import single_robot_behavior
import behavior
from enum import Enum
import main
import evaluation
import constants
import role_assignment
import robocup
import planning_priority
import time
import skills.move


class Capture(single_robot_behavior.SingleRobotBehavior):
    # Speed in m/s at which a capture will be handled by coarse and fine approach instead of intercept
    InterceptVelocityThresh = 0.3

    # (possibly deprecated)Multiplied by the speed of the ball to find a "dampened" point to move to during an intercept
    DampenMult = 0.05

    # Coarse Approach Tunables
    CourseApproachErrorThresh = 0.8

    # Distance at which to switch from coarse
    CourseApproachDist = 0.2

    # Stops at this point in coarse to avoid hitting the ball
    CourseApproachAvoidBall = 0.10

    ## Time in which to wait in delay state to confirm the robot has the ball
    DelayTime = 0.4

    # Default dribbler speed, can be overriden by self.dribbler_power
    ## Sets dribbler speed during intercept and fine approach
    DribbleSpeed = 128

    # Approach speed in fine
    FineApproachSpeed = 0.3

    # Cutoff of the angle between the ball velocity vector and the ball2bot vector
    InFrontOfBallCosOfAngleThreshold = 0.95

    class State(Enum):
        intercept = 0
        course_approach = 1
        fine_approach = 2
        delay = 3

    ## Capture Constructor
    # faceBall - If false, any turning functions are turned off,
    # useful for using capture to reflect/bounce moving balls.
    def __init__(self, faceBall=True):
        super().__init__(continuous=False)

        self.dribbler_power = Capture.DribbleSpeed

        # Declare all states to have a running state
        for state in Capture.State:
            self.add_state(state, behavior.Behavior.State.running)

        # State Transistions
        self.add_transition(
            behavior.Behavior.State.start, Capture.State.intercept,
            lambda: True, 'immediately')

        # Intercept to coarse
        #   Ball vel is slow / not moving at us
        self.add_transition(
            Capture.State.intercept, Capture.State.course_approach,
            lambda: main.ball().vel.mag() < Capture.InterceptVelocityThresh or not self.ball_moving_towards_bot(),
            'moving to coarse')

        # Coarse to intercept
        #   Ball vel is fast and moving at us
        self.add_transition(
            Capture.State.course_approach, Capture.State.intercept,
            lambda: main.ball().vel.mag() >= Capture.InterceptVelocityThresh and self.ball_moving_towards_bot(),
            'intercepting')

        # Fine to intercept
        #   Ball vel is fast and moving at us
        self.add_transition(
            Capture.State.fine_approach, Capture.State.intercept,
            lambda: main.ball().vel.mag() >= Capture.InterceptVelocityThresh and self.ball_moving_towards_bot(),
            'intercepting')

        # Coarse to fine
        #   Within a distance cutoff
        self.add_transition(
            Capture.State.course_approach, Capture.State.fine_approach,
            lambda: self.bot_near_ball(Capture.CourseApproachDist) and main.ball().valid,
            'dist to ball < threshold')

        # Fine to coarse
        #   Not within a distance cutoff
        self.add_transition(
            Capture.State.fine_approach, Capture.State.course_approach,
            lambda: not self.bot_near_ball(Capture.CourseApproachDist * 1.1) and main.ball().valid,
            'dist to ball > threshold')

        #DELAY STATES
        #   Ready to delay to make sure ball is actually caught
        self.add_transition(
            Capture.State.fine_approach, Capture.State.delay,
            lambda: evaluation.ball.robot_has_ball(self.robot),
            'has ball')

        #   Ball rolls away during wait
        self.add_transition(
            Capture.State.delay, Capture.State.fine_approach,
            lambda: not evaluation.ball.robot_has_ball(self.robot),
            'lost ball during delay')

        #   Ball stays and delay finishes
        self.add_transition(
            Capture.State.delay, behavior.Behavior.State.completed,
            lambda: time.time() - self.start_time > Capture.DelayTime and
            evaluation.ball.robot_has_ball(self.robot),
            'delay finish')

        # Last coarse target
        # Used to stop the target from constantly shifting during following updates if it's somewhat close
        # to the original target
        self.lastApproachTarget = None

        # Whether to face the ball while moving around
        self.faceBall = faceBall

    def bot_to_ball(self):
        return main.ball().pos - self.robot.pos

    # Both within distance of ball
    def bot_near_ball(self, distance):
        return (self.bot_to_ball().mag() < distance)

    # Return true if ball is moving towards us (+- cos^-1(.95)) and will reach us (Not stop before reaching us)
    def bot_in_front_of_ball(self):
        ball2bot = self.bot_to_ball() * -1
        return (ball2bot.normalized().dot(main.ball().vel) > Capture.InFrontOfBallCosOfAngleThreshold) and 
               ((ball2bot).mag() < (evaluation.ball.predict_stop(main.ball().pos, main.ball().vel) - main.ball().pos).mag())

    # Check if the ball is moving in our direction (+- 90 degrees of us)
    def ball_moving_towards_bot(self):
        return (main.ball().pos).dist_to(self.robot.pos) < (main.ball().vel + main.ball().pos).dist_to(self.robot.pos)

    # calculates intercept point for the fast moving intercept state
    def find_intercept_point(self):
        return find_robot_intercept_point(self.robot)

    # returns intercept point for the slow moving capture states
    def find_capture_point(self):
        return find_robot_capture_point(self.robot)

    # House keeping
    def execute_running(self):
        self.robot.set_planning_priority(planning_priority.CAPTURE)

        # Always face ball if set
        if (self.faceBall):
            self.robot.face(main.ball().pos)

    # Dribbles and moves to intercept point
    def execute_intercept(self):
        self.robot.set_dribble_speed(self.dribbler_power)
        self.robot.disable_avoid_ball()
        pos = self.find_intercept_point()
        self.robot.move_to(pos)

    # Clear any old data
    def on_enter_course_approach(self):
        self.lastApproachTarget == None

    # Find point to move to and try to avoid the ball when close enough
    def execute_course_approach(self):
        self.robot.set_dribble_speed(self.dribbler_power)
        pos = self.find_capture_point()

        # If the capture target is almost where it used to be, just keep the old one
        if (self.lastApproachTarget != None and
            (pos - self.lastApproachTarget).mag() < 0.1):
            pos = self.lastApproachTarget

        self.lastApproachTarget = pos

        # Stop the robot before accidently hitting the ball
        if pos.dist_to(main.ball().pos) < Capture.CourseApproachAvoidBall + constants.Robot.Radius:
            self.robot.disable_avoid_ball()
        else:
            self.robot.set_avoid_ball_radius(Capture.CourseApproachAvoidBall)

        self.robot.move_to(pos)

        # White circle is where we are trying to capture ball at
        main.system_state().draw_circle(self.lastApproachTarget,
                                        constants.Ball.Radius,
                                        constants.Colors.White, "Capture")

    def on_exit_course_approach(self):
        self.lastApproachTarget is None

    def execute_fine_approach(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(self.dribbler_power)

        # TODO(ashaw596): explain this math a bit
        bot2ball = (main.ball().pos - self.robot.pos).normalized()
        multiplier = 1.5
        aproach = self.bot_to_ball() * multiplier + bot2ball * Capture.FineApproachSpeed / 4 + main.ball().vel
        if (aproach.mag() > 1):
            aproach = aproach.normalized() * 1
        self.robot.set_world_vel(aproach)

    def on_enter_delay(self):
        self.start_time = time.time()

    def execute_delay(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(self.dribbler_power)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True

        for r in role_assignment.iterate_role_requirements_tree_leaves(reqs):
            if main.ball().valid:
                if self.state == Capture.State.intercept:
                    reqs.cost_func = lambda r: robocup.Line(main.ball().pos, main.ball().pos + main.ball().vel * 10).dist_to(r.pos)
                else:
                    reqs.cost_func = lambda r: main.ball().pos.dist_to(r.pos)
        return reqs

# Robot based helper functions
# calculates intercept point for the fast moving intercept state
def find_robot_intercept_point(robot):
    if (robot is not None):
        passline = robocup.Line(main.ball().pos, main.ball().pos + main.ball().vel * 10)
        pos = passline.nearest_point(robot.pos) + (main.ball().vel * Capture.DampenMult)
        return pos
    else:
        return None

# Finds best point to capture at for fine approach
# Samples ever X cm in front of the ball figuring out which point we can reach before the ball
def find_robot_capture_point(robot):
    if robot is None:
        return main.ball().pos

    approach_vec = approach_vector(robot)
    pos = None

    # For 50 samples
    for i in range(50):
        # Check every 5 cm along approach vector
        dist = i * 0.05
        pos = main.ball().pos + main.ball().vel + approach_vec * dist

        # How long it will take us and the ball to reach the point
        # Note: This isn't actually accurate as it doesn't account for other robots in the way
        # It also has the wrong accelerations and velocities
        ball_time = evaluation.ball.rev_predict(main.ball().vel, dist)
        robotDist = (pos - robot.pos).mag() * 0.6
        bot_time = robocup.get_trapezoidal_time(robotDist, robotDist, 2.2, 1,
                                                robot.vel.mag(), 0)

        # First time we reach the point before the ball
        if bot_time < ball_time:
            break

    return pos

# Gets direction of the ball velocity when we are far away
# and the vector between the robot and the ball when we are close
def approach_vector(robot):
    if (main.ball().vel.mag() > 0.25 and
        robot.pos.dist_to(main.ball().pos) > 0.2):
        # ball's moving, get on the side it's moving towards
        return main.ball().vel.normalized()
    else:
        return (robot.pos - main.ball().pos).normalized()
