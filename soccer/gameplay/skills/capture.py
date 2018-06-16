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
import math


class Capture(single_robot_behavior.SingleRobotBehavior):

    # tunable config values
    CourseApproachErrorThresh = 0.8
    CourseApproachDist = 0.4
    CourseApproachAvoidBall = 0.10
    DelayTime = .2
    InterceptVelocityThresh = 0.4
    DampenMult = 0.06

    SigmoidLimit = 2.0
    SigmoidExp = 2.0
    SigmoidShift = 1.0

    # Default dribbler speed, can be overriden by self.dribbler_power
    DribbleSpeed = 100
    FineApproachSpeed = 0.1

    InFrontOfBallCosOfAngleThreshold = 0.95

    class State(Enum):
        intercept = 0
        approach = 1
        capture = 2
        delay = 3

    ## Capture Constructor
    # faceBall - If false, any turning functions are turned off,
    # useful for using capture to reflect/bounce moving balls.
    def __init__(self, faceBall=True):
        super().__init__(continuous=False)

        for state in Capture.State:
            self.add_state(state,behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Capture.State.intercept, lambda: True,
                            'immediately')

        self.add_transition(
            Capture.State.intercept,
            Capture.State.approach, lambda: main.ball().vel.mag() < Capture.InterceptVelocityThresh,
            'moving to dampen')

        self.add_transition(
            Capture.State.approach,
            Capture.State.delay,
            lambda: self.robot.has_ball_raw(),
            'has ball')

        self.add_transition(
            Capture.State.delay,
            Capture.State.approach,
            lambda: not self.robot.has_ball_raw(),
            'lost ball during delay')

        self.add_transition(
            Capture.State.delay,
            behavior.Behavior.State.completed,
            lambda: time.time() - self.start_time > Capture.DelayTime and
            self.robot.has_ball_raw(),
            'delay before finish')

        self.dribbler_power = Capture.DribbleSpeed

        self.lastApproachTarget = None
        self.faceBall = faceBall

    def bot_to_ball(self):
        return main.ball().pos - self.robot.pos

    def bot_near_ball(self, distance):
        return (self.bot_to_ball().mag() < distance)

    def bot_in_front_of_ball(self):
        if (self.robot is not None):
            return self.robot.has_ball_raw()
        else:
            return None

    # calculates intercept point for the fast moving intercept state
    def find_intercept_point(self):
        return find_robot_intercept_point(self.robot)

    def find_approach_point(self):
        pos = main.ball().pos + main.ball().vel
        main.system_state().draw_circle(main.ball().pos, main.ball().vel.mag(), constants.Colors.White, "approach radius")
        return pos

    def find_capture_speed(self):
            return Capture.SigmoidLimit / (1 + math.pow(math.e, (Capture.SigmoidExp * ((main.ball().pos - self.robot.pos).mag() + Capture.SigmoidShift)))) 

    def execute_running(self):
        self.robot.set_planning_priority(planning_priority.CAPTURE)
        if (self.faceBall):
            self.robot.face(main.ball().pos)

    # sets move subbehavior
    def execute_intercept(self):
        pos = self.find_intercept_point()
        self.robot.move_to(pos)

    #TODO:set world vel only takes a vector. Also set it so that it approaches the point where the ball is at plus the radius
    def execute_approach(self):
        pos = self.find_approach_point()
        self.robot.move_to(pos)

    def on_enter_delay(self):
        self.start_time = time.time()

    def execute_delay(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(self.dribbler_power)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # find appropriate robot by finding the robot closest to the kick vector
        # TODO: multiply this by a "distance normal curve" rather than a raw multiple
        if main.ball().valid:
            reqs.cost_func = lambda r: reqs.position_cost_multiplier * (find_robot_intercept_point(r).dist_to(r.pos) / (main.ball().pos).dist_to(r.pos))
        return reqs

# calculates intercept point for the fast moving intercept state
def find_robot_intercept_point(robot):
    passline = robocup.Line(main.ball().pos, main.ball().pos + main.ball().vel * 10)
    pos = passline.nearest_point(robot.pos) + (main.ball().vel * Capture.DampenMult)
    return pos

