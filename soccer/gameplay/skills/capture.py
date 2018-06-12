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
    DelayTime = .1
    AvoidThresh = 0.10
    InterceptVelocityThresh = 0.1
    DampenMult = 0.08

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
        delay = 2

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
            Capture.State.delay, lambda: evaluation.ball.robot_has_ball(self.robot),
            'has ball')

        self.add_transition(
            Capture.State.delay,
            behavior.Behavior.State.completed,
            lambda: time.time() - self.start_time > Capture.DelayTime and
            evaluation.ball.robot_has_ball(self.robot),
            'delay before finish')

        self.add_transition(
            Capture.State.delay,
            Capture.State.approach,
            lambda: not evaluation.ball.robot_has_ball(self.robot),
            'lost ball during delay')

        self.dribbler_power = Capture.DribbleSpeed

        self.lastApproachTarget = None
        self.faceBall = faceBall

    def bot_to_ball(self):
        return main.ball().pos - self.robot.pos

    def bot_near_ball(self, distance):
        return (self.bot_to_ball().mag() < distance)

    def bot_in_front_of_ball(self):
        ball2bot = self.bot_to_ball() * -1
        return (ball2bot.normalized().dot(main.ball().vel) > Capture.InFrontOfBallCosOfAngleThreshold) and \
                ((ball2bot).mag() < (evaluation.ball.predict_stop(main.ball().pos, main.ball().vel) - main.ball().pos).mag())

    # normalized vector pointing from the ball to the point the robot should get to in course_aproach

    def find_intercept(self):
        return find_moving_robot_intercept(self.robot)

    def execute_running(self):
        self.robot.set_planning_priority(planning_priority.CAPTURE)

        if (self.faceBall):
            self.robot.face(main.ball().pos)

    def execute_intercept(self):
        pos = self.find_intercept()
        self.robot.move_to(pos)

    #TODO:set world vel only takes a vector. Also set it so that it approaches the point where the ball is at plus the radius
    def execute_approach(self):
        distToBall = (main.ball().pos - self.robot.pos).mag() 
        if (distToBall < Capture.AvoidThresh):
            self.robot.disable_avoid_ball()
            self.robot.set_dribble_speed(self.dribbler_power)
        approachSpeed = Capture.SigmoidLimit / (1 + math.pow(math.e, (Capture.SigmoidExp * (distToBall + Capture.SigmoidShift)))) 
        approachVector = approachSpeed * robocup
        self.robot.set_world_vel(approachSpeed)
        self.robot.move_to(main.ball.pos)

    def on_enter_delay(self):
        self.start_time = time.time()

    def execute_delay(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(self.dribbler_power)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # try to be near the ball
        if main.ball().valid:
            reqs.cost_func = lambda r: reqs.position_cost_multiplier * find_moving_robot_intercept(
                r).dist_to(r.pos)

        return reqs

# calculates intercept point for the fast moving intercept state
def find_moving_robot_intercept(robot):
    passline = robocup.Line(main.ball().pos, main.ball().pos + main.ball().vel * 10)
    pos = passline.nearest_point(robot.pos) + (main.ball().vel * Capture.DampenMult)
    return pos
