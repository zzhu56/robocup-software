import play
import behavior
import skills.pivot_kick
import skills.move
import enum
import main
import constants
import robocup


# this test repeatedly runs the PivotKick behavior aimed at our goal
class TestKicker(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        kick = skills.pivot_kick.PivotKick()
        kick.target = constants.Field.OurGoalSegment.center()
        kick.aim_params['desperate_timeout'] = 5
        self.add_subbehavior(kick, 'kick', required=False)

        self.timer = 0
        self.max_timer = 10 # When to print out everything
        self.timer_start = False
        self.already_printed = False
        self.cached_shot_point = None

    def execute_running(self):
        kick = self.subbehavior_with_name('kick')

        if main.ball().vel.mag() > 0.5 and not self.already_printed:
            self.timer_start = True

        if main.ball().vel.mag() < 0.5:
            self.already_printed = False

        if kick.is_done_running():
            kick.restart()

        if self.timer_start:
            self.timer += 1

        if self.timer == self.max_timer:
            self.timer = 0
            self.timer_start = False
            self.already_printed = True

        if kick.state == skills.pivot_kick.PivotKick.State.kicking:
            print("kicking now")

        if main.ball().pos.y < 0.1:
            print("low y value")

        if kick.current_shot_point() is not None:
            self.cached_shot_point = kick.current_shot_point()

        self.printKickInfo(kick)


    def printKickInfo(self, kick):
        if kick and kick.robot and self.cached_shot_point:
            ball_vel = main.ball().vel
            ball_dir = main.ball().pos - kick.robot.pos

            angle_error = ball_dir.angle_between(self.cached_shot_point - kick.robot.pos)

            print(str(ball_vel.mag()) + " m/s with the angle error " + str(angle_error * 180 / 3.14159))
        else:
            print("no robot assigned")
