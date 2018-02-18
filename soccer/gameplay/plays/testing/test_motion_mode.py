import play
import behavior
import robocup
import constants
import main
import skills.move


class TestMotionMode(play.Play):
    VELX = 1
    VELY = 1
    VELW = 1

    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        i = 0
        for r in main.our_robots():
            r.set_motion_control_mode(robocup.mc_mode.STEP)

            if (r.pos.x < 0):
                r.set_step(TestMotionMode.VELX,TestMotionMode.VELY, TestMotionMode.VELW)
            else:
                r.set_step(-TestMotionMode.VELX,-TestMotionMode.VELY, TestMotionMode.VELW)

            # mov = skills.move.Move(target)
            # mov.shell_id = i
            # self.add_subbehavior(mov, 'move' + str(i), priority=6-i, required=False) 
            i += 1
            

    def on_exit_running(self):
        self.remove_all_subbehaviors()
        for r in main.our_robots():
            r.set_motion_control_mode(robocup.mc_mode.PID)
            
    