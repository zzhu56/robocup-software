import play
import behavior
import skills.move
import robocup
import main
import enum

class TestBug(play.Play):
    class State(enum.Enum):
        setup = 1
        bugState = 2

    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(TestBug.State.setup,
                       behavior.Behavior.State.running)
        self.add_state(TestBug.State.bugState,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            TestBug.State.setup, lambda: True,
                            'immediately')

        self.add_transition(TestBug.State.setup,
                            TestBug.State.bugState, lambda: self.has_subbehavior_with_name('circle') and 
                                                            self.subbehavior_with_name('circle').robot is not None and 
                                                            self.subbehavior_with_name('circle').robot.pos.x > 0,
                            'show bug')

    def on_enter_setup(self):
        self.add_subbehavior(
            skills.move.Move(robocup.Point(1,2)),
            name='circle',
            required=True)

    def on_exit_setup(self):
        self.remove_subbehavior('circle')

    def on_enter_bugState(self):
        self.add_subbehavior(
            skills.move.Move(robocup.Point(-1,2)),
            name='circle',
            required=True)

    def on_exit_bugState(self):
        self.remove_subbehavior('circle')
