import robocup
import constants
import play
import enum
import behavior
import main
import tactics


# Maintains the state of the ball's position by keeping track of which
# half the ball is on and prints on both entering a given state and
# continously during the execution of a given state.
class triangle_pass(play.Play):
    class State(enum.Enum):
        # Define your states here.
        # eg: some_state = 0
        # -----------------------
        state1 = 0
        state2 = 1
        state3 = 2

    def __init__(self):
        super().__init__(continuous=True)

        # Register the states you defined using 'add_state'.
        # eg: self.add_state(WhichHalf.State.<???>,
        #                    behavior.Behavior.State.running)
        # ----------------------------------------------------

        # Assume we're either in tophalf or bottomhalf, no state for
        # being right on the line.
        self.add_state(triangle_pass.State.state1,
                       behavior.Behavior.State.running)
        self.add_state(triangle_pass.State.state2,
                       behavior.Behavior.State.running)
        self.add_state(triangle_pass.State.state3,
                       behavior.Behavior.State.running)

        # Add your state transitions using 'add_transition'.
        # eg: self.add_transition(behavior.Behavior.State.start,
        #                         self.State.<???>, lambda: True,
        #                         'immediately')
        # eg: self.add_transition(self.State.<???>, self.State.<???>,
        #                         lambda: <???>,
        #                         'state change message')
        # ------------------------------------------------------------


        # Rather than defining two more transitions from start to the top or
        # bottom half, we simply assume we start in bottom and let bottom's
        # transition function sort it out.
        self.add_transition(behavior.Behavior.State.start,
                            self.State.state1, lambda: True, 'immediately')

        self.add_transition(self.State.state1,
                            self.State.state2, lambda: self.all_subbehaviors_completed(), 'immediately')

        self.add_transition(self.State.state2, self.State.state3,
                            lambda: self.all_subbehaviors_completed(), 'immediately')

        self.add_transition(self.State.state3, self.State.state1,
                            lambda: self.all_subbehaviors_completed(), 'immediately')

    # Define your own 'on_enter' and 'execute' functions here.
    # eg: def on_enter_<???>(self):
    #         print('Something?')
    # eg: def execute_<???>(self):
    #         print('Something?')
    # ---------------------------------------------------------

    def on_enter_state1(self):
        move_point = robocup.Point(1,2)
        pass_tactic = tactics.coordinated_pass.CoordinatedPass(move_point)
        self.add_subbehavior(pass_tactic, "tactic")

    def on_enter_state2(self):
        move_point = robocup.Point(0,3)
        pass_tactic = tactics.coordinated_pass.CoordinatedPass(move_point)
        self.add_subbehavior(pass_tactic, "tactic")

    def on_enter_state3(self):
        move_point = robocup.Point(-1,2)
        pass_tactic = tactics.coordinated_pass.CoordinatedPass(move_point)
        self.add_subbehavior(pass_tactic, "tactic")

    def on_exit_state1(self):
        self.remove_all_subbehaviors()

    def on_exit_state2(self):
        self.remove_all_subbehaviors()

    def on_exit_state3(self):
        self.remove_all_subbehaviors()
