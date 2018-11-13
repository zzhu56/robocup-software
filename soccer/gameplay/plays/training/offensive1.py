import robocup
import constants
import play
import enum
import behavior
import main
import tactics
import skills
import evaluation

# Maintains the state of the ball's position by keeping track of which
# half the ball is on and prints on both entering a given state and
# continously during the execution of a given state.
class offensive1(play.Play):
    class State(enum.Enum):
        # Define your states here.
        # eg: some_state = 0
        # -----------------------
        starting = 0
        passing = 1
        shooting = 2

    

    def __init__(self):
        super().__init__(continuous=True)

        # Register the states you defined using 'add_state'.
        # eg: self.add_state(WhichHalf.State.<???>,
        #                    behavior.Behavior.State.running)
        # ----------------------------------------------------

        # Assume we're either in tophalf or bottomhalf, no state for
        # being right on the line.
        self.add_state(offensive1.State.starting,
                       behavior.Behavior.State.running)
        self.add_state(offensive1.State.passing,
                       behavior.Behavior.State.running)
        self.add_state(offensive1.State.shooting,
                       behavior.Behavior.State.running)

        # Add your state transitions using 'add_transition'.
        # eg: self.add_transition(behavior.Behavior.State.start,
        #                         self.State.<???>, lambda: True,
        #                         'immediately')
        # eg: self.add_transition(self.State.<???>, self.State.<???>,
        #                         lambda: <???>,
        #                         'state change message')
        # ------------------------------------------------------------
        self.move_point1 = robocup.Point(-1,4)
        self.move_point2 = robocup.Point(1,4)

       

        # Rather than defining two more transitions from start to the top or
        # bottom half, we simply assume we start in bottom and let bottom's
        # transition function sort it out.
        self.add_transition(behavior.Behavior.State.start,
                            self.State.starting, lambda: True, 'immediately')

        self.add_transition(self.State.starting,
                            self.State.shooting, lambda: (self.to_shoot() and self.all_subbehaviors_completed()), 'immediately')

        self.add_transition(self.State.starting,
                            self.State.passing, lambda: ((not self.to_shoot()) and self.all_subbehaviors_completed()), 'immediately')

        self.add_transition(self.State.passing,
                            self.State.shooting, lambda: ((not self.to_shoot()) and self.all_subbehaviors_completed()), 'immediately')
 
        self.add_transition(self.State.shooting,
                            self.State.starting, lambda: self.all_subbehaviors_completed(), 'immediately')
 

    # Define your own 'on_enter' and 'execute' functions here.
    # eg: def on_enter_<???>(self):
    #         print('Something?')
    # eg: def execute_<???>(self):
    #         print('Something?')
    # ---------------------------------------------------------
 
    def to_shoot(self):
        ball = main.ball().pos
        shot_success = evaluation.shooting.eval_shot(ball,main.our_robots())
        shot_success1 = evaluation.shooting.eval_shot(self.move_point1,main.our_robots())
        shot_success2 = evaluation.shooting.eval_shot(self.move_point2,main.our_robots())
        pass_success1 = evaluation.passing.eval_pass(ball,self.move_point1,main.our_robots())
        pass_success2 = evaluation.passing.eval_pass(ball,self.move_point2,main.our_robots())
        

        self.pass_shot1 = pass_success1 * shot_success1
        self.pass_shot2 = pass_success2 * shot_success2
        return shot_success >= self.pass_shot1 and shot_success >= self.pass_shot2

    def on_enter_starting(self):
        cap_ball = skills.capture.Capture()
        self.add_subbehavior(cap_ball, "capture")
        skill1 = skills.move.Move(self.move_point1)
        skill2 = skills.move.Move(self.move_point2)
        self.add_subbehavior(skill1, "skill1")
        self.add_subbehavior(skill2, "skill2")


    def on_enter_shooting(self):
        skill = skills.pivot_kick.PivotKick()
        self.add_subbehavior(skill, "skill")

    def on_enter_passing(self):
        if self.pass_shot1 >= self.pass_shot2:
            skill = tactics.coordinated_pass.CoordinatedPass(self.move_point1)
        else:
            skill = tactics.coordinated_pass.CoordinatedPass(self.move_point2)
        self.add_subbehavior(skill, "skill")

    def on_exit_starting(self):
        self.remove_all_subbehaviors()

    def on_exit_shooting(self):
        self.remove_all_subbehaviors()

    def on_exit_passing(self):
        self.remove_all_subbehaviors()
