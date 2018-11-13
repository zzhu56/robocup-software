import robocup
import constants
import play
import skills
import tactics

# This is a file where you can learn how skills work!
class SimpleBehaviors(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        # To make a robot move, use skills.move.Move(<point to move to>)
        # To create a point, we initialize a point using 
        # robocup.Point(<x coordinate>, <y coordinate>)
        
        # These lines moves a robot to the point (0, 0)
        move_point = robocup.Point(2,constants.Field.Length/2)
        #skill = skills.move.Move(move_point)
        #skill1 = skills.capture.Capture()
        skill2 = skills.pivot_kick.PivotKick()

        # tactics
        pass_tactic = tactics.coordinated_pass.CoordinatedPass(move_point)
        
        #defense
        
        # Adds behavior to our behavior tree, we will explain this more later
        #self.add_subbehavior(skill, "skill")
        #self.add_subbehavior(skill1, "skill1")
        #self.add_subbehavior(skill2, "skill2")
        self.add_subbehavior(pass_tactic, "tactic")