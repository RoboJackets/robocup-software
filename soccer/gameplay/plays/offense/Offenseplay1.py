import robocup
import standard_play
import behavior
import constants
import main
import skills.move
import skills.capture
import enum
import evaluation
import tactics.coordinated_pass
import tactics.defense
import play

class Offenseplay(play.Play):
	class State(enum.Enum):

		setup = 1
		passing = 2
		shooting = 3
	def __init__(self):
		super().__init__(continuous=True)

		self.add_state(Offenseplay.State.setup,
						behavior.Behavior.State.running)
		self.add_state(Offenseplay.State.passing,
						behavior.Behavior.State.running)
		self.add_state(Offenseplay.State.shooting,
						behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,
						Offenseplay.State.setup, lambda: True, 'immediately')
		self.add_transition(Offenseplay.State.setup,
                            Offenseplay.State.passing,
                            lambda: self.all_subbehaviors_completed(),
                            'moving completed')
		self.add_transition(Offenseplay.State.passing,
                            Offenseplay.State.shooting,
                            lambda: self.all_subbehaviors_completed(),
                            'passing completed')
		self.add_transition(Offenseplay.State.shooting, 
							behavior.Behavior.State.completed,
            				lambda: self.all_subbehaviors_completed(),
            				'Kick completed')
		self.r1 = robocup.Point(1,5)
		self.r2 = robocup.Point(-1,5)

		

	def on_enter_move(self):
		self.add_subbehavior(skills.move.Move(r1))
		self.add_subbehavior(skills.move.Move(r2))
		self.add_subbehavior(skills.capture.Capture())
		
		

	def on_exit_move(self):
		self.remove_all_subbehaviors()
		
		

	def on_enter_passing(self):
		passchance1 = evaluation.passing.eval_pass( main.ball().pos, self.r1, main.our_robots() )
		passchance2 = evaluation.passing.eval_pass( main.ball().pos, self.r2, main.our_robots() )
		shotchance = evaluation.shooting.eval_shot( main.ball().pos, main.our_robots() )
		shotchance1 = evaluation.shooting.eval_shot( self.r1, main.our_robots() )
		shotchance2 = evaluation.shooting.eval_shot( self.r2, main.our_robots() )
		
		if passchance1*shotchance1 >= shotchance:
				self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.r1))

		else:
				if asschance2*shotchance2 >= shotchance:
					
					self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.r2))

						

				else:
					self.remove_all_subbehaviors() 
		

		

	def on_exit_passing(self):
		self.remove_all_subbehaviors()

	def on_enter_shooting(self):
		self.add_subbehavior(skills.capture.Capture())
		kick = skills.pivot_kick.PivotKick()
		kick.target = constants.Field.TheirGoalSegment

	def on_exit_shooting(self):
		self.remove_all_subbehaviors()








