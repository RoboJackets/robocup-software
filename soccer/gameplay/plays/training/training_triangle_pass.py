import skills.move
import skills.capture
import tactics.coordinated_pass
import constants
import main
import enum
import robocup
import play
import behavior


## A demo play written during a teaching session to demonstrate play-writing
# Three robots form a triangle on the field and pass the ball A->B->C->A and so on.
class TrianglePass(play.Play):

	#set up your states here
	class State(enum.Enum):
		#example: 
		stateA = 1
		stateB = 2
	def __init__(self):
		super().__init__(continuous=True)

		'''
		Make sure you add all your states:

		Example:
		self.add_state(TrianglePass.State.stateA, behavior.Behavior.State.running)
		
		Bonus points if you can do this with a loop
		'''

		#fill in what points you want to use
		#rember: robocup.Point(x,y), constants.Field.Length, constants.Field.Width
		self.triangle_points = []

		'''
		add state transitions 

		Example:
		self.add_transition(TrianglePass.State.stateA,
                            TrianglePass.State.stateB, lambda: conditionToSwitch,
                            'whats happening with switch')
		
		Hints:
		To start into your code you will also need a state transition from
		behavior.Behavior.State.start to your first state.
		'''
	

	'''
	will need to describe your state next using the following
	useful things:
	capture.Capture()
	move.Move(robocup.Point(x,y))
	coordinated_pass.CoordinatedPass(robocup.Point(x,y))
	main.ball().pos
	
	how to add subbehaviors
	self.add_subbehavior(skill_object,  "Name of Subbehavior",
		required=False or True,  priority=10)
	
	how to remove subbehaviors
	self.remove_subbehavior('Name of Subbehavior')
	self.remove_all_subbehaviors()
	'''


	# change this name to your state
	# code that runs once at the start of the state
	# Usually good for setting up subbehaviors
	# you may or may not need this for your state

	#def on_enter_stateA:
	
	# change this name to your state
	# code that runs continously while in the state
	# usually good for updating needed values
	# you may or may not need this for your state
	
	#def execute_stateA:

	# change this name to your state
	# code that runs once at the end of the state
	# Needed to remove subbehaviors before going to the next state
	# usually necessary so we don't get too many subbehaviors
	
	#def on_exit_stateA:
