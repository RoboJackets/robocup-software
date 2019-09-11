import robocup
import play
import behavior
import skills.move
import skills.capture
import tactics.coordinated_pass
import constants
import main
import enum
import tools
import skills.latency_test
import skills.benchmark.motion_benchmark

class RunMotionBenchmark(play.Play):
    class State(enum.Enum):
        
        #Move to a central location 
        run = 1
        done = 2

    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(RunMotionBenchmark.State.run,
                       behavior.Behavior.State.running)
        self.add_state(RunMotionBenchmark.State.done,
                       behavior.Behavior.State.running)
        
        self.add_transition(behavior.Behavior.State.start,
                            RunMotionBenchmark.State.run, lambda: True,
                            'immediately')

        self.add_transition(RunMotionBenchmark.State.run,
                            behavior.Behavior.State.completed,
                            lambda: self.all_subbehaviors_completed(),
                            'all subbehaviors completed')
  
    def on_enter_run(self):
        self.add_subbehavior(skills.benchmark.motion_benchmark.MotionBenchmark(), 'move2')

