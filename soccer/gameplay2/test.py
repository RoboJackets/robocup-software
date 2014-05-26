from behavior import *
from fsm import *
from pivot_kick import *
from timed_behavior import *
from behavior_sequence import *
from capture import *
from time import *


for b in [PivotKick(), BehaviorSequence([PivotKick()]), TimedBehavior(PivotKick(), 2), Capture()]:
	b.write_diagram_png('diagrams/' + b.__class__.__name__ + '.png')


# seq2 = BehaviorSequence([PivotKick(), PivotKick(), PivotKick()])
# bhvr = BehaviorSequence([PivotKick(), TimedBehavior(PivotKick(), 0.8), PivotKick(), seq2])
# bhvr.robot = 1
# while not bhvr.is_done_running():
#     print(str(bhvr) + "\n")
#     bhvr.run()
#     sleep(0.5) # make this value bigger or smaller to control whether the TimedBehavior above will succeed

# print(str(bhvr) + "\n")
