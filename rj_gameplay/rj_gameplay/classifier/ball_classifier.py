from typing import Optional, Dict, Union, Any
from enum import Enum
import stp.rc as rc
import stp.utils.fsm as fsm
import fsm

Team = bool #bool represeting if we are blue or not

class BallClassifier(fsm.StateMachine):

    class State(Enum):
        no_possession = 1
        our_possession = 2
        our_kick = 3
        their_possession = 4
        scramble = 5
        kickoff = 6

    def __init__(self) -> None:
        
        super().__init__(start_state = BallClassifier.State.no_possession)

        self.team : Team = None #represents the team currently with posession
        self.robot : rc.Robot = None #represents the robot which currently has possesion
        self.prev_possesions : List[rc.Robot] = None #represents the previous robots which had the ball in this possesion of the ball


        self.add_transition(BallClassifier.State.no_possession, BallClassifier.State.our_possession,
            lambda: self.team_has_possession(rc.WorldState), 'our team gets possession')
        self.add_transition(BallClassifier.State.our_possession, BallClassifier.State.our_kick,
            lambda: self.we_kicked(rc.WorldState), 'our team kicks')
        self.add_transition(BallClassifier.State.our_kick, BallClassifier.State.our_possession,
            lambda: self.pass_success(rc.WorldState), 'our kick is a sucessful pass')
        self.add_transition(BallClassifier.State.our_kick, BallClassifier.State.their_possession,
            lambda: self.pass_intercepted(rc.WorldState), 'our kick is an intercepted pass')
        self.add_transition(BallClassifier.State.our_kick, BallClassifier.State.scramble,
            lambda: self.kick_to_scramble(rc.WorldState), 'our kick results in a scramble')
        self.add_transition(BallClassifier.State.our_kick, BallClassifier.State.kickoff,
            lambda: self.shot_successful(rc.WorldState), 'our kick results in a scored goal')

    def on_enter_our_possession(self) -> None:
        # Should update which robot is the self.robot and which team is self.team, add this new robot to self.prev_possesions
        return 

    def on_enter_scramble(self) -> None:
        # return ros message describing what happened
        return 

    def on_enter_their_possession(self) -> None:
        # return ros message describing what happend
        return 

    def on_enter_scramble(self) -> None:
        # return ros message describing what happend
        return

    def on_enter_kickoff(self) -> None:
        # return ros message describing what happend
        return 



    def team_has_possession(self, world_state: rc.WorldState) -> bool:
        return False

    def we_kicked(self, world_state: rc.WorldState) -> bool:
        return False

    def pass_success(self, world_state: rc.WorldState) -> bool:
        return False

    def pass_intercepted(self, world_state: rc.WorldState) -> bool:
        return False

    def kick_to_scramble(self, world_state: rc.WorldState) -> bool:
        return False

    def shot_successful(self, world_state: rc.WorldState) -> bool:
        return False



