from typing import Optional, Dict, Union, Any
from enum import Enum
import stp.rc as rc
import fsm

Team = bool #bool represeting if we are blue or not

class BallClassifier(fsm.StateMachine):

    class State(Enum):
        start = 1
        no_possession = 2
        our_possession = 3
        our_kick = 4
        their_possession = 5
        scramble = 6
        kickoff = 7 #represents our goal

    def __init__(self) -> None:
        
        super().__init__(start_state = BallClassifier.State.start)

        self.team : Team = None #represents the team currently with posession
        self.robot : rc.Robot = None #represents the robot which currently has possesion
        self.prev_possesions : List[rc.Robot] = None #represents the previous robots which had the ball in this possesion of the ball

        for state in BallClassifier.State:
            self.add_state(state, None)

        self.add_transition(BallClassifier.State.start, BallClassifier.State.no_possession, lambda: True, 'start')
        self.add_transition(BallClassifier.State.no_possession, BallClassifier.State.our_possession, lambda: self.team_has_possession(), 'our team gets possession')
        self.add_transition(BallClassifier.State.our_possession, BallClassifier.State.our_kick, lambda: self.we_kicked(), 'our team kicks')
        self.add_transition(BallClassifier.State.our_kick, BallClassifier.State.our_possession, lambda: self.pass_success(), 'our kick is a sucessful pass')
        self.add_transition(BallClassifier.State.our_kick, BallClassifier.State.their_possession, lambda: self.pass_intercepted(), 'our kick is an intercepted pass')
        self.add_transition(BallClassifier.State.our_kick, BallClassifier.State.scramble, lambda: self.kick_to_scramble(), 'our kick results in a scramble')
        self.add_transition(BallClassifier.State.our_kick, BallClassifier.State.kickoff, lambda: self.shot_successful(), 'our kick results in a scored goal')



    # for these functions use self._world_state to get the most recent world_state

    def on_enter_our_possession(self) -> None:
        # Should update which robot is the self.robot and which team is self.team, add this new robot to self.prev_possesions
        return 

    def on_enter_scramble(self) -> None:
        # return ros message describing what happened
        return 

    def on_enter_their_posession(self) -> None:
        # return ros message describing what happend
        return 

    def on_enter_scramble(self) -> None:
        # return ros message describing what happend
        return

    def on_enter_kickoff(self) -> None:
        # return ros message describing what happend
        return 



    def team_has_possession(self) -> bool:
        return False

    def we_kicked(self) -> bool:
        return False

    def pass_success(self) -> bool:
        return False

    def pass_intercepted(self) -> bool:
        return False

    def kick_to_scramble(self) -> bool:
        return False

    def shot_successful(self) -> bool:
        return False




