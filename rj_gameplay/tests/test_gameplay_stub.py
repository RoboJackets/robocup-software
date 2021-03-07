import stp.play as play
import stp.tactic as tactic

import rj_gameplay.tactic as tactic
import rj_gameplay.tactic as tactic
import stp.role as role
from rj_gameplay.play import stub_basic122
import stp.testing as testing
import stp.rc as rc
import rclpy
import numpy as np
import stp.coordinator as coordinator
import stp.situation as situation
import stp
import rj_gameplay.gameplay_node as Gameplay_node

from typing import Dict, Tuple

class PlaySelector(situation.IPlaySelector):
    def select(self, world_state: rc.WorldState) -> Tuple[situation.ISituation, stp.play.IPlay]:
        return (None, stub_basic122.Basic122())

def make_our_robots():
    our_bots = []
    for i in range(6):
        r_id = i
        is_ours = True
        pos = np.array([1,i/3, 0])
        twist = np.array([1,i/3, 0])
        visible = True
        has_ball_sense = False
        kicker_charged = False
        kicker_healthy= False
        lethal_fault = False
        our_bots.append(rc.Robot(r_id,is_ours,pos, twist, visible, has_ball_sense, kicker_charged, kicker_healthy, lethal_fault))
    return our_bots

our_robots = make_our_robots()
world_state = testing.generate_test_worldstate(our_robots=our_robots)


basic = stub_basic122.Basic122()

new_role_results, skills = basic.tick(world_state, {}, None)
play_selector = PlaySelector()

gameplay = coordinator.Coordinator(play_selector)
gameplay._props = {}

for i in range(5):
    print('Tick #',i+1)
    gameplay.tick(world_state)
    print('\n')