# import rj_msgs
# import interfaces
# import typing
from tactic import goalie, waller, marker
from utils import find_wall_pts

class PBasicDefense(IPlay):
    """
    Basic Defense assigns 1 goalie (robot 0), 3 wallers, and 2 markers based on every robot's distance to the ball. 
    """
    def __init__(self):
        self.tactics = [
            goalie.Goalie(0),
            waller.Waller(1),
            waller.Waller(2),
            waller.Waller(3),
            marker.Marker(4),
            marker.Marker(5)
        ]
        self.goalie_id = 0

    def tick(
        self,
        world_state: rc.WorldState,
        situation: ISituation
        ) -> List[msg.RobotIntent]:

        unused_ids = [i for i in range(6)]

        unused_ids -= [self.goalie_id]

        # find three closest robots to wall_pt 
        wall_pts = utils.find_wall_pts(3, world_state)
        closest_robots = []
        # ...
        for i in closest_robots:
            self.tactics[i] = waller.Waller(i)
        unused_ids -= closest_robots

        # two other non-goalies are thus markers
        for i in unused_ids:
            self.tactics[i] = marker.Marker(i)

        intents = [tactic.tick(world_state, situation) for tactic in self.tactics]
        return intents

    def is_done(self, world_state):
        # find closest robot to ball
        closest_robot = 0
        closest_dist = np.linalg.norm(world_state.our_robots[i] - world_state.ball.pos)

        for i in range(6):
            dist_to_ball = np.linalg.norm(world_state.our_robots[i] - world_state.ball.pos)
            if dist_to_ball < closest_dist:
                closest_dist = dist_to_ball
                closest_robot = i

        return self.tactics[closest_robot].is_done()
