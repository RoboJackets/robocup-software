import play
import behavior
import skills.bump
import standard_play
import robocup
import main
import constants
import evaluation.space
import math


class TestSpaceEvals(standard_play.StandardPlay):
    def __init__(self):
        super().__init__(continuous=True)
        self.frame = 0
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def execute_running(self):
        if self.frame % 500 == 0:
            print(math.pi)
            print("Overall:" ,evaluation.space.get_closest_opponent_distance_to_point(main.ball().pos))
            print("Downfield:" ,evaluation.space.get_closest_downfield_opponent_distance_to_point(main.ball().pos))
            print("Upfield:" ,evaluation.space.get_closest_upfield_opponent_distance_to_point(main.ball().pos))
            rp = evaluation.space.get_radius_points(main.ball().pos, r=1, n=5)
            #print("Radius Points from Ball", rp)
            best_downfield_point = evaluation.space.get_best_downfield_space_point()
            print("Best Downfield Point: ",best_downfield_point)
            if best_downfield_point is None:
                print('WELL FUCK')
            else:
                main.system_state().draw_circle(best_downfield_point, 0.1, constants.Colors.Green,"Best")
            #for i, pt in enumerate(rp):
            #    print("Plotting pt")
            #    main.system_state().draw_circle(pt, 0.1, (255,255,22),"Point"+str(i))
        self.frame += 1

