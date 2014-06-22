import play
import behavior
import skills.move
import robocup


class LineUp(play.Play):

    def __init__(self):
        super().__init__(continuous=False)

        self._subbehaviors = None

        self.add_transition(behavior.Behavior.State.start, behavior.Behavior.State.running, lambda: True, 'immediately')
        self.add_transition(behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.all_subbehaviors_completed(),
            'all robots reach target positions'
            )
        self.add_transition(Behavior.State.completed,
            Behavior.State.running,
            lambda: not self.all_subbehaviors_completed(),
            lambda: 'robots arent lined up')


    def all_subbehaviors_completed(self):
        if self.subbehaviors != None:
            return all([b.behavior_state == behavior.Behavior.State.completed for b in self.subbehaviors])
        else:
            return True


    @play.Play.robots.setter
    def robots(self, robots):
        # super()._set_robots(robots)
        self._robots = robots

        if robots != None:
            # build a list of Points for where the robots should go
            start_x = -1.0
            start_y = 0.5
            spacing_y = 0.25
            points = [robocup.Point(start_x, start_y + i * spacing_y) for i in range(0, len(robots))]

            # print("LineUp set destinations: " + str(points))

            self.subbehaviors = [Move(pt) for pt in points]

            # FIXME: assign behaviors more smartly
            for i in range(0, len(robots)):
                self.subbehaviors[i].robot = self.robots[i]


    def execute_running(self):
        # print("LineUp running...")
        if self.subbehaviors != None:
            for subbehavior in self.subbehaviors:
                print("\trunning subbehavior")
                subbehavior.run()


    # a list of Move behaviors
    @property
    def subbehaviors(self):
        return self._subbehaviors
    @subbehaviors.setter
    def subbehaviors(self, value):
        self._subbehaviors = value
        self.transition(behavior.Behavior.State.running)
