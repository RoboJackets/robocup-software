import robocup
import play
import behavior
import skills.move
import skills.capture
import tactics.coordinated_pass
import constants
import main
import enum
import copy


## A testing play to demonstrate our ability to pass and receive balls
# One robot will pursue the ball while three other robots will pass the ball amongst themselves
# they can only pass the ball at the corners and receive at the corners and they cannot move
# through the center of the square
class FourCornerPass(play.Play):
    class State(enum.Enum):
        ## One robot goes captures the ball while the other two gets on corners
        # pursuing robot does not move on the first instance
        setup = 1

        ## The robots pass to each other and begin setting up the next pass
        # while the chasing robot goes for the ball.
        passing = 2

    def __init__(self):
        super().__init__(continuous=True)

        # register states - they're both substates of "running"
        self.add_state(FourCornerPass.State.setup,
                       behavior.Behavior.State.running)
        self.add_state(FourCornerPass.State.passing,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            FourCornerPass.State.setup, lambda: True,
                            'immediately')
        #transition when ball is captured
        self.add_transition(
            FourCornerPass.State.setup,
            FourCornerPass.State.passing, lambda: self.subbehavior_with_name(
                'capture').state == behavior.Behavior.State.completed,
            'captured ball')

        #transition when ball is passed or failed to pass
        self.add_transition(
            FourCornerPass.State.passing, FourCornerPass.State.setup, lambda:
            self.subbehavior_with_name('passer').state == behavior.Behavior.
            State.completed or self.subbehavior_with_name('passer').state ==
            behavior.Behavior.State.failed, 'passed ball')

        #change this to adjust the square size
        self.variable_square = 5.5

        #NEVER CHANGE THIS
        self.length = constants.Field.Length
        self.width = constants.Field.Width

        #if the square size is smaller than the width shrink it down otherwise
        #max value is the width of the field.
        self.variable_square = min(self.variable_square, self.width,
                                   self.length)

        #radius of robot
        self.bot_buffer = constants.Robot.Radius * 3
        # the largest and smallest x and y position possible for the square
        self.max_x = (self.variable_square / 2 - self.bot_buffer)
        self.min_x = -self.max_x
        self.max_y = self.length / 2 + self.max_x
        self.min_y = self.length / 2 - self.max_x

        # the four courners
        self.square_points = [
            robocup.Point(self.max_x, self.max_y),
            robocup.Point(self.min_x, self.max_y),
            robocup.Point(self.min_x, self.min_y),
            robocup.Point(self.max_x, self.min_y)
        ]

        # speed of the hunting robots
        #TODO Create python pull from Config values. Currently this breaks the world.
        #tmp = robocup.Configuration.FromRegisteredConfigurables().nameLookup("MotionConstraints/Max Velocity").value
        self.normal_speed = 1.0  #tmp 
        # speed of the defending robots can decrease value to make it easier for offense
        self.defense_speed = self.normal_speed * 2 / 3  #self.normal_speed/2.0

        # picks the direction to pass to. TODO make actual smart pass selection
        self.direction = 1

    # constantly running changes and updates
    def execute_running(self):
        # checks if there is an offensive behavior for the various plays and set the normal speed
        for sub in self.subbehaviors_by_name():
            if (sub != 'passer'):
                robo = self.subbehavior_with_name(sub).robot
                # checks if there is a chasing robot and set their speed to defense speed
                if (robo != None):
                    if (sub == 'chasing'):
                        robo.set_max_speed(self.defense_speed)
                    # otherwise sets to the offensive normal speed rate
                    else:
                        robo.set_max_speed(self.normal_speed)

        # chasing robot position should always follow the ball within a smaller inside box of the
        # four corners.
        if (self.has_subbehavior_with_name('chasing')):
            self.chaser.pos = self.cut_off_pos(main.ball().pos)

        #draw the four courner field
        # takes in the square points and form lines and create a square on the field
        for i in range(len(self.square_points)):
            main.debug_drawer().draw_line(
                robocup.Line(self.square_points[i],
                             self.square_points[(i + 1) % 4]), (135, 0, 255),
                "Square")

        # speed of the hunting robots
        #TODO Create python pull from Config values. Currently this breaks the world.
        #tmp = copy.deepcopy(robocup.Configuration.FromRegisteredConfigurables().nameLookup("MotionConstraints/Max Velocity").value)
        self.normal_speed = 1.0  # tmp
        #speed of the defending robots can decrease value to make it easier for offense
        self.defense_speed = 2 * self.normal_speed / 3.0  #self.normal_speed/2.0#self.variable_square/(min(self.width,self.length) * 2) * self.normal_speed

    def on_enter_start(self):
        # if we have too many robots isolate one of the robots so they don't help in the play
        goalie = main.root_play(
        ).goalie_id  #.system_state().game_state.get_goalie_id()
        print(goalie)
        numRobots = len(main.our_robots()) - 4
        if (goalie != None):
            numRobots = numRobots - 1

        for i in range(numRobots):
            iso = skills.move.Move(
                robocup.Point(-constants.Field.Width / 2 + self.bot_buffer * i,
                              0))
            self.add_subbehavior(
                iso, 'iso' + str(i), required=True, priority=1)

    def on_enter_setup(self):
        # find where we will consider the ball is closest too and passing from
        closestPt = min(self.square_points,
                        key=lambda pt: pt.dist_to(main.ball().pos))

        closestPtIdx = self.square_points.index(closestPt)

        #points that the other robots can move to
        self.otherPts = list(self.square_points)
        self.otherPts.remove(closestPt)

        # remove the point that's diagonal of the closest point
        farthestPt = max(self.otherPts,
                         key=lambda pt: pt.dist_to(main.ball().pos))

        self.otherPts.remove(farthestPt)

        # decide which direction you're passing to
        self.direction = self.safer_pass()
        # move the other two robots to the other point locations
        self.add_subbehavior(
            skills.move.Move(
                self.square_points[(closestPtIdx + self.direction) % 4]),
            'move',
            required=False)
        # send the closest robot to capture the ball
        self.add_subbehavior(skills.capture.Capture(), 'capture')

    def on_exit_setup(self):
        self.remove_subbehavior('move')
        self.remove_subbehavior('capture')

    def on_enter_passing(self):
        # get the point passing from
        kickFrom = min(self.square_points,
                       key=lambda pt: pt.dist_to(main.ball().pos))
        # get the index of where you're passing from
        kickFromIdx = self.square_points.index(kickFrom)
        # based on which direction you're passing to pick that index
        kickToIdx = (kickFromIdx + self.direction) % len(self.square_points)
        # get that point
        kickToPt = self.square_points[kickToIdx]

        # get ready to pass the ball
        passing_action = tactics.coordinated_pass.CoordinatedPass(kickToPt)
        # while pass is preparing get ready for the next pass
        premove = skills.move.Move(self.square_points[(
            kickToIdx + self.direction) % len(self.square_points)])

        self.add_subbehavior(
            passing_action, 'passer', required=True, priority=10)
        self.add_subbehavior(premove, 'premove', required=True, priority=2)

        # add a robot to chase after the ball.
        if (not self.has_subbehavior_with_name('chasing')):
            self.chaser = skills.move.Move(self.cut_off_pos(main.ball().pos))
            self.add_subbehavior(
                self.chaser, 'chasing', required=True, priority=2)

    # remove subbehaviors 
    def on_exit_passing(self):
        self.remove_subbehavior('passer')
        self.remove_subbehavior('chasing')
        self.remove_subbehavior('premove')

    #Decide which the better direction to pass is
    def safer_pass(self):
        if (True):
            return 1
        else:
            return -1

    # if a point is outside of a smaller box that is the square points - 3 robot radius'
    # return the closest point inside that box.
    def cut_off_pos(self, point):
        # Projects the point given onto the edge of the smaller rectangle
        # Each coordinate is found independently so we don't have to project to each
        # segment of the rectangle and choose the closest
        segment_x = robocup.Segment(
            robocup.Point(self.min_x + self.bot_buffer, 0),
            robocup.Point(self.max_x - self.bot_buffer, 0))
        segment_y = robocup.Segment(
            robocup.Point(0, self.min_y + self.bot_buffer),
            robocup.Point(0, self.max_y - self.bot_buffer))

        x_pos = segment_x.nearest_point(point)
        y_pos = segment_y.nearest_point(point)

        return robocup.Point(x_pos.x, y_pos.y)
