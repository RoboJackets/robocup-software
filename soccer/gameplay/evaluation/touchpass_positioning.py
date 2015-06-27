import robocup
import constants
import main
import evaluation.passing

# The Touchpass Positioner finds the best location within a rectangle

# Example usage:
# tpos = TouchpassPositioner()
# best_pt = tpos.evaluate(ball.pos, robot2.pos)
#
class TouchpassPositioner:

    def __init__(self):
        self.debug = False

    # Defaults to False
    # if True, uses the system state drawing methods to draw Windows
    @property
    def debug(self):
        return self._debug
    @debug.setter
    def debug(self, value):
        self._debug = value

    # Returns a robocup.Rect object that is the default location to be evaluated
    # Takes a current ball position/initial kick position (robocup.Point)
    #
    # Mainly for internal use
    def generate_default_rectangle(self, kick_point):
        offset_from_edge = 0.3

        if kick_point.x > 0:
            # Ball is on right side of field
            toReturn = robocup.Rect(robocup.Point(0, min(constants.Field.Length - offset_from_edge, main.ball().pos.y - 0.5)),
                    robocup.Point(-constants.Field.Width / 2 - offset_from_edge, min(constants.Field.Length * 3 / 4, main.ball().pos.y - 2)))
        else:
            # Ball is on left side of field
            toReturn = robocup.Rect(robocup.Point(0, min(constants.Field.Length - offset_from_edge, main.ball().pos.y - 0.5)),
                    robocup.Point(constants.Field.Width / 2 - offset_from_edge, min(constants.Field.Length * 3 / 4, main.ball().pos.y - 2)))
        return toReturn

    def get_points_from_rect(self, rect, threshold=0.50):
        outlist = []
        currentx = rect.min_x()
        currenty = rect.max_y()

        # Loop through from top left to bottom right

        while currenty > rect.min_y():
            while currentx < rect.max_x():
                candiate = robocup.Point(currentx, currenty)
                if not constants.Field.TheirGoalShape.contains_point(candiate):
                    outlist.extend([candiate])
                currentx = currentx + threshold
            currenty = currenty - threshold
            currentx = rect.min_x()
        return outlist

    def eval_best_receive_point(self, kick_point, evaluation_zone=None):
        # Autogenerate kick point
        if evaluation_zone == None:
            evaluation_zone = self.generate_default_rectangle(kick_point)

        # print(evaluation_zone.min_x())
        # print(evaluation_zone.min_y())
        # print(evaluation_zone.max_x())
        # print(evaluation_zone.max_y())
        points = self.get_points_from_rect(evaluation_zone)

        # TODO Check for empty list
        best = points[0]
        bestChance = 0


        for point in points:
            currentChance = evaluation.passing.eval_pass(kick_point, point)
            # TODO dont only aim for center of goal
            targetPoint = constants.Field.TheirGoalCenter
            currentChance = currentChance * evaluation.passing.eval_pass(point, targetPoint)
            if currentChance > bestChance:
                bestChance = currentChance
                best = point

        print("BEST VALUE: " + str(best))
        return best, targetPoint

