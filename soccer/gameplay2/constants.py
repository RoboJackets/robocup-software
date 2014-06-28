import robocup
import math



DegreesToRadians = math.pi / 180.0
RadiansToDegrees = 180.0 / math.pi



class Robot:
    Radius = 0.09


class Ball:
    Radius = 0.0215
    

class Field:
    Length = 6.05
    Width = 4.05

    # diameter of the center circle 
    CenterRadius = 0.5

    LineWidth = 0.01

    GoalWidth = 0.700
    GoalDepth = 0.180
    GoalHeight = 0.160

    # Distance of the penalty marker from the goal line 
    PenaltyDist = 0.750
    PenaltyDiam = 0.010

    # flat area for defence markings 
    GoalFlat = 0.35

    # Radius of the goal arcs 
    ArcRadius = 0.8

    Border = 0.25
    FloorLength = Length + 2.0 * Border;
    FloorWidth = Width + 2.0 * Border;

    # TODO: make these out of rectangles and circles, then we can check if the ball is in there
    OurGoalShape = None
    TheirGoalShape = None


    TheirGoalSegment = robocup.Segment(robocup.Point(GoalWidth / 2.0, Length),
                                        robocup.Point(-GoalWidth / 2.0, Length))
    OurGoalSegment = robocup.Segment(robocup.Point(GoalWidth / 2.0, 0),
                                        robocup.Point(-GoalWidth / 2.0, 0))
