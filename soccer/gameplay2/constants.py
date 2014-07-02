import robocup
import math



DegreesToRadians = math.pi / 180.0
RadiansToDegrees = 180.0 / math.pi




class Colors:
    White = (255, 255, 255)
    Black = (0, 0, 0)



class Robot:
    Radius = 0.09

    class Dribbler:
        MaxPower = 127

    class Chipper:
        MaxPower = 255

    class Kicker:
        MaxPower = 255


class Ball:
    Radius = 0.0215
    Mass = 0.04593 # mass of golf ball (kg)
    

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

    OurGoalZoneShape = robocup.CompositeShape()
    OurGoalZoneShape.add_shape(robocup.Circle(robocup.Point(-GoalFlat / 2.0, 0), ArcRadius))
    OurGoalZoneShape.add_shape(robocup.Circle(robocup.Point(GoalFlat / 2.0, 0), ArcRadius))
    OurGoalZoneShape.add_shape(robocup.Rect(robocup.Point(-GoalFlat / 2.0, ArcRadius), robocup.Point(GoalFlat / 2.0, 0)))

    TheirGoalShape = robocup.CompositeShape()
    TheirGoalShape.add_shape(robocup.Circle(robocup.Point(-GoalFlat / 2.0, Length), ArcRadius))
    TheirGoalShape.add_shape(robocup.Circle(robocup.Point(GoalFlat / 2.0, Length), ArcRadius))
    TheirGoalShape.add_shape(robocup.Rect(robocup.Point(-GoalFlat / 2.0, Length), robocup.Point(GoalFlat / 2.0, Length - ArcRadius)))


    TheirGoalSegment = robocup.Segment(robocup.Point(GoalWidth / 2.0, Length),
                                        robocup.Point(-GoalWidth / 2.0, Length))
    OurGoalSegment = robocup.Segment(robocup.Point(GoalWidth / 2.0, 0),
                                        robocup.Point(-GoalWidth / 2.0, 0))
