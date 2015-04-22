import robocup
import math



DegreesToRadians = math.pi / 180.0
RadiansToDegrees = 180.0 / math.pi




class Colors:
    White = (255, 255, 255)
    Black = (0, 0, 0)
    Green = (0, 255, 0)
    Red = (255, 0, 0)
    Blue = (0, 0, 255)



class Robot:
    Radius = 0.09

    class Dribbler:
        MaxPower = 127


class Ball:
    Radius = 0.0215
    Mass = 0.04593 # mass of golf ball (kg)
    

class Field:
    Length = 6.05
    Width = 4.05
    Border = 0.25

    LineWidth = 0.01

    GoalWidth = 0.700
    GoalDepth = 0.180
    GoalHeight = 0.160

    # Distance of the penalty marker from the goal line
    PenaltyDist = 0.750
    PenaltyDiam = 0.010

    # Radius of the goal arcs
    ArcRadius = 0.8

    # diameter of the center circle 
    CenterRadius = 0.5
    CenterDiameter = 1.0

    # flat area for defense markings 
    GoalFlat = 0.35

    FloorLength = Length + 2.0 * Border;
    FloorWidth = Width + 2.0 * Border;

    CenterPoint = robocup.Point(0.0, Length / 2.0)

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

    TheirHalf = robocup.Rect(robocup.Point(-Width/2, Length), robocup.Point(Width/2, Length/2))
    OurHalf = robocup.Rect(robocup.Point(-Width/2, 0), robocup.Point(Width/2, Length/2))


def setFieldConstantsFromField_Dimensions(value):
    Field.Length = value.Length()
    Field.Width = value.Width()
    Field.Border = value.Border()
    Field.LineWidth = value.LineWidth()
    Field.GoalWidth = value.GoalWidth()
    Field.GoalDepth = value.GoalDepth()
    Field.GoalHeight = value.GoalHeight()
    Field.PenaltyDist = value.PenaltyDist()
    Field.PenaltyDiam = value.PenaltyDiam()
    Field.ArcRadius = value.ArcRadius()
    Field.CenterRadius = value.CenterRadius()
    Field.CenterDiameter = value.CenterDiameter()
    Field.GoalFlat = value.GoalFlat()
    Field.FloorLength = value.FloorLength()
    Field.FloorWidth = value.FloorWidth()

    Field.CenterPoint = robocup.Point(0.0, Field.Length / 2.0)

    Field.OurGoalZoneShape = robocup.CompositeShape()
    Field.OurGoalZoneShape.add_shape(robocup.Circle(robocup.Point(-Field.GoalFlat / 2.0, 0), Field.ArcRadius))
    Field.OurGoalZoneShape.add_shape(robocup.Circle(robocup.Point(Field.GoalFlat / 2.0, 0), Field.ArcRadius))
    Field.OurGoalZoneShape.add_shape(robocup.Rect(robocup.Point(-Field.GoalFlat / 2.0, Field.ArcRadius), robocup.Point(Field.GoalFlat / 2.0, 0)))

    Field.TheirGoalShape = robocup.CompositeShape()
    Field.TheirGoalShape.add_shape(robocup.Circle(robocup.Point(-Field.GoalFlat / 2.0, Field.Length), Field.ArcRadius))
    Field.TheirGoalShape.add_shape(robocup.Circle(robocup.Point(Field.GoalFlat / 2.0, Field.Length), Field.ArcRadius))
    Field.TheirGoalShape.add_shape(robocup.Rect(robocup.Point(-Field.GoalFlat / 2.0, Field.Length), robocup.Point(Field.GoalFlat / 2.0, Field.Length - Field.ArcRadius)))


    Field.TheirGoalSegment = robocup.Segment(robocup.Point(Field.GoalWidth / 2.0, Field.Length),
                                        robocup.Point(-Field.GoalWidth / 2.0, Field.Length))
    Field.OurGoalSegment = robocup.Segment(robocup.Point(Field.GoalWidth / 2.0, 0),
                                        robocup.Point(-Field.GoalWidth / 2.0, 0))

    Field.TheirHalf = robocup.Rect(robocup.Point(-Field.Width/2, Field.Length), robocup.Point(Field.Width/2, Field.Length/2))
    Field.OurHalf = robocup.Rect(robocup.Point(-Field.Width/2, 0), robocup.Point(Field.Width/2, Field.Length/2))

