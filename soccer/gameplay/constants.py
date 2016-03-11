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
    MaxKickSpeed = 8  # m/s

    class Dribbler:
        MaxPower = 127


class Ball:
    Radius = 0.0215
    Mass = 0.04593  # mass of golf ball (kg)


Field = robocup.Field_Dimensions.SingleFieldDimensions
