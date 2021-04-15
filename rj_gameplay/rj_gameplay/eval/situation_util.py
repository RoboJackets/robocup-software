

class FieldLoc(Enum):
    """Enum for representing where the ball is on the field."""
    DEFEND_SIDE = 1
    MIDFIELD = 2
    ATTACK_SIDE = 3

def situationFieldLoc(stp.rc.WorldState) -> 


def ballToRobotDist(stp.rc.Robot, stp.rc.Ball) -> float:
    pass


def closestRobotToBall(stp.rc.WorldState) -> (stp.rc.Robot, float):
    pass


def ourRobotClosestToBall(stp.rc.WorldState) -> bool:
    pass

def bothTeamsClosest(stp.rc.WorldState) -> (stp.rc.Robot, stp.rc.Robot):
    pass


def ballClosenessRatio(stp.rc.WorldState) -> float:
    pass



