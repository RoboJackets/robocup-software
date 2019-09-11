import main
import time
import robocup
import math
from enum import Enum
import constants
from typing import List, Dict


## Class for breaking gameplay down into discrete states to aid in play selection
# 
# An instance of this class exists in main.py where its updateAnalysis function 
# every frame.
#
# The purpose of this class is to inform plays of the current gameplay situation
# so that they can change their score function based on their applicability to
# the current situation
#
# Organization of tunable parameters is something that is still under consideration
#
#
class SituationalPlaySelector:

    ## Enum for representing the current game situation, each of which acts as a catagory of play to be run
    #  
    # The none situation should never be encountered during gameplay
    # 
    class Situation(Enum):
        NONE = 0 #This situation should never be encountered during gameplay
        KICKOFF = 1  #Plays that can perform our kickoff
        DEFEND_RESTART_OFFENSIVE = 2  #Plays for defending our opponents restart on their side of the field
        DEFEND_RESTART_MIDFIELD = 3  #Plays for defending our opponents restart in the midfield
        DEFEND_RESTART_DEFENSIVE = 4  #Plays for defending our opponents restart on our side of the field
        CLEAR = 5  #play for clearing the ball from our side of the field (should include defensive caution)
        DEFEND_CLEAR = 6  #Plays for defending the opponents clear, when the ball is on their side.
        DEFEND_GOAL = 7  #Plays for defending our goal from opponents near it with the ball
        MIDFIELD_CLEAR = 8  #Plays for when we possess the ball in the midfield
        ATTACK_GOAL = 9  #Plays for attacking the opponents goal, when we have the ball near it
        OFFENSIVE_SCRAMBLE = 10  #Plays for getting a loose ball when the ball is on the opponents half
        MIDFIELD_SCRAMBLE = 11  #Plays for getting a loose ball when the ball is in the midfield
        DEFENSIVE_SCRAMBLE = 12  #Plays for getting a loose ball when the ball is on our half
        SAVE_BALL = 13  #Plays that will trigger when the ball is headed out of the field with no obstuctions
        SAVE_SHOT = 14  #Plays that will trigger when the ball is headed directly at our goal
        OFFENSIVE_PILEUP = 15  #Plays to handle a pile up on their side of the field
        MIDFIELD_PILEUP = 16  #Plays to handle a pile up in the midfield
        DEFENSIVE_PILEUP = 17  #Plays to handle a pile up on our side of the field
        MIDFIELD_DEFEND_CLEAR = 18  #Plays to defend a clear when the ball is in the midfield
        SHOOTOUT = 19  #Plays for making shootout shots
        DEFEND_SHOOTOUT = 20  #Plays for defending shootout shots
        PENALTY = 21  #Plays for making penalty shots
        DEFEND_PENALTY = 22  #Plays for defending penalty shots
        OFFENSIVE_KICK = 23  #Plays for direct and indirect kicks on their side
        DEFENSIVE_KICK = 24  #Plays for direct and indirect kicks on our side
        MIDFIELD_KICK = 25  #Plays for direct and indirect kicks in the midfield
        GOALIE_CLEAR = 26  #Plays for clearing the ball when our goalie possesses the ball

    ##Enum for representing where the ball is on the field
    #
    # The regions are defined in the update function
    #
    class FieldLoc(Enum):
        DEFENDSIDE = 1
        MIDFIELD = 2
        ATTACKSIDE = 3

    ##Enum for representing the possession of the ball
    class BallPos(Enum):
        OURBALL = 1
        FREEBALL = 2
        THEIRBALL = 3

    ##An empty constructor
    def __init__(self):
        pass

    ## Holds the current situation enum
    currentSituation = Situation.NONE

    ##Holds the current possession enum
    currentPossession = BallPos.FREEBALL

    ##Holds the ball location enum
    ballLocation = FieldLoc.MIDFIELD

    ##Represents if there is currently a pileup for the purposes of situation selection
    currentPileup = False

    ##Holds the last time that currentPileup was changed by the pileup update function 
    pileupTime = time.time()

    ##Variable used to ensure setup is done 
    isSetup = False

    ##Holds the game state object
    gameState = None

    ##Holds the system state object
    systemState = None

    ##Holds a list of all the robots
    robotList: List[robocup.Robot] = list()

    ##Holds a list of the currently visible robots
    activeRobots: List[robocup.Robot] = list()

    ##Function is run a single time to setup the class
    #
    # Gets the game state, system state, and context objects
    #
    # Builds the robotList for later use
    #
    def setupStates(self):
        self.gameState = main.game_state()
        self.systemState = main.system_state()
        self.context = main.context()
        for g in self.systemState.our_robots:
            self.robotList.append(g)
        for g in self.systemState.their_robots:
            self.robotList.append(g)

        self.updateRobotList()

    ## Function that updates the activeRobots list
    # 
    # Updates the activerobots list every frame to include only the currently visible robots
    #
    def updateRobotList(self):
        self.activeRobots.clear()
        for g in self.robotList:
            if (g.visible):
                self.activeRobots.append(g)


    ## Main update function
    #   Is called every frame from main.py to update the situation information 
    #   mostly calls all the other update functions
    #   
    #   Will also run the setup if isSetup is false and has functionality to get
    #   the running time of the module
    def updateAnalysis(self):
        #startTime = time.time()
        if (not self.isSetup):
            self.setupStates()
            self.isSetup = True
        else:
            self.updateRobotList()

        self.updatePileup()
        self.locationUpdate()
        self.ballPossessionUpdate()
        self.situationUpdate()

        #print(abs(startTime - time.time()))

        #Print the current situation in the corner of the soccer gui
        self.context.debug_drawer.draw_text(self.currentSituation.name,
                                            robocup.Point(-3, -0.3), (0, 0, 0),
                                            "hat")

    ## Returns a list of the robots in the path of the ball
    # 
    # builds a dict of ingress info, which is expensive
    #
    def in_ball_path(self):
        ingress_info = dict()
        robotsInBallPath = list()
        for g in self.activeRobots:
            ingress_info[g] = self.ball_ingress(self.systemState.ball.pos,
                                                self.systemState.ball.vel, g)
            if (ingress_info[g] != None and ingress_info[g][0] != None and
                    abs(ingress_info.get(g)[0]) < 0.1):
                robotsInBallPath.append(g)

        return robotsInBallPath

    ##Returns a tuple containing the robots distance from the balls path, 
    #the distance the ball has to travel to get to that intercept point, 
    #the balls speed, and the angle between the balls velocity and the robot's position
    def ball_ingress(self, ballPos, ballVel, robot, slowThresh = 0.15):

        robotx = robot.pos.x
        roboty = robot.pos.y

        ballSpeed = ballVel.mag()
        if (ballSpeed < slowThresh):
            return (None, None, None, None)

        robotToBall = robot.pos - ballPos
        ballDist = robotToBall.mag()
        angle = ballVel.angle_between(robotToBall)

        if (abs(angle) > 100):
            return (None, None, None, None)

        robotToBallDOTBallVel = robotToBall.x * ballVel.x + robotToBall.y * ballVel.y
        scalar = robotToBallDOTBallVel / (ballSpeed**2)

        ballDirection = ballVel.normalized()
        robotOntoVelocity = ballDirection * ballDirection.dot(
            robotToBall
        )  #The robots position relative to the ball projected onto the balls velocity

        projectedToRobot = robocup.Point(
            robotOntoVelocity.x - robotToBall.x,
            robotOntoVelocity.y - robotToBall.
            y)  #The vector from the projected vector to the robots position

        distanceFromPath = projectedToRobot.mag()
        interceptDistance = robotOntoVelocity.mag()

        return (distanceFromPath, interceptDistance, ballSpeed, angle)

    ##Returns the closest robot in the balls path as a tuple of the robot and its distance 
    def closestReciever(self):
        botsInPath = self.in_ball_path()
        if (len(botsInPath) == 0):
            return (None, None)
        closestRobot = None
        closestRobotDistance = 0.0
        ballLocation = self.systemState.ball.pos
        for g in botsInPath:
            roboDist = self.ballToRobotDist(g)
            if (closestRobot == None or roboDist < closestRobotDistance):
                closestRobot = g
                closestRobotDistance = roboDist

        return (closestRobot, closestRobotDistance)


    ##A function that determines if the ball is in the mouth of a given robot
    #
    # Is purly geometric, looks at angle and distance based on the thresholds
    # given as parameters
    #
    def possesses_the_ball(self,
                           ballPos,
                           robot,
                           distThresh=0.14,
                           angleThresh=35):

        distance = (ballPos - robot.pos).mag()
        #math.sqrt((ballPos.x - robot.pos.x)**2 + (ballPos.y - robot.pos.y)**2)
        angle = math.degrees(
            math.atan2(ballPos.y - robot.pos.y, ballPos.x - robot.pos.x) -
            robot.angle)
        if (distance < distThresh and abs(angle) < angleThresh):
            return True
        else:
            return False

    ##Keeps track of if each robot possesses the ball
    hasBall: Dict[robocup.Robot, bool] = dict()

    ##Keeps track of when possession last changed for each robot
    posChangeTime: Dict[robocup.Robot, float] = dict()

    ##Keeps track of how long the last possession or lack of possession was for each robot
    posDuration: Dict[robocup.Robot, float] = dict()

    #recvProb: Dict[robocup.Robot, float] = dict()
    #ballDist: Dict[robocup.Robot, float] = dict()

    ##Keeps track of what the situaion was for the last frame for the purpose of preemption
    lastSituation = None

    ##Keeps track of the time at which the situation last changed
    situationChangeTime = None

    ##The time after a situation changes before preempting the current play
    playPreemptTime = 0.20
    
    ##Keeps track of if the current play should be preempted 
    currentPreempt = False

    ## Returns true if no one posseses the ball
    def isFreeBall(self):
        return self.currentPossession == self.BallPos.FREEBALL

    ## Returns true if we possess the ball
    def isOurBall(self):
        return self.currentPossession == self.BallPos.OURBALL

    ## Returns true if our opponents possess the ball
    def isTheirBall(self):
        return self.currentPossession == self.BallPos.THEIRBALL

    ## Returns true if the ball is on our opponents side of the field, but not in the midfield
    def isAttackSide(self):
        return self.ballLocation == self.FieldLoc.ATTACKSIDE

    ## Returns true if the ball is on our side of the field, but not in the midfield
    def isDefendSide(self):
        return self.ballLocation == self.FieldLoc.DEFENDSIDE

    ## Returns true if the ball is in the midfield
    def isMidfield(self):
        return self.ballLocation == self.FieldLoc.MIDFIELD

    ##Returns true if there is currently determined to be a pileup
    def isPileup(self):
        return self.currentPileup

    ##Returns if we are in the specified situation without regard to capitalization
    #If the check variable is true, it will check if the situation exists and throw an 
    #exception is it does now.
    def isSituation(self, situation, check=False):
        up = situation.upper()

        if (check):
            found = False
            for g in self.Situation:
                if (up == g):
                    found = True
                    break
            if not found:
                raise Exception("Passed situation " + situation + " / " + up +
                                " is not an existing situation")

        if (self.currentSituaion.name == up):
            return True
        else:
            return False

    ##Update determining if we want to preempt the current play or not 
    #
    # Preemption is still an open question but this is a prototype of 
    # of how a non-invasive preemption system might work
    #
    def updatePreempt(self):
        if (self.lastSituation != self.currentSituaion and
                self.situationChangeTime == None):
            self.situationChangeTime = time.time()

        if (self.currentPreempt):
            self.currentPreempt = False
        elif (self.situationChangeTime != None):
            if (abs(time.time() - self.situationChangeTime) >
                    self.playPreemptTime):
                self.situationChangeTime = None
                self.currentPreempt = True
                self.LastSituation = self.currentSituaion

    '''def addPreempt(play) possibly a function to add transition out of every state to the completed state with preemptPlay as the lambda
        for g in states:
            play.add_transition(g -> completed , preemptPlay)'''


    ##A function to determine if the currently running play should be preempted
    def preemptPlay(self):
        return self.currentPreempt

    ##Returns the distance from a given robot to the ball
    def ballToRobotDist(self, robot):
        return (robot.pos - self.systemState.ball.pos).mag()

    ##Returns a tuple of the closest robot to the ball and the distance that robot is away from the ball
    #
    #
    def closestRobot(self):
        closestRobot = None
        closestRobotDistance = 0.0
        ballLocation = self.systemState.ball.pos
        for g in self.activeRobots:
            roboDist = self.ballToRobotDist(g)
            if (closestRobot == None or roboDist < closestRobotDistance):
                closestRobot = g
                closestRobotDistance = roboDist

        return (closestRobot, closestRobotDistance)

    ##Returns true if our robot is closest to the ball
    #
    #
    def ourRobotClosest(self):
        return self.closestRobot()[0].is_ours()

    ##Returns a tuple of the distance from the ball of our closest robot and our opponents closest robot
    #(our distance, theirDistance)
    #
    def bothTeamsClosest(self):
        ourClosest = None
        ourClosestDist = 0.0
        theirClosest = None
        theirClosestDist = 0.0

        ballLocation = self.systemState.ball.pos

        for g in self.activeRobots:
            roboDist = self.ballToRobotDist(g)
            if (g.is_ours()):
                if (ourClosest == None or roboDist < ourClosestDist):
                    ourClosest = g
                    ourClosestDist = roboDist
            else:
                if (theirClosest == None or roboDist < theirClosestDist):
                    theirClosest = g
                    theirClosestDist = roboDist

        return (ourClosestDist, theirClosestDist)

    ##Returns the ratio of our closest distance to the ball and the opponents closest distance to the ball
    def ballClosenessRatio(self):
        distances = self.bothTeamsClosest()
        try:
            return distances[0] / distances[1]
        except:
            return 1.0

    ##Returns the robot that last had the ball, when they last had the ball, and how long they had the ball for
    #
    def hadBallLast(self):
        lastRobot = None
        lastRobotTime = 0.0

        for g in self.activeRobots:
            if (self.hasBall[g]):
                return (g, 0.0, abs(time.time() - self.posChangeTime[g]))
            timeSincePoss = abs(time.time() - self.posChangeTime.get(
                g, float("inf")))
            if (lastRobot == None or timeSincePoss < lastRobotTime):
                lastRobot = g
                lastRobotTime = timeSincePoss

        if (self.posChangeTime.get(lastRobot, float("inf")) == float("inf")):
            return (None, None, None)

        return (lastRobot, lastRobotTime, self.posDuration.get(lastRobot, 0.0))

    ##Returns true if we had the ball last
    def weHadBallLast(self):
        if (self.hadBallLast()[0].is_ours()):
            return True
        else:
            return False

    ##Returns a list of robots that currently possess the ball
    def robotsWithTheBall(self):
        robotsWithTheBall = list()

        for g in self.activeRobots:
            if (self.hasBall.get(g, False)):
                robotsWithTheBall.append(g)

        return robotsWithTheBall

    ##
    #Returns the number of robots that currenly possess the ball
    #
    def withBallCount(self):
        ourBots = 0
        theirBots = 0
        for g in self.robotsWithTheBall():
            if (g.is_ours()):
                ourBots += 1
            else:
                theirBots += 1

        return (ourBots, theirBots)

    ##
    # Returns a list of the robots that are currenly near the ball based on the passed distance parameter
    # 
    def robotsNearTheBall(self, distance=0.5):
        robotsNearTheBall = list()

        for g in self.activeRobots:
            if (self.ballToRobotDist(g) < distance):
                robotsNearTheBall.append(g)

        return robotsNearTheBall

    ##
    # Returns the number of robots on each team that are near the ball based on the passed distance parameter
    #
    def nearBallCount(self, distance=0.35):
        ourBots = 0
        theirBots = 0
        for g in self.robotsNearTheBall(distance):
            if (g.is_ours()):
                ourBots += 1
            else:
                theirBots += 1

        return (ourBots, theirBots)

    ##
    # Updates the self.currentPileup bool with its determination of if we are in a pileup state
    # called every frame
    #
    def updatePileup(self):

        pileupBufferTimeIn = 0.3  #the number of seconds of pile up conditions before a pile up is declared
        pileupBufferTimeOut = 0.3  #the number of seconds of not pile up conditions before a pile up is un declared

        botsNearBall = self.nearBallCount(distance=0.35)
        botsWithBall = self.withBallCount()
        totalNearBall = sum(botsNearBall)
        totalWithBall = sum(botsWithBall)
        #print("Pileup info------------------") 
        #print(botsNearBall)
        #print(botsWithBall)
        pileUpCalc = False

        if (totalNearBall >= 3 and botsNearBall[0] > 0 and
                botsNearBall[1] > 0):
            pileUpCalc = True

        if (totalWithBall >= 2 and botsWithBall[0] > 0 and
                botsWithBall[1] > 0):
            pileUpCalc = True

        pileUpDecision = bool(self.currentPileup)

        if (not self.currentPileup and pileUpCalc and
                abs(time.time() - self.pileupTime) > pileupBufferTimeIn):
            pileUpDecision = True
        if (self.currentPileup and not pileUpCalc and
                abs(time.time() - self.pileupTime) > pileupBufferTimeOut):
            pileUpDecision = False

        if (pileUpDecision != self.currentPileup):
            self.pileupTime = time.time()

        self.currentPileup = pileUpDecision

    ## Function that updates self.currentPossession enum with the current possession
    # 
    # Looks at a number of factors including the balls trajectory towards robots
    # to determine possession
    #
    def ballPossessionUpdate(self):

        minimumPassSpeed = 2.2  #The minumum speed for the ball to be traveling to look for recieving robots
        ballRatioFactor = 6.0  #The ratio of robot closeness for automatic possession

        intercept_time = 0.7  #The remaining travel time for the ball to a robot for that robot to be considered recieving the ball

        for g in self.activeRobots:
            hasBall = self.possesses_the_ball(self.systemState.ball.pos, g)

            hadBall = self.hasBall.get(g)
            if (hadBall == None):
                hadBall = False
                self.hasBall[g] = False

            if (hasBall and not hadBall):
                self.posChangeTime[g] = time.time()

            if (hadBall and not hasBall):
                self.posDuration[g] = abs(time.time() - self.posChangeTime[g])
                self.posChangeTime[g] = time.time()

            self.hasBall[g] = hasBall

        if (self.currentPileup):
            self.currentPossession = self.BallPos.FREEBALL
            return None

        ballPossessionDurationThreshold = 0.07
        botsWithBall = self.robotsWithTheBall()
        if (len(botsWithBall) == 1 and
                abs(self.posChangeTime[botsWithBall[0]] - time.time()) >
                ballPossessionDurationThreshold):
            if (botsWithBall[0].is_ours()):
                self.currentPossession = self.BallPos.OURBALL
            else:
                self.currentPossession = self.BallPos.THEIRBALL
            return None

        lastInfo = self.hadBallLast()
        lastDurationThreshold = 0.5
        lastDurationLengthThreshold = 0.5
        if (lastInfo[0] != None and lastInfo[1] < lastDurationThreshold and
                lastInfo[2] > lastDurationLengthThreshold):
            if (lastInfo[0].is_ours()):
                self.currentPossession = self.BallPos.OURBALL
            else:
                self.currentPossession = self.BallPos.THEIRBALL
            return None

        ballDistRatio = self.ballClosenessRatio()
        if (ballDistRatio > ballRatioFactor):
            self.currentPossession = self.BallPos.THEIRBALL
            return None
        if (ballDistRatio < (1.0 / ballRatioFactor)):
            self.currentPossession = self.BallPos.OURBALL
            return None

        #This should probably eventually be updated to take the intercept location into accoutn rather 
        #than the current location for determining what situation we are in. Or not, I could see an arguement for both.
        if (self.systemState.ball.vel.mag() > minimumPassSpeed):
            recvr = self.closestReciever()
            if (recvr[0] != None and
                (recvr[1] / self.systemState.ball.vel.mag()) < intercept_time):

                if (recvr[0].is_ours()):
                    self.currentPossession = self.BallPos.OURBALL
                else:
                    self.currentPossession = self.BallPos.THEIRBALL
                return None

        self.currentPossession = self.BallPos.FREEBALL

    ##
    # Function that is called every frame to update the self.ballLocation enum value
    # based on the current ball location.
    # 
    # We have set the midfield factor to zero for div B fields
    #
    # @param midfieldFactor The fraction of the field to be designated as the midfield
    def locationUpdate(self, midfieldFactor = 0.0):

        ballPos = self.systemState.ball.pos

        fieldLen = constants.Field.Length
        midfield = fieldLen / 2

        if (ballPos.y < midfield - (midfieldFactor / 2) * fieldLen):
            self.ballLocation = self.FieldLoc.DEFENDSIDE
        elif (ballPos.y > midfield + (midfieldFactor / 2) * fieldLen):
            self.ballLocation = self.FieldLoc.ATTACKSIDE
        else:
            self.ballLocation = self.FieldLoc.MIDFIELD

    ## Function to determine if the ball is inside the goal zone
    def ballInGoalZone(self,
                       buff=constants.Robot.Radius + constants.Ball.Radius):
        ballPos = self.systemState.ball.pos
        if (ballPos.x - buff > constants.Field.OurGoalZoneShape.min_x() and
                ballPos.x + buff < constants.Field.OurGoalZoneShape.max_x() and
                ballPos.y + buff < constants.Field.OurGoalZoneShape.max_y() and
                ballPos.y > constants.Field.OurGoalZoneShape.min_y()):
            return True

        return False

    ##Function that determines if our goalie has the ball safely inside our goal zone 
    # 
    # Looks for if our goalie has the ball in a position to not be harassed by the other team
    # intended to interact with the GOALIE_CLEAR situation
    #
    def cleanGoaliePossession(self):
        goalieID = self.gameState.get_goalie_id()
        goalieBot = None

        for g in self.activeRobots:
            if (g.shell_id() == goalieID):
                goalieBot = g
                break

        goalieHasBall = self.hasBall.get(goalieBot)
        return goalieHasBall and self.ballInGoalZone()

    ##This function will detect if the ball is about to go out of bounds, or is headed towards the goal 
    #
    # Function not currently implemented but is intended to trigger the SAVE_BALL situation
    # and possibly the SAVE_GOAL situation
    #
    def ballTrajectoryUpdate(self, ballPos, ballVel, factor=0.5):
        #Find function that determines if a point is in bounds
        pass

    ## Sets the current situation to NONE
    def clearSituation(self):
        self.currentSituation = self.Situation.NONE

    ## Function that is called every frame to actually change currentSituation
    # 
    #  This function looks at the game state, the balls location, and the possession of the ball
    #  to make a determination as to the current situation, from there it sets an enum
    #
    def situationUpdate(self):
        #I've made all branches make an assignment for ease of debugging,
        #none assignments have been marked

        if (self.gameState.is_our_kickoff()):
            self.currentSituation = self.Situation.KICKOFF
        elif (self.gameState.is_our_penalty()):
            self.currentSituation = self.Situation.NONE  #Warning: assigns none
        elif (self.gameState.is_our_direct() or
              self.gameState.is_our_indirect()):
            if (self.isAttackSide()):
                self.currentSituation = self.Situation.OFFENSIVE_KICK
            elif (self.isMidfield()):
                self.currentSituation = self.Situation.MIDFIELD_KICK
            elif (self.isDefendSide()):
                self.currentSituation = self.Situation.DEFENSIVE_KICK
            else:
                self.currentSituation = self.Situation.NONE  #Warning: assigns none
        elif (self.gameState.is_our_free_kick()):
            self.currentSituation = self.Situation.NONE  #Warning: assigns none
        elif (self.gameState.is_their_kickoff()):
            self.currentSituation = self.Situation.DEFEND_RESTART_DEFENSIVE
        elif (self.gameState.is_their_penalty()):
            self.currentSituation = self.Situation.NONE  #Warning: assigns none
        elif (self.gameState.is_their_direct() or
              self.gameState.is_their_indirect()):
            if (self.isDefendSide()):
                self.currentSituation = self.Situation.DEFEND_RESTART_DEFENSIVE
            elif (self.isAttackSide()):
                self.currentSituation = self.Situation.DEFEND_RESTART_OFFENSIVE
            elif (self.isMidfield()):
                self.currentSituation = self.Situation.DEFEND_RESTART_MIDFIELD
        elif (self.gameState.is_their_free_kick()):
            self.currentSituation = self.Situation.NONE  #Warning: assigns none
        elif (self.isDefendSide()):
            if (self.cleanGoaliePossession(
            )):  #This does not trigger correctly currently
                self.currentSituaion = self.Situation.GOALIE_CLEAR
            elif (self.isPileup()):
                self.currentSituation = self.Situation.DEFENSIVE_PILEUP
            elif (self.isFreeBall()):
                self.currentSituation = self.Situation.DEFENSIVE_SCRAMBLE
            elif (self.isOurBall()):
                self.currentSituation = self.Situation.CLEAR
            elif (self.isTheirBall):
                self.currentSituation = self.Situation.DEFEND_GOAL
            else:
                self.currentSituation = self.Situation.NONE  #Warning: assigns none

        elif (self.isAttackSide()):
            if (self.isPileup()):
                self.currentSituation = self.Situation.OFFENSIVE_PILEUP
            elif (self.isFreeBall()):
                self.currentSituation = self.Situation.OFFENSIVE_SCRAMBLE
            elif (self.isOurBall()):
                self.currentSituation = self.Situation.ATTACK_GOAL
            elif (self.isTheirBall()):
                self.currentSituation = self.Situation.DEFEND_CLEAR
            else:
                self.currentSituation = self.Situation.NONE  #Warning: assigns none

        elif (self.isMidfield()):
            if (self.isPileup()):
                self.currentSituation = self.Situation.MIDFIELD_PILEUP
            elif (self.isFreeBall()):
                self.currentSituation = self.Situation.MIDFIELD_SCRAMBLE
            elif (self.isOurBall()):
                self.currentSituation = self.Situation.MIDFIELD_CLEAR
            elif (self.isTheirBall()):
                self.currentSituation = self.Situation.MIDFIELD_DEFEND_CLEAR
            else:
                self.currentSituation = self.Situation.NONE  #Warning: assigns none
        else:
            self.currentSituation = self.Situation.NONE  #Warning: assigns none
