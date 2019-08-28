
import main
import time
import robocup
import math
from enum import Enum
import constants


class SituationalPlaySelector:


    #Enum for representing the current game situation, each of which acts as a catagory of play to be run
    class situation(Enum):
            NONE = 0
            KICKOFF = 1 #Plays that can perform our kickoff
            DEFEND_RESTART_OFFENSIVE = 2 #Plays for defending our opponents restart on their side of the field
            DEFEND_RESTART_MIDFIELD = 3 #Plays for defending our opponents restart in the midfield
            DEFEND_RESTART_DEFENSIVE = 4 #Plays for defending our opponents restart on our side of the field
            CLEAR = 5 #play for clearing the ball from our side of the field (should include defensive caution)
            DEFEND_CLEAR = 6 #Plays for defending the opponents clear, when the ball is on their side.
            DEFEND_GOAL = 7 #Plays for defending our goal from opponents near it with the ball
            MIDFIELD_CLEAR = 8 #Plays for when we possess the ball in the midfield
            ATTACK_GOAL = 9 #Plays for attacking the opponents goal, when we have the ball near it
            OFFENSIVE_SCRAMBLE = 10 #Plays for getting a loose ball when the ball is on the opponents half
            MIDFIELD_SCRAMBLE = 11 #Plays for getting a loose ball when the ball is in the midfield
            DEFENSIVE_SCRAMBLE = 12 #Plays for getting a loose ball when the ball is on our half
            SAVE_BALL = 13 #Plays that will trigger when the ball is headed out of the field with no obstuctions
            SAVE_SHOT = 14 #Plays that will trigger when the ball is headed directly at our goal
            OFFENSIVE_PILEUP = 15 #Plays to handle a pile up on their side of the field
            MIDFIELD_PILEUP = 16 #Plays to handle a pile up in the midfield
            DEFENSIVE_PILEUP = 17 #Plays to handle a pile up on our side of the field
            MIDFIELD_DEFEND_CLEAR = 18 #Plays to defend a clear when the ball is in the midfield
            SHOOTOUT = 19 #Plays for making shootout shots
            DEFEND_SHOOTOUT = 20 #Plays for defending shootout shots
            PENALTY = 21 #Plays for making penalty shots
            DEFEND_PENALTY = 22 #Plays for defending penalty shots
            OFFENSIVE_KICK = 23 #Plays for direct and indirect kicks on their side
            DEFENSIVE_KICK = 24 #Plays for direct and indirect kicks on our side
            MIDFIELD_KICK = 25 #Plays for direct and indirect kicks in the midfield
            GOALIE_CLEAR = 26 #Plays for clearing the ball when our goalie possesses the ball

    #Enum for representing where the ball is on the field
    class fieldLoc(Enum):
        DEFENDSIDE = 1
        MIDFIELD = 2
        ATTACKSIDE = 3

    #Enum for representing the possession of the ball
    class ballPos(Enum):
        OURBALL = 1
        FREEBALL = 2
        THEIRBALL = 3

    def __init__(self):
        print("Don't make an instance of this class you bafoon!") 
        exit() #This is a joke I'll need to remove at some point

    currentSituation = situation.NONE
    currentPossession = ballPos.FREEBALL
    ballLocation = fieldLoc.MIDFIELD 
    currentPileup = False

    pileupTime = None #The first time at which a pileup was detected

    isSetup = False
    gameState = None
    systemState = None
    robotList = list()
    activeRobots = list()

    @classmethod
    def setupStates(cls):
        cls.gameState = main.game_state()
        cls.systemState = main.system_state()
        cls.context = main.context()
        for g in cls.systemState.our_robots:
            cls.robotList.append(g)
        for g in cls.systemState.their_robots:
            cls.robotList.append(g)

        cls.updateRobotList()

    @classmethod 
    def updateRobotList(cls): 
       cls.activeRobots.clear()
       for g in cls.robotList:
           if(g.visible):
               cls.activeRobots.append(g)

    #Calls all needed update functions
    #Needs to be called in main loop to make the module work
    @classmethod
    def updateAnalysis(cls):
        startTime = time.time()  
        if(not cls.isSetup):
            cls.setupStates()
            cls.isSetup = True
        else:
            cls.updateRobotList()

        cls.updatePileup()
        cls.ballLocation = cls.locationUpdate()
        cls.ballPossessionUpdate()
        cls.situationUpdate()
        

        cls.context.debug_drawer.draw_text(cls.currentSituation.name, robocup.Point(-3,-0.3), (0,0,0),"hat")
        #print(cls.currentSituation.name)
        #print(abs(time.time() - startTime))


    #It would be interesting to evaluate characteristics about our enemy
    #Like some kind of manuverability/speed characteristic
    #Would have to have one for each team
   
    '''
    def ballVelFactor(ballVel):
        return 1.0 

    @staticmethod
    def clip(x, minValue=-1.0, maxValue=1.0):
        if(x > maxValue):
            return maxValue
        elif(x < minValue):
            return minValue
        else:
            return x


    @classmethod
    def trajectory(cls, x, y, v, velWidthNon=0.2, velWidthLin=1.0):
        factor = v**velWidthNon * velWidthLin * ((math.sqrt((x - y)**2 + (y - x)**2)) / (math.sqrt(x**2 + y**2)))
        return cls.clip(factor) #I either have to make these class methods or do this manually fml


    @classmethod
    def falloff(cls, x, y, v, falloffLin=0.03, falloffNon=1.0):
        factor = 1.0 - falloffLin * (1.0 / v)**falloffNon * math.sqrt(x**2 + y**2)
        return cls.clip(factor,minValue = 0.0)
  
    @classmethod
    def obstruction(cls, x, y, obstDist, obsDrop=0.2):
        factor = obsDrop * cls.clip((math.sqrt(x**2 + y**2) - obstDist) * 0.5, minValue=0.0) * (3 / math.sqrt((x-y)**2 + (y-x)**2))
        return factor

    #A triweight kernal approximation of a gaussian
    @staticmethod
    def triweight(x):
        return (34.0 / 35.0) * ((1 - x**2)**3)

    #Velocity is a scalar here and the robots x and y are in the balls refrence frame
    @classmethod
    def ballRecieveFunction(cls, x, y, v, od):
        if(x + y < 0.0):
            return 0.0
        return cls.clip(cls.triweight(cls.trajectory(x,y,v)))
     
    #Transforms position vector from global to the ball (where the ball is always traveling in the pi / 4 direction)
    #(Really just a general transform but its made to do this in particular)
    @staticmethod
    def transformToBall(pos, ballPos, ballVel):
        #Current hypothesis is that I want to rotate the vector the angle of the
        #ball velocity plus pi / 4, will see how that plays out
        rotA = math.atan2(ballVel.y - pos[1], ballVel.x - pos[0]) + (math.pi / 4.0)
        x = pos[0] * math.cos(rotA) - pos[1] * math.sin(rotA) + ballPos.x
        y = pos[0] * math.sin(rotA) + pos[1] * math.cos(rotA) + ballPos.y
        return (x, y)

    @staticmethod
    def rotate_point(x, y, heading_deg):
        c = math.cos(math.radians(heading_deg))
        s = math.sin(math.radians(heading_deg))
        xr = x * c + y * -s
        yr = x * s + y * c
        return xr, yr

    @classmethod
    def get_obstruct_dist(cls):
        return 999999
    '''




    #These two functions are really a mess, its kind of bad
    @classmethod
    def in_ball_path(cls):
        ingress_info = dict()
        robotsInBallPath = list()
        for g in cls.activeRobots:
            ingress_info[g] = cls.ball_ingress(cls.systemState.ball.pos, cls.systemState.ball.vel, g)
            if(ingress_info[g] != None and ingress_info[g][0] != None and abs(ingress_info.get(g)[0]) < 0.1):
                robotsInBallPath.append(g)
       
        return robotsInBallPath
        '''if(ingress_info[cls.activeRobots[0]] != None):
            toPrint = ingress_info[cls.activeRobots[0]][0]
            if(toPrint != None):
                print(toPrint)'''

    #Second mess function
    @classmethod
    def ball_ingress(cls, ballPos, ballVel, robot):
        robotx = robot.pos.x
        roboty = robot.pos.y
        
        ballSpeed = math.sqrt(ballVel.x**2 + ballVel.y**2)
        if(ballSpeed < 0.15):
            return (None, None, None, None)
        
        robotToBall = (robotx - ballPos.x, roboty - ballPos.y)
        
        ballDist = math.sqrt(robotToBall[0]**2 + robotToBall[1]**2)
        angle = math.degrees(math.atan2(ballVel.y, ballVel.x) - math.atan2(robotToBall[1], robotToBall[0]))

        if(abs(angle) > 100):
            return (None, None, None, None)
       
        robotToBallDOTBallVel = robotToBall[0] * ballVel.x + robotToBall[1] * ballVel.y
        scalar = robotToBallDOTBallVel / (ballSpeed**2)
        robotOntoVelocity = (scalar * ballVel.x, scalar * ballVel.y) #The robots position relative to the ball projected onto the balls velocity
        projectedToRobot = (robotOntoVelocity[0] - robotToBall[0], robotOntoVelocity[1] - robotToBall[1]) #The vector from the projected vector to the robots position
        
        distanceFromPath = math.sqrt(projectedToRobot[0]**2 +  projectedToRobot[1]**2) 
        interceptDistance = math.sqrt(robotOntoVelocity[0]**2 + robotOntoVelocity[1]**2)

        return (distanceFromPath, interceptDistance, ballSpeed, angle)


    @classmethod
    def closestReciever(cls):
        botsInPath = cls.in_ball_path()
        if(len(botsInPath) == 0):
            return (None, None)
        closestRobot = None
        closestRobotDistance = 0.0
        ballLocation = cls.systemState.ball.pos
        for g in botsInPath:
            roboDist = cls.ballToRobotDist(g)
            if(closestRobot == None or roboDist < closestRobotDistance):
                closestRobot = g
                closestRobotDistance = roboDist

        return (closestRobot, closestRobotDistance)

        

    #A function that determines if the ball is in the mouth of a given robot
    @staticmethod
    def possesses_the_ball(ballPos, robot, distThresh=0.14, angleThresh=35):
        distance = math.sqrt((ballPos.x - robot.pos.x)**2 + (ballPos.y - robot.pos.y)**2)
        angle = math.degrees(math.atan2(ballPos.y - robot.pos.y, ballPos.x - robot.pos.x) - robot.angle)
        if(distance < distThresh and abs(angle) < angleThresh):
            return True
        else:
            return False


    hasBall = dict()
    posChangeTime = dict()
    posDuration = dict()
    recvProb = dict()
    ballDist = dict()

    lastSituation = None
    situationChangeTime = None
    playPreemptTime = 0.20 #The time after a situation changes before preempting the current play
    currentPreempt = False #If we are preempting the current play


    @classmethod
    def isFreeBall(cls):
        return cls.currentPossession == cls.ballPos.FREEBALL

    @classmethod
    def isOurBall(cls):
        return cls.currentPossession == cls.ballPos.OURBALL

    @classmethod
    def isTheirBall(cls):
        return cls.currentPossession == cls.ballPos.THEIRBALL

    @classmethod
    def isAttackSide(cls):
        return cls.ballLocation == cls.fieldLoc.ATTACKSIDE

    @classmethod
    def isDefendSide(cls):
        return cls.ballLocation == cls.fieldLoc.DEFENDSIDE

    @classmethod
    def isMidfield(cls):
        return cls.ballLocation == cls.fieldLoc.MIDFIELD

    @classmethod
    def isPileup(cls):
        return cls.currentPileup

    #Returns if we are in the specified situation without regard to 
    @classmethod
    def isSituation(cls, situation, check = False):
        up = situation.upper()

        if(check):
            found = False
            for g in cls.situation:
                if(up == g):
                    found = True
                    break
            if not found:
                raise Exception("Passed situation " + situaion + " / " + up + " is not an existing situation")

        
        if(cls.currentSituaion.name == up):
            return true
        else:
            return false
        

    #Update determining if we want to preempt the current play or not
    @classmethod
    def updatePreempt(cls):
        if(cls.lastSituation != cls.currentSituaion and cls.situationChangeTime == None):
            cls.situationChangeTime = time.time()

        if(cls.currentPreempt):
            cls.currentPreempt = False
        elif(cls.situationChangeTime != None):
            if(abs(time.time() - cls.situationChangeTime) > cls.playPreemptTime):
                cls.situationChangeTime = None
                cls.currentPreempt = True
                cls.LastSituation = cls.currentSituaion

    '''def addPreempt(play) possibly a function to add transition out of every state to the completed state with preemptPlay as the lambda
        for g in states:
            play.add_transition(g -> completed , preemptPlay)'''

    #You will also need to make sure you delete all subbehaviors on enter_completed in the play

    #A function to determine if the currently running play should be preempted
    @classmethod
    def preemptPlay():
        return currentPreempt

    @classmethod
    def ballToRobotDist(cls, robot):
        return math.sqrt((robot.pos.x - cls.systemState.ball.pos.x)**2 + (robot.pos.y -  cls.systemState.ball.pos.y)**2)

    #Returns a tuple of the closest robot to the ball and the distance that robot is away from the ball
    @classmethod
    def closestRobot(cls): 
        closestRobot = None
        closestRobotDistance = 0.0
        ballLocation = cls.systemState.ball.pos
        for g in cls.activeRobots:
            roboDist = ballToRobotDist(g)
            if(closestRobot == None or roboDist < closestRobotDistance):
                closestRobot = g
                closestRobotDistance = roboDist

        return (closestRobot, closestRobotDistance)


    #Returns true if our robot is closest to the ball
    @classmethod
    def ourRobotClosest(cls):
        return cls.closestRobot()[0].is_ours()

    #Returns a tuple of the distance from the ball of our closest robot and our opponents closest robot
    #(our distance, theirDistance)
    @classmethod
    def bothTeamsClosest(cls):
        ourClosest = None
        ourClosestDist = 0.0
        theirClosest = None
        theirClosestDist = 0.0 

        ballLocation = cls.systemState.ball.pos

        for g in cls.activeRobots:
            roboDist = cls.ballToRobotDist(g)
            if(g.is_ours()):
                if(ourClosest == None or roboDist < ourClosestDist):
                    ourClosest = g
                    ourClosestDist = roboDist
            else: 
                if(theirClosest == None or roboDist < theirClosestDist):
                    theirClosest = g
                    theirClosestDist = roboDist

        return (ourClosestDist, theirClosestDist)
   
    #Returns the ratio of our closest distance to the ball and the opponents closest distance to the ball
    @classmethod
    def ballClosenessRatio(cls):
        distances = cls.bothTeamsClosest()
        try:
            return distances[0] / distances[1]
        except:
            return 1.0

    #Returns the robot that last had the ball and how long it was since they had the ball
    @classmethod
    def hadBallLast(cls): 
        lastRobot = None
        lastRobotTime = 0.0
        
        for g in cls.activeRobots:
            if(cls.hasBall[g]):
               return (g, 0.0, abs(time.time() - cls.posChangeTime[g]))
            timeSincePoss = abs(time.time() - cls.posChangeTime.get(g ,float("inf")))
            if(lastRobot == None or timeSincePoss < lastRobotTime):
                lastRobot = g
                lastRobotTime = timeSincePoss
        
        if(cls.posChangeTime.get(lastRobot, float("inf")) == float("inf")):
            return (None, None, None)

        return (lastRobot, lastRobotTime, cls.posDuration.get(lastRobot,0.0))

    #Returns true if we had the ball last
    @classmethod
    def weHadBallLast(cls):
        if(hadBallLast()[0].is_ours()):
            return True
        else:
            return False

    @classmethod
    def updatePileup(cls):
        cls.currentPileup = cls.isPileup()

    @classmethod
    def robotsWithTheBall(cls):
        robotsWithTheBall = list()

        for g in cls.activeRobots:
            if(cls.hasBall.get(g, False)):
                robotsWithTheBall.append(g)

        return robotsWithTheBall

    @classmethod
    def withBallCount(cls):
        ourBots = 0
        theirBots = 0
        for g in cls.robotsWithTheBall():
            if(g.is_ours()):
                ourBots += 1
            else:
                theirBots += 1

        return (ourBots, theirBots)
        

    @classmethod
    def robotsNearTheBall(cls, distance = 0.5):
        robotsNearTheBall = list()
       
        for g in cls.activeRobots:
            if(cls.ballToRobotDist(g) < distance):
                robotsNearTheBall.append(g)

        return robotsNearTheBall

    @classmethod
    def nearBallCount(cls, distance = 0.2):
        ourBots = 0
        theirBots = 0
        for g in cls.robotsNearTheBall(distance):
            if(g.is_ours()):
                ourBots += 1
            else:
                theirBots += 1

        return (ourBots, theirBots)

    @classmethod
    def isPileup(cls):

        pileupBufferTime = 0.8 #the number of seconds of pile up conditions before a pile up is declared

        botsNearBall = cls.nearBallCount()
        botsWithBall = cls.withBallCount()
        totalNearBall = sum(botsNearBall)
        totalWithBall = sum(botsWithBall)

        currentPileup = False
        
        if(totalNearBall >= 3 and botsNearBall[0] > 0 and botsNearBall[1] > 0):
            currentPileup = True

        if(totalWithBall >= 2 and botsWithBall[0] > 0 and botsWithBall[1] > 0):
            currentPileup = True

        if(cls.pileupTime == None and currentPileup):
            cls.pileupTime = time.time()
        elif(cls.pileupTime != None and not currentPileup):
            cls.pileupTime = None

        return cls.pileupTime != None and abs(time.time() - cls.pileupTime) >= pileupBufferTime 

    @classmethod
    def ballPossessionUpdate(cls):

        minimumPassSpeed = 2.2 #The minumum speed for the ball to be traveling to look for recieving robots
        ballRatioFactor = 6.0 #The ratio of robot closeness for automatic possession

        for g in cls.activeRobots:
            #printPoint1 = robocup.Point(g.pos.x + 0.1, g.pos.y)
            #printPoint2 = robocup.Point(g.pos.x + 0.1, g.pos.y - 0.12)
            #printPoint3 = robocup.Point(g.pos.x + 0.1, g.pos.y - 0.24)
            #printPoint4 = robocup.Point(g.pos.x + 0.1, g.pos.y - 0.36)

            hasBall = cls.possesses_the_ball(cls.systemState.ball.pos,g)

            hadBall = cls.hasBall.get(g)
            if(hadBall == None):
                hadBall = False
                cls.hasBall[g] = False

            if(hasBall and not hadBall):
                cls.posChangeTime[g] = time.time()

            if(hadBall and not hasBall):
                cls.posDuration[g] = abs(time.time() - cls.posChangeTime[g])
                cls.posChangeTime[g] = time.time()

            cls.hasBall[g] = hasBall

            #cls.systemState.draw_text(str(round(cls.ball_recieve_prob(cls.systemState.ball.pos,cls.systemState.ball.vel,g), 3)), g.pos, (0.3,0,0),"hat")
            #cls.systemState.draw_text(str(cls.possesses_the_ball(cls.systemState.ball.pos,g)), printPoint, (0.3,0,0),"hat")
            #if(cls.hasBall[g]):
            #    cls.systemState.draw_text(str(abs(time.time() - cls.posChangeTime[g])), printPoint1, (0,0,0),"hat")
            #else:

            #try:
            #    cls.systemState.draw_text(str(round(cls.posDuration[g],3)), printPoint2, (0,0,0),"hat")
            #except:
            #    cls.systemState.draw_text("N/A", printPoint2, (0,0,0),"hat")

            #cls.recvProb[g] = cls.ball_recieve_prob(cls.systemState.ball.pos, cls.systemState.ball.vel, g)

            #cls.systemState.draw_text(str(round(cls.recvProb[g],3)), printPoint3, (0,0,0), "hat")

            #try:
            #except:
            #    pass

        if(cls.currentPileup):
            cls.currentPossession = cls.ballPos.FREEBALL
            return None

        ballPossessionDurationThreshold = 0.07
        botsWithBall = cls.robotsWithTheBall()
        if(len(botsWithBall) == 1 and abs(cls.posChangeTime[botsWithBall[0]] - time.time()) > ballPossessionDurationThreshold):
            #print(abs(cls.posChangeTime[botsWithBall[0]] - time.time())) 
            if(botsWithBall[0].is_ours()):
                cls.currentPossession = cls.ballPos.OURBALL
            else:
                cls.currentPossession = cls.ballPos.THEIRBALL
            return None

        lastInfo = cls.hadBallLast()
        lastDurationThreshold = 0.5
        lastDurationLengthThreshold = 0.5
        #print(str(lastInfo[1]) + " " + str(lastInfo[2]))
        if(lastInfo[0] != None and lastInfo[1] < lastDurationThreshold and lastInfo[2] > lastDurationLengthThreshold):
            if(lastInfo[0].is_ours()):
                cls.currentPossession = cls.ballPos.OURBALL
            else:
                cls.currentPossession = cls.ballPos.THEIRBALL
            return None


        ballDistRatio = cls.ballClosenessRatio()
        #print(ballDistRatio)
        if(ballDistRatio > ballRatioFactor):
            cls.currentPossession = cls.ballPos.THEIRBALL
            return None
        if(ballDistRatio < (1.0 / ballRatioFactor)):
            cls.currentPossession = cls.ballPos.OURBALL
            return None

        if(math.sqrt(cls.systemState.ball.vel.x**2 + cls.systemState.ball.vel.y**2) > minimumPassSpeed):
            recvr = cls.closestReciever()
            if(recvr[0] != None):
                if(recvr[0].is_ours()):
                    cls.currentPossession = cls.ballPos.OURBALL
                else:
                    cls.currentPossession = cls.ballPos.THEIRBALL
                return None
       
        cls.currentPossession = cls.ballPos.FREEBALL
        
        
        '''
        ourScore = 0.0
        theirScore = 0.0

        for g in cls.activeRobots:
            if(g.is_ours()):
                if(cls.hasBall[g]):
                    ourScore += abs(time.time() - cls.posChangeTime.get(g,time.time()))
                elif(cls.posDuration.get(g,0) > 0 and abs(time.time() - cls.posChangeTime[g]) < 4):
                    ourScore += cls.posDuration[g] / abs(time.time() - cls.posChangeTime[g])

                ourScore += cls.recvProb.get(g,0) * 15
            else:
                if(cls.hasBall[g]):
                    theirScore += abs(time.time() - cls.posChangeTime.get(g,time.time()))
                elif(cls.posDuration.get(g,0) > 0 and abs(time.time() - cls.posChangeTime[g]) < 4):
                    theirScore += cls.posDuration[g] / abs(time.time() - cls.posChangeTime[g])
 
                theirScore += cls.recvProb.get(g,0) * 15

        cls.ballPossessionScore = ourScore - theirScore
        
        thresh = 0.3

        if(cls.ballPossessionScore > thresh):
            cls.ourBall = True
        elif(cls.ballPossessionScore < -1 * thresh):
            cls.theirBall = True
        else:
            cls.freeBall = True
        '''
    
    
    @classmethod
    def locationUpdate(cls):
        #This will basically just figure out what part of the field the ball is in.
        midfieldFactor = 0.0 #We have concluded to change the midfield factor to zero for the div B field  
       
        ballPos = cls.systemState.ball.pos

        fieldLen = constants.Field.Length
        midfield = fieldLen / 2

        if(ballPos.y < midfield - (midfieldFactor / 2) * fieldLen):
            return cls.fieldLoc.DEFENDSIDE
        elif(ballPos.y > midfield + (midfieldFactor / 2) * fieldLen):
            return cls.fieldLoc.ATTACKSIDE
        else:
            return cls.fieldLoc.MIDFIELD


    @classmethod
    def ballInGoalZone(cls, buff = constants.Robot.Radius + constants.Ball.Radius):
        ballPos = cls.systemState.ball.pos
        if(ballPos.x - buff > constants.Field.OurGoalZoneShape.min_x()):
            if(ballPos.x + buff < constants.Field.OurGoalZoneShape.max_x()):
                if(ballPos.y + buff < constants.Field.OurGoalZoneShape.max_y()):
                    if(ballPos.y > constants.Field.OurGoalZoneShape.min_y()):
                        return True

        return False


    #Function that determines if our goalie has the ball safely inside our goal zone
    @classmethod
    def cleanGoaliePossession(cls):
        goalieID = cls.gameState.get_goalie_id()
        goalieBot = None

        for g in cls.activeRobots:
            if(g.shell_id() == goalieID):
                goalieBot = g
                break
      
        goalieHasBall = cls.hasBall.get(goalieBot)
        return goalieHasBall and cls.ballInGoalZone()

        #print(constants.Field.OurGoalZoneShape)

        #return False

    #This function will detect if the ball is about to go out of bounds, or is headed towards the goal
    @classmethod
    def ballTrajectoryUpdate(cls, ballPos, ballVel, factor=0.5):
        #Find function that determines if a point is in bounds
        pass

    @classmethod
    def clearSituation(cls):
        cls.currentSituation = cls.situation.NONE

    @classmethod
    def situationUpdate(cls):
        #I've made all branches make an assignment for ease of debugging,
        #none assignments have been marked

        if(cls.gameState.is_our_kickoff()):
            cls.currentSituation = cls.situation.KICKOFF
        elif(cls.gameState.is_our_penalty()):
            cls.currentSituation = cls.situation.NONE #Warning: assigns none
        elif(cls.gameState.is_our_direct() or cls.gameState.is_our_indirect()):
            if(cls.isAttackSide()):
                cls.currentSituation = cls.situation.OFFENSIVE_KICK
            elif(cls.isMidfield()):
                cls.currentSituation = cls.situation.MIDFIELD_KICK
            elif(cls.isDefendSide()):
                cls.currentSituation = cls.situation.DEFENSIVE_KICK
            else:
                cls.currentSituation = cls.situation.NONE #Warning: assigns none
        elif(cls.gameState.is_our_free_kick()):
            cls.currentSituation = cls.situation.NONE #Warning: assigns none
        elif(cls.gameState.is_their_kickoff()):
            cls.currentSituation = cls.situation.DEFEND_RESTART_DEFENSIVE
        elif(cls.gameState.is_their_penalty()):
            cls.currentSituation = cls.situation.NONE #Warning: assigns none
        elif(cls.gameState.is_their_direct() or cls.gameState.is_their_indirect()):
            if(cls.isDefendSide()):
                cls.currentSituation = cls.situation.DEFEND_RESTART_DEFENSIVE
            elif(cls.isAttackSide()):
                cls.currentSituation = cls.situation.DEFEND_RESTART_OFFENSIVE
            elif(cls.isMidfield()):
                cls.currentSituation = cls.situation.DEFEND_RESTART_MIDFIELD
        elif(cls.gameState.is_their_free_kick()):
            cls.currentSituation = cls.situation.NONE #Warning: assigns none
        elif(cls.isDefendSide()):
            if(cls.cleanGoaliePossession()): #This does not trigger correctly currently
                cls.currentSituaion = cls.situation.GOALIE_CLEAR
            elif(cls.isPileup()):
                cls.currentSituation = cls.situation.DEFENSIVE_PILEUP
            elif(cls.isFreeBall()):
                cls.currentSituation = cls.situation.DEFENSIVE_SCRAMBLE
            elif(cls.isOurBall()):
                cls.currentSituation = cls.situation.CLEAR
            elif(cls.isTheirBall):
                cls.currentSituation = cls.situation.DEFEND_GOAL
            else:
                cls.currentSituation = cls.situation.NONE #Warning: assigns none
        
        elif(cls.isAttackSide()):
            if(cls.isPileup()):
                cls.currentSituation = cls.situation.OFFENSIVE_PILEUP
            elif(cls.isFreeBall()):
                cls.currentSituation = cls.situation.OFFENSIVE_SCRAMBLE
            elif(cls.isOurBall()):
                cls.currentSituation = cls.situation.ATTACK_GOAL
            elif(cls.isTheirBall()):
                cls.currentSituation = cls.situation.DEFEND_CLEAR
            else:
                cls.currentSituation = cls.situation.NONE #Warning: assigns none
        
        elif(cls.isMidfield()):
            if(cls.isPileup()):
                cls.currentSituation = cls.situation.MIDFIELD_PILEUP
            elif(cls.isFreeBall()):
                cls.currentSituation = cls.situation.MIDFIELD_SCRAMBLE
            elif(cls.isOurBall()):
                cls.currentSituation = cls.situation.MIDFIELD_CLEAR
            elif(cls.isTheirBall()):
                cls.currentSituation = cls.situation.MIDFIELD_DEFEND_CLEAR
            else:
                cls.currentSituation = cls.situation.NONE #Warning: assigns none
        else:
            cls.currentSituation = cls.situation.NONE #Warning: assigns none




