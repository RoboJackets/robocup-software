
import main
import time
import robocup
import math
from enum import Enum
import constants


class SituationalPlaySelector:


    class situation(Enum):
            none = 0
            kickoff = 1 #Plays that can perform our kickoff
            indirect_kick = 2 #Plays that can perform our indirect kicks
            direct_kick = 3 #Plays that can perform our direct kicks
            defend_restart_offensive = 4 #Plays for defending our opponents restart on their side of the field
            defend_restart_midfield = 5 #Plays for defending our opponents restart in the midfield
            defend_restart_defensive = 6 #Plays for defending our opponents restart on our side of the field
            clear = 7 #play for clearing the ball from our side of the field (should include defensive caution)
            defend_clear = 8 #Plays for defending the opponents clear, when the ball is on their side.
            defend_goal = 9 #Plays for defending our goal from opponents near it with the ball
            midfield_clear = 10 #Plays for when we possess the ball in the midfield
            attack_goal = 11 #Plays for attacking the opponents goal, when we have the ball near it
            offensive_scramble = 12 #Plays for getting a loose ball when the ball is on the opponents half
            midfield_scramble = 13 #Plays for getting a loose ball when the ball is in the midfield
            defensive_scramble = 14 #Plays for getting a loose ball when the ball is on our half
            save_ball = 15 #Plays that will trigger when the ball is headed out of the field with no obstuctions
            save_shot = 16 #Plays that will trigger when the ball is headed directly at our goal
            offensive_pile_up = 17 #Plays to handle a pile up on their side of the field
            midfield_pile_up = 18 #Plays to handle a pile up in the midfield
            defensive_pile_up = 19 #Plays to handle a pile up on our side of the field
            midfield_defend_clear = 20 #Plays to defend a clear when the ball is in the midfield
            shootout = 21 #Plays for making shootout shots
            defend_shootout = 22 #Plays for defending shootout shots
            penalty = 23 #Plays for making penalty shots
            defend_penalty = 24 #Plays for defending penalty shots

    def __init__(self):
        print("Don't make an instance of this class you bafoon!") 
        exit() #This is a joke I'll need to remove at some point

    currentSituation = None

    ballPossessionScore = 0.0

    isSetup = False
    gameState = None
    systemState = None
    robotList = list()
    activeRobots = list()

    ballLocation = None

    currentPileup = False

    @classmethod
    def setupStates(cls):
        cls.gameState = main.game_state()
        cls.systemState = main.system_state()
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

    @classmethod
    def updateAnalysis(cls):

        if(not cls.isSetup):
            cls.setupStates()
            cls.isSetup = True
        else:
            cls.updateRobotList()

        cls.scoreUpdate()
        cls.ballLocation = cls.locationUpdate()
        cls.ballPossessionUpdate()
        printPoint1 = robocup.Point(0,0)
        #cls.systemState.draw_text(str(cls.getCurrentSituations()[0]), printPoint1, (0,0,0),"hat")

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

    #So this was going to be super fancy but for now its very simple, basically just based on the angle
    @classmethod
    def ball_recieve_prob(cls, ballPos, ballVel, robot):
        robotx = robot.pos.x
        roboty = robot.pos.y
        
        if(math.sqrt(ballVel.x**2 + ballVel.y**2) < 0.4):
            return 0.0

        robotToBall = [robotx - ballPos.x, roboty - ballPos.y]
        angle = math.degrees(math.atan2(ballVel.y, ballVel.x) - math.atan2(robotToBall[1], robotToBall[0]));
        if(abs(angle) > 90):
            return 0.0
        return (1 - abs(angle / 90))**4

    #A function that determines if the ball is in the mouth of a given robot
    @staticmethod
    def possesses_the_ball(ballPos, robot, distThresh=0.14, angleThresh=35):
        distance = math.sqrt((ballPos.x - robot.pos.x)**2 + (ballPos.y - robot.pos.y)**2)
        angle = math.degrees(math.atan2(ballPos.y - robot.pos.y, ballPos.x - robot.pos.x) - robot.angle)
        if(distance < distThresh and abs(angle) < angleThresh):
                return True
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

    #Update determining if we want to preempt the current play or not 
    def updatePreempt():
        pass

    #def addPreempt(play) possibly a function to add transition out of every state to the completed state with preemptPlay as the lambda
    # for g in states
    #    .add_transition(g -> completed , preemptPlay)
    #
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


    #Returns the robot that last had the ball and how long it was since they had the ball
    @classmethod
    def hadBallLast(cls): 
        lastRobot = None
        lastRobotTime = 0.0
        
        for g in cls.activeRobots:
           if()

        return (lastBot, timeSince, posTime)

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
            if(cls.hasBall(g)):
                robotsWithTheBall.append(g)

        return robotsWithTheBall

    @classmethod
    def withBallCount(cls):
        ourBots = 0
        theirBots = 0
        for g in robotsWithTheBall():
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
    def nearBallCount(cls, distance = 0.5):
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
        botsNearBall = nearBallCount()
        botsWithBall = withBallCount()
        totalNearBall = sum(botsNearBall)
        totalWithBall = sum(botsWithBall)

        if(totalNearBall >= 3 and botsNearBall[0] > 0 and botsNearBall[1] > 0):
            return True

        if(totalWithBall >= 2 and bothWithBall[0] > 0 and botsWithBall[1] > 0):
            return True

        return False

    @classmethod
    def ballPossessionUpdate(cls):

        for g in cls.activeRobots:
            printPoint1 = robocup.Point(g.pos.x + 0.1, g.pos.y)
            printPoint2 = robocup.Point(g.pos.x + 0.1, g.pos.y - 0.12)
            printPoint3 = robocup.Point(g.pos.x + 0.1, g.pos.y - 0.24)
            printPoint4 = robocup.Point(g.pos.x + 0.1, g.pos.y - 0.36)

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
            if(cls.hasBall[g]):
                cls.systemState.draw_text(str(abs(time.time() - cls.posChangeTime[g])), printPoint1, (0,0,0),"hat")
            else:
                cls.systemState.draw_text("N/A", printPoint1, (0,0,0),"hat")

            try:
                cls.systemState.draw_text(str(round(cls.posDuration[g],3)), printPoint2, (0,0,0),"hat")
            except:
                cls.systemState.draw_text("N/A", printPoint2, (0,0,0),"hat")

            cls.recvProb[g] = cls.ball_recieve_prob(cls.systemState.ball.pos, cls.systemState.ball.vel, g)

            cls.systemState.draw_text(str(round(cls.recvProb[g],3)), printPoint3, (0,0,0), "hat")

            try:
                cls.systemState.draw_text(str(round(abs(cls.posChangeTime[g] - time.time()),3)), printPoint4, (0,0,0), "hat")
            except:
                pass

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

        cls.ourBall = False
        cls.theirBall = False
        cls.freeBall = False


        thresh = 0.3

        if(cls.ballPossessionScore > thresh):
            cls.ourBall = True
        elif(cls.ballPossessionScore < -1 * thresh):
            cls.theirBall = True
        else:
            cls.freeBall = True

    
    class fieldLoc(Enum):
        defendSide = 1
        midfield = 2
        attackSide = 3

    class ballPos(Enum):
        ourBall = 1
        freeBall = 2
        theirBall = 3

    @classmethod
    def locationUpdate(cls):
        #This will basically just figure out what part of the field the ball is in.
        #This should probably be appended at some point to account where the ball will be in the near future
        
        midfieldFactor = 0.33  
        
        ballPos = cls.systemState.ball.pos

        fieldLen = constants.Field.Length
        midfield = fieldLen / 2


        if(ballPos.y < midfield - (midfieldFactor / 2) * fieldLen):
            return cls.fieldLoc.defendSide
        elif(ballPos.y > midfield + (midfieldFactor / 2) * fieldLen):
            return cls.fieldLoc.attackSide
        else:
            return cls.fieldLoc.midfield

    #This function will detect if the ball is about to go out of bounds, or is headed towards the goal
    @classmethod
    def ballTrajectoryUpdate(cls, ballPos, ballVel, factor=0.5):
        #Find function that determines if a point is in bounds
        pass

    @classmethod
    def clearSituation(cls):
        cls.currentSituation = cls.situation.none

    @classmethod
    def scoreUpdate(cls):

        restart = False
        if(cls.gameState.is_our_kickoff()):
            cls.currentSituation = cls.situation.kickoff
        if(cls.gameState.is_our_penalty()):
            pass
        if(cls.gameState.is_our_direct()):
            cls.currentSituation = cls.situation.direct_kick
        if(cls.gameState.is_our_indirect()):
            cls.currentSituation = cls.situation.indirect_kick
        if(cls.gameState.is_our_free_kick()):
            pass
        if(cls.gameState.is_their_kickoff()):
            cls.currentSituation = cls.situation.defend_restart_defensive
        if(cls.gameState.is_their_penalty()):
            pass 
        if(cls.gameState.is_their_direct() or cls.gameState.is_their_indirect()):
            if(cls.ballLocation == cls.fieldLoc.defendSide):
                cls.currentSituation = cls.situation.defend_restart_defensive
            elif(cls.ballLocation == cls.fieldLoc.attackSide):
                cls.currentSituation = cls.situation.defend_restart_offensive
            elif(cls.ballLocation == cls.fieldLoc.midfield):
                cls.currentSituation = cls.situation.defend_restart_midfield
        if(cls.gameState.is_their_free_kick()):
            pass

        if(cls.ballLocation == cls.fieldLoc.defendSide):
            if(cls.currentPileup):
                cls.currentSituation = cls.situation.defensive_pileup
            elif(cls.freeBall):
                cls.currentSituation = cls.situation.defensive_scramble
            elif(cls.ourBall):
                cls.currentSituation = cls.situation.clear
            elif(cls.theirBall):
                cls.currentSituation = cls.situation.defend_goal
            else:
                print("Situation analysis has done broke")
        
        elif(cls.ballLocation == cls.fieldLoc.attackSide):
            if(cls.currentPileup):
                cls.currentSituation = cls.situation.offensive_pileup
            elif(cls.freeBall):
                cls.currentSituation = cls.situation.offensive_scramble
            elif(cls.ourBall):
                cls.currentSituation = cls.situation.attack_goal
            elif(cls.theirBall):
                cls.currentSituation = cls.situation.defend_clear
            else:
                print("Situation analysis has done broke")
        
        elif(cls.ballLocation == cls.fieldLoc.midfield):
            if(cls.currentPileup):
                cls.currentSituation = cls.situation.midfield_pileup
            elif(cls.freeBall):
                cls.currentSituation = cls.situation.midfield_scramble
            elif(cls.ourBall):
                cls.currentSituation = cls.situation.midfield_clear
            elif(cls.theirBall):
                cls.currentSituation = cls.situation.midfield_defend_clear
            else:
                print("Situation analysis has done broke")




