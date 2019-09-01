
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
        #raise Exception("Situation Analysis is intended to be a static class, and so instances should not be made")
        #raise Exception("Congruadulations, you've made a situational analysis object!!")
        pass 

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

    
    def setupStates(self):
        self.gameState = main.game_state()
        self.systemState = main.system_state()
        self.context = main.context()
        for g in self.systemState.our_robots:
            self.robotList.append(g)
        for g in self.systemState.their_robots:
            self.robotList.append(g)

        self.updateRobotList()

     
    def updateRobotList(self): 
       self.activeRobots.clear()
       for g in self.robotList:
           if(g.visible):
               self.activeRobots.append(g)

    #Calls all needed update functions
    #Needs to be called in main loop to make the module work
    
    def updateAnalysis(self):
        startTime = time.time()  
        if(not self.isSetup):
            self.setupStates()
            self.isSetup = True
        else:
            self.updateRobotList()

        self.updatePileup()
        self.ballLocation = self.locationUpdate()
        self.ballPossessionUpdate()
        self.situationUpdate()
        

        self.context.debug_drawer.draw_text(self.currentSituation.name, robocup.Point(-3,-0.3), (0,0,0),"hat")
        #print(self.currentSituation.name)
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


    
    def trajectory(self, x, y, v, velWidthNon=0.2, velWidthLin=1.0):
        factor = v**velWidthNon * velWidthLin * ((math.sqrt((x - y)**2 + (y - x)**2)) / (math.sqrt(x**2 + y**2)))
        return self.clip(factor) #I either have to make these class methods or do this manually fml


    
    def falloff(self, x, y, v, falloffLin=0.03, falloffNon=1.0):
        factor = 1.0 - falloffLin * (1.0 / v)**falloffNon * math.sqrt(x**2 + y**2)
        return self.clip(factor,minValue = 0.0)
  
    
    def obstruction(self, x, y, obstDist, obsDrop=0.2):
        factor = obsDrop * self.clip((math.sqrt(x**2 + y**2) - obstDist) * 0.5, minValue=0.0) * (3 / math.sqrt((x-y)**2 + (y-x)**2))
        return factor

    #A triweight kernal approximation of a gaussian
    @staticmethod
    def triweight(x):
        return (34.0 / 35.0) * ((1 - x**2)**3)

    #Velocity is a scalar here and the robots x and y are in the balls refrence frame
    
    def ballRecieveFunction(self, x, y, v, od):
        if(x + y < 0.0):
            return 0.0
        return self.clip(self.triweight(self.trajectory(x,y,v)))
     
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

    
    def get_obstruct_dist(self):
        return 999999
    '''




    #These two functions are really a mess, its kind of bad
    
    def in_ball_path(self):
        ingress_info = dict()
        robotsInBallPath = list()
        for g in self.activeRobots:
            ingress_info[g] = self.ball_ingress(self.systemState.ball.pos, self.systemState.ball.vel, g)
            if(ingress_info[g] != None and ingress_info[g][0] != None and abs(ingress_info.get(g)[0]) < 0.1):
                robotsInBallPath.append(g)
       
        return robotsInBallPath
        '''if(ingress_info[self.activeRobots[0]] != None):
            toPrint = ingress_info[self.activeRobots[0]][0]
            if(toPrint != None):
                print(toPrint)'''

    #Second mess function
    
    def ball_ingress(self, ballPos, ballVel, robot):
       
        robotx = robot.pos.x
        roboty = robot.pos.y
        
        ballSpeed = ballVel.mag()
        if(ballSpeed < 0.15):
            return (None, None, None, None)
        
        robotToBall = robot.pos - ballPos
        ballDist = robotToBall.mag() #math.sqrt(robotToBall[0]**2 + robotToBall[1]**2)
        angle = math.degrees(math.atan2(ballVel.y, ballVel.x) - math.atan2(robotToBall.y, robotToBall.x))

        if(abs(angle) > 100):
            return (None, None, None, None)
         
        robotToBallDOTBallVel = robotToBall.x * ballVel.x + robotToBall.y * ballVel.y
        scalar = robotToBallDOTBallVel / (ballSpeed**2)
        robotOntoVelocity = robocup.Point(scalar * ballVel.x, scalar * ballVel.y) #The robots position relative to the ball projected onto the balls velocity
        projectedToRobot = robocup.Point(robotOntoVelocity.x - robotToBall.x, robotOntoVelocity.y - robotToBall.y) #The vector from the projected vector to the robots position
        
        distanceFromPath = projectedToRobot.mag() #math.sqrt(projectedToRobot[0]**2 +  projectedToRobot[1]**2) 
        interceptDistance = robotOntoVelocity.mag()#math.sqrt(robotOntoVelocity[0]**2 + robotOntoVelocity[1]**2)

        return (distanceFromPath, interceptDistance, ballSpeed, angle)


    
    def closestReciever(self):
        botsInPath = self.in_ball_path()
        if(len(botsInPath) == 0):
            return (None, None)
        closestRobot = None
        closestRobotDistance = 0.0
        ballLocation = self.systemState.ball.pos
        for g in botsInPath:
            roboDist = self.ballToRobotDist(g)
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


    
    def isFreeBall(self):
        return self.currentPossession == self.ballPos.FREEBALL

    
    def isOurBall(self):
        return self.currentPossession == self.ballPos.OURBALL

    
    def isTheirBall(self):
        return self.currentPossession == self.ballPos.THEIRBALL

    
    def isAttackSide(self):
        return self.ballLocation == self.fieldLoc.ATTACKSIDE

    
    def isDefendSide(self):
        return self.ballLocation == self.fieldLoc.DEFENDSIDE

    
    def isMidfield(self):
        return self.ballLocation == self.fieldLoc.MIDFIELD

    
    def isPileup(self):
        return self.currentPileup

    #Returns if we are in the specified situation without regard to 
    
    def isSituation(self, situation, check = False):
        up = situation.upper()

        if(check):
            found = False
            for g in self.situation:
                if(up == g):
                    found = True
                    break
            if not found:
                raise Exception("Passed situation " + situaion + " / " + up + " is not an existing situation")

        
        if(self.currentSituaion.name == up):
            return true
        else:
            return false
        

    #Update determining if we want to preempt the current play or not
    
    def updatePreempt(self):
        if(self.lastSituation != self.currentSituaion and self.situationChangeTime == None):
            self.situationChangeTime = time.time()

        if(self.currentPreempt):
            self.currentPreempt = False
        elif(self.situationChangeTime != None):
            if(abs(time.time() - self.situationChangeTime) > self.playPreemptTime):
                self.situationChangeTime = None
                self.currentPreempt = True
                self.LastSituation = self.currentSituaion

    '''def addPreempt(play) possibly a function to add transition out of every state to the completed state with preemptPlay as the lambda
        for g in states:
            play.add_transition(g -> completed , preemptPlay)'''

    #You will also need to make sure you delete all subbehaviors on enter_completed in the play

    #A function to determine if the currently running play should be preempted
    
    def preemptPlay():
        return currentPreempt

    
    def ballToRobotDist(self, robot):
        return math.sqrt((robot.pos.x - self.systemState.ball.pos.x)**2 + (robot.pos.y -  self.systemState.ball.pos.y)**2)

    #Returns a tuple of the closest robot to the ball and the distance that robot is away from the ball
    
    def closestRobot(self): 
        closestRobot = None
        closestRobotDistance = 0.0
        ballLocation = self.systemState.ball.pos
        for g in self.activeRobots:
            roboDist = ballToRobotDist(g)
            if(closestRobot == None or roboDist < closestRobotDistance):
                closestRobot = g
                closestRobotDistance = roboDist

        return (closestRobot, closestRobotDistance)


    #Returns true if our robot is closest to the ball
    
    def ourRobotClosest(self):
        return self.closestRobot()[0].is_ours()

    #Returns a tuple of the distance from the ball of our closest robot and our opponents closest robot
    #(our distance, theirDistance)
    
    def bothTeamsClosest(self):
        ourClosest = None
        ourClosestDist = 0.0
        theirClosest = None
        theirClosestDist = 0.0 

        ballLocation = self.systemState.ball.pos

        for g in self.activeRobots:
            roboDist = self.ballToRobotDist(g)
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
    
    def ballClosenessRatio(self):
        distances = self.bothTeamsClosest()
        try:
            return distances[0] / distances[1]
        except:
            return 1.0

    #Returns the robot that last had the ball and how long it was since they had the ball
    
    def hadBallLast(self): 
        lastRobot = None
        lastRobotTime = 0.0
        
        for g in self.activeRobots:
            if(self.hasBall[g]):
               return (g, 0.0, abs(time.time() - self.posChangeTime[g]))
            timeSincePoss = abs(time.time() - self.posChangeTime.get(g ,float("inf")))
            if(lastRobot == None or timeSincePoss < lastRobotTime):
                lastRobot = g
                lastRobotTime = timeSincePoss
        
        if(self.posChangeTime.get(lastRobot, float("inf")) == float("inf")):
            return (None, None, None)

        return (lastRobot, lastRobotTime, self.posDuration.get(lastRobot,0.0))

    #Returns true if we had the ball last
    
    def weHadBallLast(self):
        if(hadBallLast()[0].is_ours()):
            return True
        else:
            return False

    
    def updatePileup(self):
        self.currentPileup = self.isPileup()

    
    def robotsWithTheBall(self):
        robotsWithTheBall = list()

        for g in self.activeRobots:
            if(self.hasBall.get(g, False)):
                robotsWithTheBall.append(g)

        return robotsWithTheBall

    
    def withBallCount(self):
        ourBots = 0
        theirBots = 0
        for g in self.robotsWithTheBall():
            if(g.is_ours()):
                ourBots += 1
            else:
                theirBots += 1

        return (ourBots, theirBots)
        

    
    def robotsNearTheBall(self, distance = 0.5):
        robotsNearTheBall = list()
       
        for g in self.activeRobots:
            if(self.ballToRobotDist(g) < distance):
                robotsNearTheBall.append(g)

        return robotsNearTheBall

    
    def nearBallCount(self, distance = 0.2):
        ourBots = 0
        theirBots = 0
        for g in self.robotsNearTheBall(distance):
            if(g.is_ours()):
                ourBots += 1
            else:
                theirBots += 1

        return (ourBots, theirBots)

    
    def isPileup(self):

        pileupBufferTime = 0.8 #the number of seconds of pile up conditions before a pile up is declared

        botsNearBall = self.nearBallCount()
        botsWithBall = self.withBallCount()
        totalNearBall = sum(botsNearBall)
        totalWithBall = sum(botsWithBall)

        currentPileup = False
        
        if(totalNearBall >= 3 and botsNearBall[0] > 0 and botsNearBall[1] > 0):
            currentPileup = True

        if(totalWithBall >= 2 and botsWithBall[0] > 0 and botsWithBall[1] > 0):
            currentPileup = True

        if(self.pileupTime == None and currentPileup):
            self.pileupTime = time.time()
        elif(self.pileupTime != None and not currentPileup):
            self.pileupTime = None

        return self.pileupTime != None and abs(time.time() - self.pileupTime) >= pileupBufferTime 

    
    def ballPossessionUpdate(self):

        minimumPassSpeed = 2.2 #The minumum speed for the ball to be traveling to look for recieving robots
        ballRatioFactor = 6.0 #The ratio of robot closeness for automatic possession

        intercept_time = 0.7 #The remaining travel time for the ball to a robot for that robot to be considered recieving the ball

        for g in self.activeRobots:
            #printPoint1 = robocup.Point(g.pos.x + 0.1, g.pos.y)
            #printPoint2 = robocup.Point(g.pos.x + 0.1, g.pos.y - 0.12)
            #printPoint3 = robocup.Point(g.pos.x + 0.1, g.pos.y - 0.24)
            #printPoint4 = robocup.Point(g.pos.x + 0.1, g.pos.y - 0.36)

            hasBall = self.possesses_the_ball(self.systemState.ball.pos,g)

            hadBall = self.hasBall.get(g)
            if(hadBall == None):
                hadBall = False
                self.hasBall[g] = False

            if(hasBall and not hadBall):
                self.posChangeTime[g] = time.time()

            if(hadBall and not hasBall):
                self.posDuration[g] = abs(time.time() - self.posChangeTime[g])
                self.posChangeTime[g] = time.time()

            self.hasBall[g] = hasBall

            #self.systemState.draw_text(str(round(self.ball_recieve_prob(self.systemState.ball.pos,self.systemState.ball.vel,g), 3)), g.pos, (0.3,0,0),"hat")
            #self.systemState.draw_text(str(self.possesses_the_ball(self.systemState.ball.pos,g)), printPoint, (0.3,0,0),"hat")
            #if(self.hasBall[g]):
            #    self.systemState.draw_text(str(abs(time.time() - self.posChangeTime[g])), printPoint1, (0,0,0),"hat")
            #else:

            #try:
            #    self.systemState.draw_text(str(round(self.posDuration[g],3)), printPoint2, (0,0,0),"hat")
            #except:
            #    self.systemState.draw_text("N/A", printPoint2, (0,0,0),"hat")

            #self.recvProb[g] = self.ball_recieve_prob(self.systemState.ball.pos, self.systemState.ball.vel, g)

            #self.systemState.draw_text(str(round(self.recvProb[g],3)), printPoint3, (0,0,0), "hat")

            #try:
            #except:
            #    pass

        if(self.currentPileup):
            self.currentPossession = self.ballPos.FREEBALL
            return None

        ballPossessionDurationThreshold = 0.07
        botsWithBall = self.robotsWithTheBall()
        if(len(botsWithBall) == 1 and abs(self.posChangeTime[botsWithBall[0]] - time.time()) > ballPossessionDurationThreshold):
            #print(abs(self.posChangeTime[botsWithBall[0]] - time.time())) 
            if(botsWithBall[0].is_ours()):
                self.currentPossession = self.ballPos.OURBALL
            else:
                self.currentPossession = self.ballPos.THEIRBALL
            return None

        lastInfo = self.hadBallLast()
        lastDurationThreshold = 0.5
        lastDurationLengthThreshold = 0.5
        #print(str(lastInfo[1]) + " " + str(lastInfo[2]))
        if(lastInfo[0] != None and lastInfo[1] < lastDurationThreshold and lastInfo[2] > lastDurationLengthThreshold):
            if(lastInfo[0].is_ours()):
                self.currentPossession = self.ballPos.OURBALL
            else:
                self.currentPossession = self.ballPos.THEIRBALL
            return None


        ballDistRatio = self.ballClosenessRatio()
        #print(ballDistRatio)
        if(ballDistRatio > ballRatioFactor):
            self.currentPossession = self.ballPos.THEIRBALL
            return None
        if(ballDistRatio < (1.0 / ballRatioFactor)):
            self.currentPossession = self.ballPos.OURBALL
            return None

        if(self.systemState.ball.vel.mag() > minimumPassSpeed):
            recvr = self.closestReciever()
            #if(recvr[0] != None):
                #print(recvr[1] / self.systemState.ball.vel.mag())
            if(recvr[0] != None and (recvr[1] / self.systemState.ball.vel.mag()) < intercept_time):
                
                if(recvr[0].is_ours()):
                    self.currentPossession = self.ballPos.OURBALL
                else:
                    self.currentPossession = self.ballPos.THEIRBALL
                return None
       
        self.currentPossession = self.ballPos.FREEBALL
        
        
        '''
        ourScore = 0.0
        theirScore = 0.0

        for g in self.activeRobots:
            if(g.is_ours()):
                if(self.hasBall[g]):
                    ourScore += abs(time.time() - self.posChangeTime.get(g,time.time()))
                elif(self.posDuration.get(g,0) > 0 and abs(time.time() - self.posChangeTime[g]) < 4):
                    ourScore += self.posDuration[g] / abs(time.time() - self.posChangeTime[g])

                ourScore += self.recvProb.get(g,0) * 15
            else:
                if(self.hasBall[g]):
                    theirScore += abs(time.time() - self.posChangeTime.get(g,time.time()))
                elif(self.posDuration.get(g,0) > 0 and abs(time.time() - self.posChangeTime[g]) < 4):
                    theirScore += self.posDuration[g] / abs(time.time() - self.posChangeTime[g])
 
                theirScore += self.recvProb.get(g,0) * 15

        self.ballPossessionScore = ourScore - theirScore
        
        thresh = 0.3

        if(self.ballPossessionScore > thresh):
            self.ourBall = True
        elif(self.ballPossessionScore < -1 * thresh):
            self.theirBall = True
        else:
            self.freeBall = True
        '''
    
    
    
    def locationUpdate(self):
        #This will basically just figure out what part of the field the ball is in.
        midfieldFactor = 0.0 #We have concluded to change the midfield factor to zero for the div B field  
       
        ballPos = self.systemState.ball.pos

        fieldLen = constants.Field.Length
        midfield = fieldLen / 2

        if(ballPos.y < midfield - (midfieldFactor / 2) * fieldLen):
            return self.fieldLoc.DEFENDSIDE
        elif(ballPos.y > midfield + (midfieldFactor / 2) * fieldLen):
            return self.fieldLoc.ATTACKSIDE
        else:
            return self.fieldLoc.MIDFIELD


    
    def ballInGoalZone(self, buff = constants.Robot.Radius + constants.Ball.Radius):
        ballPos = self.systemState.ball.pos
        if(ballPos.x - buff > constants.Field.OurGoalZoneShape.min_x() and
                ballPos.x + buff < constants.Field.OurGoalZoneShape.max_x() and
                ballPos.y + buff < constants.Field.OurGoalZoneShape.max_y() and
                ballPos.y > constants.Field.OurGoalZoneShape.min_y()):
            return True

        return False


    #Function that determines if our goalie has the ball safely inside our goal zone
    
    def cleanGoaliePossession(self):
        goalieID = self.gameState.get_goalie_id()
        goalieBot = None

        for g in self.activeRobots:
            if(g.shell_id() == goalieID):
                goalieBot = g
                break
      
        goalieHasBall = self.hasBall.get(goalieBot)
        return goalieHasBall and self.ballInGoalZone()

        #print(constants.Field.OurGoalZoneShape)

        #return False

    #This function will detect if the ball is about to go out of bounds, or is headed towards the goal
    
    def ballTrajectoryUpdate(self, ballPos, ballVel, factor=0.5):
        #Find function that determines if a point is in bounds
        pass

    
    def clearSituation(self):
        self.currentSituation = self.situation.NONE

    
    def situationUpdate(self):
        #I've made all branches make an assignment for ease of debugging,
        #none assignments have been marked

        if(self.gameState.is_our_kickoff()):
            self.currentSituation = self.situation.KICKOFF
        elif(self.gameState.is_our_penalty()):
            self.currentSituation = self.situation.NONE #Warning: assigns none
        elif(self.gameState.is_our_direct() or self.gameState.is_our_indirect()):
            if(self.isAttackSide()):
                self.currentSituation = self.situation.OFFENSIVE_KICK
            elif(self.isMidfield()):
                self.currentSituation = self.situation.MIDFIELD_KICK
            elif(self.isDefendSide()):
                self.currentSituation = self.situation.DEFENSIVE_KICK
            else:
                self.currentSituation = self.situation.NONE #Warning: assigns none
        elif(self.gameState.is_our_free_kick()):
            self.currentSituation = self.situation.NONE #Warning: assigns none
        elif(self.gameState.is_their_kickoff()):
            self.currentSituation = self.situation.DEFEND_RESTART_DEFENSIVE
        elif(self.gameState.is_their_penalty()):
            self.currentSituation = self.situation.NONE #Warning: assigns none
        elif(self.gameState.is_their_direct() or self.gameState.is_their_indirect()):
            if(self.isDefendSide()):
                self.currentSituation = self.situation.DEFEND_RESTART_DEFENSIVE
            elif(self.isAttackSide()):
                self.currentSituation = self.situation.DEFEND_RESTART_OFFENSIVE
            elif(self.isMidfield()):
                self.currentSituation = self.situation.DEFEND_RESTART_MIDFIELD
        elif(self.gameState.is_their_free_kick()):
            self.currentSituation = self.situation.NONE #Warning: assigns none
        elif(self.isDefendSide()):
            if(self.cleanGoaliePossession()): #This does not trigger correctly currently
                self.currentSituaion = self.situation.GOALIE_CLEAR
            elif(self.isPileup()):
                self.currentSituation = self.situation.DEFENSIVE_PILEUP
            elif(self.isFreeBall()):
                self.currentSituation = self.situation.DEFENSIVE_SCRAMBLE
            elif(self.isOurBall()):
                self.currentSituation = self.situation.CLEAR
            elif(self.isTheirBall):
                self.currentSituation = self.situation.DEFEND_GOAL
            else:
                self.currentSituation = self.situation.NONE #Warning: assigns none
        
        elif(self.isAttackSide()):
            if(self.isPileup()):
                self.currentSituation = self.situation.OFFENSIVE_PILEUP
            elif(self.isFreeBall()):
                self.currentSituation = self.situation.OFFENSIVE_SCRAMBLE
            elif(self.isOurBall()):
                self.currentSituation = self.situation.ATTACK_GOAL
            elif(self.isTheirBall()):
                self.currentSituation = self.situation.DEFEND_CLEAR
            else:
                self.currentSituation = self.situation.NONE #Warning: assigns none
        
        elif(self.isMidfield()):
            if(self.isPileup()):
                self.currentSituation = self.situation.MIDFIELD_PILEUP
            elif(self.isFreeBall()):
                self.currentSituation = self.situation.MIDFIELD_SCRAMBLE
            elif(self.isOurBall()):
                self.currentSituation = self.situation.MIDFIELD_CLEAR
            elif(self.isTheirBall()):
                self.currentSituation = self.situation.MIDFIELD_DEFEND_CLEAR
            else:
                self.currentSituation = self.situation.NONE #Warning: assigns none
        else:
            self.currentSituation = self.situation.NONE #Warning: assigns none




