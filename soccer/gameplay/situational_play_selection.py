
import main
import time
import robocup
import math

class SituationalPlaySelector:


    scoreBonus = 100

    ballPossessionScore = 0.0 #I'm thinking positive scores for our possession, and negative for our opponents
    #I am thinking about wheither this should be based on the last frame, or totally fresh each frame.
    #I'm thinking the latter makes more sense, some timers can handle that type of behavior.

    situations = {
            'kickoff' : 0, #Plays that can perform our kickoff
            'indirect_kick' : 0, #Plays that can perform our indirect kicks
            'direct_kick' : 0, #Plays that can perform our direct kicks
            'defend_restart_offensive' : 0, #Plays for defending our opponents restart on their side of the field
            'defend_restart_defensive' : 0, #Plays for defending our opponents restart on our side of the field
            'clear' : 0, #play for clearing the ball from our side of the field (should include defensive caution)
            'defend_clear' : 0, #Plays for defending the opponents clear, when the ball is on their side.
            'defend_goal' : 0, #Plays for defending our goal from opponents near it with the ball
            'attack_goal' : 0, #Plays for attacking the opponents goal, when we have the ball near it
            'offensive_scramble' : 0, #Plays for getting a loose ball when the ball is on the opponents half
            'defensive_scramble' : 0, #Plays for getting a loose ball when the ball is on our half
            'save_ball' : 0, #Plays that will trigger when the ball is headed out of the field with no obstuctions
            'save_shot' : 0, #Plays that will trigger when the ball is headed directly at our goal
            'offensive_pile_up' : 0, #Plays to handle a pile up on their side of the field
            'defensive_pile_up': 0} #Plays to handle a pile up on our side of the field
       

    def __init__(self):
        print("Don't make an instance of this class you bafoon!") 
        exit() #This is a joke I'll need to remove at some point

    isSetup = False
    gameState = None
    systemState = None
    robotList = list()
    activeRobots = list()


    lastFullPossession = dict()
    possessionLength = dict()

    possessionLastCalc = 0.0

    robotPossessionTimer = 0.0

    count = 0

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

        cls.ballPossessionUpdate()
        #cls.scoreUpdate()

    #It would be interesting to evaluate characteristics about our enemy
    #Like some kind of manuverability/speed characteristic
    #Would have to have one for each team
   
    
    #there should also probably be a factor for when the ball is moving too
    #fast to reasonably be captured, if that is a concern.

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
        if(math.sqrt(x**2 + y**2) > 4 * od or x + y < 0.0):
            return 0.0
        return cls.clip(cls.triweight(cls.trajectory(x,y,v) * cls.falloff(x,y,v) - cls.obstruction(x,y,od)), minValue=0.0)
     

    #penalty for not facing the ball as a function of ball velocity and distance to the ball 
    #@staticmethod
    #def rotationFactor(ballPos, ballVel, robotPos):
    #    return 1.0

    #Transforms position vector from global to the ball (where the ball is always traveling in the pi / 4 direction)
    #(Really just a general transform but its made to do this in particular)
    @staticmethod
    def transformToBall(pos, ballPos, ballVel):
        #Current hypothesis is that I want to rotate the vector the angle of the
        #ball velocity plus pi / 4, will see how that plays out
        rotA = math.atan2(ballVel.y, ballVel.x) + (math.pi / 4.0)
        x = pos[0] * math.cos(rotA) - pos[1] * math.sin(rotA) + ballPos.x
        y = pos[0] * math.cos(rotA) + pos[1] * math.cos(rotA) + ballPos.y
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

    @classmethod
    def ball_recieve_prob(cls, ballPos, ballVel, robot):
        if(math.sqrt(ballVel.x**2 + ballVel.y**2) < 0.2):
            return 0.0
        robotPos = (robot.pos.x, robot.pos.y)
        robotPos_ball = cls.transformToBall(robotPos, ballPos, ballVel)
        return cls.ballRecieveFunction(robotPos_ball[0], robotPos_ball[1], math.sqrt(ballVel.x**2 + ballVel.y**2), cls.get_obstruct_dist())

    #A function that determines if the ball is in the mouth of a given robot
    def possesses_the_ball(ballPos, robot):
        return math.sqrt(ballPos[0]**2 + ballPos[1]**2) < thresh and abs(math.atan2(ballPos[1] - robotPos[1], ballPos[0] - robotPos[0]) - robot.angle) < angleThresh


    @classmethod
    def ballPossessionUpdate(cls):


        #Ok, so we want to look at a couple of factors here

        #For one, if a bot has the ball in its mouth, than they probably are in possession of the ball
        #But we want to make sure that the ball is actually in the mouth, not just bouncing off.

        #So we want to look at the duration, direction, and distance away from the ball

        #We will want to think about how we want to register possession when the ball is within a close radius

        #1. Direct ball possession
            #Distance away from robot mouth
            #Probably sum of distance and rotation error times some factors

        #2. Recent ball possession
            #Should include factor for if ball was impelled (kicked)
            #A quick decaying timer from when the robot genuially had the ball in its mouth past the settle timer

        #3. Ball ingress towards robot
            #Probably a cone that is narrower based on velocity I would think
            #Might need to be a continuous function

        #4. 

        
        '''
        for i in cls.systemState.our_robots:
            #print(i)
            pass

        for i in cls.systemState.their_robots:
            #print(i)
            pass
        '''
        for g in cls.activeRobots:
            cls.systemState.draw_text(str(round(cls.ball_recieve_prob(cls.systemState.ball.pos,cls.systemState.ball.vel,g), 3)), g.pos, (0,0,0),"hat")
        print(cls.systemState.ball.pos)

        pass

    @classmethod
    def locationUpdate(cls):
        #This will basically just figure out what part of the field the ball is in.
        pass


    #This function will detect if the ball is about to go out of bounds, or is headed towards the goal
    @classmethod
    def ballTrajectoryUpdate(cls):
        pass

    @classmethod
    def scoreUpdate(cls):
        #This will actually update the dict
        pass

    #This method is stupid and does nothing for us (I need to figure out what it was supposed to do)
    @classmethod
    def getBonus(cls):
        return max([situations.get(t) for t in cls.situations])













