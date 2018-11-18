import single_robot_composite_behavior
import behavior
from enum import Enum
import main
import constants
import robocup
import skills.move
import time
import datetime
import numpy as np
import math


## Motion Benchmark V0.0.0.0
#
# A skill for testing the capabilities of our robots motion control
# 
# Note: I think I might need to add at least two different rotation tests, one for big rotations and one for small rotations
#
#
class MotionBenchmark(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    
    class State(Enum):
        #Noise and latency test
        setup = 1
        noise = 2
        move1 = 3

        #Basic motion triangle
        BasicMotion0 = 4
        BasicMotionEnd = 5
        EndAll = 6


  

    #General Result Variables
    resultsToWrite = [] #should possibly make a seperate list for more/less verbose output
    
    dateString = datetime.datetime.now().strftime("Run time: %I:%M%p on %B %d, %Y")
        #I also need this info for the file name
    versionString = "Version: 0.0"
    #Note: Add the type of processor used
    #Note: If possible, add the battery voltage of the robot that is performing the test
                #Add a warning message if the battery is below almost full
    #Note: add a function for things that only go into the file
                #Add a section to the file to put notes about the current run
                #Might also put the actual data, mabye a csv if I'm feeling frisky
    #Note: add a warning if the field size is too small to run the tests
   

    class BasicMotionTest:

        title = "No Name Test"
        startTime = 0.0

        lineErrorTimer = 0.0
        rotErrorTimer = 0.0
        
        point0 = None
        point1 = None
        point2 = None

        facePoint0 = None
        facePoint1 = None
        facePoint2 = None
        
        timeTaken = None
        posEndError = None
        lineFollowError = None
        rotationalFollowError = None
        finalRotationalError = None
        maxOvershoot = None

        count = 0
        runs = 5

        motionNumber = 0

        def __init__(self, nRuns):
            self.runs = nRuns
            timeTaken = [0.0] * self.runs
            posEndError = [0.0] * self.runs
            lineFollowError = [0.0] * self.runs
            rotationalFollowError = [0.0] * self.runs
            finalRotationalError = [0.0] * self.runs
            maxOvershoot = [(0,0)] * self.runs

        currentStart = None
        currentEnd = None
        currentFacePoint = None

        def startRun(self):
            points = [point0, point1, point2]
            facePoints = [facePoint0, facePoint1, facePoint2]
            self.startTime = time.time()
            self.currentStart = points[2] if motionNumber % 3 == 0 else points[(motionNumber % 3) - 1]
            self.currentEnd = points[motionNumber % 3]
            self.currentFacePoint = facePoints[motionNumber % 3]
            
        def processRun(self):
            if(currentEnd != None):
                integrateLineError()
                updateOvershoot()
            if(currentFace != None):
                integrateRotError()
            

        def endRun(self):
            self.timeTaken[motionNumber] = abs(startTime - time.time())
            finalRotationError()
            finalPosError()
            self.motionNumber = self.motionNumber + 1


        def finalRotaionalError(self):
            self.finalRotationalError[motionNumber] = getAngleError(currentFacePoint)

        def finalPosError(self):
            self.posEndError[motionNumber] = getPosError(currentEnd)

        def integrateLineError(self):
            deltat = abs(self.lineErrorTimer - time.time())
            self.lineErrorTimer = time.time()
            lineFollowError[motionNumber] += getLineError(currentStart, currentEnd) * deltat

        def integrateRotationalError(self):
            deltat = abs(rotErrorTimer - time.time())
            rotErrorTimer = time.time()
            rotFollowError[motionNumber] += getAngleError(currentFacePoint) * deltat

        def updateOvershoot(self):
            perOvershoot = pOvershoot(currentStart, currentEnd)
            if(perOvershoot[0] > maxOvershoot[motionNumber][0]):
                maxOvershoot[motionNumber] =  perOvershoot

        def isComplete(self):
            if(motionNumber * 3 >= runs):
                return True
            else:
                return False
        
          #End General Result Variables


    #Movement test points START
    setupPoint = robocup.Point(0, 1.2)

        #BasicMid
    BasicMid0Point = robocup.Point(1.2,1.2)
    BasicMid1Point = robocup.Point(-1.2,1.2)
    BasicMid2Point = robocup.Point(0,3.5)

        #BasicSmall
    BasicSmall0Point = robocup.Point(-0.75, 1.2)
    BasicSmall1Point = robocup.Point(0.75, 1.2)
    BasicSmall2Point = robocup.Point(0,2.4)

        #BasicLarge
    BasicLarge0Point = robocup.Point(-1.7,1.5)
    BasicLarge1Point = robocup.Point(0, 4.8)
    BasicLarge2Point = robocup.Point(1.7, 1.5)

        #BasicSmaller
    BasicSmaller0Point = robocup.Point(0.25, 1.2)
    BasicSmaller1Point = robocup.Point(-0.25, 1.2)
    BasicSmaller2Point = robocup.Point(0, 1.7)
    
        #BasicTiny
    BasicTiny0Point = robocup.Point(0.085, 1.2)
    BasicTiny1Point = robocup.Point(-0.085, 1.2)
    BasicTiny2Point = robocup.Point(0, 1.285)

        #Micro
    Micro0Point = robocup.Point(0.034, 1.2)
    Micro1Point = robocup.Point(-0.034, 1.2)
    Micro2Point = robocup.Point(0,1.242)

        #PureRot
    startPoint = robocup.Point(0,1.5)
    PureRot0FacePoint = robocup.Point(0, 2.5)
    PureRot1FacePoint = robocup.Point(-1,1.5)
    PureRot2FacePoint = robocup.Point(1,1.5)

        #MidFace
    MidFace0Point = BasicMid0Point
    MidFace1Point = BasicMid1Point
    MidFace2Point = BasicMid2Point

    MidFace0FacePoint = robocup.Point(0,2.8)
    MidFace1FacePoint = robocup.Point(0,0)
    MidFace2FacePoint = robocup.Point(2,0)

    #Movement test points END


    #Latency Measurement Variables
    noiseStartTime = 0.0
    noiseStartPos = None

    noiseMeasured = False
    noiseMaxX = 0.0
    noiseMinX = 0.0
    noiseMaxY = 0.0
    noiseMinY = 0.0

    NoiseTest = 0.0
    NoiseTestDone = False
    noiseStartTime = 0.0

    noiseResult = 0.0

    moveStartTime = 0.0
    moveEndTime = 0.0
    #End Latency Measurement Variables

    def __init__(self):
        super().__init__(continuous=False) 

        #The list of states to be registered
        allStates = [MotionBenchmark.State.setup, 
                     MotionBenchmark.State.noise,
                     MotionBenchmark.State.move1,
                     MotionBenchmark.State.BasicMotion0,
                     MotionBenchmark.State.BasicMotionEnd,
                     MotionBenchmark.State.EndAll]
                    

        #Register states in the previously defined list
        for g in allStates: 
            self.add_state(g, behavior.Behavior.State.running)

        #TRANSITIONS 

        #Reset
        self.add_transition(behavior.Behavior.State.start,
                            MotionBenchmark.State.setup, lambda: True,
                            'immediately')

        #Setup -> Noise
        self.add_transition(MotionBenchmark.State.setup,
                            MotionBenchmark.State.noise,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #Noise -> Move1
        self.add_transition(MotionBenchmark.State.noise,
                            MotionBenchmark.State.move1,
                            lambda: self.noiseMeasured,
                            'The noise has been measured')

        #Move1 -> BasicMotion0
        self.add_transition(MotionBenchmark.State.move1,
                            MotionBenchmark.State.BasicMotion0,
                            lambda: self.all_subbehaviors_completed(), 'In Position')
        
        #BasicMotion0 -> BasicMotion0
        self.add_transition(MotionBenchmark.State.BasicMotion0,
                            MotionBenchmark.State.BasicMotion0,
                            lambda: self.all_subbehaviors_completed() and not self.currentBasicMotion.isCompleted(), 'In Position')

        #BasicMid0 -> BasicMotionEnd
        self.add_transition(MotionBenchmark.State.BasicMotion0,
                            MotionBenchmark.State.BasicMotionEnd,
                            lambda: self.all_subbehaviors_completed() and self.currentBasicMotion.isCompleted(), 'In Position')

        #BasicMotionEnd -> BasicMotion0
        self.add_transition(MotionBenchmark.State.BasicMotionEnd,
                            MotionBenchmark.State.BasicMotion0,
                            lambda: self.basicMotionCount < len(self.basicMotionTests), 'In Position')


        #BasicMotionEnd -> exit the behavior  
        self.add_transition(MotionBenchmark.State.BasicMotionEnd,
                            behavior.Behavior.State.completed,
                            lambda: self.basicMotionCount >= len(self.basicMotionTests), 'In Position')

        #END TRANSITIONS

    #Utility functions START

    #A function that both prints and adds to the output file list to be written
    def resultOut(self, result):
        print(result)
        self.resultsToWrite.append(result)


    def fileOnly(self, result):
        self.resultsToWrite.append(result)



    #A function to determine if the robot has broken the bounding box
    #created by measuring the noise
    def brokenNoise(self):
        deltaX = self.noiseStartPos.x - self.robot.pos.x 
        deltaY = self.noiseStartPos.y - self.robot.pos.y
        if(deltaX > self.noiseMaxX):
            return True
        if(deltaX < self.noiseMinX):
            return True
        if(deltaY < self.noiseMinY):
            return True
        if(deltaY > self.noiseMaxY):
            return True
        return False


    def getAngleError(point):
        targetAngle = robot.angle
        betweenVec = robot.pos - point
        currentAngle = math.atan2(betweenVec.x, betweenVec.y)
        return targetAngle - currentAngle

    def getPosError(point):
        xErr = robot.pos.x - point.x
        yErr = robot.pos.y - point.y
        return math.sqrt(xErr**2 + yErr**2)

    def getLineError(start, end):
        startNumpy=np.array([start.x,start.y])
        endNumpy=np.array([end.x,end.y])
        robotNumpy=np.array([robot.pos.x,robot.pos.y])
        d = np.cross(endNumpy-endNumpy,endNumpy-robotNumpy)/np.linalg.norm(endNumpy-startNumpy)
        return d
 


    def getOvershoot(start, end):
       distToStart = math.sqrt((robot.pos.x - start.x)**2 + (robot.pos.x - start.y)**2)
       startToEnd = math.sqrt((start.x - end.x)**2 + (start.y - end.y)**2)
       overshoot = distToStart - starToEnd
       if(overshoot <= 0):
           return 0
       else:
           return overshoot


    def pOvershoot(start, end):
        overshoot = getOvershoot(start, end) 
        if(overshoot > 0):
            moveDist = math.sqrt((start.x - end.x)**2 + (start.y - end.y)**2)
            return (overshoot, overshoot / moveDist)
        else:
            return (0,0)


    #Utility functions END


    basicMotionTests = []

    superBasicTest = BasicMotionTest(3)

    basicMotionTests.append(superBasicTest)

    basicMotionIndex = 0
    currentBasicMotion = basicMotionTests[basicMotionIndex]



    #Setup state functions (for the latency test)

    def on_enter_setup(self):
        move_point = robocup.Point(0, constants.Field.Width / 4)
        self.add_subbehavior(skills.move.Move(move_point), 'move') 

    def on_exit_setup(self):
        self.remove_all_subbehaviors()

    #End setup state functions



    #Noise state functions

    def on_enter_noise(self):
        self.noiseStartTime = time.time()
        self.noiseStartPos = self.robot.pos

    def execute_noise(self):
        if(abs(self.noiseStartTime - time.time()) >=  5):
            self.noiseMeasured = True
        deltaX = self.noiseStartPos.x - self.robot.pos.x 
        deltaY = self.noiseStartPos.y - self.robot.pos.y
        if(deltaX > self.noiseMaxX):
            self.noiseMaxX = deltaX
        if(deltaX < self.noiseMinX):
            self.noiseMinX = deltaX
        if(deltaY < self.noiseMinY):
            self.noiseMinY = deltaY
        if(deltaY > self.noiseMaxY):
            self.noiseMaxY = deltaY

    #End noise state functions

    #move1 functions START

    def on_enter_move1(self):
        self.moveStartTime = time.time()
        self.add_subbehavior(skills.move.Move(robocup.Point(0, constants.Field.Width / 2)), 'move')


    def execute_move1(self):
        noiseTestDone = self.brokenNoise()
        if(noiseTestDone):
            self.remove_all_subbehaviors()

    def on_exit_move1(self):
        self.moveEndTime = time.time()
        self.remove_all_subbehaviors() 
        self.noiseResult = abs(self.moveStartTime - self.moveEndTime)
        print("------------------LATENCY TEST RESULTS---------------------")
        print("Latency (seconds) = " + str(self.noiseResult))
        print("X noise = " + str((abs(self.noiseMaxX) + abs(self.noiseMinX))))
        print("Y noise = " + str((abs(self.noiseMaxY) + abs(self.noiseMinY))))
        print("-----------------------------------------------------------")


    def on_enter_BasicMotion0(self):
        self.currentBasicMotion.startRun()
        if (self.currentBasicMotion.currentEnd is not None):
            self.add_subbehavior(skills.move.Move(self.currentBasicMotion.currentEnd), 'move')
        if (self.currentBasicMotion.currentFacePoint is not None):
            robot.face(self.currentBasicMotion.currentFacePoint)

    def execute_BasicMotion0(self):
        if (self.currentBasicMotion.currentFacePoint is not None):
            robot.face(self.currentBasicMotion.currentFacePoint)
        self.currentBasicMotion.processRun()

    def on_exit_BasicMotion0(self):
        self.currentBasicMotion.endRun()


    def on_enter_BasicMotionEnd(self):
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa Hats")        









    #nothing special for role requirements
    def role_requirements(self):
        reqs = super().role_requirements()
        return reqs
