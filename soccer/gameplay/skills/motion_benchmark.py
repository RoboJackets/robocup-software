import single_robot_composite_behavior
import single_robot_behavior
import behavior
from enum import Enum
import main
import constants
import robocup
import skills.move
import skills.face
import time
import datetime
import numpy as np
import math
import role_assignment
import composite_behavior
from operator import truediv
import statistics
from abc import ABC, abstractmethod

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
        ProcessBasicMotion = 6
        BasicMotionBuffer = 7

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

    #An abstract base class for motion tests
    class MotionTest(ABC):
        
        title = "No Name Test"

        sParams = dict()
        sResults = dict()
        sScore = dict()

        theMotionBenchmark = None
        
        motionStartTime = 0.0
        timeSinceLastCalc = 0.0

        runs = 0
   
        def __init__(self, nRuns, benchmark):
            self.runs = nRuns
            self.theMotionBenchmark = benchmark

        #Just a distance calculation
        def calcDist(self, point1, point2):
            if(point1 is not None and point2 is not None):
                return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
            else:
                return 0

        #Title for prints
        def __str__(self):
            return self.title

        @abstractmethod
        def setupTest(self):
            pass

        @abstractmethod
        def startMotion(self):
            pass

        @abstractmethod
        def endMotion(self):
            pass

        @abstractmethod
        def processMotion(self):
            pass

        @abstractmethod
        def motionCompleted(self):
            pass
        
        @abstractmethod
        def testCompleted(self):
            pass

        @abstractmethod
        def processResults(self):
            pass


        def scaleResult(self, value, expectedMin, expectedMax, outMin, outMax):
            if(value < expectedMin or value > expectedMax):
                return None
            retValue = ((outMax - outMin)*(value - expectedMin)) / (expectedMax - expectedMin) + outMin
            if(retValue > outMax or retValue < outMin):
                raise ValueError('A very specific bad thing happened.')
            return retValue

        def scaleHundred(self, value, expectedMin, expectedMax):
            return scaleResult(value,expectedMin,expectedMax,0.0,100.0)


    #Class to calculate the noise and latency from the vision system
    class VisionTest(MotionTest):
        pass
        #Note, add angular noise?

    #Test that makes the robot navigate a virtual field with virtual obstacles
    class ObstacleMotionTest(MotionTest):
        pass


    #Tests how a robot captures an inbound or stationary ball
    class InboundBallTest(MotionTest):
        pass


    #Test that causes the robot to move in triangular motions
    class BasicMotionTest(MotionTest):

        #Test information - 

        started = False
     
        points = []
        distances = []
        facePoints = []
        
        timeTaken = None
        posEndError = None
        lineFollowError = None
        rotationalFollowError = None
        finalRotationalError = None
        maxOvershoot = None
        maxVel = None
        totalVel = None

        maximumSpeed = -1
        maximumAcc = -1
        lastSpeed = -1

        motionNumber = -1


        currentStart = None
        currentEnd = None
        currentFacePoint = None

        positions = 0
        motions = 0 

        #Sets up the test to be run
        def setupTest(self):
            self.positions = max(len(self.points),len(self.facePoints))
            self.motions = self.runs * self.positions
            self.timeTaken = [0.0] * self.motions
            self.posEndError = [0.0] * self.motions
            self.lineFollowError = [0.0] * self.motions
            self.rotationalFollowError = [0.0] * self.motions
            self.finalRotationalError = [0.0] * self.motions
            self.maxOvershoot = [0.0] * self.motions
            self.endVel = [0.0] * self.motions
            self.totalVel = [0.0] * self.motions
            self.motionNumber = -1

        startIndex = 0
        endIndex = 1
        faceIndex = 0

        #Sets up the next motion
        def startMotion(self):
            if (self.motionNumber is -1):
                self.currentStart = self.points[0]
                self.currentEnd = self.points[0]
                self.currentFacePoint = None
                self.theMotionBenchmark.robot.face_none()
            else:
                if(self.startIndex >= len(self.points)):
                    self.startIndex = 0
                if(self.endIndex >= len(self.points)):
                    self.endIndex = 0
                if(self.faceIndex >= len(self.facePoints)):
                    self.faceIndex = 0
                if(len(self.facePoints) == 0):
                    self.theMotionBenchmark.robot.face_none()
                    self.faceIndex = -1

                self.currentStart = self.points[self.startIndex]
                self.currentEnd = self.points[self.endIndex]
                self.currentFacePoint = self.facePoints[self.faceIndex]
                self.startIndex += 1
                self.endIndex += 1
                self.faceIndex += 1

            self.motionStartTime = time.time()
            self.timeSinceLastCalc = time.time()


       
        #Calls all the functions that need to be called every frame
        def processMotion(self):
            deltat = abs(self.timeSinceLastCalc - time.time())
            self.timeSinceLastCalc = time.time()
            if(self.currentEnd is not None and self.currentStart is not None):
                self.integrateLineError(deltat)
                self.updateOvershoot()
            if(self.currentFacePoint != None):
                self.integrateRotationalError(deltat)
        
            self.updateVel(deltat)


        def updateVel(self, deltat):
            vel = MotionBenchmark.getVel(self.theMotionBenchmark)
            speed = MotionBenchmark.getSpeed(self.theMotionBenchmark)

            if speed > self.maximumSpeed:
                maximumSpeed = speed
           
            self.totalVel[self.motionNumber] += speed * deltat

            accl = abs(self.lastSpeed - speed) / deltat

            self.lastSpeed = speed


        def endMotion(self):
            self.timeTaken[self.motionNumber] = abs(self.motionStartTime - time.time())
            self.endVel[self.motionNumber] = MotionBenchmark.getSpeed(self.theMotionBenchmark)
            self.calcFinalRotationError()
            self.calcFinalPosError()
            if(self.started):
                self.motionNumber = self.motionNumber + 1
            else:
                self.started = True

            if (len(self.facePoints) is 0):
                self.theMotionBenchmark.robot.face_none()
                
        def calcFinalRotationError(self):
            if(self.currentFacePoint is not None):
                self.finalRotationalError[self.motionNumber] = MotionBenchmark.getAngleError(self.theMotionBenchmark, self.currentFacePoint)

        def calcFinalPosError(self):
            if(self.currentEnd is not None):
                self.posEndError[self.motionNumber] = MotionBenchmark.getPosError(self.theMotionBenchmark, self.currentEnd)

        def integrateLineError(self, deltat):
            self.lineFollowError[self.motionNumber] += MotionBenchmark.getLineError(self.theMotionBenchmark,self.currentStart, self.currentEnd) * deltat

        def integrateRotationalError(self, deltat):
            self.rotationalFollowError[self.motionNumber] += abs(MotionBenchmark.getAngleError(self.theMotionBenchmark,self.currentFacePoint)) * deltat

        def updateOvershoot(self):
            overshoot = MotionBenchmark.getOvershoot(self.theMotionBenchmark,self.currentStart, self.currentEnd)
            if(overshoot > self.maxOvershoot[self.motionNumber]):
                self.maxOvershoot[self.motionNumber] = overshoot
        
        def motionCompleted(self, motionBenchmark):
            return True

        def testCompleted(self):
            if (self.motionNumber >= self.motions):
                return True
            else:
                return False
        
   
        def processResults(self):

            timeTaken = self.timeTaken
            self.sResults['avgMotionTime'] = sum(self.timeTaken) / len(self.timeTaken)
            self.sResults['motionTimeVar'] = statistics.pvariance(self.timeTaken)
            self.sResults['avgEndPosError'] = sum(self.posEndError) / len(self.posEndError)
            self.sResults['maxEndPosError'] = max(self.posEndError)
            self.sResults['lineError'] = sum(self.lineFollowError) / len(self.lineFollowError)
            self.sResults['unitLineError'] = list(map(truediv, self.lineFollowError, self.timeTaken))
            self.sResults['lineErrorPerTime'] = sum(unitLineError) / len(unitLineError)
            rotationalError = sum(self.rotationalFollowError) / len(self.rotationalFollowError)
            unitRotError = list(map(truediv, self.rotationalFollowError, self.timeTaken))
            rotErrorPerTime = sum(unitRotError) / len(unitRotError)
           
            distances = [g.dist0, g.dist1, g.dist2]

            overshoot = self.maxOvershoot
            perOvershoot = []
            for i in range(0, len(self.maxOvershoot)):
                perOvershoot.append(self.maxOvershoot[i] / distances[i % 3])


            avgAbsOvershoot = sum(overshoot) / len(overshoot)
            avgPerOvershoot = sum(perOvershoot) / len(perOvershoot)
            maxAbsOvershoot = max(overshoot)
            maxPerOvershoot = max(perOvershoot)

            avgMotionVelocity = []
            for i in range(0, len(self.timeTaken) - 1): 
                avgMotionVelocity.append(distances[i % 3] / self.timeTaken[i])
           
            calcVels = []
            for i in range(0, len(g.totalVel) - 1):
                calcVels.append(self.totalVel[i] / self.timeTaken[i])

            avgCalcVelocity = sum(calcVels) / len(calcVels)
            maxCalcVelocity = max(calcVels)
            varCalcVelocity = statistics.variance(calcVels)

            avgTestVelocity = sum(avgMotionVelocity) / len(avgMotionVelocity)
            maxTestVelocity = max(avgMotionVelocity)

            self.sResults['title'] = self.title



            paramList = []




            if self.sParams.get('avgMotionTime',False): 
                self.sResults['avgMotionTime'] = avgMotionTime
                self.sScore['avgMotionTime'] = self.scaleHundred(avgMotionTime,self.sParams.get('avgMotionTimeBest'),self.sParams.get('avgMotionTimeWorst'))

            if self.sParams.get('motionTimeVar',False): 
                self.sResults['motionTimeVar'] = motionTimeVar
                self.sScore['motionTimeVar'] = self.scaleHundred(motionTimeVar,self.sParams.get('motionTimeVarBest'),self.sParams.get('motionTimeVarWorst'))

            if self.sParams.get('avgEndPosError',False): 
                self.sResults['avgEndPosError'] = motionTimeVar
                self.sScore['avgEndPosError'] = self.scaleHundred(avgEndPosError,self.sParams.get('avgEndPosErrorBest'),self.sParams.get('avgEndPosErrorWorst'))

            if self.sParams.get('avgEndPosError',False): 
                self.sResults['avgEndPosError'] = motionTimeVar
                self.sScore['avgEndPosError'] = self.scaleHundred(motionTimeVar,self.sParams.get('avgEndPosErrorBest'),self.sParams.get('avgEndPosErrorWorst'))





    basicMotionTests = []
    basicMotionIndex = 0
    currentBasicMotion = None


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
                     MotionBenchmark.State.ProcessBasicMotion,
                     MotionBenchmark.State.BasicMotionBuffer]
                    

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
        
        #BasicMotion0 -> BasicMotionBuffer
        self.add_transition(MotionBenchmark.State.BasicMotion0,
                            MotionBenchmark.State.BasicMotionBuffer,
                            lambda: self.all_subbehaviors_completed() and not self.currentBasicMotion.testCompleted(), 'In Position')
       
        #BasicMotionBuffer -> BasicMotion0
        self.add_transition(MotionBenchmark.State.BasicMotionBuffer,
                            MotionBenchmark.State.BasicMotion0,
                            lambda: True, 'In Position')

        #BasicMid0 -> BasicMotionEnd
        self.add_transition(MotionBenchmark.State.BasicMotion0,
                            MotionBenchmark.State.BasicMotionEnd,
                            lambda: self.all_subbehaviors_completed() and self.currentBasicMotion.testCompleted(), 'In Position')

        #BasicMotionEnd -> BasicMotion0
        self.add_transition(MotionBenchmark.State.BasicMotionEnd,
                            MotionBenchmark.State.BasicMotion0,
                            lambda: self.basicMotionIndex < len(self.basicMotionTests) - 1, 'In Position')


        #BasicMotionEnd -> ProcessBasicMotion
        self.add_transition(MotionBenchmark.State.BasicMotionEnd,
                            MotionBenchmark.State.ProcessBasicMotion,
                            lambda: self.basicMotionIndex >= len(self.basicMotionTests) - 1, 'In Position')
                            
        #ProcessBasicMotion -> exit
        self.add_transition(MotionBenchmark.State.ProcessBasicMotion,
                            behavior.Behavior.State.completed,
                            lambda: False, 'In Position') #Should change this to true if I want it to end

        #Setup the BasicMotionTests

    #Movement test points START
    setupPoint = robocup.Point(0, 1.25)


    #PureRot
    startPoint = robocup.Point(0,1.5)
    PureRot0FacePoint = robocup.Point(0, 2.5)
    PureRot1FacePoint = robocup.Point(-1,1.5)
    PureRot2FacePoint = robocup.Point(1,1.5)

    #Movement test points END


    #Utility functions START

    #A function that both prints and adds to the output file list to be written
    def resultOut(self, result):
        print(result)
        self.resultsToWrite.append(result)

    def fileOnly(self, result):
        self.resultsToWrite.append(result)

    def checkSub(self):
        print(self.subbehaviors_by_name)


    def setupBasicMotionTests(self):
        self.basicMotionTests = []
        numberOfRuns = 3
        print(self.basicMotionTests)
    
        superBasicTest = self.BasicMotionTest(numberOfRuns, self)
        superBasicTest.points.append(robocup.Point(1.2,1.2))
        superBasicTest.points.append(robocup.Point(1.2,2.2))
        superBasicTest.points.append(robocup.Point(2.2,1.2))
        superBasicTest.title = "Test Triangle"
        #self.basicMotionTests.append(superBasicTest)

        basicMid = self.BasicMotionTest(numberOfRuns, self)
        basicMid.points.append(robocup.Point(1.2,1.2))
        basicMid.points.append(robocup.Point(-1.2,1.2))
        basicMid.points.append(robocup.Point(0,3.5))
        basicMid.title = "Mid Size Motion Triangle"
        self.basicMotionTests.append(basicMid)

        basicSmall = self.BasicMotionTest(numberOfRuns, self)
        basicSmall.points.append(robocup.Point(-0.75, 1.2))
        basicSmall.points.append(robocup.Point(0.75, 1.2))
        basicSmall.points.append(robocup.Point(0,2.4))
        basicSmall.title = "Small Motion Triangle"
        self.basicMotionTests.append(basicSmall)

        basicLarge = self.BasicMotionTest(numberOfRuns, self)
        basicLarge.points.append(robocup.Point(-1.7,1.5))
        basicLarge.points.append(robocup.Point(0, 4.8))
        basicLarge.points.append(robocup.Point(1.7, 1.5))
        basicLarge.title = "Large Motion Triangle"
        self.basicMotionTests.append(basicLarge)

        basicSmaller = self.BasicMotionTest(numberOfRuns, self)
        basicSmaller.points.append(robocup.Point(0.25, 1.2))
        basicSmaller.points.append(robocup.Point(-0.25, 1.2))
        basicSmaller.points.append(robocup.Point(0, 1.7))
        basicSmaller.title = "Smaller Motion Triangle"
        self.basicMotionTests.append(basicSmaller)
        
        basicTiny = self.BasicMotionTest(numberOfRuns, self)
        basicTiny.points.append(robocup.Point(0.085, 1.2))
        basicTiny.points.append(robocup.Point(-0.085, 1.2))
        basicTiny.points.append(robocup.Point(0, 1.285))
        basicTiny.title = "Tiny Motion Triangle"
        self.basicMotionTests.append(basicTiny)

        basicMicro = self.BasicMotionTest(numberOfRuns, self)
        basicMicro.points.append(robocup.Point(0.034, 1.2))
        basicMicro.points.append(robocup.Point(-0.034, 1.2))
        basicMicro.points.append(robocup.Point(0,1.242))
        basicMicro.title = "Micro Motion Triangle"
        self.basicMotionTests.append(basicMicro)
        
        midFace = self.BasicMotionTest(numberOfRuns, self)
        midFace.points.append(robocup.Point(1.2,1.2))
        midFace.points.append(robocup.Point(-1.2,1.2))
        midFace.points.append(robocup.Point(0,3.5))
        midFace.facePoints.append(robocup.Point(0,2.8))
        midFace.facePoints.append(robocup.Point(0,0))
        midFace.facePoints.append(robocup.Point(2,0))
        midFace.title = "Mid size triangle while facing points #1"
        self.basicMotionTests.append(midFace)
        
        midFace2 = self.BasicMotionTest(numberOfRuns, self)
        midFace2.points.append(robocup.Point(1.2,1.2))
        midFace2.points.append(robocup.Point(-1.2,1.2))
        midFace2.points.append(robocup.Point(0,3.5))
        midFace2.facePoints.append(robocup.Point(0,2.8))
        midFace2.facePoints.append(robocup.Point(0,0))
        midFace2.facePoints.append(robocup.Point(2,0))
        midFace2.title = "Mid size triangle while facing points #2"
        self.basicMotionTests.append(midFace)

        pureRot = self.BasicMotionTest(numberOfRuns, self)
        pureRot.facePoints.append(robocup.Point(0,2.8))
        pureRot.facePoints.append(robocup.Point(0,0))
        pureRot.facePoints.append(robocup.Point(2,0))
        pureRot.title = "Pure Rotational Test"
        #self.basicMotionTests.append(midFace)
        
        self.basicMotionIndex = 0
        self.currentBasicMotion = self.basicMotionTests[self.basicMotionIndex]


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


    def getVel(self):
        return self.robot.vel

    def getSpeed(self):
        a = self.getVel()
        return math.sqrt(a.x**2 + a.y**2)


    def getAngleError(self, point):
        targetAngle = self.robot.angle
        betweenVec = self.robot.pos - point
        currentAngle = math.atan2(betweenVec.x, betweenVec.y)
        return targetAngle - currentAngle

    def getPosError(self, point):
        xErr = self.robot.pos.x - point.x
        yErr = self.robot.pos.y - point.y
        return math.sqrt(xErr**2 + yErr**2)

    def getLineError(self,start, end):
        startNumpy=np.array([start.x,start.y])
        endNumpy=np.array([end.x,end.y])
        robotNumpy=np.array([self.robot.pos.x,self.robot.pos.y])
        d = np.cross(startNumpy-endNumpy,endNumpy-robotNumpy)/np.linalg.norm(endNumpy-startNumpy)
        return abs(d)
 

    def getOvershoot(self, start, end):
       distToStart = math.sqrt((self.robot.pos.x - start.x)**2 + (self.robot.pos.y - start.y)**2)
       startToEnd = math.sqrt((start.x - end.x)**2 + (start.y - end.y)**2)
       overshoot = distToStart - startToEnd
       if(overshoot <= 0):
           return 0
       else:
           return overshoot

    #Setup state functions (for the latency test)

    def on_enter_setup(self):
        #self.remove_all_subbehaviors()
        move_point = self.setupPoint
        self.add_subbehavior(skills.move.Move(move_point), 'move') 

        self.setupBasicMotionTests()

    
        print("TESTS TO BE RUN --------------------------------")
        for g in self.basicMotionTests:
            print(g)
        print("TESTS TO BE RUN --------------------------------")

    def on_exit_setup(self):
        self.remove_all_subbehaviors()
        #pass

    #End setup state functions



    #Noise state functions

    def on_enter_noise(self):
        self.noiseStartTime = time.time()
        self.noiseStartPos = self.robot.pos

    def execute_noise(self):
        #print(self.noiseStartTime - time.time())
        #print(self.noiseMeasured)
        if(abs(self.noiseStartTime - time.time()) >=  5):
            self.noiseMeasured = True
            self.noiseStartTime = time.time()
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
        self.remove_all_subbehaviors()
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
        self.currentBasicMotion.setupTest()


    def on_enter_BasicMotion0(self):
        self.currentBasicMotion.startMotion()
        if (self.currentBasicMotion.currentEnd is not None):
            self.add_subbehavior(skills.move.Move(self.currentBasicMotion.currentEnd), 'move')
        else:
            print("There is no current end")

        if (self.currentBasicMotion.currentFacePoint is not None):
            self.robot.face(self.currentBasicMotion.currentFacePoint)

    def execute_BasicMotion0(self):
        if (self.currentBasicMotion.currentFacePoint is not None):
            self.robot.face(self.currentBasicMotion.currentFacePoint)
        self.currentBasicMotion.processMotion()

    def on_exit_BasicMotion0(self):
        self.currentBasicMotion.endMotion()
        self.remove_all_subbehaviors()

    def on_enter_BasicMotionEnd(self):
        pass

    def on_exit_BasicMotionEnd(self):
        self.basicMotionIndex += 1
        if(self.basicMotionIndex < len(self.basicMotionTests)):
            self.currentBasicMotion = self.basicMotionTests[self.basicMotionIndex]
            self.currentBasicMotion.setupTest()
        else:
            self.currentBasicMotion = None

    #ProcessBasicMotion
    def on_enter_ProcessBasicMotion(self):
        print("Should be processing basic motion")

        '''

            self.resultOut("\nResults for test " + theTitle + " - - - - - - - - - - - - - - - \n")
            self.resultOut("Average Motion Time: " + str(avgMotionTime))
            self.resultOut("Motion Time Variance: " + str(motionTimeVar))
            self.resultOut("Average Positional Error: " + str(avgEndPosError))
            self.resultOut("Maximum Positional Error: " + str(maxEndPosError))
            self.resultOut("Positional Error Score (0-100): " + str(self.scaleResult(avgEndPosError, 0.0, 0.15, 0, 100 )))
            self.resultOut("Line Follow Error: " +  str(lineError))
            self.resultOut("Line Follow Error per Time" + str(lineErrorPerTime))
            self.resultOut("Rotational Error: " + str(rotationalError))
            self.resultOut("Rotational Error per Time: " + str(rotErrorPerTime))
            self.resultOut("Average Absolute Overshoot: " + str(avgAbsOvershoot))
            self.resultOut("Average Perportional Overshoot: " + str(avgPerOvershoot))
            self.resultOut("Max Absolute Overshoot: " + str(maxAbsOvershoot))
            self.resultOut("Max Perportional Overshoot: " + str(maxPerOvershoot))
            self.resultOut("Average Test Velocity: " + str(avgTestVelocity))
            self.resultOut("Max Test Velocity: " + str(maxTestVelocity))
            self.resultOut("\nEnd of results - - - - - - - - - - - - - - - - - - - - - - - - - \n")
            '''
    def role_requirements(self):

        '''reqs = role_assignment.RoleRequirements()
        if self.robot is not None:
            reqs.previous_shell_id = self.robot.shell_id()
        return reqs'''

        #b = single_robot_behavior.SingleRobotBehavior.role_requirements(self)
        c = super().role_requirements()

        if isinstance(c, role_assignment.RoleRequirements):
            if(c.previous_shell_id is not None):
                c.required_shell_id = c.previous_shell_id
            else:
                print("We have gotten a role requirements object with no previous shell id")
        else:
            if(c['move'].previous_shell_id is not None):
                c['move'].required_shell_id = c['move'].previous_shell_id
            else:
                print("Its a dict")

        return c


        #This is what was in the role requiremnts before, what I have now works but is not ideal
        
        '''
        a = role_assignment.RoleRequirements()
        print("--------------------------ROLE_REQUIREMENTS_THING--------------------")
        print(a)
        print(b)
        print(c)
        print(type(c))

        
        reqs = composite_behavior.CompositeBehavior.role_requirements(self)
        print(type(reqs))
        print(reqs)
        return reqs
        if(type(c) is not dict and c.previous_shell_id is not None):
            c.required_shell_id = c.previous_shell_id
        else:
            print(c)
            print(type(c))
            print(c['move'])
        #print("---------------------------------------------------------------------")
        return c

        
        #So this returns a dict instead of a role requirements object, idk
        reqs = composite_behavior.CompositeBehavior.role_requirements(self) 
        if self.robot is not None:
            for req in role_assignment.iterate_role_requirements_tree_leaves(reqs):
                    req.previous_shell_id = self.robot.shell_id()
            return reqs

        else:
            reqs = super().role_requirements()
            #reqs = role_assignment.RoleRequirements()
            #reqs.destination_shape = robocup.Point(0,0)
            return reqs
        '''

