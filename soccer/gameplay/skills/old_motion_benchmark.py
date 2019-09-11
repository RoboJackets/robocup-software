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
#import numpy as np
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
        #setup
        setup = 1 #Setup state
        TestMotion = 2 #Perform a test motion
        TestEnd = 3 #The end of a test
        ProcessAllTests = 4 #a state in which we process all test results
        TestBuffer = 5

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

        def __init__(self, nRuns, benchmark):
            self.runs = nRuns
            self.theMotionBenchmark = benchmark
            self.title = "No Name Test"

            self.motionPause = 0.5

            self.sParams = dict() #Scoring parameters
            self.sResults = dict() #Raw results
            self.sScore = dict() #Actual scores

            #Is used to determine if the test has actually
            #started, to exclude setup motions from results.
            self.started = False

            #This holds a refrence to the calling class
            #self.theMotionBenchmark = None
          
            #The time at which the motion pause started
            self.motionPauseStart = 0.0

            #The time at which the current motion started
            self.motionStartTime = 0.0

            #The time at which the last iterative calculation was made
            self.timeOfLastCalc = 0.0

            #The number of test runs remaining
            #self.runs = 0


        #Just a distance calculation
        def calcDist(self, point1, point2):
            if(point1 is not None and point2 is not None):
                return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
            else:
                return 0

        #Title for prints
        def __str__(self):
            return self.title

        def startMotionPause(self):
            self.motionPauseStart = time.time()

        def isMotionPause(self):
            if(abs(time.time() - self.motionPauseStart) > self.motionPause):
                return True
            else:
                return False
    
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

        #Scales a test result into the desired range
        def scaleResult(self, value, expectedMin, expectedMax, outMin, outMax):
            if(value < expectedMin or value > expectedMax):
                return None
            retValue = ((outMax - outMin)*(value - expectedMin)) / (expectedMax - expectedMin) + outMin
            if(retValue > outMax or retValue < outMin):
                raise ValueError('A very specific bad thing happened.')
            return retValue

        #Scales a test result into a 0 to 100 range
        def scaleHundred(self, value, expectedMin, expectedMax):
            return scaleResult(value,expectedMin,expectedMax,0.0,100.0)


    #Tests with continuous move commands, often incountered in defence
    class ContinuousIssueMotionTest(MotionTest):
        pass

    #Test of pivoting around the ball
    #May use Line kick or move, I'm not sure yet
    class BallProximityTest(MotionTest):
        pass

    #Test that tests the vision system and radio for performance
    class VisionTest(MotionTest):

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

        #Note, add angular noise?


    #Test that makes the robot navigate virtual obstacles
    class ObstacleMotionTest(MotionTest):
        pass

    #Tests how a robot captures an inbound or stationary ball using a simulated one
    class InboundBallTest(MotionTest):
        pass

    #Test that causes the robot to move in triangular motions,
    #recording movement characteristics
    class BasicMotionTest(MotionTest):

        #Test information - 
 

        def __init__(self, nRuns, benchmark):
            super().__init__(nRuns, benchmark)
            
            self.points = [] #The points to move to
            self.distances = [] #The distances of each motion
            self.facePoints = [] #The points to face while making motions
          
            #An array of the time taken for each motion
            self.timeTaken = None

            #An array of the ending positionl errors
            self.posEndError = None

            #An array of the approximate integrals of the error from a perfect line
            self.lineFollowError = None

            #An array of the approximate integrals of error from the desired face
            self.rotationalFollowError = None

            #An array of the final Rotational Error
            self.finalRotationalError = None

            #An array of the largest observed overshoot on each motion
            self.maxOvershoot = None

            #An array of the highest observed velocity on each motion
            self.maxVel = None

            #An array of the integral of the velocity on each motion
            #Note: I can't quite remember what this is for
            self.totalVel = None

            self.maximumSpeed = -1 #Stores the current max speed
            self.maximumAcc = -1 #Stores the current mac accleeration
            self.lastSpeed = -1 #Stores the last speed (probably a bad method)


            #The index of the currently running motion 
            self.motionNumber = -1 


            self.currentStart = None #The start point for the current motion
            self.currentEnd = None #The end point for the current motion
            self.currentFacePoint = None #The face point for the current motion

            self.positions = 0 #The number of positions that are in the motion cycle
            self.motions = 0 #The total number of motions that the robot is going to make

            self.isSetup = False #If setupTest has been run

            self.startIndex = 0 #The index of the current start point
            self.endIndex = 1 #The index of the current end point
            self.faceIndex = 0 #The index of the current face point


        #Sets up the test to be run
        #Largly initalizes arrays to arrays of zeros
        def setupTest(self):
            print("Setting up test: " + self.title)
            #if(len(self.points) == 0 or len(self.facePoints) == 1 or len(self.points) == len(self.facePoints)):
            #    raise RuntimeError("Invalid number of face points")
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
            #print("This test has: " + str(self.runs) + " runs")
            #print("This test has: " + str(self.positions) + " positions")
            #print(self.points)
            #print(self.facePoints)
            #print("So this test will have: " + str(self.motions) + "total motions in it")

       
        #Sets up the next motion to be done
        def startMotion(self):
            if(not self.isSetup):
                self.setupTest()
                self.isSetup = True

            if(len(self.points) == 0):
                self.points = [None]

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
                
                if(len(self.points) != 0):
                    self.currentStart = self.points[self.startIndex]
                    self.currentEnd = self.points[self.endIndex]
                else:
                    self.currentStart = None
                    self.currentEnd = None

                if(len(self.facePoints) != 0):
                    self.currentFacePoint = self.facePoints[self.faceIndex]
                else:
                    self.currentFacePoint = None
                self.startIndex += 1
                self.endIndex += 1
                self.faceIndex += 1

            self.motionStartTime = time.time()
            self.timeOfLastCalc = time.time()


        def updateDistances():
            self.distances = []
            for i in range(0,len(self.points)):
                if(i < len(self.points)):
                   self.distances.append(math.sqrt((self.points[i][0] - self.points[i + 1])**2 + (self.points[i][1] - self.points[i + 1][1])**2)) 
                else:
                   self.distances.append(math.sqrt((self.points[0][0] - self.points[i])**2 + (self.points[0][1] - self.points[i][1])**2))

        #Calls all the functions that need to be called every frame
        def processMotion(self):
            deltat = abs(self.timeOfLastCalc - time.time())
            self.timeOfLastCalc = time.time()
            if(self.currentEnd is not None and self.currentStart is not None):
                self.integrateLineError(deltat)
                self.updateOvershoot()
            if(self.currentFacePoint != None):
                self.integrateRotationalError(deltat)
        
            self.updateVel(deltat)

        #Update the velocity related stats
        def updateVel(self, deltat):
            vel = MotionBenchmark.getVel(self.theMotionBenchmark)
            speed = MotionBenchmark.getSpeed(self.theMotionBenchmark)

            if speed > self.maximumSpeed:
                maximumSpeed = speed
           
            self.totalVel[self.motionNumber] += speed * deltat

            accl = abs(self.lastSpeed - speed) / deltat

            self.lastSpeed = speed

        #Records the data at the end of the motion
        def endMotion(self):
            print("Motion " + str(self.motionNumber) + " out of " + str(self.motions) + " in test " + self.title + " results: ")
            self.timeTaken[self.motionNumber] = abs(self.motionStartTime - time.time())
            print("Time taken: " + str(self.timeTaken[self.motionNumber]) + " seconds")
            self.endVel[self.motionNumber] = MotionBenchmark.getSpeed(self.theMotionBenchmark)
            print("Ending velocity: " + str(self.endVel[self.motionNumber]) + " m/s")
            self.calcFinalRotationError()
            self.calcFinalPosError()
            print("Overshoot: " + str(self.maxOvershoot[self.motionNumber]) + " meters")

            if(self.started):
                self.motionNumber = self.motionNumber + 1
            else:
                self.started = True

            if (len(self.facePoints) is 0):
                self.theMotionBenchmark.robot.face_none()
              
        #Calculates the final rotational error based on self.currentFacePoint
        def calcFinalRotationError(self):
            if(self.currentFacePoint is not None):
                self.finalRotationalError[self.motionNumber] = MotionBenchmark.getAngleError(self.theMotionBenchmark, self.currentFacePoint)
                print("End rotational error: " + str(self.finalRotationalError[self.motionNumber]) + " degrees")

        #Calculates the final positional error based 
        def calcFinalPosError(self):
            if(self.currentEnd is not None):
                self.posEndError[self.motionNumber] = MotionBenchmark.getPosError(self.theMotionBenchmark, self.currentEnd)
                print("End positional error: " + str(self.posEndError[self.motionNumber]) + " meters")

        #Update the line follow error integral
        def integrateLineError(self, deltat):
            self.lineFollowError[self.motionNumber] += MotionBenchmark.getLineError(self.theMotionBenchmark,self.currentStart, self.currentEnd) * deltat

        #Update the rotation error integral
        def integrateRotationalError(self, deltat):
            self.rotationalFollowError[self.motionNumber] += abs(MotionBenchmark.getAngleError(self.theMotionBenchmark,self.currentFacePoint)) * deltat

        #Check and update the overshoot stats
        def updateOvershoot(self):
            overshoot = MotionBenchmark.getOvershoot(self.theMotionBenchmark,self.currentStart, self.currentEnd)
            if(overshoot > self.maxOvershoot[self.motionNumber]):
                self.maxOvershoot[self.motionNumber] = overshoot
       
        #Returns true if the current motion is completed
        #Will be used mostly for pure rotation as it does not have an end
        def motionCompleted(self):
            return self.theMotionBenchmark.all_subbehaviors_completed()

        #Returns true when the entire test has been completed, false otherwise
        def testCompleted(self):
            if (self.motionNumber >= self.motions - 1):
                print("Test " + self.title + " Completed")
                return True
            else:
                return False
        
        #A function that processes the results and stores them to the relavant maps 
        def processResults(self):
            timeTaken = self.timeTaken
            avgMotionTime = sum(self.timeTaken) / len(self.timeTaken)
            motionTimeVar = statistics.pvariance(self.timeTaken)
            avgEndPosError = sum(self.posEndError) / len(self.posEndError)
            maxEndPosError = max(self.posEndError)
            avgLineError = sum(self.lineFollowError) / len(self.lineFollowError)
            unitLineError = list(map(truediv, self.lineFollowError, self.timeTaken))
            lineErrorPerTime = sum(unitLineError) / len(unitLineError)
            rotationalError = sum(self.rotationalFollowError) / len(self.rotationalFollowError)
            unitRotError = list(map(truediv, self.rotationalFollowError, self.timeTaken))
            rotErrorPerTime = sum(unitRotError) / len(unitRotError)

            overshoot = self.maxOvershoot
            perOvershoot = []
            for i in range(0, len(self.maxOvershoot)):
                perOvershoot.append(self.maxOvershoot[i] / distances[i % len(self.distances)])


            avgAbsOvershoot = sum(overshoot) / len(overshoot)
            avgPerOvershoot = sum(perOvershoot) / len(perOvershoot)
            maxAbsOvershoot = max(overshoot)
            maxPerOvershoot = max(perOvershoot)

            avgMotionVelocity = []
            for i in range(0, len(self.timeTaken) - 1): 
                avgMotionVelocity.append(distances[i % len(distances)] / self.timeTaken[i])
           
            calcVels = []
            for i in range(0, len(g.totalVel) - 1):
                calcVels.append(self.totalVel[i] / self.timeTaken[i])

            avgCalcVelocity = sum(calcVels) / len(calcVels)
            maxCalcVelocity = max(calcVels)
            varCalcVelocity = statistics.variance(calcVels)

            avgTestVelocity = sum(avgMotionVelocity) / len(avgMotionVelocity)
            maxTestVelocity = max(avgMotionVelocity)

            self.sResults['title'] = self.title

            if(self.sParams.get('avgMotionTime',False)): 
                self.sResults['avgMotionTime'] = avgMotionTime
                self.sScore['avgMotionTime'] = self.scaleHundred(avgMotionTime,self.sParams.get('avgMotionTimeBest'),self.sParams.get('avgMotionTimeWorst'))

            if(self.sParams.get('motionTimeVar',False)): 
                self.sResults['motionTimeVar'] = motionTimeVar
                self.sScore['motionTimeVar'] = self.scaleHundred(motionTimeVar,self.sParams.get('motionTimeVarBest'),self.sParams.get('motionTimeVarWorst'))

            if(self.sParams.get('avgEndPosError',False)): 
                self.sResults['avgEndPosError'] = motionTimeVar
                self.sScore['avgEndPosError'] = self.scaleHundred(avgEndPosError,self.sParams.get('avgEndPosErrorBest'),self.sParams.get('avgEndPosErrorWorst'))

            if(self.sParams.get('avgAbsOvershoot',False)): 
                self.sResults['avgAbsOvershoot'] = avgAbsOvershoot
                self.sScore['avgAbsOvershoot'] = self.scaleHundred(avgAbsOvershoot,self.sParams.get('avgAbsOvershootBest'),self.sParams.get('avgAbsOvershootWorst'))

            if(self.sParams.get('avgPerOvershoot',False)): 
                self.sResults['avgPerOvershoot'] = avgAbsOvershoot
                self.sScore['avgPerOvershoot'] = self.scaleHundred(avgAbsOvershoot,self.sParams.get('avgPerOvershootBest'),self.sParams.get('avgPerOvershootWorst'))



    #An list that stores the tests to be run (Subclasses of MotionTest)
    tests = []
    #The index of the currently running Test
    testIndex = 0
    #The currently running test
    currentTest = None

    def __init__(self):
        super().__init__(continuous=False) 

        #The list of states to be registered
        allStates = [MotionBenchmark.State.setup, 
                     MotionBenchmark.State.TestMotion,
                     MotionBenchmark.State.TestEnd,
                     MotionBenchmark.State.ProcessAllTests,
                     MotionBenchmark.State.TestBuffer]
                    

        #Register states in the previously defined list
        for g in allStates: 
            self.add_state(g, behavior.Behavior.State.running)

        #TRANSITIONS 

        #Reset
        self.add_transition(behavior.Behavior.State.start,
                            MotionBenchmark.State.setup, lambda: True,
                            'immediately')

        #Setup -> TestMotion
        self.add_transition(MotionBenchmark.State.setup,
                            MotionBenchmark.State.TestMotion,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #TestMotion -> TestBuffer
        self.add_transition(MotionBenchmark.State.TestMotion,
                            MotionBenchmark.State.TestBuffer,
                            lambda: self.currentTest.motionCompleted() and not self.currentTest.testCompleted(), 'In Position')
       
        #TestBuffer -> TestMotion
        #Note that this transition happens instantly, as the buffer is just to reset the state machine, causing on_enter and on_exit to trigger
        self.add_transition(MotionBenchmark.State.TestBuffer,
                            MotionBenchmark.State.TestMotion,
                            lambda: self.currentTest.isMotionPause(), 'In Position')

        #TestMotion -> TestEnd
        #We have to check both subbehavior completion and testCompleted() because some tests aren't subbehaviors
        self.add_transition(MotionBenchmark.State.TestMotion,
                            MotionBenchmark.State.TestEnd, #I should probably change this to just testCompleted(), its a bit of a hack right now
                            lambda: self.currentTest.testCompleted(), 'In Position')

        #TestEnd -> TestMotion
        self.add_transition(MotionBenchmark.State.TestEnd,
                            MotionBenchmark.State.TestMotion,
                            lambda: self.currentTest != None, 'Next test exists')


        #BasicMotionEnd -> ProcessAllTests
        self.add_transition(MotionBenchmark.State.TestEnd,
                            MotionBenchmark.State.ProcessAllTests,
                            lambda: self.testIndex >= len(self.tests), 'No next test exists')
                            
        #ProcessTest -> exit
        self.add_transition(MotionBenchmark.State.ProcessAllTests,
                            behavior.Behavior.State.completed,
                            lambda: True, 'In Position') #Should change this to true if I want it to end

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
        self.tests = []
        numberOfRuns = 3
        print(self.tests)
    
        superBasicTest = self.BasicMotionTest(numberOfRuns, self)
        superBasicTest.points.append(robocup.Point(1.2,1.2))
        superBasicTest.points.append(robocup.Point(1.2,2.2))
        superBasicTest.points.append(robocup.Point(2.2,1.2))
        superBasicTest.title = "Test Triangle"
        self.tests.append(superBasicTest)

        basicMid = self.BasicMotionTest(numberOfRuns, self)
        basicMid.points.append(robocup.Point(1.2,1.2))
        basicMid.points.append(robocup.Point(-1.2,1.2))
        basicMid.points.append(robocup.Point(0,3.5))
        basicMid.title = "Mid Size Motion Triangle"
        self.tests.append(basicMid)

        basicSmall = self.BasicMotionTest(numberOfRuns, self)
        basicSmall.points.append(robocup.Point(-0.75, 1.2))
        basicSmall.points.append(robocup.Point(0.75, 1.2))
        basicSmall.points.append(robocup.Point(0,2.4))
        basicSmall.title = "Small Motion Triangle"
        self.tests.append(basicSmall)

        basicLarge = self.BasicMotionTest(numberOfRuns, self)
        basicLarge.points.append(robocup.Point(-1.7,1.5))
        basicLarge.points.append(robocup.Point(0, 4.8))
        basicLarge.points.append(robocup.Point(1.7, 1.5))
        basicLarge.title = "Large Motion Triangle"
        self.tests.append(basicLarge)

        basicSmaller = self.BasicMotionTest(numberOfRuns, self)
        basicSmaller.points.append(robocup.Point(0.25, 1.2))
        basicSmaller.points.append(robocup.Point(-0.25, 1.2))
        basicSmaller.points.append(robocup.Point(0, 1.7))
        basicSmaller.title = "Smaller Motion Triangle"
        self.tests.append(basicSmaller)
        
        basicTiny = self.BasicMotionTest(numberOfRuns, self)
        basicTiny.points.append(robocup.Point(0.085, 1.2))
        basicTiny.points.append(robocup.Point(-0.085, 1.2))
        basicTiny.points.append(robocup.Point(0, 1.285))
        basicTiny.title = "Tiny Motion Triangle"
        self.tests.append(basicTiny)

        basicMicro = self.BasicMotionTest(numberOfRuns, self)
        basicMicro.points.append(robocup.Point(0.034, 1.2))
        basicMicro.points.append(robocup.Point(-0.034, 1.2))
        basicMicro.points.append(robocup.Point(0,1.242))
        basicMicro.title = "Micro Motion Triangle"
        self.tests.append(basicMicro)
        
        midFace = self.BasicMotionTest(numberOfRuns, self)
        midFace.points.append(robocup.Point(1.2,1.2))
        midFace.points.append(robocup.Point(-1.2,1.2))
        midFace.points.append(robocup.Point(0,3.5))
        midFace.facePoints.append(robocup.Point(0,2.8))
        midFace.facePoints.append(robocup.Point(0,0))
        midFace.facePoints.append(robocup.Point(2,0))
        midFace.title = "Mid size triangle while facing points #1"
        self.tests.append(midFace)
        
        midFace2 = self.BasicMotionTest(numberOfRuns, self)
        midFace2.points.append(robocup.Point(1.2,1.2))
        midFace2.points.append(robocup.Point(-1.2,1.2))
        midFace2.points.append(robocup.Point(0,3.5))
        midFace2.facePoints.append(robocup.Point(0,2.8))
        midFace2.facePoints.append(robocup.Point(0,0))
        midFace2.facePoints.append(robocup.Point(2,0))
        midFace2.title = "Mid size triangle while facing points #2"
        self.tests.append(midFace2)

        pureRot = self.BasicMotionTest(numberOfRuns, self)
        pureRot.facePoints.append(robocup.Point(0,2.8))
        pureRot.facePoints.append(robocup.Point(0,0))
        pureRot.facePoints.append(robocup.Point(2,0))
        pureRot.title = "Pure Rotational Test"
        self.tests.append(pureRot)
        
        self.testIndex = 0
        self.currentTest = self.tests[self.testIndex]

        #for g in self.tests:
        #    print("The points of " + g.title + ":")
        #    print(g.points)



    def getVel(self):
        return self.robot.vel

    def getSpeed(self):
        a = self.getVel()
        return math.sqrt(a.x**2 + a.y**2)

    def getAngleError(self, point):
        targetAngle = self.robot.angle
        betweenVec = self.robot.pos - point
        currentAngle = math.degrees(math.atan2(betweenVec.x, betweenVec.y))
        return targetAngle - currentAngle

    def getPosError(self, point):
        xErr = self.robot.pos.x - point.x
        yErr = self.robot.pos.y - point.y
        return math.sqrt(xErr**2 + yErr**2)

    def getLineError(self,start, end):
        #startNumpy=np.array([start.x,start.y])
        #endNumpy=np.array([end.x,end.y])
        #robotNumpy=np.array([self.robot.pos.x,self.robot.pos.y])
        #d = np.cross(startNumpy-endNumpy,endNumpy-robotNumpy)/np.linalg.norm(endNumpy-startNumpy)
        d = 0.1
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
        for g in self.tests:
            print(g)
        print("TESTS TO BE RUN --------------------------------")

    def on_exit_setup(self):
        self.remove_all_subbehaviors()
        #pass

    #End setup state functions



    #Noise state functions

   #def on_enter_noise(self):
   #     self.noiseStartTime = time.time()
   #     self.noiseStartPos = self.robot.pos

    '''def execute_noise(self):
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
    '''
    #This functionality needs to be moved into the visionTest class
    '''
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
        self.currentTest.setupTest()

    '''

    def on_enter_TestMotion(self):
        self.currentTest.startMotion()
        if (self.currentTest.currentEnd is not None):
            self.add_subbehavior(skills.move.Move(self.currentTest.currentEnd), 'move')
        else:
            print("There is no current end")

        if (self.currentTest.currentFacePoint is not None):
            self.robot.face(self.currentTest.currentFacePoint)

    def execute_TestMotion(self):
        if (self.currentTest.currentFacePoint is not None):
            self.robot.face(self.currentTest.currentFacePoint)
        self.currentTest.processMotion()

    def on_enter_TestBuffer(self):
        self.currentTest.startMotionPause()

    def on_exit_TestMotion(self):
        self.currentTest.endMotion()
        self.remove_all_subbehaviors()

    def on_enter_TestEnd(self):
        print("Ending test " + self.currentTest.title)
        pass

    def on_exit_TestEnd(self):
        self.testIndex += 1
        if(self.testIndex < len(self.tests)):
            self.currentTest = self.tests[self.testIndex]
            #self.currentTest.setupTest()
        else:
            self.currentTest = None

    #We need to
    def on_enter_ProcessAllTest(self):
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

