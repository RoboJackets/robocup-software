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


    #An list that stores the tests to be run (Subclasses of MotionTest)
    tests = []
    #The index of the currently running Test
    testIndex = 0
    #The currently running test
    currentTest = None

    #Variable to control leaving the setup state, exists so the the play can setup the tests to be run
    isSetup = False

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
                            lambda: self.isSetup, 'Ready to start running')

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
                            lambda: self.testIndex >= len(self.tests) or self.currentTest == None, 'No next test exists')
                            
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


    
    ##
    # Appends a test to the list of tests
    def addTest(self, a):
        self.tests.append(a)


    #A function that both prints and adds to the output file list to be written
    def resultOut(self, result):
        print(result)
        self.resultsToWrite.append(result)

    def fileOnly(self, result):
        self.resultsToWrite.append(result)

    def checkSub(self):
        print(self.subbehaviors_by_name)

    ##
    # This function used to do something
    #
    def setupTests(self):
       pass 

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
        self.setupTests()
   
    def on_exit_setup(self):
        self.currentTest = self.tests[0] 
        print("TESTS TO BE RUN --------------------------------")
        for g in self.tests:
            print(g)
        print("TESTS TO BE RUN --------------------------------")

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


    def done_with_setup(self):
        self.isSetup = True

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
    def on_enter_ProcessAllTests(self):
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

