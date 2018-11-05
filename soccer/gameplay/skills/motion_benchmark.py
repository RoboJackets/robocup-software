import single_robot_composite_behavior
import behavior
from enum import Enum
import main
import constants
import robocup
import skills.move
import time
import datetime



## Motion Benchmark V0.0.0.0
#
# A skill for testing the capabilities of our robots motion control
# 
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
        BasicMotion1 = 5
        BasicMotion2 = 6
        BasicMotionEnd = 7

        #Pure Rotations
        PureRot0 = 8
        PureRot1 = 9
        PureRot2 = 10
        PureRotEnd = 11


        #Medium Triangle while facing
        MidFace0 = 12
        MidFace1 = 13
        MidFace2 = 14
        MidFaceEnd = 15

        #Small movements with a specified end orientation
        SmallRot0 = 16
        SmallRot1 = 17
        SmallRot2 = 18
        SmallRotEnd = 19

        #Very small movementes with a specified orientation
        MicroRot0 = 20
        MicroRot1 = 21
        MicroRot2 = 22
        MicroRotEnd = 23
        
        EndAll = 24


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
        timer = 0.0

        point0 = None
        point1 = None
        point2 = None
        
        facePoint0 = None
        facePoint1 = None
        facePoint2 = None
        
        timeTaken = []
        posEndError = []
        lineFollowError = []
        rotationalFollowError = []
        finalRotationalError = []

        count = 0
        runs = 5

        currentStart = None
        currentEnd = None
        currentFacePoint

        def startRun(self, startPoint, endPoint, facePoint):
            timer = time.time()
            currentStart = startPoint
            currentEnd = endPoint
            currentFacePoint = facePoint
            
            
        def endRun(self):
            timeTaken.append(abs(timer - time.time()))



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
                     MotionBenchmark.State.BasicMotion1,
                     MotionBenchmark.State.BasicMotion2,
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

        #Move1 -> BasicMid0
        self.add_transition(MotionBenchmark.State.move1,
                            MotionBenchmark.State.BasicMotion0,
                            lambda: self.all_subbehaviors_completed(), 'In Position')
        
        #BasicMid0 -> BasicMid1
        self.add_transition(MotionBenchmark.State.BasicMotion0,
                            MotionBenchmark.State.BasicMotion1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #BasicMid1 -> BasicMid2
        self.add_transition(MotionBenchmark.State.BasicMotion1,
                            MotionBenchmark.State.BasicMotion2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')


        #BasicMid2 -> BasicMid0
        self.add_transition(MotionBenchmark.State.BasicMotion2,
                            MotionBenchmark.State.BasicMotion0,
                            lambda: self.all_subbehaviors_completed() and BasicMidCount < BasicMidLoops, 'In Position')

        #BasicMid2 -> BasicMidEnd
        self.add_transition(MotionBenchmark.State.BasicMotion2,
                            MotionBenchmark.State.BasicMotionEnd,
                            lambda: self.all_subbehaviors_completed() and BasicMidCount >= BasicMidLoops, 'In Position')


        #END TRANSITIONS









    #Utility functions START

    #A function that both prints and adds to the output file list to be written
    def resultOut(self, result):
        print(result)
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


    #Utility functions END



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







    #nothing special for role requirements
    def role_requirements(self):
        reqs = super().role_requirements()
        return reqs
