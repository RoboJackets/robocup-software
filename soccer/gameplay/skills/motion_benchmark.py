import single_robot_composite_behavior
import behavior
from enum import Enum
import main
import constants
import robocup
import skills.move
import time



## Motion Benchmark V0.0.0.0
class MotionBenchmark(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    class State(Enum):
        #Noise and latency test
        setup = 1
        noise = 2
        move1 = 3
        #Basic motion triangle
        BasicMid0 = 4
        BasicMid1 = 5
        BasicMid2 = 6
        BasicSmall0 = 7
        BasicSmall1 = 8
        BasicSmall2 = 9
        BasicLarge0 = 10
        BasicLarge1 = 11
        BasicLarge2 = 12
        BasicEnd = 13
        #Micro Motions
        Micro0 = 14
        Micro1 = 15
        Micro2 = 16
        #Pure Rotations
        PureRot0 = 17
        PureRot1 = 18
        PureRot2 = 19
        #Medium Triangle while facing
        MidFace0 = 20
        MidFace1 = 21
        MidFace2 = 22
        #Small movements with a specified end orientation
        SmallRot0 = 23
        SmallRot1 = 24
        SmallRot2 = 25
        #Very small movementes with a specified orientation
        MicroRot0 = 26
        MicroRot1 = 27
        MicroRot2 = 28
         
        





        
    #Latency Measurement
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


    #Phase 1: Basic Motion

    MediumTrials = 6
    LargeTrials = 6
    SmallTrials = 6

    startTime = 0.0
    endTime = 0.0

    MediumTimes = []
    LargeTimes = []
    SmallTimes = []

    MaxOvershootMedium = []
    MaxOvershootLarge = []
    MaxOvershootSmall = []    

    MediumDone = False
    LargeDone = False
    SmallDone = False

    
    def __init__(self):
        super().__init__(continuous=False) 


        #Latency Test
        self.add_state(MotionBenchmark.State.setup,
                       behavior.Behavior.State.running)

        self.add_state(MotionBenchmark.State.noise,
                       behavior.Behavior.State.running)

        self.add_state(MotionBenchmark.State.move1,
                       behavior.Behavior.State.running)


        #Mid basic motion triangle

        self.add_state(MotionBenchmark.State.BasicMid0,
                       behavior.Behavior.State.running)

        self.add_state(MotionBenchmark.State.BasicMid1,
                       behavior.Behavior.State.running)

        self.add_state(MotionBenchmark.State.BasicMid2,
                       behavior.Behavior.State.running)

        #Small basic Motion triangle

        self.add_state(MotionBenchmark.State.BasicSmall0,
                       behavior.Behavior.State.running)

        self.add_state(MotionBenchmark.State.BasicSmall1,
                       behavior.Behavior.State.running)

        self.add_state(MotionBenchmark.State.BasicSmall2,
                       behavior.Behavior.State.running)

        #Large basic motion triangle

        self.add_state(MotionBenchmark.State.BasicLarge0,
                       behavior.Behavior.State.running)

        self.add_state(MotionBenchmark.State.BasicLarge1,
                       behavior.Behavior.State.running)

        self.add_state(MotionBenchmark.State.BasicLarge2,
                       behavior.Behavior.State.running)

        #End of basic motion

        self.add_state(MotionBenchmark.State.BasicEnd,
                       behavior.Behavior.State.running)


        #TRANSITIONS 

        self.add_transition(behavior.Behavior.State.start,
                            MotionBenchmark.State.setup, lambda: True,
                            'immediately')

        self.add_transition(MotionBenchmark.State.setup,
                            MotionBenchmark.State.noise,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        self.add_transition(MotionBenchmark.State.noise,
                            MotionBenchmark.State.move1,
                            lambda: self.noiseMeasured,
                            'The noise has been measured')

        self.add_transition(MotionBenchmark.State.move1,
                            behavior.Behavior.State.completed,
                            lambda: self.all_subbehaviors_completed(),
                            'Noise Test Completed')

    def on_enter_setup(self):
        move_point = robocup.Point(0, constants.Field.Width / 4)

        self.add_subbehavior(skills.move.Move(move_point), 'move') 

    def on_exit_setup(self):
        self.remove_all_subbehaviors()

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

    def role_requirements(self):
        reqs = super().role_requirements()
        return reqs
