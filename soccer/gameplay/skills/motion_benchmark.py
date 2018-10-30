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
        BasicMidEnd = 7


        BasicSmall0 = 8
        BasicSmall1 = 9
        BasicSmall2 = 10
        BasicSmallEnd = 11

        BasicLarge0 = 12
        BasicLarge1 = 13
        BasicLarge2 = 14
        BasicLargeEnd = 15


        #Micro Motions
        Micro0 = 16
        Micro1 = 17
        Micro2 = 18
        MicroEnd = 19


        #Pure Rotations
        PureRot0 = 20
        PureRot1 = 21
        PureRot2 = 22
        PureRotEnd = 23


        #Medium Triangle while facing
        MidFace0 = 24
        MidFace1 = 25
        MidFace2 = 26
        MidFaceEnd = 27

        #Small movements with a specified end orientation
        SmallRot0 = 28 
        SmallRot1 = 29
        SmallRot2 = 30
        SmallRotEnd = 31


        #Very small movementes with a specified orientation
        MicroRot0 = 32
        MicroRot1 = 33
        MicroRot2 = 34
        MicroRotEnd = 35
        
        EndAll = 36





        
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




    #State Control Variables
    BasicMidCount = 0
    BasicMidLoops = 5

    BasicSmallCount = 0
    BasicSmallLoops = 5

    BasicLargeCount = 0
    BasicLargeLoops = 5

    MicroCount = 0
    MicroLoops = 5

    PureRotCount = 0
    PureRotLoops = 5


    MidFaceCount = 0
    MidFaceLoops = 5

    SmallRotCount = 0
    SmallRotLoops = 5

    MicroRotCount = 0
    MicroRotLoops = 5


    #Phase 1: Basic Motion


'''
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
'''
    
    def __init__(self):
        super().__init__(continuous=False) 



        allStates = [MotionBenchmark.State.setup, 
                     MotionBenchmark.State.noise,
                     MotionBenchmark.State.move1,
                     MotionBenchmark.State.BasicMid0,
                     MotionBenchmark.State.BasicMid1,
                     MotionBenchmark.State.BasicMid2,
                     MotionBenchmark.State.BasicMidEnd,
                     MotionBenchmark.State.BasicLarge0,
                     MotionBenchmark.State.BasicLarge1,
                     MotionBenchmark.State.BasicLarge2,
                     MotionBenchmark.State.BasicLargeEnd,
                     MotionBenchmark.State.BasicSmall0,
                     MotionBenchmark.State.BasicSmall1,
                     MotionBenchmark.State.BasicSmall2,
                     MotionBenchmark.State.BasicSmallEnd,
                     MotionBenchmark.State.Micro0,
                     MotionBenchmark.State.Micro1,
                     MotionBenchmark.State.Micro2,
                     MotionBenchmark.State.MicroEnd,
                     MotionBenchmark.State.PureRot0,
                     MotionBenchmark.State.PureRot1,
                     MotionBenchmark.State.PureRot2,
                     MotionBenchmark.State.PureRotEnd,
                     MotionBenchmark.State.MidFace0,
                     MotionBenchmark.State.MidFace1,
                     MotionBenchmark.State.MidFace2,
                     MotionBenchmark.State.MidFaceEnd,
                     MotionBenchmark.State.SmallRot0,
                     MotionBenchmark.State.SmallRot1,
                     MotionBenchmark.State.SmallRot2,
                     MotionBenchmark.State.SmallRotEnd,
                     MotionBenchmark.State.MicroRot0,
                     MotionBenchmark.State.MicroRot1,
                     MotionBenchmark.State.MicroRot2,
                     MotionBenchmark.State.MicroRotEnd,
                     MotionBenchmark.State.EndAll]



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
                            MotionBenchmark.State.BasicMid0,
                            lambda: self.all_subbehaviors_completed(), 'In Position')
        
        #BasicMid0 -> BasicMid1
        self.add_transition(MotionBenchmark.State.BasicMid0,
                            MotionBenchmark.State.BasicMid1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #BasicMid1 -> BasicMid2
        self.add_transition(MotionBenchmark.State.BasicMid1,
                            MotionBenchmark.State.BasicMid2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')


        #BasicMid2 -> BasicMid0
        self.add_transition(MotionBenchmark.State.BasicMid2,
                            MotionBenchmark.State.BasicMid0,
                            lambda: self.all_subbehaviors_completed() and BasicMidCount < BasicMidLoops, 'In Position')

        #BasicMid2 -> BasicMidEnd
        self.add_transition(MotionBenchmark.State.BasicMid2,
                            MotionBenchmark.State.BasicMidEnd,
                            lambda: self.all_subbehaviors_completed() and BasicMidCount >= BasicMidLoops, 'In Position')

        #BasicMidEnd -> BasicSmall0
        self.add_transition(MotionBenchmark.State.BasicMidEnd,
                            MotionBenchmark.State.BasicSmall0,
                            lambda: True, 'In Position')

        #BasicSmall0 -> BasicSmall1
        self.add_transition(MotionBenchmark.State.BasicSmall0,
                            MotionBenchmark.State.BasicSmall1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #BasicSmall1 -> BasicSmall2
        self.add_transition(MotionBenchmark.State.BasicSmall1,
                            MotionBenchmark.State.BasicSmall2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #BasicSmall2 -> BasicSmall0
        self.add_transition(MotionBenchmark.State.BasicSmall2,
                            MotionBenchmark.State.BasicSmall0,
                            lambda: self.all_subbehaviors_completed() and BasicSmallCount < BasicSmallLoops, 'In Position')

        #BasicSmall2 -> BasicSmallEnd
        self.add_transition(MotionBenchmark.State.BasicSmall0,
                            MotionBenchmark.State.BasicSmallEnd,
                            lambda: self.all_subbehaviors_completed() and BasicSmallCount >= BasicSmallLoops, 'In Position')


        #BasicSmallEnd -> BasicLarge0
        self.add_transition(MotionBenchmark.State.BasicSmallEnd,
                            MotionBenchmark.State.BasicLarge0,
                            lambda: True, 'In Position')
        
      
        #BasicLarge0 -> BasicLarge1
        self.add_transition(MotionBenchmark.State.BasicLarge0,
                            MotionBenchmark.State.BasicLarge1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #BasicLarge1 -> BasicLarge2
        self.add_transition(MotionBenchmark.State.BasicLarge1,
                            MotionBenchmark.State.BasicLarge2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #BasicLarge2 -> BasicLarge0
        self.add_transition(MotionBenchmark.State.BasicLarge2,
                    MotionBenchmark.State.BasicLarge0,
                    lambda: self.all_subbehaviors_completed() and BasicLargeCount < BasicLargeLoops, 'In Position')


        #BasicLarge2 -> BasicLargeEnd
        self.add_transition(MotionBenchmark.State.BasicLarge2,
                    MotionBenchmark.State.BasicLargeEnd,
                    lambda: self.all_subbehaviors_completed() and BasicLargeCount >= BasicLargeLoops, 'In Position')

        #BasicSmallEnd -> BasicLarge0
        self.add_transition(MotionBenchmark.State.BasicLargeEnd,
                            MotionBenchmark.State.Micro0,
                            lambda: True, 'In Position')

 
        #Micro0 -> Micro1
        self.add_transition(MotionBenchmark.State.Micro0,
                            MotionBenchmark.State.Micro1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #Micro1 -> Micro2
        self.add_transition(MotionBenchmark.State.Micro1,
                            MotionBenchmark.State.Micro2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #Micro2 -> Micro0
        self.add_transition(MotionBenchmark.State.Micro2,
                    MotionBenchmark.State.Micro0,
                    lambda: self.all_subbehaviors_completed() and MicroCount < MicroLoops, 'In Position')


        #Micro2 -> MicroEnd
        self.add_transition(MotionBenchmark.State.Micro2,
                    MotionBenchmark.State.MicroEnd,
                    lambda: self.all_subbehaviors_completed() and MicroCount >= MicroLoops, 'In Position')

        #MicroEnd -> BasicLarge0
        self.add_transition(MotionBenchmark.State.MicroEnd,
                            MotionBenchmark.State.PureRot0,
                            lambda: True, 'In Position')




        #End transition
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
