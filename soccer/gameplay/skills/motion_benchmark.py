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

        BasicSmaller0 = 41
        BasicSmaller1 = 42
        BasicSmaller2 = 43
        BasicSamller3 = 44

        BasicTiny0 = 37
        BasicTiny1 = 38
        BasicTiny2 = 39
        BasicTinyEnd = 40

        #Micro Motions (really just a continuation of Basic Tests)
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



    #State Control Variables
    BasicMidCount = 0
    BasicMidLoops = 5

    BasicSmallCount = 0
    BasicSmallLoops = 5

    BasicLargeCount = 0
    BasicLargeLoops = 5

    BasicTinyCount = 0
    BasicTinyLoops = 5

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
    # End State Control Variables












    def __init__(self):
        super().__init__(continuous=False) 


        #The list of states to be registered
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
                     MotionBenchmark.State.BasicSmaller0,
                     MotionBenchmark.State.BasicSmaller1,
                     MotionBenchmark.State.BasicSmaller2,
                     MotionBenchmark.State.BasicSmallerEnd,
                     MotionBenchmark.State.BasicTiny0,
                     MotionBenchmark.State.BasicTiny1,
                     MotionBenchmark.State.BasicTiny2,
                     MotionBenchmark.State.BasicTinyEnd,
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

        #BasicLargeEnd -> Micro0
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

        #MicroEnd -> Smaller0
        self.add_transition(MotionBenchmark.State.BasicLargeEnd,
                            MotionBenchmark.State.Micro0,
                            lambda: True, 'In Position')

 
        #Smaller0 -> Smaller1
        self.add_transition(MotionBenchmark.State.Micro0,
                            MotionBenchmark.State.Micro1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #Smaller1 -> Smaller2
        self.add_transition(MotionBenchmark.State.Micro1,
                            MotionBenchmark.State.Micro2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #Smaller2 -> Smaller0
        self.add_transition(MotionBenchmark.State.Micro2,
                    MotionBenchmark.State.Micro0,
                    lambda: self.all_subbehaviors_completed() and MicroCount < MicroLoops, 'In Position')


        #Smaller2 -> SmallerEnd
        self.add_transition(MotionBenchmark.State.Micro2,
                    MotionBenchmark.State.MicroEnd,
                    lambda: self.all_subbehaviors_completed() and MicroCount >= MicroLoops, 'In Position')


        
        #SmallerEnd -> Tiny0
        self.add_transition(MotionBenchmark.State.BasicLargeEnd,
                            MotionBenchmark.State.Micro0,
                            lambda: True, 'In Position')

 
        #Tiny0 -> Tiny1
        self.add_transition(MotionBenchmark.State.Micro0,
                            MotionBenchmark.State.Micro1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #Tiny1 -> Tiny2
        self.add_transition(MotionBenchmark.State.Micro1,
                            MotionBenchmark.State.Micro2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #Tiny2 -> Tiny0
        self.add_transition(MotionBenchmark.State.Micro2,
                    MotionBenchmark.State.Micro0,
                    lambda: self.all_subbehaviors_completed() and MicroCount < MicroLoops, 'In Position')


        #Tiny2 -> TinyEnd
        self.add_transition(MotionBenchmark.State.Micro2,
                    MotionBenchmark.State.MicroEnd,
                    lambda: self.all_subbehaviors_completed() and MicroCount >= MicroLoops, 'In Position')

        #TinyEnd -> PureRot0
        self.add_transition(MotionBenchmark.State.MicroEnd,
                            MotionBenchmark.State.PureRot0,
                            lambda: True, 'In Position')


        #PureRot0 -> PureRot1
        self.add_transition(MotionBenchmark.State.Micro0,
                            MotionBenchmark.State.Micro1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #PureRot1 -> PureRot2
        self.add_transition(MotionBenchmark.State.Micro1,
                            MotionBenchmark.State.Micro2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #PureRot2 -> PureRot0
        self.add_transition(MotionBenchmark.State.Micro2,
                    MotionBenchmark.State.Micro0,
                    lambda: self.all_subbehaviors_completed() and MicroCount < MicroLoops, 'In Position')


        #PureRot2 -> PureRotEnd
        self.add_transition(MotionBenchmark.State.Micro2,
                    MotionBenchmark.State.MicroEnd,
                    lambda: self.all_subbehaviors_completed() and MicroCount >= MicroLoops, 'In Position')


        #PureRotEnd -> MidFace0
        self.add_transition(MotionBenchmark.State.MicroEnd,
                            MotionBenchmark.State.MidFace0,
                            lambda: True, 'In Position')

        #MidFace0 -> MidFace1
        self.add_transition(MotionBenchmark.State.MidFace0,
                            MotionBenchmark.State.MidFace1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #MidFace1 -> MidFace2
        self.add_transition(MotionBenchmark.State.MidFace1,
                            MotionBenchmark.State.MidFace2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #MidFace2 -> MidFace0
        self.add_transition(MotionBenchmark.State.MidFace2,
                            MotionBenchmark.State.MidFace0,
                            lambda: self.all_subbehaviors_completed() and MidFaceCount < MidFaceLoops, 'In Position')


        #MidFace2 -> MidFaceEnd
        self.add_transition(MotionBenchmark.State.MidFace2,
                    MotionBenchmark.State.MidFaceEnd,
                    lambda: self.all_subbehaviors_completed() and MidFaceCount >= MidFaceLoops, 'In Position')



        #MidFaceEnd -> SmallRot0
        self.add_transition(MotionBenchmark.State.MidFaceEnd,
                            MotionBenchmark.State.SmallRot0,
                            lambda: True, 'In Position')
        
        #SmallRot0 -> SmallRot1
        self.add_transition(MotionBenchmark.State.SmallRot0,
                            MotionBenchmark.State.SmallRot1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #SmallRot1 -> SmallRot2
        self.add_transition(MotionBenchmark.State.SmallRot1,
                            MotionBenchmark.State.SmallRot2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #SmallRot2 -> SmallRot0
        self.add_transition(MotionBenchmark.State.SmallRot2,
                            MotionBenchmark.State.SmallRot0,
                            lambda: self.all_subbehaviors_completed() and MidFaceCount < MidFaceLoops, 'In Position')


        #SmallRot2 -> SmallRotEnd
        self.add_transition(MotionBenchmark.State.SmallRot2,
                    MotionBenchmark.State.SmallRotEnd,
                    lambda: self.all_subbehaviors_completed() and MidFaceCount >= MidFaceLoops, 'In Position')



        #SmallRotEnd -> MicroRot0
        self.add_transition(MotionBenchmark.State.SmallRotEnd,
                            MotionBenchmark.State.MicroRot0,
                            lambda: True, 'In Position')


        #MicroRot0 -> MicroRot1
        self.add_transition(MotionBenchmark.State.Micro0,
                            MotionBenchmark.State.Micro1,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #MicroRot1 -> MicroRot2
        self.add_transition(MotionBenchmark.State.Micro1,
                            MotionBenchmark.State.Micro2,
                            lambda: self.all_subbehaviors_completed(), 'In Position')

        #MicroRot2 -> MicroRot0
        self.add_transition(MotionBenchmark.State.Micro2,
                            MotionBenchmark.State.Micro0,
                            lambda: self.all_subbehaviors_completed() and MicroCount < MicroLoops, 'In Position')


        #SmallRot2 -> SmallRotEnd
        self.add_transition(MotionBenchmark.State.SmallRot2,
                    MotionBenchmark.State.SmallRotEnd,
                    lambda: self.all_subbehaviors_completed() and MidFaceCount >= MidFaceLoops, 'In Position')


        #End transition
        self.add_transition(MotionBenchmark.State.MicroRotEnd,
                            behavior.Behavior.State.completed,
                            lambda: True, 'All Tests completed')
        
        
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


    #move1 functions END


    #BasicMid0 functions START









    #BasicMid0 functions END



    #BasicMid1 functions START

    #BasicMid1 functions END



    #BasicMid2 functions START

    #BasicMid2 functions END



    #BasicMidEnd functions START

    #BasicMidEnd functions END



    #BasicSmall0 functions START

    #BasicSmall0 functions END

    #BasicSmall1 functions START

    #BasicSmall1 functions END

    #BasicSmall2 functions START

    #BasicSmall2 functions END

    #BasicSmallEnd functions START

    #BasicSmallEnd functions END

    #BasicLarge0 functions START

    #BasicLarge0 functions END

    #BasicLarge1 functions START

    #BasicLarge1 functions END

    #BasicLarge2 functions START


    #BasicLarge2 functions END


    #BasicLargeEnd functions START


    #BasicLargeEnd functions END


    #Micro0 functions START


    #Micro0 functions END


    #Micro1 functions START


    #Micro1 functions END


    #Micro2 functions START


    #Micro2 functions END


    #MicroEnd functions START


    #MicroEnd functions END

    
    #PureRot0 functions START


    #PureRot0 functions END


    #PureRot1 functions START


    #PureRot1 functions END


    #PureRot2 functions START


    #PureRot2 functions END


    #PureRotEnd functions START


    #PureRotEnd functions END



    #MidFace0 functions START


    #MidFace0 functions END


    #MidFace1 functions START



    #MidFace1 functions END


    #MidFace2 functions START


    #MidFace2 functions END


    #MidFaceEnd functions START


    #MidFaceEnd functions END

    
    #SmallRot0 functions START


    #SmallRot0 functions END


    #SmallRot1 functions START


    #SmallRot1 functions END


    #SmallRot2 functions START


    #SmallRot2 functions END


    #SmallRotEnd functions START


    #SmallRotEnd functions END

    
    
    #MicroRot0 functions START


    #MicroRot0 functions END


    #MicroRot1 functions START


    #MicroRot1 functions END


    #MicroRot2 functions START


    #MicroRot2 functions END


    #MicroRotEnd  functions START

    
    #MicroRotEnd  functions END

    
    
    #EndAll functions START



    #EndAll functions END





















    #nothing special for role requirements
    def role_requirements(self):
        reqs = super().role_requirements()
        return reqs
