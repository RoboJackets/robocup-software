import robocup
import play
import behavior
import skills.move
import skills.capture
import tactics.coordinated_pass
import constants
import main
import enum
import tools
import skills.latency_test
import skills.benchmark.motion_benchmark
import skills.benchmark.basic_motion_test
import robocup

class RunMotionBenchmark(play.Play):

    benchmark = skills.benchmark.motion_benchmark.MotionBenchmark()

    class State(enum.Enum):
        
        #Move to a central location 
        run = 1
        done = 2

    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(RunMotionBenchmark.State.run,
                       behavior.Behavior.State.running)
        self.add_state(RunMotionBenchmark.State.done,
                       behavior.Behavior.State.running)
        
        self.add_transition(behavior.Behavior.State.start,
                            RunMotionBenchmark.State.run, lambda: True,
                            'immediately')

        self.add_transition(RunMotionBenchmark.State.run,
                            behavior.Behavior.State.completed,
                            lambda: self.all_subbehaviors_completed(),
                            'all subbehaviors completed')

       
        numberOfRuns = 3


        
        superBasicTest = skills.benchmark.basic_motion_test.BasicMotionTest(numberOfRuns, self.benchmark)
        superBasicTest.points.append(robocup.Point(1.2,1.2))
        superBasicTest.points.append(robocup.Point(1.2,2.2))
        superBasicTest.points.append(robocup.Point(2.2,1.2))
        superBasicTest.title = "Test Triangle"
        self.benchmark.addTest(superBasicTest)

        basicMid = skills.benchmark.basic_motion_test.BasicMotionTest(numberOfRuns, self.benchmark)
        basicMid.points.append(robocup.Point(1.2,1.2))
        basicMid.points.append(robocup.Point(-1.2,1.2))
        basicMid.points.append(robocup.Point(0,3.5))
        basicMid.title = "Mid Size Motion Triangle"
        self.benchmark.addTest(basicMid)

        basicSmall = skills.benchmark.basic_motion_test.BasicMotionTest(numberOfRuns, self.benchmark)
        basicSmall.points.append(robocup.Point(-0.75, 1.2))
        basicSmall.points.append(robocup.Point(0.75, 1.2))
        basicSmall.points.append(robocup.Point(0,2.4))
        basicSmall.title = "Small Motion Triangle"
        self.benchmark.addTest(basicSmall)

        basicLarge = skills.benchmark.basic_motion_test.BasicMotionTest(numberOfRuns, self.benchmark)
        basicLarge.points.append(robocup.Point(-1.7,1.5))
        basicLarge.points.append(robocup.Point(0, 4.8))
        basicLarge.points.append(robocup.Point(1.7, 1.5))
        basicLarge.title = "Large Motion Triangle"
        self.benchmark.addTest(basicLarge)

        basicSmaller = skills.benchmark.basic_motion_test.BasicMotionTest(numberOfRuns, self.benchmark)
        basicSmaller.points.append(robocup.Point(0.25, 1.2))
        basicSmaller.points.append(robocup.Point(-0.25, 1.2))
        basicSmaller.points.append(robocup.Point(0, 1.7))
        basicSmaller.title = "Smaller Motion Triangle"
        self.benchmark.addTest(basicSmaller)
        
        basicTiny = skills.benchmark.basic_motion_test.BasicMotionTest(numberOfRuns, self.benchmark)
        basicTiny.points.append(robocup.Point(0.085, 1.2))
        basicTiny.points.append(robocup.Point(-0.085, 1.2))
        basicTiny.points.append(robocup.Point(0, 1.285))
        basicTiny.title = "Tiny Motion Triangle"
        self.benchmark.addTest(basicTiny)

        basicMicro = skills.benchmark.basic_motion_test.BasicMotionTest(numberOfRuns, self.benchmark)
        basicMicro.points.append(robocup.Point(0.034, 1.2))
        basicMicro.points.append(robocup.Point(-0.034, 1.2))
        basicMicro.points.append(robocup.Point(0,1.242))
        basicMicro.title = "Micro Motion Triangle"
        self.benchmark.addTest(basicMicro)
       
        

        midFace = skills.benchmark.basic_motion_test.BasicMotionTest(numberOfRuns, self.benchmark)
        midFace.points.append(robocup.Point(1.2,1.2))
        midFace.points.append(robocup.Point(-1.2,1.2))
        midFace.points.append(robocup.Point(0,3.5))
        midFace.facePoints.append(robocup.Point(0,2.8))
        midFace.facePoints.append(robocup.Point(0,0))
        midFace.facePoints.append(robocup.Point(2,0))
        midFace.title = "Mid size triangle while facing points #1"
        self.benchmark.addTest(midFace)
        
        midFace2 = skills.benchmark.basic_motion_test.BasicMotionTest(numberOfRuns, self.benchmark)
        midFace2.points.append(robocup.Point(1.2,1.2))
        midFace2.points.append(robocup.Point(-1.2,1.2))
        midFace2.points.append(robocup.Point(0,3.5))
        midFace2.facePoints.append(robocup.Point(0,2.8))
        midFace2.facePoints.append(robocup.Point(0,0))
        midFace2.facePoints.append(robocup.Point(2,0))
        midFace2.title = "Mid size triangle while facing points #2"
        self.benchmark.addTest(midFace2)


        #Pure rotational Tests do not currently work
        '''
        pureRot = skills.benchmark.basic_motion_test.BasicMotionTest(numberOfRuns, self.benchmark)
        pureRot.facePoints.append(robocup.Point(0,2.8))
        pureRot.facePoints.append(robocup.Point(0,0))
        pureRot.facePoints.append(robocup.Point(2,0))
        pureRot.title = "Pure Rotational Test"
        self.benchmark.addTest(pureRot)
        '''

        self.benchmark.done_with_setup()

  
    def on_enter_run(self):
        self.add_subbehavior(self.benchmark, 'Runs the benchmark')

