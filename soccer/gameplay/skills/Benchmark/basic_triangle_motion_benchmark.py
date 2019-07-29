

class BasicTriangleMotionBenchmark(BaseMotionBenchmark):
   
    
    def __init__(self):
        super().__init__()

        self.title = "Basic Triangle MotionTest"


    def setupTests(self):
        self.tests = []
        numberOfRuns = 3
        print(self.tests)
    
        superBasicTest = BasicMotionTest(numberOfRuns, self)
        superBasicTest.points.append(robocup.Point(1.2,1.2))
        superBasicTest.points.append(robocup.Point(1.2,2.2))
        superBasicTest.points.append(robocup.Point(2.2,1.2))
        superBasicTest.title = "Test Triangle"
        self.tests.append(superBasicTest)

        basicMid = BasicMotionTest(numberOfRuns, self)
        basicMid.points.append(robocup.Point(1.2,1.2))
        basicMid.points.append(robocup.Point(-1.2,1.2))
        basicMid.points.append(robocup.Point(0,3.5))
        basicMid.title = "Mid Size Motion Triangle"
        self.tests.append(basicMid)

        basicSmall = BasicMotionTest(numberOfRuns, self)
        basicSmall.points.append(robocup.Point(-0.75, 1.2))
        basicSmall.points.append(robocup.Point(0.75, 1.2))
        basicSmall.points.append(robocup.Point(0,2.4))
        basicSmall.title = "Small Motion Triangle"
        self.tests.append(basicSmall)

        basicLarge = BasicMotionTest(numberOfRuns, self)
        basicLarge.points.append(robocup.Point(-1.7,1.5))
        basicLarge.points.append(robocup.Point(0, 4.8))
        basicLarge.points.append(robocup.Point(1.7, 1.5))
        basicLarge.title = "Large Motion Triangle"
        self.tests.append(basicLarge)

        basicSmaller = BasicMotionTest(numberOfRuns, self)
        basicSmaller.points.append(robocup.Point(0.25, 1.2))
        basicSmaller.points.append(robocup.Point(-0.25, 1.2))
        basicSmaller.points.append(robocup.Point(0, 1.7))
        basicSmaller.title = "Smaller Motion Triangle"
        self.tests.append(basicSmaller)
        
        basicTiny = BasicMotionTest(numberOfRuns, self)
        basicTiny.points.append(robocup.Point(0.085, 1.2))
        basicTiny.points.append(robocup.Point(-0.085, 1.2))
        basicTiny.points.append(robocup.Point(0, 1.285))
        basicTiny.title = "Tiny Motion Triangle"
        self.tests.append(basicTiny)

        basicMicro = BasicMotionTest(numberOfRuns, self)
        basicMicro.points.append(robocup.Point(0.034, 1.2))
        basicMicro.points.append(robocup.Point(-0.034, 1.2))
        basicMicro.points.append(robocup.Point(0,1.242))
        basicMicro.title = "Micro Motion Triangle"
        self.tests.append(basicMicro)
        
        midFace = BasicMotionTest(numberOfRuns, self)
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
 
    



