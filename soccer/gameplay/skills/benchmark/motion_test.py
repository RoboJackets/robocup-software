from abc import ABC, abstractmethod

#An abstract base class for motion tests to be run by subclasses of the BaseMotionBenchmark skill
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


