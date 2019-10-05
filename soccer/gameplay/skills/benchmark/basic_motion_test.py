import skills.benchmark.motion_test
import time
import skills.benchmark.motion_benchmark
#import Motion
#import motion_test.MotionTest




##Runs motion tests involving polygonal motions with or without a face command
#So if we end up defining everything inside of the plays, this should probably just be called motion test?
class BasicMotionTest(skills.benchmark.motion_test.MotionTest):

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

    
    ##
    # Appends a point to the list of movement points
    def addPoint(self, point):
        self.points.append(a)

    
    ##
    # Appends a point to the list of face points
    def addFacePoint(self, point):
        self.facePoints.append(a)


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
        vel = self.theMotionBenchmark.getVel() #MotionBenchmark.getVel(self.theMotionBenchmark)
        speed = self.theMotionBenchmark.getSpeed()

        if speed > self.maximumSpeed:
            maximumSpeed = speed
       
        self.totalVel[self.motionNumber] += speed * deltat

        accl = abs(self.lastSpeed - speed) / deltat

        self.lastSpeed = speed

    #Records the data at the end of the motion
    def endMotion(self):
        if(self.motionNumber != -1):
            print("Motion " + str(self.motionNumber) + " out of " + str(self.motions) + " in test " + self.title + " results: ")
            self.timeTaken[self.motionNumber] = abs(self.motionStartTime - time.time())
            print("Time taken: " + str(self.timeTaken[self.motionNumber]) + " seconds")
            self.endVel[self.motionNumber] = self.theMotionBenchmark.getSpeed()
            print("Ending velocity: " + str(self.endVel[self.motionNumber]) + " m/s")
            self.calcFinalRotationError()
            self.calcFinalPosError()
            print("Overshoot: " + str(self.maxOvershoot[self.motionNumber]) + " meters")
            print("Line follow Error (area between path and line): " + str(self.lineFollowError[self.motionNumber]))
            print("Rotational Follow Error: " + str(self.rotationalFollowError[self.motionNumber]))

        if(self.started):
            self.motionNumber = self.motionNumber + 1
        else:
            self.started = True

        if (len(self.facePoints) is 0):
            self.theMotionBenchmark.robot.face_none()
          
    #Calculates the final rotational error based on self.currentFacePoint
    def calcFinalRotationError(self):
        if(self.currentFacePoint is not None):
            self.finalRotationalError[self.motionNumber] = self.theMotionBenchmark.getAngleError(self.currentFacePoint)
            print("End rotational error: " + str(self.finalRotationalError[self.motionNumber]) + " degrees")

    #Calculates the final positional error based 
    def calcFinalPosError(self):
        if(self.currentEnd is not None):
            self.posEndError[self.motionNumber] = self.theMotionBenchmark.getPosError(self.currentEnd)
            print("End positional error: " + str(self.posEndError[self.motionNumber]) + " meters")

    #Update the line follow error integral
    def integrateLineError(self, deltat):
        self.lineFollowError[self.motionNumber] += self.theMotionBenchmark.getLineError(self.currentStart, self.currentEnd) * deltat

    #Update the rotation error integral
    def integrateRotationalError(self, deltat):
        self.rotationalFollowError[self.motionNumber] += abs(self.theMotionBenchmark.getAngleError(self.currentFacePoint)) * deltat

    #Check and update the overshoot stats
    def updateOvershoot(self):
        overshoot = self.theMotionBenchmark.getOvershoot(self.currentStart, self.currentEnd)
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
        unitLineError = list(map(truediv, self.lineFollowError, self.timeTaken)) #What even is this?
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



