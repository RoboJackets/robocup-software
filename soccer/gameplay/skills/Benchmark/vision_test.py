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


