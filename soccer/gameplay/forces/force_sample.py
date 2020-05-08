import robocup
import copy


#This is probably more complex than it might neec to be, but I was attached to the merge function lambda function
# and wanted to make that work as well as possible
class ForceSample():

    origin = None
    vector = None

    def __init__(self, origin=robocup.Point(0,0), vector=robocup.Point(0,0)):
        self.origin=origin
        self.vector=vector

    def __str__(self):
        return "Origin: (" + str(origin.x) + ", " + str(origin.y) + "), Sample: (" + str(vector.x) + ", " + str(vector.y) + ")"

    def checkOrigin(self, other):
        if(self.origin != other.origin):
            raise ValueError("The origins of the force samples are not aligned, which is not allowed for most operations")

    def mag(self):
        return self.vector.mag()

    def __eq__(self, other):
        return self.origin == other.origin and self.vector == other.vector

    def __ne__(self, other):
        return not self.__eq__(self, other) 

    def __lt__(self, other):
        self.checkOrigin(self, other)
        return self.mag() < other.mag()

    def __le__(self, other):
        self.checkOrigin(self, other)
        return self.mag() <= other.mag()

    def __gt__(self, other):
        self.checkOrigin(self, other)
        return self.mag() <= other.mag()

    def __ge__(self, other):
        self.checkOrigin(self, other)
        return self.mag() >= other.mag()

    def __add__(self, other):
        self.checkOrigin(self, other)
        return ForceSample(origin=self.origin, vector=self.vector + other.vector)

    def __sub__(self, other):
        self.checkOrigin(self, other)
        return ForceSample(origin=self.origin, vector=self.vector - other.vector)

    def __mul__(self, other):
        newSample = copy.deepcopy(sample)        
        if(isinstance(other, ForceSample)):
            self.checkOrigin(self, other)
            newSample.vector.x *= other.vector.x
            newSample.vector.y *= other.vector.y
        else:
            num = float(other)
            newSample.vector.x *= num
            newSample.vector.y *= num
        return newSample

    def __div__(self, other):
        newSample = copy.deepcopy(self)        
        if(isinstance(other, ForceSample)):
            self.checkOrigin(self, other)
            newSample.vector.x /= other.vector.x
            newSample.vector.y /= other.vector.y
        else:
            num = float(other)
            newSample.vector.x /= num
            newSample.vector.y /= num
        return newSample

    def __abs__(self):
        newSample = copy.deepcopy(self)
        newSample.vector.x = abs(newSample.vector.x)
        newSample.vector.y = abs(newSample.vector.y)
        return newSample

    def __neg__(self):
        newSample = copy.deepcopy(self)
        newSample.vector.x = -1 * newSample.vector.x
        newSample.vector.y = -1 * newSample.vector.y
        return newSample




