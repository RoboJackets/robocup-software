import robocup

#What is returned when you sample a force?
#This could basically just be a tuple but it might be more confusing.
class ForceSample():

    origin = None
    vector = None

    def __init__(self, origin=robocup.Point(0,0), vector=robocup.Point(0,0)):
        self.origin=origin
        self.vector=vector
