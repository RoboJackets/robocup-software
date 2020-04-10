class ForceField():

    #This is dumb, I shouldn't do this

    samples = None
    force = None #Should I do it this way???

    def __init__(self, samples=None, force=None):
        self.samples = samples
        self.force = force 
     
    #Regenerate the field
    def generate(self):
        self.samples = list()
        pass

    #Augment the current field, some things to figure out here
    def augment(self, func)
        self.samples = [func(x) for x in samples]


