class ForceField():

    #This is dumb, I shouldn't do this

    samples = None
    force = None #Should I do it this way???


    def __init__(self, samples=None):
        if(samples is not None):
            self.samples = samples

    def sampleThis


    #This won't work because the lambda would need unknown parameters, IDK if this class should actually exist then
    #No wait, you can just put those into the lambda? Is there already an easy way to perform a lambda on each element of a list?
    #I bet there is, list comprehension should work
    def augment(func)
        samples = [func(x) for x in samples]


    def lazy(point, lazy_type="hat"):

