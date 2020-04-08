class ForceField():

    #This is dumb, I shouldn't do this

    samples = None

    def __init__(self, samples=list()):
        self.samples = samples

    #This won't work because the lambda would need unknown parameters, IDK if this class should actually exist then
    #No wait, you can just put those into the lambda? Is there already an easy way to perform a lambda on each element of a list?
    #I bet there is, list comprehension should work
    def augment(func)
        samples = [func(x) for x in samples]

