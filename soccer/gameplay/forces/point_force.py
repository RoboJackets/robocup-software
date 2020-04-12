from forces import force
from forces import force_utils
from forces import force_sample

class PointForce(forces.force):


    point = None

    responce_function = lambda x : force_utils.log_responce(x, 2.0, 1.0)
    responce_direction = direction.Direction.PUSH  

    def __init__(self, point, responce_function=None, responce_direction=None):
        self.point = point
        if(responce_function is not None):
            self.responce_function = responce_function
        if(responce_direction is not None):
            self.responce_direction = responce_direction

    def sample(self, sample_point):
        vec = force_utils.push(self.point, sample_point)
        vec = force_utils.vec_scale(vec,responce_function(norm.mag())) 

        if(self.responce_direction is direction.Direction.PUSH):
            vec = force_utils.vec_invert(vec)
           
        return force_sample.ForceSample(vec, sample_point)


