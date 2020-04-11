
from forces import force_utils


#A force origonating from a number of points
class PointsForce(force.Force):
    
    points = list()

    ## The responce function 
    responce_function = lambda x : force_utils.log_responce()
    
    responce_direction = direction.Direction.PUSH

    #Actually merge functions might need some modifications
    #Yeah it's not going to work as is, because I don't think
    merge_function = lambda x : max(x)

    def sample(self, point):
        forces = list()

        merge_function


