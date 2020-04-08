
import math
import robocup

def push(anchor, sample):
    return anchor - sample

def pull(anchor, sample):
    return sample - anchor

#Be really careful when changing clipLow, keeping in mind that this will allow your force to change directions i.e. from a push to a pull, in the middle of the field, which can get confusing
def log_push(anchor, sample, base, decay, clipLow=0.0, clipHigh=float('inf')):
    pushVector = push(anchor, sample)
    vectorMag = pushVector.mag()
    vectorNorm = pushVector.norm()
    logMag = log_responce(vectorMag, base, decay, clipLow, clipHigh)
    return vectorNorm * logMag 

def log_pull(anchor, sample, base, decay, clipLow=0.0, clipHigh=float('inf')):
    return vec_invert(log_push(anchor, sample, base, decay, clipLow, clipHigh))

def log_responce(mag, base, decay, clipLow, clipHigh):
    if(decay < 1):
        decay -= 1
     return clipLowHigh((base - math.log(mag + 1, 1 + (1 / decay))))

def clipLowHigh(x, low, high):
    if(x < low):
        return low
    elif(x > high):
        return high
    else:
        return x

##
#
#
# x0 is essentially your base here
#
def poly_push(anchor, sample, x0=0, x1=0, x2=0, x3=0, x4=0, clipLow=0.0, clipHigh=float('inf')):
    pushVector = push(anchor, sample)
    vectorMag = pushVector.mag()
    vectorNorm = pushVector.norm()
    polyMag = poly_responce(vectorMag, x0, x1, x2, x3, x4, clipLow, clipHigh)
    return vectorNorm * logMag 


def poly_pull(anchor, sample, x0=0, x1=0, x2=0, x3=0, x4=0, clipLow=0.0, clipHigh=float('inf')):
    return vec_invert(poly_push(anchor, sample, x0))

def poly_responce(mag, x0=0, x1=0, x2=0, x3=0, x4=0, clipLow=0.0, clipHigh=float('inf')):
    responce = x0 + mag * x1 + mag * x2**2 + mag * x3**3 + mag * x4**4
    return clipLowHigh(responce, clipLow, clipHigh)


def trig_pull(anchor, sample, base, decay, decay_range, clipLow=0.0, clipHigh=float('inf')):
    return None

def trig_push(anchor, sample, base, decay, decay_range, clipLow=0.0, clipHigh=float('inf')):
    return None

def trig_response(mag, base, decay, decay_range, clipLow=0.0, clipHigh=float('inf')):
    responce = base - ((2 * responce_range)/(math.pi)) * math.atan(mag * decay)
    return clipLowHigh(responce, clipLow, clipHigh)

def force_thermal_color(force, minimum=0, maximum=10):
    return thermal_rgb_convert(force.mag(), minimum, maximum)

##
#
# Turns a magnatude into rgb values according to a "thermal" colorscheme
# taken from stack overflow here:
# https://stackoverflow.com/questions/20792445/calculate-rgb-value-for-a-range-of-values-to-create-heat-map
#
def thermal_rgb_convert(value, minimum=0, maximum=10):
    minimum, maximum = float(minimum), float(maximum)
    ratio = 2 * (value-minimum) / (maximum - minimum)
    b = int(max(0, 255*(1 - ratio)))
    r = int(max(0, 255*(ratio - 1)))
    g = 255 - b - r
    return r, g, b



#Laziness is something that I'll have to consider, the intent is to scale down nearby force vectors? 
def linear_lazy(origin, sample, force):
    return None

def log_lazy(origin, sample, force):
    return None

def poly_lazy(origin, sample, force):
    return None

def trig_lazy(origin, sample, force):
    return None

def lazy_clip(force, threshold):



def force_clip_low_high(force, clipLow=0.0, clipHigh=float('inf')):
    return None






#Flips a vector 
def vec_invert(input_vec):
    return input_vec * -1


#Rotates a vector
def rotate(input_vec, degrees):
    toRadians = math.radians(degrees)
    return robocup.Point(input_vec.x * math.cos(toRadians), input_vec.y * math.sin(toRadians))


'''
def log_compress(input_vec, base=2):
    mag = input_vec.mag()
    norm = 
    return input_vec
    #I should probably not allow negatives here

#Roots are not as intense as log functinos in many cases
def scale_root_compress(input_vec, root=2, signed=False):
    return input_vec

def scale_clip_inc(input_vec, mag_max=float("inf"), mag_min=0, signed=False):
    return input_vec
    #If signed is false its just looking at the magnitude

def scale_poly_inc(input_vec,x0=0,x1=0,x2=0,x3=0):
    #This is basically going to be a polynomial function for scaling
    return input_vec

def vec_inv_inc(input_vec, add_one=True):
    #This is basically going to be 1/x or 1/(x+1)
    return input_vec

def vec_arctan_inc(input_vec):
    #Inverse tangent is anot
    return input_vec



##
# The decreasing functions are for making a force smaller as it's magnatude increases
#
#
##

#Intended to be used with robocup points
def scale_log_dec(input_vec, base=2, signed=False):
    return input_vec
    #I should probably not allow negatives here

#Roots are not as intense as log functinos in many cases
def scale_root_dec(input_vec, root=2, signed=False):
    return input_vec

def scale_clip_dec(input_vec, mag_max=float("inf"), mag_min=0, signed=False):
    return input_vec
    #If signed is false its just looking at the magnitude

def scale_poly_dec(input_vec,x0=0,x1=0,x2=0,x3=0):
    #This is basically going to be a polynomial function for scaling
    return input_vec

def vec_inv_dec(input_vec, add_one=True):
    #This is basically going to be 1/x or 1/(x+1)
    return input_vec

def vec_inv_tan_dec(input_vec):
    #Inverse tangent is anot
    return input_vec



#Returns a normalized push away from the target
#hmm, I guess I should try to make it so that the target can be basically any geometry 2d.
#Like a rectangle or whatnot
def push(sample_point, target):
    return robocup.Point(0,0)

#Returns a pull vector towards the target
def pull(sample_point, target)
    return robocup.Point(0,0)

#These are kind of hard to explain, its like you want the line to move away from the point
def push_line_away_from_point(sample_point, target, push_point, high_clip=None):
    return None

#Its like pull line away from point but the oppisite
def pull_line_to_point():
    return None
'''




