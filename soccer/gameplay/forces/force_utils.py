
import math
import robocup


#
# A bunch of functions that will help with using the forces system
# If you want to create or tune a force, this should be the starting point
#
# It will definatly all be butifully documented in doxygen format because it will be super confusing otherwise
#

def push(anchor, sample):
    return anchor - sample

def pull(anchor, sample):
    return sample - anchor

#Be really careful when changing clipLow, keeping in mind that this will allow your force to change directions i.e. from a push to a pull, in the middle of the field, which can get confusing
def log_push(anchor, sample, base, decay, clipLow=0.0, clipHigh=float('inf'), threshold=float("-inf")):
    pushVector = push(anchor, sample)
    vectorMag = pushVector.mag()
    vectorNorm = pushVector.norm()
    logMag = log_responce(vectorMag, base, decay, clipLow, clipHigh, threshold)
    return vectorNorm * logMag 

def log_pull(anchor, sample, base, decay, clipLow=0.0, clipHigh=float('inf'), threshold=float("-inf")):
    return vec_invert(log_push(anchor, sample, base, decay, clipLow, clipHigh, threshold))

def log_responce(mag, base, decay, clipLow=0.0, clipHigh=float('inf'), threshold=float('-inf')):
    if(decay < 1):
        decay -= 1
    responce = base - math.log(mag + 1, 1 + (1 / decay))
    if(responce < threshold):
        return 0.0
    return clipLowHigh(responce)

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
# realizing that offset should be included for poly and trig responces, oof, going to be a lot of optional parameters
#
def poly_push(anchor, sample, offset=0.0, x0=0, x1=0, x2=0, x3=0, x4=0, clipLow=0.0, clipHigh=float('inf'), threshold=float('-inf')):
    pushVector = push(anchor, sample)
    vectorMag = pushVector.mag()
    vectorNorm = pushVector.norm()
    polyMag = poly_responce(vectorMag, x0, x1, x2, x3, x4, clipLow, clipHigh, threshold)
    return vectorNorm * polyMag 

def poly_pull(anchor, sample, offset=0.0, x0=0, x1=0, x2=0, x3=0, x4=0, clipLow=0.0, clipHigh=float('inf'), threshold=float('-inf')):
    return vec_invert(poly_push(anchor, sample, offset, x0, x1, x2, x3, x4, clipLow, clipHigh, threshold))

def poly_responce(mag, offset=0.0, x0=0, x1=0, x2=0, x3=0, x4=0, clipLow=0.0, clipHigh=float('inf'), threshold=float('-inf')):
    mag += offset 
    responce = x0 + mag * x1 + mag * x2**2 + mag * x3**3 + mag * x4**4
    if(responce < threshold):
        return 0.0
    return clipLowHigh(responce, clipLow, clipHigh)

def trig_pull(anchor, sample, base, decay, decay_range, offset=0.0, clipLow=0.0, clipHigh=float('inf'), threshold=float('-inf')):
    pushVector = push(anchor, sample)
    vectorMag = pushVector.mag()
    vectorNorm = pushVector.norm()
    trigMag = trig_responce(vectorMag, base, decay, decay_range, offset, clipLow, clipHigh)
    return vectorNorm * trigMag 

def trig_push(anchor, sample, base, decay, decay_range, offset=0.0, clipLow=0.0, clipHigh=float('inf')):
    return vec_invert(trig_pull(anchor, sample, base, decay, decay_range, offset, clipLow, clipHigh))

def trig_response(mag, base, decay, decay_range, offset=0.0, clipLow=0.0, clipHigh=float('inf'), theshold=float('-inf')):
    mag += offset
    responce = base - ((2 * responce_range)/(math.pi)) * math.atan(mag * decay)
    if(responce < threshold):
        return 0.0
    return clipLowHigh(responce, clipLow, clipHigh)

def force_thermal_color(force, minimum=0, maximum=10):
    return thermal_rgb_convert(force.mag(), minimum, maximum)

#UGGGG, I don't know how I should do this but these functions are getting redic but I want to add thresholding but that would mean another argument. Do i
#threshold(offset(clip_low_high(trig_responce()),1),0.2)


##
#
# Turns a magnatude into rgb values according to a "thermal" colorscheme
# taken from stack overflow here:
# https://stackoverflow.com/questions/20792445/calculate-rgb-value-for-a-range-of-values-to-create-heat-map
#
def thermal_rgb_convert(value, minimum=0, maximum=10):
    if(value > maximum):
        value = maximum
    if(value < minimum):
        value = minimum
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

##
# Drops the force to zero if it is below the threshold
def lazy_threshold(force, threshold):
    mag = force.mag()
    if(mag < threshold):
        return robocup.Point(0,0)


def threshold(mag, )


#Clips the magnitude of a vector low and high
def force_clip_low_high(input_vec, clipLow=0.0, clipHigh=float('inf')):
    return force.norm() * clipLowHigh(force.mag(), clipLow, clipHigh) 

#Flips a vector 
def vec_invert(input_vec):
    return input_vec * -1

#Rotates a vector
def rotate(input_vec, degrees):
    toRadians = math.radians(degrees)
    return robocup.Point(input_vec.x * math.cos(toRadians), input_vec.y * math.sin(toRadians))

#Scales a vector to the 
def vec_scale(input_vec, scale):
    return input_vec.norm() * scale

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




