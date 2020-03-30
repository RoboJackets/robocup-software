
import math


#Ok, so I should have an increasing and decreasing version of each function?
#I've got some figuring out to do.
#I really want like n-log(2,x) where n is the magnatude at 0


#Out of all the bullcrap I've written I think these are probably what I would want
#I can't think of a reason that you would need more granular control?

#I should possibly change "push" and "pull" arguments to sample, to represent that is the point you are sampling the field at

#It's also possible that I should just do a "push" and then have you negate that to get a pull, but they need to decay and be offset differently so IDK.
#I think this can be made to work to cover all types of fields that you would want.


#Be really careful when changing clipLow, keeping in mind that this will allow your force to change directions i.e. from a push to a pull, in the middle of the field, which can get confusing
def log_push(anchor, sample, base, decay, clipLow=0.0, clipHigh=float('inf')):
    return None

def log_pull(anchor, sample, base, decay, clipLow=0.0, clipHigh=float('inf')):
    return None



def poly_push(anchor, sample, x0=0, x1=0, x2=3, x3=4):
    return None

def poly_pull(anchor, sample):

#This might have 
def distance_maigntain()


#Flips a vector 
def invert(input_vec):
    return input_vec

def rotate(input_vec, degrees):
    return input_vec


def norm_split(input_vec):
    return input_vec.norm(), input_vec.mag()


def log_compress(input_vec, base=2):
    mag = input_vec.mag()
    norm = 
    return input_vec
    #I should probably not allow negatives here



def log_decay(input_vec, base=2):
    return None



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




