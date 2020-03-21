
import math


#Ok, so I should have an increasing and decreasing version of each function?
#I've got some figuring out to do.
#I really want like n-log(2,x) where n is the magnatude at 0





def scale_log_inc(input_vec, base=2, signed=False):
    return input_vec
    #I should probably not allow negatives here

#Roots are not as intense as log functinos in many cases
def scale_root_inc(input_vec, root=2, signed=False):
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

def vec_inv_tan_inc(input_vec):
    #Inverse tangent is anot
    return input_vec



##
# The decreasing functions are for making a force smaller 
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




