#-------------------------------------------------------------#
#functions.py file, part of the anti-flocking algorithm
#Lars Oerlemans, 4880110
# AE4350 Bio-Inspired Intelligence and Aerospace Applications
#-------------------------------------------------------------#
#This file consist of all the functions for the code


import math
import numpy as np
from Constants import Constants

def norm2(p1,p2):
    return np.linalg.norm(p1-p2) #scalar distance between two points 

#repulsion function: z distance between UAV and obstacle and d is threshold
def s(z, d):
    return max(0, 1 + np.cos(np.pi * (z + d) / (2 * d))) if z <= d else 0

def unitary_vector(p1,p2):
    if (p1==p2).all():
        # return a random velocity for each
        return np.random.uniform(low=-1,high=1, size=(2,))
    return (p2-p1)/norm2(p1,p2)

def outside_area(x,y):
    if (x<Constants.Geo_Fence_width) or (y<Constants.Geo_Fence_width) or \
        (x>Constants.area_width-Constants.Geo_Fence_width) or (y>Constants.area_length-Constants.Geo_Fence_width):
        return True
    return False

def angle_between(vec1,vec2):

    def norm1(p):
        """Scalar magnitude of a vector from (0,0)"""
        return math.sqrt(p[0]*p[0]+p[1]*p[1])

    """ Compute angle between vector1 and vector2 """
    divisor = norm1(vec1)*norm1(vec2)
    dot_product = np.dot(vec1,vec2)
    
    # Ensure divisor is not zero to avoid error in division
    if divisor == 0: return 0 
    value = dot_product/divisor

    # Ensure a value between -1 and 1 is passed to acos function
    if value > 1: value = 1 
    elif value < -1: value = -1
    
    # Final computation for angle between vectors
    return math.degrees(math.acos(value))

# %%
class Obstacle(object):
    """
        Class to define Obstacle properties
    """
    def __init__(self,ld,ru):
        """ Init function """

        # Left-down coordinates
        self.ld = ld 

        # Right-up coordinates
        self.ru = ru 
                

