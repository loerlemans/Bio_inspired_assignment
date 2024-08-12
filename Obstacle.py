#-------------------------------------------------------------#
#Obstacle.py file, part of the anti-flocking algorithm
#Lars Oerlemans, 4880110
# AE4350 Bio-Inspired Intelligence and Aerospace Applications
#-------------------------------------------------------------#
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
                
