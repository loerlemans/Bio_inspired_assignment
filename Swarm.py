#-------------------------------------------------------------#
#Swarm.py file, part of the anti-flocking algorithm
#Lars Oerlemans, 4880110
# AE4350 Bio-Inspired Intelligence and Aerospace Applications
#-------------------------------------------------------------#



import numpy as np
import random, math
from Constants import Constants
from functions import norm2,unitary_vector
import numpy as np
import random, math

#--------------------------------# 
#swarming Behaviour class
#--------------------------------#
class Swarm():
    #initialise The UAV parameters and create all zero arrays
    def __init__(self, num, obstacles):
      
        #Array UAV attitudes (heading/position)
        self.heading_angle, self.pos = self.init_positions(num)
        
        # w: Control input for changing heading angle
        self.control_input = np.zeros(num)
        
        # θ: Difference angle --> angle between desired and actual angle
        self.diff_angle = np.zeros(num)
        
        # Initialise the Coverage map
        self.coverage_map = self.init_coverage_map(num, obstacles)
        
        #UAV Velocity terms arrays
        self.vel_actual = np.zeros((num, 2))
        self.vel_desired = np.zeros((num, 2))
        self.vel_obs = np.zeros((num, 2)) #obstacle/collission avoidance
        self.vel_dec = np.zeros((num, 2)) # decentering
        self.vel_sel = np.zeros((num, 2))   #selfishness
        self.vel_bou = np.zeros((num, 2)) #boundary velocity
        
        # Array for storing neighbors indexes
        self.neighbors = [[] for i in range(num)]
        
        # UAV Goals Arrays (x,y) current and previous
        self.goal = np.zeros_like(self.pos)
        self.prev_goal = np.zeros_like(self.pos)
        
        # Accumulated percentage coverage
        self.coverage_percentage = 0
        
        # Instantaneous coverage map --> each UAV has its own coverage map
        self.instantaneous_coverage_map = np.zeros((Constants.area_width, Constants.area_length))  

    # Mark with -1 all cells with obstacle inside
    def init_coverage_map(self, num_UAVS, obstacles):
        cov_map = np.zeros((Constants.area_width,Constants.area_length)) 
        for obs in obstacles:
            for row in range(obs.ld[0],obs.ru[0]):
                cov_map[row][obs.ld[1]:obs.ru[1]] = Constants.Obstacle_Value
        return np.array([cov_map]*num_UAVS)    

    #Position function agent with different initialisation configurations
    def init_positions(self, num):
        middle = [Constants.area_length / 2, Constants.area_width / 2]
        positions = []
        angles = []
        
        if Constants.INIT_RANDOM:
            for _ in range(num):
                positions.append((Constants.area_width / 2 + random.uniform(-1, 1), 
                                  Constants.area_length / 2 + random.uniform(-1, 1)))
                angles.append(360 * random.random())
    
        if Constants.INIT_CIRCUMFERENCE:
            init_angle=(2*math.pi)/num # in radians
            angle=init_angle
            for _ in range(num):
                positions.append((round(middle[0]+math.cos(angle),2),round(middle[1]+math.sin(angle),2)))
                angles.append(math.degrees(angle))
                angle = angle+init_angle

        return np.array(angles), np.array(positions)

