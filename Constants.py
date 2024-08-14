#-------------------------------------------------------------#
#Constant.py file, part of the anti-flocking algorithm
#Lars Oerlemans, 4880110
# AE4350 Bio-Inspired Intelligence and Aerospace Applications
#-------------------------------------------------------------#

import numpy as np


class Constants(object):
    
#Mission area:
    area_length =50  # Length of the mission area in meters
    area_width = 50 # Width of the mission area in meters
    Geo_Fence_width = 3 # Geo-fence width
    
        
#UAV constants
#-------------------#
    R_C=10  # Communication range 
    R_S = 5     # Sensor-perception range
    W_MAX = 45      # Max heading angle change in degrees
    num_UAVS= 4    # number of UAVS
    UAV_Velocity=3  #velocity UAV m/s



#Agent dynamics control/behaviour weights
#--------------------------------------------#
    K_O= 0.8    # Obstacle and neighbor avoidance
    K_C= 0.5  # Decentering
    K_S= 0.5    # Selfishness
    K_B= 1   # Boundary control
    K_W= 0.8    # Weight for Control input update

    #Distance/avoidance constants
    
    D_O = R_S    # For obstacle avoidance given an emergency situation
    D_N = 2   # For neighbor avoidance given an emergency situation
    D_C = 2*R_S # For decentering term --> neighbor avoidance threshold whether or not decentering term will be taken into account
    MIN_GOAL_DIST=R_S #make sure agents do not move to same area

    #boundary control weights
    Q_M= 0.6    # Weight for center of mission area
    Q_B= 0.8    # Weight for nearest boundary point

    #UAV dynamics fitness function constants
    RHO= 0.2
    ALPHA= 0.04 # Weight for distance: UAV <--> point
    BETA= 0.01  # Weight for distance: previous_goal <--> point

#Evolutionary Parameter constants
#--------------------------------------------#
    Cost_factor=0.2              #penalty factor per UAV number --> preventing unneccessary increasing the number of UAVs. 


#Simulation constants
#-------------------------------------#
    timestep=0.2
    Obstacle_Value=-40
    Max_coverage = 90   #percentage max coverage
    Max_iterations=100 # max iterations setting a max on run time

    #Simulation/Switch/Plot constants to start certain functions:
    CIRCLE_SENSOR = True            # Draw the Sensor Radius R_S circle in canvas
    CIRCLE_COMMUNICATION = True    # Draw the Communication Radius R_C circle in canvas
    INIT_RANDOM=False              #random agent initialisation
    INIT_CIRCUMFERENCE=True        # circular agent initialisation
    SIMULATE_FAILURES=False        # simulate agent failures
    PLOTS=True
    TRAJECTORY_PLOT=True
    CUMULATIVE_PERCENTAGE = True
    INSTANTANEOUS_PERCENTAGE= True
    COVERAGE_FIRST=False
    MODE = "continuous"             #Coverage map time dependency

    