#-------------------------------------------------------------#
#Behaviour.py file, part of the anti-flocking algorithm
#Lars Oerlemans, 4880110
# AE4350 Bio-Inspired Intelligence and Aerospace Applications
#-------------------------------------------------------------#


#This file consist of all the anti-flocking behaviour rules to satisfy the following main rules:
# 1) Collision avoidance
# 2) Decentering
# 3) Selfishness

import time
import numpy as np
import math
from Constants import Constants
from functions import norm2, s, unitary_vector, outside_area, angle_between

#initialise the agent
def init(swarm,agent):
    swarm.prev_goal[agent]=swarm.goal[agent] # save previous goal
    swarm.vel_obs[agent]=np.zeros((1,2))       #set obstacle velocity to zero

#function to get neighbors
def get_neighbors(agent,swarm):
    for agent2 in range(agent+1,Constants.num_UAVS):
        inter_dist = norm2(swarm.pos[agent],swarm.pos[agent2])
        if inter_dist<Constants.R_C:
            swarm.neighbors[agent].append(agent2)
            swarm.neighbors[agent2].append(agent)

            # if neighbor is within "danger zone", update obstacle avoidance velocity for both agents
            swarm.vel_obs[agent] += s(inter_dist,Constants.D_O)*unitary_vector(swarm.pos[agent2],swarm.pos[agent])
            swarm.vel_obs[agent2] += s(inter_dist,Constants.D_O)*unitary_vector(swarm.pos[agent],swarm.pos[agent2])
            
        

#1) Collission avoidance rule
def collision_avoidance(swarm,agent):    
    for neighbor in swarm.neighbors[agent]:
        inter_dist = norm2(swarm.pos[agent], swarm.pos[neighbor])
        if inter_dist < Constants.D_O:  # Only consider if within obstacle distance
            direction = unitary_vector(swarm.pos[neighbor], swarm.pos[agent])
            swarm.vel_obs[agent] += s(inter_dist, Constants.D_N) * direction
            

#2) Decentering rule
def decentering(swarm,agent):
    num_neighbors = len(swarm.neighbors[agent])

    if num_neighbors >0:
        mean=np.array(swarm.pos[agent])
        for neighbors in swarm.neighbors[agent]:
            mean[0] += swarm.pos[neighbors][0]
            mean[1] += swarm.pos[neighbors][1]

            #sharing of the coverage maps by the UAVS --> local map sharing
            new = np.maximum(swarm.coverage_map[agent],swarm.coverage_map[neighbors])
            swarm.coverage_map[agent] = new
            swarm.coverage_map[neighbors] = new
        mean = mean/(num_neighbors+1)

        #update decentering velocity of the agent
        swarm.vel_dec[agent]=s(norm2(mean,swarm.pos[agent]),Constants.D_C)*unitary_vector(mean,swarm.pos[agent])
    else:
        swarm.vel_dec[agent]= np.zeros((2))

#3) selfishness rule
#if there is no obstacle or no other agent is near then the drone moves with 
# a unitary vector towards the agents goal
def selfish(swarm,agent):
    swarm.vel_sel[agent]=unitary_vector(swarm.pos[agent],swarm.goal[agent])



#Benefit/fitness function to create a target grid selection mode for the UAV 
#function based on papers formula 12
def fitness_function(swarm,agent,Start_Time,point,x,y,dist_to_point):
    if Constants.MODE=="continuous": # surveillance mode (with time)
        dt=time.monotonic()-Start_Time-swarm.coverage_map[agent][x][y]
        # distance to previous goal
        dist_to_prev_goal= norm2(np.array([x,y]),swarm.prev_goal[agent])
        #exponent
        exponent= -Constants.ALPHA*dist_to_point-Constants.BETA*dist_to_prev_goal
        # final fitness calculation for point (x,y)
        fitness = dt*(Constants.RHO+(1-Constants.RHO)*math.exp(exponent))
        #print(f"Agent {agent} - Point ({x}, {y}) Fitness: {fitness}")
        return fitness
    
    if Constants.MODE=="unique": # no time consideration
        if swarm.coverage_map[agent][x][y] == 1:
            return 0
        else:
            if dist_to_point > 2*Constants.R_S:
                return 1
            elif dist_to_point > 1.5*Constants.R_S:
                return 2
            elif dist_to_point < 1.5*Constants.R_S:
                return 3
    return 0

#boundary function
def boundary(swarm,agent):  
    agent_pos = swarm.pos[agent]    # Get the position of the current agent

    if outside_area(*swarm.pos[agent]):
        p_m = np.array((Constants.area_width/2,Constants.area_length/2))
        # Nearest boundary point calculations
        x_bound = 0 if agent_pos[0] < Constants.area_width / 2 else Constants.area_width
        y_bound = 0 if agent_pos[1] < Constants.area_length / 2 else Constants.area_length
        p_b = np.array([x_bound, y_bound])
        swarm.vel_bou[agent] = (Constants.Q_M*unitary_vector(agent_pos,p_m)) - (Constants.Q_B*unitary_vector(agent_pos,p_b))




#----------------Big iterationloop per agent over all the gridpoints---------#
#"Loop over all the grid points for each agent and implement all the above rules/functions and
#"update the speed of the UAVS etc. 
#"Main components: 
#"- each rule
#"- Coverage map
#"- Target Grid generation
#"- Final velocity, position"


def grid_iteration(Start_Time, swarm, agent):

    init(swarm,agent)
    get_neighbors(agent,swarm)


     # LOCAL VARIABLES
    max_fitness = 0 # Variable for storing the max_fitness
    best_angle = 180 # Variable for storing the best angle for choosing optimum goal cell

    for x in range(0,Constants.area_width):
        for y in range(0,Constants.area_length):
            
            point = np.array((x,y)) # grid cell under consideration in current iteration
            dist_to_point = norm2(swarm.pos[agent],point) # distance agent <--> grid_cell

            #additional obstacle avoidance
            # Obstacles are marked with OBSTACLE_VALUE in coverage map
            # If obstacles are inside D_0 radius, repulsion
            if swarm.coverage_map[agent][x][y] ==Constants.Obstacle_Value:
                swarm.vel_obs[agent] += (s(dist_to_point,Constants.D_O)* unitary_vector(point,swarm.pos[agent]))


        #update coverage map
            # If inside not an obstacle and is inside the sensor range, update timestamp
            elif dist_to_point < Constants.R_S: 
                if Constants.MODE=="continuous": swarm.coverage_map[agent][x][y] = time.monotonic()-Start_Time
                if Constants.MODE=="unique": swarm.coverage_map[agent][x][y] = 1
                if swarm.instantaneous_coverage_map[x][y] == 0:
                    swarm.instantaneous_coverage_map[x][y] = 1


        #----------Target grid selection fuction--------
        #If there is no obstacle or UAV in radius --> target grid selection with 
        # the use of fitness function,  # that are:  R_S < cells < 2*R_S 
            elif (swarm.coverage_map[agent][x][y] != Constants.Obstacle_Value) and (not outside_area(x,y)):
                
                # Compute closest neighbor to this point
                closest = True
                for neighbors in swarm.neighbors[agent]:
                    dist_to_neighbors = norm2(point,swarm.pos[neighbors])
                    dist_to_neig_goal = norm2(point,swarm.goal[neighbors])

                    #to ensure more smoother path skip point if neighbor is closer than agent
                    #to the point or if the point is too close to neighbor goal. 
                    #this ensures that no 2 agents move towards the same uncovered area. 
                    if dist_to_neighbors<dist_to_point or dist_to_neig_goal < Constants.MIN_GOAL_DIST:
                        closest = False


                #implementation of fitness function Equation 2.11 of own report Included the scenario that similar fitness function values are calculated 
                if closest:
                    fitness=fitness_function(swarm,agent,Start_Time,point,x,y,dist_to_point)

                    if fitness > max_fitness:
                        max_fitness=fitness
                        swarm.goal[agent]=point # set point as goal /update
                        vel_goal= unitary_vector(swarm.pos[agent],point)
                        best_angle=angle_between(swarm.vel_actual[agent],vel_goal)

                        # if fitness== max_fitness and fitness!=0. Tolerance is set to 0.01. 
                    elif abs(max_fitness-fitness)<0.010: 
                        vel_goal=unitary_vector(swarm.pos[agent],point)

                        # Compute angle between actual velocity and possible goal
                        angle = angle_between(vel_goal,swarm.vel_actual[agent])

                        # Update goal if new angle is better.
                        if angle < best_angle:
                            best_angle = angle
                            max_fitness = fitness
                            swarm.goal[agent] = point # Update goal with point (x,y)

    
    #--------all function in the loop--------------#
    decentering(swarm,agent)
    selfish(swarm,agent)
    boundary(swarm,agent)
    collision_avoidance(swarm,agent)


    #-------------Agent dynamics----------------------
    #final velocity all  rules ->Eta_total (Collision avoidance+decentering+selfish, boundary avoidance)
    swarm.vel_desired[agent]=Constants.K_O*swarm.vel_obs[agent] + Constants.K_C*swarm.vel_dec[agent] \
                            + Constants.K_S*swarm.vel_sel[agent] + Constants.K_B*swarm.vel_bou[agent]\
                            
    #compute difference angle between desired velocity and actual direction
    sign_result = math.copysign(1,np.cross(np.append(swarm.vel_actual[agent],0),np.append(swarm.vel_desired[agent],0))[2])
    swarm.diff_angle[agent] = angle_between(swarm.vel_desired[agent],swarm.vel_actual[agent])*sign_result


    #update control input i.e. the changing rate of the heading angle
    if swarm.diff_angle[agent]>=0:
        swarm.control_input[agent] = min(Constants.W_MAX,Constants.K_W*swarm.diff_angle[agent])
    else:
        swarm.control_input[agent] = max(-Constants.W_MAX,Constants.K_W*swarm.diff_angle[agent])


    #update the position/heading angle of agent (kinematic laws)
    swarm.heading_angle[agent] = swarm.heading_angle[agent] + Constants.UAV_Velocity*Constants.timestep*swarm.control_input[agent]

    # Update the actual velocity once the heading angle is updated
    swarm.vel_actual[agent] = np.array([math.cos(math.radians(swarm.heading_angle[agent])),math.sin(math.radians(swarm.heading_angle[agent]))]) 

    # Agent Position determination
    swarm.pos[agent][0] = swarm.pos[agent][0] + Constants.UAV_Velocity*Constants.timestep*math.cos(math.radians(swarm.heading_angle[agent]))
    swarm.pos[agent][1] = swarm.pos[agent][1] + Constants.UAV_Velocity*Constants.timestep*math.sin(math.radians(swarm.heading_angle[agent]))



#---------------------------------------------------------------------------------------------#
def percentage_covered(swarm):
    area = Constants.area_width * Constants.area_length
    if Constants.num_UAVS >= 2:
        aux_max = np.maximum(swarm.coverage_map[0],swarm.coverage_map[1])
        for i in range(2,Constants.num_UAVS): aux_max = np.maximum(aux_max,swarm.coverage_map[i])
    else:
        aux_max = swarm.coverage_map[0]

    if Constants.COVERAGE_FIRST:
        aux_max = swarm.coverage_map[0]
    return aux_max, (np.count_nonzero(aux_max)/area)*100
