#-------------------------------------------------------------#
#Main.py file, part of the anti-flocking algorithm
#Lars Oerlemans, 4880110
# AE4350 Bio-Inspired Intelligence and Aerospace Applications
#-------------------------------------------------------------#

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from Obstacle import Obstacle
from Swarm import Swarm
from Constants import Constants
import time
from Plot import trajectory_patch, plot_simulation_map, draw_obstacles, assign_agent_colors, get_screen_dimensions
from Behaviour import grid_iteration, percentage_covered
from functions import norm2

# Command line arguments processing
# Files and directories creation


# ======================================================
#           SWARM AND OBSTACLES INITIALIZATION
# ======================================================

obs1 = Obstacle(ld=[10,10],ru=[18,17])
obs2 = Obstacle(ld=[32,36],ru=[40,40])

obs3 = Obstacle(ld=[37,6],ru=[43,43])
obs4 = Obstacle(ld=[25,30],ru=[35,40])
obs5 = Obstacle(ld=[7,20],ru=[15,40])

o11 = Obstacle(ld=[10,40],ru=[12,42])
o12 = Obstacle(ld=[20,40],ru=[22,42])
o13 = Obstacle(ld=[30,40],ru=[32,42])
o14 = Obstacle(ld=[40,40],ru=[42,42])

o21 = Obstacle(ld=[12,30],ru=[14,32])
o22 = Obstacle(ld=[22,30],ru=[24,32])
o23 = Obstacle(ld=[32,30],ru=[34,32])
o24 = Obstacle(ld=[42,30],ru=[44,32])

o31 = Obstacle(ld=[10,20],ru=[12,22])
o32 = Obstacle(ld=[20,20],ru=[22,22])
o33 = Obstacle(ld=[30,20],ru=[32,22])
o34 = Obstacle(ld=[40,20],ru=[42,22])

o41 = Obstacle(ld=[12,10],ru=[14,12])
o42 = Obstacle(ld=[22,10],ru=[24,12])
o43 = Obstacle(ld=[32,10],ru=[34,12])
o44 = Obstacle(ld=[42,10],ru=[44,12])

# obstacles = [obs1, obs2]
# obstacles = [obs3, obs4, obs5]
obstacles = [o11,o12,o13,o14,
             o21,o22,o23,o24,
             o31,o32,o33,o34,
             o41,o42,o43,o44]
#obstacles = []

if Constants.MODE=="continuous": Start_Time = time.monotonic()
if Constants.MODE=="unique": Start_Time = 0


swarm = Swarm(Constants.num_UAVS, obstacles)
if Constants.SearchandRescue:
    lost_agent = LostAgent(Constants.area_width, Constants.area_length)
history_x = [[swarm.pos[i][0]] for i in range(Constants.num_UAVS)]
history_y = [[swarm.pos[i][1]] for i in range(Constants.num_UAVS)]
history_percentage = [0]
instant_coverage = [0]
# ======================================================
#             CREATING PLOTS/GRAPH AND OBSTACLES
# ======================================================

fig, (ax_trajectories, ax_inst_graph, ax_cov_graph) = plt.subplots(3, 1, gridspec_kw={'height_ratios': [3, 1, 1]})
fig.tight_layout()

width_in, height_in = get_screen_dimensions() 
fig.set_size_inches(round(width_in)-1,round(height_in)-1)

# Create and init plots
if Constants.TRAJECTORY_PLOT:
    ax_trajectories = plot_simulation_map(ax_trajectories, history_x,history_y)

if Constants.CUMULATIVE_PERCENTAGE:
    ax_cov_graph.set_ylim(0,100)   
    ax_cov_graph.yaxis.tick_right()
    ax_cov_graph.set_title("Total Cumulative Area Coverage (%)")
    ax_cov_graph.set_xlabel("Iterations")
    ax_cov_graph.set_ylabel("Area coverage (%)")
    
if Constants.INSTANTANEOUS_PERCENTAGE:
    #ax_inst_graph.set_ylim(0,100)   
    ax_inst_graph.set_title("Instantaneous Area Coverage (%) for whole UAV Swarm")
    ax_inst_graph.set_xlabel("Iterations")
    ax_inst_graph.set_ylabel("Instantaneous Area Coverage")


draw_obstacles(obstacles,ax_trajectories)
agent_colors = assign_agent_colors()

# ======================================================
#             Running the Simulation
# ======================================================
def run_simulation():
    
    #specially implemented if Evolutionary Learning is running --> then the simulation gets reset everytime it runs. 
    swarm = Swarm(Constants.num_UAVS, obstacles)
    if Constants.SearchandRescue:
        lost_agent = LostAgent(Constants.area_width, Constants.area_length)
    history_x = [[swarm.pos[i][0]] for i in range(Constants.num_UAVS)]
    history_y = [[swarm.pos[i][1]] for i in range(Constants.num_UAVS)]
    history_percentage = [0]
    instant_coverage = [0]

    # Iteration counter
    iter = 0
    FINAL_CONDITION = True

    # MAIN EXECUTION LOOP

    while FINAL_CONDITION: 

        if Constants.SIMULATE_FAILURES and Constants.num_UAVS > 1 and iter > 1:
            # Kill UAV every 50 iterations
            if iter==50:
                # --------- Failures ------------
                Constants.num_UAVS = Constants.num_UAVS-1
                ax_trajectories.scatter(*swarm.pos[Constants.num_UAVS],color='r',marker="x", zorder=2, s=130, linewidth=3)
            if iter==100:
                # --------- New members ------------
                Constants.num_UAVS = Constants.num_UAVS+1
                ax_trajectories.scatter(*swarm.pos[Constants.num_UAVS-1],color='black',marker="o", zorder=2, s=60)

        iter = iter + 1 # Increment iteration counter
        swarm.neighbors = [[] for i in range(Constants.num_UAVS)] # Initialization of neighbors
        swarm.instantaneous_coverage_map = np.zeros((Constants.area_width, Constants.area_length)) # Initialize instantaneous coverage map




        # MAIN LOOP 2
        
        for agent in range(Constants.num_UAVS):
            
            #Grid iteration function
            grid_iteration(Start_Time,swarm,agent)

            # ===================================================
            #                 PLOTTING GRAPHS
            # ===================================================
            history_x[agent].append(swarm.pos[agent][0])
            history_y[agent].append(swarm.pos[agent][1])

            # Trajectory drawing
            if Constants.TRAJECTORY_PLOT:
                ax_trajectories.add_patch(trajectory_patch(history_x,history_y,agent,agent_colors))

                # Add sensor and communication radius circles
                if Constants.CIRCLE_SENSOR:
                    ax_trajectories.add_patch(Circle(swarm.pos[agent],Constants.R_S,edgecolor="cornflowerblue",fill=False,linestyle="--"))
                if Constants.CIRCLE_COMMUNICATION:
                    ax_trajectories.add_patch(Circle(swarm.pos[agent],Constants.R_C,edgecolor="lavender",fill=False,linestyle="--"))


        # COMPUTE OVERALL COVERAGE PERCENTAGE 
        total_coverage_map, swarm.coverage_percentage = percentage_covered(swarm)

        # COMPUTE INSTANTANEOUS COVERAGE
        instant_coverage_percentage = (np.count_nonzero(swarm.instantaneous_coverage_map==1)/(Constants.area_width*Constants.area_length))*100
        instant_coverage.append(instant_coverage_percentage)

        # PLOT COVERAGE PERCENTAGE GRAPH
        if Constants.CUMULATIVE_PERCENTAGE:
            history_percentage.append(swarm.coverage_percentage)
            if Constants.PLOTS:
                ax_cov_graph.plot(history_percentage, color="b")

                #CURRENT COVERAGE PERCENTAGE
                ax_cov_graph.annotate(" "+str(round(swarm.coverage_percentage,2))+"% ",
                    xy=(0.86,0.9), xycoords='axes fraction',
                    size=14,
                    bbox=dict(boxstyle="round", fc=(0.5, 0.8, 1.0), ec="none"))

        # PLOT INSTANTANEOUS COVERAGE (%)
        if Constants.INSTANTANEOUS_PERCENTAGE and Constants.PLOTS:
            ax_inst_graph.plot(instant_coverage, color="b")

            ax_inst_graph.annotate(" "+str(round(instant_coverage_percentage,2))+"% ",
                xy=(0.86,0.1), xycoords='axes fraction',
                size=14,
                bbox=dict(boxstyle="round", fc=(0.5, 0.8, 1.0), ec="none"))
        
        # Update canvas with new changes
        if Constants.PLOTS:
            fig.canvas.draw_idle()
            plt.pause(0.01)

        # FINAL CONDITION to exit loop
        if Constants.Max_coverage <= swarm.coverage_percentage or Constants.Max_iterations <iter:
            FINAL_CONDITION = False


#------------------------------------------#
#           Results                        #
#------------------------------------------#

    exec_time = round(time.monotonic() - Start_Time, 2)
    total_iter = iter
    total_cov_area = round(swarm.coverage_percentage, 2)
    average_inst_cov_area = round(np.mean(instant_coverage), 2)
    mission_time = iter * Constants.timestep

    return exec_time, total_iter, total_cov_area, average_inst_cov_area, mission_time

# Arrays to hold results
exec_times = []
total_iters = []
total_cov_areas = []
average_inst_cov_areas = []
mission_times = []

# Run simulation i times
for i in range(1):
    print("Iteration:",i)
    results = run_simulation()
    exec_times.append(results[0])
    total_iters.append(results[1])
    total_cov_areas.append(results[2])
    average_inst_cov_areas.append(results[3])
    mission_times.append(results[4])

    # Reinitialize the Swarm and LostAgent before each simulation
    swarm = Swarm(Constants.num_UAVS, obstacles)
    if Constants.SearchandRescue:
        lost_agent = LostAgent(Constants.area_width, Constants.area_length)
    history_x = [[swarm.pos[i][0]] for i in range(Constants.num_UAVS)]
    history_y = [[swarm.pos[i][1]] for i in range(Constants.num_UAVS)]
    history_percentage = [0]
    instant_coverage = [0]

    # Reset the start time for each run
    if Constants.MODE == "continuous":
        Start_Time = time.monotonic()
    else:
        Start_Time = 0

# Calculate averages
avg_exec_time = np.mean(exec_times)
avg_total_iter = np.mean(total_iters)
avg_total_cov_area = np.mean(total_cov_areas)
avg_average_inst_cov_area = np.mean(average_inst_cov_areas)
avg_mission_time = np.mean(mission_times)

# Print results
print(f"Average Execution Time: {avg_exec_time}")
print(f"Average Total Iterations: {avg_total_iter}")
print(f"Average Total Coverage Area: {avg_total_cov_area}")
print(f"Average Instantaneous Coverage Area: {avg_average_inst_cov_area}")
print(f"Average Mission Time: {avg_mission_time}")

