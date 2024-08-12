#-----------------------------------------------------------------------------------#
#Average Cumulative coverage area per R_C and R_S
#-----------------------------------------------------------------------------------#




import numpy as np
import matplotlib.pyplot as plt
from Obstacle import Obstacle
from Swarm import Swarm, LostAgent
from Constants import Constants
from Behaviour import grid_iteration, percentage_covered
from functions import norm2
import time

# Initialize obstacles as in your initial code
o11 = Obstacle(ld=[10, 40], ru=[12, 42])
o12 = Obstacle(ld=[20, 40], ru=[22, 42])
o13 = Obstacle(ld=[30, 40], ru=[32, 42])
o14 = Obstacle(ld=[40, 40], ru=[42, 42])

o21 = Obstacle(ld=[12, 30], ru=[14, 32])
o22 = Obstacle(ld=[22, 30], ru=[24, 32])
o23 = Obstacle(ld=[32, 30], ru=[34, 32])
o24 = Obstacle(ld=[42, 30], ru=[44, 32])

o31 = Obstacle(ld=[10, 20], ru=[12, 22])
o32 = Obstacle(ld=[20, 20], ru=[22, 22])
o33 = Obstacle(ld=[30, 20], ru=[32, 22])
o34 = Obstacle(ld=[40, 20], ru=[42, 22])

o41 = Obstacle(ld=[12, 10], ru=[14, 12])
o42 = Obstacle(ld=[22, 10], ru=[24, 12])
o43 = Obstacle(ld=[32, 10], ru=[34, 12])
o44 = Obstacle(ld=[42, 10], ru=[44, 12])

obstacles = [o11, o12, o13, o14,
             o21, o22, o23, o24,
             o31, o32, o33, o34,
             o41, o42, o43, o44]

def run_simulation(num_uavs, R_C, R_S):
    Constants.num_UAVS = num_uavs
    Constants.R_C = R_C
    Constants.R_S = R_S
    swarm = Swarm(num_uavs, obstacles)
    if Constants.SearchandRescue:
        lost_agent = LostAgent(Constants.area_width, Constants.area_length)
    history_percentage = [0]
    time_steps = [0]

    iter = 0
    FINAL_CONDITION = True

    while FINAL_CONDITION:  # 95% coverage or max iterations

        iter += 1  # Increment iteration counter
        swarm.neighbors = [[] for _ in range(num_uavs)]  # Initialization of neighbors
        swarm.instantaneous_coverage_map = np.zeros((Constants.area_width, Constants.area_length))  # Initialize instantaneous coverage map

        if Constants.SearchandRescue:
            lost_agent.move()
            LostAgent.update_swarm_behaviour(swarm, lost_agent)

        for agent in range(num_uavs):
            if Constants.SearchandRescue:
                distance_to_lost = norm2(swarm.pos[agent], lost_agent.position)
                if distance_to_lost <= Constants.R_S:
                    swarm.goal[agent] = lost_agent.position

            grid_iteration(0, swarm, agent)

        total_coverage_map, swarm.coverage_percentage = percentage_covered(swarm)

        history_percentage.append(swarm.coverage_percentage)
        time_steps.append(iter * Constants.timestep)

        if Constants.Max_iterations <= iter or Constants.Max_coverage <= swarm.coverage_percentage:
            FINAL_CONDITION = False

    return time_steps, history_percentage

def main():
    num_uavs_list = [3, 4, 5, 6, 7, 8]
    R_C_values = [10, 20, 30]  # Example communication ranges
    R_S_values = [5, 10, 15]   # Example sensor ranges
    num_runs = 5  # Number of runs per UAV configuration

    coverage_results = {}

    for R_C in R_C_values:
        for R_S in R_S_values:
            for num_uavs in num_uavs_list:
                all_runs = []
                for run in range(num_runs):
                    print(f"Running simulation {run+1}/{num_runs} for {num_uavs} UAVs with R_C={R_C} and R_S={R_S}")
                    time_steps, coverage = run_simulation(num_uavs, R_C, R_S)
                    all_runs.append((time_steps, coverage))

                # Average coverage across runs
                max_length = max(len(run[1]) for run in all_runs)
                averaged_coverage = np.zeros(max_length)
                common_time_steps = np.linspace(0, max_length * Constants.timestep, max_length)

                for run in all_runs:
                    current_time_steps, coverage = run
                    coverage_interp = np.interp(common_time_steps, current_time_steps, coverage)
                    averaged_coverage += coverage_interp

                averaged_coverage /= num_runs
                coverage_results[(num_uavs, R_C, R_S)] = (common_time_steps, averaged_coverage)

    # Plotting the results
    num_plots = len(R_C_values) * len(R_S_values)
    plt.figure(figsize=(15, 10))
    plot_num = 1
    for R_C in R_C_values:
        for R_S in R_S_values:
            plt.subplot(len(R_C_values), len(R_S_values), plot_num)
            for num_uavs in num_uavs_list:
                time_steps, averaged_coverage = coverage_results[(num_uavs, R_C, R_S)]
                
                # Find the index where time is >= 2 seconds
                start_index = next(i for i, t in enumerate(time_steps) if t >= 2)
                
                plt.plot(time_steps[start_index:], averaged_coverage[start_index:], label=f'{num_uavs} UAVs')
            
            plt.title(f'R_C={R_C}, R_S={R_S}')
            plt.xlabel('Time (s)')
            plt.ylabel('Cumulative Area Coverage (%)')
            plt.legend()
            plt.grid(True)
            plot_num += 1

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
