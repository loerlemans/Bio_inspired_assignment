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

def run_simulation(num_uavs):
    Constants.num_UAVS = num_uavs
    swarm = Swarm(num_uavs, obstacles)
    if Constants.SearchandRescue:
        lost_agent = LostAgent(Constants.area_width, Constants.area_length)
    history_percentage = [0]
    instant_coverage = []

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
        instant_coverage.append(np.sum(swarm.instantaneous_coverage_map) / (Constants.area_width * Constants.area_length) * 100)

        if Constants.Max_iterations <= iter or Constants.Max_coverage <= swarm.coverage_percentage:
            FINAL_CONDITION = False

    return history_percentage, instant_coverage

def main():
    num_uavs_list = [3, 4, 5, 6, 7, 8]
    num_runs = 5 # Number of runs per UAV configuration
    cumulative_coverage_results = {}
    instant_coverage_results = {}

    for num_uavs in num_uavs_list:
        all_cumulative_runs = []
        all_instant_runs = []
        for run in range(num_runs):
            print(f"Running simulation {run+1}/{num_runs} for {num_uavs} UAVs")
            cumulative_coverage, instant_coverage = run_simulation(num_uavs)
            all_cumulative_runs.append(cumulative_coverage)
            all_instant_runs.append(instant_coverage)

        # Average cumulative coverage across runs
        max_length_cumulative = max(len(run) for run in all_cumulative_runs)
        averaged_cumulative_coverage = np.zeros(max_length_cumulative)

        for run in all_cumulative_runs:
            averaged_cumulative_coverage[:len(run)] += run

        averaged_cumulative_coverage /= num_runs
        cumulative_coverage_results[num_uavs] = averaged_cumulative_coverage

        # Average instantaneous coverage across runs
        all_instant_coverage = np.array([np.mean(run) for run in all_instant_runs])
        average_instant_coverage = np.mean(all_instant_coverage)
        instant_coverage_results[num_uavs] = average_instant_coverage

    # Plotting the instantaneous coverage results
    uavs = list(instant_coverage_results.keys())
    instant_coverage = list(instant_coverage_results.values())

    plt.figure(figsize=(10, 6))
    plt.plot(uavs, instant_coverage, 'o-', label='Instantaneous Coverage')
    plt.xlabel('Number of UAVs')
    plt.ylabel('Instantaneous Area Coverage (%)')
    plt.title('Instantaneous Area Coverage vs Number of UAVs')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
