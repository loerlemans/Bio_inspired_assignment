import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import random
from copy import deepcopy
from Constants import Constants
from Swarm import Swarm
from Obstacle import Obstacle
from Main import run_simulation

class EvolutionaryParameters:
    def __init__(self, num_UAVs, R_C, R_S, K_O, K_C, K_S):
        self.num_UAVs = num_UAVs
        self.R_C = R_C
        self.R_S = R_S
        self.K_O = K_O
        self.K_C = K_C
        self.K_S = K_S
     # Bounds for each parameter
    bounds = {
        'num_UAVs': (1, 6),
        'R_C': (5, 15),
        'R_S': (2, 10),
        'K_O': (0.1, 1),
        'K_C': (0.1, 1),
        'K_S': (0.1, 1)
    }

def initialize_population(size=5):
    population = []
    for _ in range(size):
        params = EvolutionaryParameters(
            num_UAVs=random.randint(*EvolutionaryParameters.bounds['num_UAVs']),
            R_C=random.uniform(*EvolutionaryParameters.bounds['R_C']),
            R_S=random.uniform(*EvolutionaryParameters.bounds['R_S']),
            K_O=random.uniform(*EvolutionaryParameters.bounds['K_O']),
            K_C=random.uniform(*EvolutionaryParameters.bounds['K_C']),
            K_S=random.uniform(*EvolutionaryParameters.bounds['K_S'])
        )
        population.append(params)
    return population

def Run_EL(params):
    Constants.num_UAVS = params.num_UAVs
    Constants.R_C = params.R_C
    Constants.R_S = params.R_S
    Constants.K_O = params.K_O
    Constants.K_C = params.K_C
    Constants.K_S = params.K_S

    results = run_simulation()
    total_cov_area = results[2]
    mission_times = results[4]
    print("Total Coverage Area:", total_cov_area)
    print("Mission Time:", mission_times)
    print("Number of UAVS",Constants.num_UAVS)
    fitness = ((total_cov_area * Constants.weight_total_cov_area) /
               (mission_times * Constants.Weight_missiontimes)) - (Constants.num_UAVS**2) * Constants.Cost_factor
    return fitness

def select(population, fitness_scores, tournament_size=4):
    selected = []
    for _ in range(len(population)):
        competitors = random.sample(list(zip(population, fitness_scores)), tournament_size)
        winner = max(competitors, key=lambda x: x[1])
        selected.append(deepcopy(winner[0]))
    return selected

def crossover(parent1, parent2):
    child = deepcopy(parent1)
    child.num_UAVs = random.choice([parent1.num_UAVs, parent2.num_UAVs])
    if random.random() < 0.5:
        child.R_C = parent2.R_C
    if random.random() < 0.5:
        child.R_S = parent2.R_S
    if random.random() < 0.5:
        child.K_O = parent2.K_O
    if random.random() < 0.5:
        child.K_C = parent2.K_C
    if random.random() < 0.5:
        child.K_S = parent2.K_S
    return child

def mutate(individual, average_fitness, individual_fitness, base_rate=0.1):
    mutation_rate = base_rate * (5 if individual_fitness < average_fitness else 0.5)
    if random.random() < mutation_rate:
        individual.num_UAVs = max(1, individual.num_UAVs + random.choice([-1, 1]))
        individual.num_UAVs = max(min(individual.num_UAVs, EvolutionaryParameters.bounds['num_UAVs'][1]), EvolutionaryParameters.bounds['num_UAVs'][0])
    if random.random() < mutation_rate:
        individual.R_C += random.uniform(-2, 2)
        individual.R_C = max(min(individual.R_C, EvolutionaryParameters.bounds['R_C'][1]), EvolutionaryParameters.bounds['R_C'][0])
    if random.random() < mutation_rate:
        individual.R_S += random.uniform(-1, 1)
        individual.R_S = max(min(individual.R_S, EvolutionaryParameters.bounds['R_S'][1]), EvolutionaryParameters.bounds['R_S'][0])
    if random.random() < mutation_rate:
        individual.K_O += random.uniform(-0.1, 0.1)
        individual.K_O = max(min(individual.K_O, EvolutionaryParameters.bounds['K_O'][1]), EvolutionaryParameters.bounds['K_O'][0])
    if random.random() < mutation_rate:
        individual.K_C += random.uniform(-0.1, 0.1)
        individual.K_C = max(min(individual.K_C, EvolutionaryParameters.bounds['K_C'][1]), EvolutionaryParameters.bounds['K_C'][0])
    if random.random() < mutation_rate:
        individual.K_S += random.uniform(-0.1, 0.1)
        individual.K_S = max(min(individual.K_S, EvolutionaryParameters.bounds['K_S'][1]), EvolutionaryParameters.bounds['K_S'][0])

def run_evolutionary_algorithm(generations=20, population_size=5):
    population = initialize_population(population_size)
    best_fitness_per_generation = []
    for generation in range(generations):
        fitness_scores = [Run_EL(individual) for individual in population]
        average_fitness = sum(fitness_scores) / len(fitness_scores)
        best_fitness_per_generation.append(max(fitness_scores))

        print(f"Generation {generation + 1}: Best Fitness: {max(fitness_scores)}")

        # Print the parameters for each individual
        for individual in population:
            print(f"Individual Parameters - Num_UAVS:{individual.num_UAVs}, R_C: {individual.R_C}, R_S: {individual.R_S}, K_O: {individual.K_O}, K_C: {individual.K_C}, K_S: {individual.K_S}")

        selected = select(population, fitness_scores)
        offspring = [crossover(selected[i], selected[(i + 1) % len(selected)]) for i in range(len(selected))]

        for idx, individual in enumerate(offspring):
            mutate(individual, average_fitness, fitness_scores[idx])

        population = offspring

    return best_fitness_per_generation

# Run the evolutionary algorithm and plot the results
fitness_data = run_evolutionary_algorithm()
plt.figure(figsize=(10, 5))
plt.plot(fitness_data, marker='o', linestyle='-', color='b')
plt.title("Best Fitness Value Per Generation")
plt.xlabel("Generation")
plt.ylabel("Best Fitness Value")
plt.grid(True)
plt.show()
