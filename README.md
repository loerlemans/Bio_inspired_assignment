# Bio_inspired_assignment
Evolutionary Learning Applied to a Distributed Anti-Flocking Algorithm for Area Coverage. An assignment for the course AE4350; bio-inspired intelligence for Aerospace applications. The proposed anti-flocking algorithm is based on simulated flocking behaviour proposed in a paper by Reynolds and the anti-flocking algorithm described in a paper by Zhang and a paper by Miñano. 


The Code is setup accordingly:

Main.py --> this is the main file that is runned for the complete simulation.     
Behaviour.py --> This file consist of the main part of the Anti-Flocking Algorithm methodology (each of the rules and the agent dynamics).    
swarm.py --> This file consist of the agent swarming class where each agent characteristic gets initialised.    
functions.py --> This file consist of all the mathematic functions used in the algorithm/code.    
Plot.py --> This file consist of the plotting code/logic.     
Constants.py --> This file consist of all the predetermined constants used in the algorithm.     
Obstacle.py --> This file consist of only the obstacle logic.     

EvolutionaryLearning_final.py --> This file is runned for the EA algorithm. This calls the main.py and other files to run the algorithm and implement the EA algorithm in the code. Highly adaptable file where different parameters could be called to perform a EA on. For now 6 parameters are included. (NUM_UAV, RC,RS,KO,KC,KS) This file also provide a plot with the fitness function over the generations.

SA_Main_outliers.py --> This file is an updated main.py file where different outliers are analysed as part of the sensitivity analysis.
SA_Main_obstacles.py --> This file is an update main.py file where different obstacle configurations are analysed as part of the sensitivity analysis.

Res_RC_2.py --> This file consist of on of the many developed codes to show the result of the self-organised swarm. This file provide plots for different RC,RS and UAV number and their mission time and their area coverage performance (as shown in the report)



References:    
Xiangke Wang Mengge Zhang Jie Li. Distributed anti-flocking method for area coverage of mul-
tiple unmanned aerial vehicles. July 2020. URL: ÿlkhttps : / / ieeexplore . ieee . org / stamp /
stamp.jsp?tp=%5C&arnumber=9188393.    

Miguel Espinosa Mi nano. Self-awareness in a drone swarm for the complete coverage of its
surroundings. July 2021. URL: ÿlkhttps://drive.google.com/file/d/1A_35Dyw4siyMmg9ASH
MDx_kQeNBcnG2t/view.      

Craig W. Reynolds. Flocks, herds and schools: A distributed behavioral model. July 1987. URL:
ÿlkhttps://dl.acm.org/doi/pdf/10.1145/37402.37406.    
