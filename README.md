# Agent Bassed Modelling - Escape Panic Simulation with Social Force Model

This repository contains a set of Python scripts designed to simulate the impact of obstacle geometry on escape efficiency, considering both individual and group behaviors. The core function revolves around modeling the movement of agents through environments with various obstacles, calculating evacuation times under different conditions, and analyzing the impact of different group sizes and obstacle configurations.

## Components
#### Functions:
`astar.py`: Path-finding Algorithm, designed to calculate the most efficient path from a starting point to a predefined goal on a grid:
- Construct barriers and obstacles on the grid
- The use of Manhattan distance as a heuristic to estimate the cost from any node on the grid to the goal
- Provides the next directional step from the start location towards the goal for step-by-step navigation.

`force.py`: Calculates different types of forces that influence the movement of agents:
- `self_driven_force`: Calculates the force that drives an agent towards the desired direction.
- `ped_repulsive_force`: Calculates the repulsive force exerted by other nearby agents to prevent collisions and maintain personal space.
- `ped_obstacle_rep_force`: Calculates the repulsive forces between an agent and obstacles.
- `group_vis_force`: Calculates the force related to the agent’s field of vision and the group’s direction.
- `group_att_force`: Calculates an attractive force that acts on an agent if she is far from the centroid of their group.

`people.py`: Simulate the movement and interactions of agents:
- Agents are grouped to simulate social groups moving together.
- Using A-star for path-finding, using a directional matrix to guide agent movement. 
- Calculates and updates agent positions based on a combination of forces.

`gui.py`: Implements a GUI for an escape simulation using Tkinter:
- Initializes a canvas with a gray background and draws a grid.
- Obstacles and boundaries placement, represent agents with ovals, connect group members with lines to represent their interactions.
- Updates timeto track the simulation's progress in seconds.

`main_functions.py`: Simulation and plot functions: 
- Determines the distribution of agents into groups.
- Conducts the simulation either visually via a GUI or simply for data collection.
- Plots evacuation times with confidence intervals.

### Sensitive analysis and experiments:
`Global_Analysis.ipynb`: global sensitive analysis:
- First order sensitivity, considering r1, beta1, beta2 and A.
- Second order sensitivity, considering (beta1, r1), (beta2, r1), (beta1, beta2), (A, r1), (A, beta1), (A, beta2).
- Total sensitivity, considering r1, beta1, beta2 and A.

`local_beta_1.py`: local sensitive analysis for beta1.
`local_beta_2.py`: local sensitive analysis for beta2.
`local_individual.py`: local sensitive analysis for r1.
`local_social.py`: local sensitive analysis for force factor A.

`experiment.ipynb`: Two experiments, namely optimal obstacle searching and group escaping scenarios:
- Considers horizontal panel, vertical panel and squared pillar, determining the best obstacle for individually escaping scenario.
- Splits agents into groups, examining the impact of obstacles.
- Plots heatmaps for both escaping with and without obstacles.


## Installation
Clone this repository to your local machine using:
```
git clone https://github.com/ann0215/ABM.git
```
Ensure you have Python installed, along with the necessary packages: `numpy`, `matplotlib`, `scipy`, and `tkinter` (if GUI functionality is desired).

## Usage
1. Configure Simulation Parameters:
Adjust the parameters in the script according to your scenario requirements. Parameters include the number of pedestrians, group configurations, obstacle placements, etc.

2. Run Simulation:
Execute the simulation script. If GUI is enabled, the simulation will display a window showing the movement of pedestrians. For non-GUI runs, the simulation will process in the background and directly output the results.

3. Analyze Results:
Use the plotting functions to generate visualizations of the evacuation times and analyze the impacts of different configurations.


## Examples
#### Global sensitive analysis example:
To run a first order sensitive analysis:
```
plot_index(si, problem['names'], '1', 'First order sensitivity')
plt.savefig("first_order.png", dpi=150)
```

#### Local sensitive analysis example:
To run a local sensitive analysis for force factor parameter A:
```
python local_social.py
```

#### Experiment example:
To run a simulation for a group splitting and obstacle scenario with a graphical display:
```
r_1 = 0.6
r_2 = 0.35
r_3 = 0.05
N = 15*9
barrier_set = [(0, 0), (27, 17), (26, 8), (22, 7), (23, 10)]
delta_time = 0.005
seed = 41
percent_threshold = 80
A_p=2000
A_o=2000
B=-0.08
beta1=1
beta2=3
time,time_perct = one_simulation(barrier_set, percent_threshold, 
                   r_1, r_2, r_3, A_p, A_o, B, beta1, beta2,
                   N, delta_time, seed, gui_display=True)
```

To generate a plot comparing evacuation times for 80% escaping with and without obstacles:
```
# suppose you already have the evacuation times with obstacles(`time_perct_list_g_noobs`) and evacuation times without obstacles(`time_perct_list_g_noobs`)
x_axis = ["$r_3=0.3$ \n $r_2=0.1$ \n $r_1=0.6$", 
                 "$r_3=0.1$ \n $r_2=0.3$ \n $r_1=0.6$", 
                 "$r_3=0$ \n $r_2=0$ \n $r_1=1$"]
plot_with_two_ci_bar(time_perct_list_g_noobs, time_perct_list_g_obs, x_axis, scenario='for 80% People')
```

## Reference
The pathfinding component of this code is inspired by the A-star algorithm developed by lc6chang, while the graphical interface is based on the gui code[1]. The simulation of escape panic is modeled after the study "Simulating dynamical features of escape panic" by Helbing et al., published in 2000[2]. For the group escape aspect, the group dynamics draw from the research "The walking behaviour of pedestrian social groups" by Moussaïd et al., published in 2010[3].

[1] https://github.com/lc6chang/Social_Force_Model/tree/master?tab=readme-ov-file
[2] Helbing, D., Farkas, I., & Vicsek, T. (2000). Simulating dynamical features of escape panic. Nature, 407(6803), 487-490.
[3] Moussaïd, M., Perozo, N., Garnier, S., Helbing, D., & Theraulaz, G. (2010). The walking behaviour of pedestrian social groups and its impact on crowd dynamics. PloS one, 5(4), e10047.
