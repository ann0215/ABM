# Agent Bassed Modelling - Escape Panic Simulation with Social Force Model

This repository contains a set of Python scripts designed to simulate the impact of obstacle geometry on escape efficiency, considering both individual and group behaviors. The core function revolves around modeling the movement of agents through environments with various obstacles, calculating evacuation times under different conditions, and analyzing the impact of different group sizes and obstacle configurations.

## Components

`astar`: Path-finding Algorithm, designed to calculate the most efficient path from a starting point to a predefined goal on a grid.
- Construct barriers and obstacles on the grid
- The use of Manhattan distance as a heuristic to estimate the cost from any node on the grid to the goal
- Provides the next directional step from the start location towards the goal for step-by-step navigation.

`force`: Calculates different types of forces that influence the movement of agents:
- `self_driven_force`: Calculates the force that drives an agent towards the desired direction.
- `ped_repulsive_force`: Calculates the repulsive force exerted by other nearby agents to prevent collisions and maintain personal space.
- `ped_obstacle_rep_force`: Calculates the repulsive forces between an agent and obstacles.
- `group_vis_force`: Calculates the force related to the agent’s field of vision and the group’s direction.
- `group_att_force`: Calculates an attractive force that acts on an agent if she is far from the centroid of their group.

`people`: Simulate the movement and interactions of agents:
- Agents are grouped to simulate social groups moving together.
- Using A-star for path-finding, using a directional matrix to guide agent movement. 
- Calculates and updates agent positions based on a combination of forces.

`gui`: Implements a GUI for an escape simulation using Tkinter.
- Initializes a canvas with a gray background and draws a grid.
- Obstacles and boundaries placement, represent agents with ovals, connect group members with lines to represent their interactions.
- Updates timeto track the simulation's progress in seconds.

`simulator`: Simulation and plot functions: 
- Determines the distribution of agents into groups.
- Conducts the simulation either visually via a GUI or simply for data collection.
- Plots evacuation times with confidence intervals.



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
