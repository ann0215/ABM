"""
Explore the impact of the group-attraction parameter (beta2) on the evacuation time of agent with different obstacles.
Run multiple iterations for each beta2 value ranging from 1 to 3.5, measuring the average evacuation times and CI.
Finally the results are visualized.
"""

import importlib
import gui
import people
import astar
import main_functions
importlib.reload(gui)
importlib.reload(people)
importlib.reload(astar)
importlib.reload(main_functions)
from gui import GUI
from people import PeopleList
from main_functions import group_num_calculator,one_simulation, batch_simulation, intense_simulation
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats
import pandas as pd


r_1 = 0.6 # ratio of 1-ped group
r_2 = 0.35 # ratio of 2-ped group
r_3 = 0.05 # ratio of 3-ped group
N = 15*9 # Total pedestrians

barrier_set = [(0, 0), (27, 17), (26, 8), (22,6), (23, 11)]
no_barrier_set = [(0, 0), (27, 17), (26,8)]
delta_time = 0.005
percent_threshold = 80
A_p=2000
A_o=2000
B=-0.08
beta1=1

mean_time_list = []
ci_lower_list = []
ci_upper_list = []

nobarrier_time_list = []
nobarrier_ci_lower_list = []
nobarrier_ci_upper_list = []


beta_2_list = np.linspace(1,3.5,6)


for i in beta_2_list:
    
    result=intense_simulation(barrier_set, percent_threshold, 
                   r_1, r_2, r_3, A_p, A_o, B, beta1, i,
                   N, delta_time, sim_run=12, gui_display=False)
    
    nobarrier_result=intense_simulation(no_barrier_set, percent_threshold, 
                   r_1, r_2, r_3, A_p, A_o, B, beta1, i,
                   N, delta_time, sim_run=12, gui_display=False)
    
    print("finish",i)
    
    mean_time_list.append(result[0])
    ci_lower_list.append(result[1])
    ci_upper_list.append(result[2])

    nobarrier_time_list.append(nobarrier_result[0])
    nobarrier_ci_lower_list.append(nobarrier_result[1])
    nobarrier_ci_upper_list.append(nobarrier_result[2])


df=pd.DataFrame({'beta2':beta_2_list,'mean_time':mean_time_list,'ci_lower':ci_lower_list,'ci_upper':ci_upper_list,
                 'nobarrier_time':nobarrier_time_list,'nobarrier_ci_lower':nobarrier_ci_lower_list,'nobarrier_ci_upper':nobarrier_ci_upper_list})

error_lower = df['mean_time'] - df['ci_lower']
error_upper = df['ci_upper'] - df['mean_time']
errors=[error_lower,error_upper]

no_barrier_error_lower = df['nobarrier_time'] - df['nobarrier_ci_lower']
no_barrier_error_upper = df['nobarrier_ci_upper'] - df['nobarrier_time']
no_barrier_errors=[no_barrier_error_lower,no_barrier_error_upper]

plt.figure(figsize=(10, 5))
plt.errorbar(df['beta2'], df['mean_time'], yerr=errors, fmt='o', capsize=5, capthick=2, color='blue', label='Mean Time with Barrier')
plt.errorbar(df['beta2'], df['nobarrier_time'], yerr=no_barrier_errors, fmt='o', capsize=5, capthick=2, color='red', label='Mean Time without Barrier')

plt.plot(df['beta2'], df['mean_time'], linestyle='-', marker='o', color='deepskyblue')
plt.plot(df['beta2'], df['nobarrier_time'], linestyle='-', marker='o', color='red')

plt.title('Average time via different group-attraction beta2')
plt.xlabel('group-attraction beta2')
plt.ylabel('Average Time')
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
plt.savefig('group-attraction.png', dpi=150)





