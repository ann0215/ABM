"""
Evaluate the evacuation times of agents from an area with and without obstacles.
Vary the interaction strength between agents and between agents and obstacles. 
Each set of parameters is tested multiple times. Finally the results are visualized.
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
# approximate 9 hours to run

r_1 = 0.6 # ratio of 1-ped group
r_2 = 0.35 # ratio of 2-ped group
r_3 = 0.05 # ratio of 3-ped group
N = 15*9 # Total pedestrians
barrier_set = [(0, 0), (27, 17), (26, 8), (22,6), (23, 11)]
no_barrier_set = [(0, 0), (27, 17), (26, 8)]
delta_time = 0.005
seed = 41
percent_threshold = 80
B=-0.08
beta1=1
beta2=3

mean_time_list = []
ci_lower_list = []
ci_upper_list = []

nobarrier_time_list = []
nobarrier_ci_lower_list = []
nobarrier_ci_upper_list = []


A_list = np.linspace(1000,2000,5)


for i in A_list:
    
    result=intense_simulation(barrier_set, percent_threshold, 
                   r_1, r_2, r_3, i, i, B, beta1, beta2,
                   N, delta_time, sim_run=10, gui_display=False)
    
    nobarrier_result=intense_simulation(no_barrier_set, percent_threshold, 
                   r_1, r_2, r_3, i, i, B, beta1, beta2,
                   N, delta_time, sim_run=10, gui_display=False)
    
    print("finish",i)
    
    mean_time_list.append(result[0])
    ci_lower_list.append(result[1])
    ci_upper_list.append(result[2])

    nobarrier_time_list.append(nobarrier_result[0])
    nobarrier_ci_lower_list.append(nobarrier_result[1])
    nobarrier_ci_upper_list.append(nobarrier_result[2])

df=pd.DataFrame({'A_factor':A_list,'mean_time':mean_time_list,'ci_lower':ci_lower_list,'ci_upper':ci_upper_list,
                 'nob_mean_time':nobarrier_time_list,'nob_ci_lower':nobarrier_ci_lower_list,'nob_ci_upper':nobarrier_ci_upper_list})

error_lower = df['mean_time'] - df['ci_lower']
error_upper = df['ci_upper'] - df['mean_time']
errors=[error_lower,error_upper]

nob_error_lower = df['nob_mean_time'] - df['nob_ci_lower']
nob_error_upper = df['nob_ci_upper'] - df['nob_mean_time']
nob_errors=[nob_error_lower,nob_error_upper]

plt.figure(figsize=(10, 5))
plt.errorbar(df['A_factor'], df['mean_time'], yerr=errors, fmt='o', capsize=5, capthick=2, color='blue', label='Mean Time with Barrier')
plt.errorbar(df['A_factor'], df['nob_mean_time'], yerr=nob_errors, fmt='o', capsize=5, capthick=2, color='red', label='Mean Time without Barrier')

plt.plot(df['A_factor'], df['mean_time'], linestyle='-', marker='o', color='deepskyblue')
plt.plot(df['A_factor'], df['nob_mean_time'], linestyle='-', marker='o', color='red')

plt.title('Average time via different pep-pep-obstacle factor')
plt.xlabel('pep-pep/obstacle A factor')
plt.ylabel('Average Time')
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
plt.savefig('pep-pep-obstacle.png', dpi=150)

