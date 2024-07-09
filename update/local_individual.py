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
from main_functions import group_num_calculator,one_simulation, batch_simulation
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats
import pandas as pd


#r_1 = 0.6 # ratio of 1-ped group
#r_2 = 0.35 # ratio of 2-ped group
#r_3 = 0.05 # ratio of 3-ped group

N = 15*9 # Total pedestrians
barrier_set = [(0, 0), (27, 17), (26, 8), (22,6), (23, 11)]
no_barrier_set = [(0, 0), (27, 17), (26,8)] 
delta_time = 0.005
percent_threshold = 80
A_p=2000
A_o=2000
B=-0.08
beta1=1
beta2=3

mean_time_list = []
ci_lower_list = []
ci_upper_list = []

nobarrier_time_list = []
nobarrier_ci_lower_list = []
nobarrier_ci_upper_list = []


r1_list = np.linspace(0.4,1,13)

for i in r1_list:
    
    result=batch_simulation(barrier_set, percent_threshold,
                    i, A_p, A_o, B, beta1, beta2,
                    N, delta_time,  sim_run=15, gui_display=False)
    
    nobarrier_result=batch_simulation(no_barrier_set, percent_threshold,
                    i, A_p, A_o, B, beta1, beta2,
                    N, delta_time,  sim_run=15, gui_display=False)
    
    print("finish",i) 
    
    mean_time_list.append(result[0])
    ci_lower_list.append(result[1])
    ci_upper_list.append(result[2])

    nobarrier_time_list.append(nobarrier_result[0])
    nobarrier_ci_lower_list.append(nobarrier_result[1])
    nobarrier_ci_upper_list.append(nobarrier_result[2])


df=pd.DataFrame({'r1':r1_list,'mean_time':mean_time_list,'ci_lower':ci_lower_list,'ci_upper':ci_upper_list,
                 'nobarrier_time':nobarrier_time_list,'nobarrier_ci_lower':nobarrier_ci_lower_list,'nobarrier_ci_upper':nobarrier_ci_upper_list})

error_lower = df['mean_time'] - df['ci_lower']
error_upper = df['ci_upper'] - df['mean_time']
errors=[error_lower,error_upper]

nobarrier_error_lower = df['nobarrier_time'] - df['nobarrier_ci_lower']
nobarrier_error_upper = df['nobarrier_ci_upper'] - df['nobarrier_time']
nobarrier_errors=[nobarrier_error_lower,nobarrier_error_upper]

plt.figure(figsize=(10, 5))
plt.errorbar(df['r1'], df['mean_time'], yerr=errors, fmt='o', capsize=5, capthick=2, color='blue', label='Mean Time with CI')
plt.errorbar(df['r1'], df['nobarrier_time'], yerr=nobarrier_errors, fmt='o', capsize=5, capthick=2, color='red', label='Mean Time without Barrier')

plt.plot(df['r1'], df['mean_time'], linestyle='-', marker='o', color='deepskyblue')
plt.plot(df['r1'], df['nobarrier_time'], linestyle='-', marker='o', color='red')

plt.title(' Average time by individual ratio of 1-ped group')
plt.xlabel('Individual ratio of 1-ped')
plt.ylabel('Average Time')
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
plt.savefig('indiviudal_ratio_15.png')
plt.show()

