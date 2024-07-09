'''
Simulation and plot functions.

Key Features:
1. Determines the distribution of agents into groups.
2. Conducts the simulation either visually via a GUI or simply for data collection.
3. Plots evacuation times with confidence intervals.
'''
import numpy as np
from gui import GUI
from people import PeopleList, People
import scipy.stats as stats
import matplotlib.pyplot as plt


def group_num_calculator(N, r1, r2, r3):
    """Calculation of the number of groups according to the corresponding ratio (pederstains ratio) """
    """N: Total population; r_i: ratio of i-peds group"""
    """return: the number of each group"""
    group2 = int(N * r2) 
    group3 = int(N * r3)
    
    num2 = int(group2 / 2)
    num3 = int(group3 / 3)
    num1 = N - num2*2 - num3*3
    
    assert num1 + num2*2 + num3*3 == N, "Total number of people does not match N"
    
    return num1, num2, num3


def one_simulation(barrier_set, percent_threshold, 
                   r_1, r_2, r_3, A_p, A_o, B, beta1, beta2,
                   N = 135, delta_time=0.005, seed=41, gui_display=False):
    """
    Runs a single simulation.

    Args:
    barrier_set: Specifications of boundaries and obstacles as tuples.
    percent_threshold: Percentage to record the time once this fraction of the population exited.
    r_1, r_2, r_3: Ratios for different group sizes.
    A_p, A_o, B, beta1, beta2: Constant coefficient.
    N: Total number of agents.
    delta_time: Time increment for the simulation.
    seed: Seed for random initialization.

    Returns:
    tuple: Total simulation time and time at which percent_threshold of pedestrians have exited.
    """

    initial_people_count = None
    recorded_time = False
    
    if gui_display:
        gui = GUI()
        gui.add_barrier(barrier_set)
        gui.update_gui()

    # Initialize simulation environment
    people_list = PeopleList(seed)
    people_list.direction_matrix(barrier_set)
    
    # Calculate and assign group configurations
    (_, n_2, n_3) = group_num_calculator(N, r_1, r_2, r_3)
    group_split = {3: n_3, 2: n_2}
    people_list.assign_groups(group_split, seed)

    time = 0  # Initialize simulation time

    if gui_display:
        for person in people_list.list:
            gui.add_line(person, people_list.list, person.group_id)
            gui.add_oval(person.loc[0] - person.r, person.loc[1] - person.r,
                         person.loc[0] + person.r, person.loc[1] + person.r, person.id)
        gui.update_gui()

    initial_people_count = len(people_list.list)

    # Simulation loop
    while people_list.list:
        i = 0
        while i < len(people_list.list):
            # Remove GUI elements for exiting pedestrians
            if gui_display:
                gui.del_line(people_list.list[i].group_id)
                gui.del_oval(people_list.list[i].id)

            # Check exit condition for pedestrians
            if people_list.list[i].loc[0] > 1040 and 300 < people_list.list[i].loc[1] < 380:
                people_list.list.pop(i)  # Pedestrian exits the simulation boundary
                continue
            i += 1

        # Calculate the percentage of exited pedestrians
        remaining_people_count = len(people_list.list)
        moved_out_percentage = ((initial_people_count - remaining_people_count) / initial_people_count) * 100

        # Record the simulation time when specified percentage has exited
        if not recorded_time and moved_out_percentage >= percent_threshold:
            time_perct = round(time, 3)
            recorded_time = True

        # Move pedestrians and apply forces
        people_list.move(barrier_set, delta_time=delta_time, A_p=A_p, A_o=A_o, B=B,
                         threshold_ped=1.2, threshold_obs=120, threshold_group=2, q_A=1, beta1=beta1, beta2=beta2)

        # Update positions and redraw GUI elements if enabled
        if gui_display:
            for person in people_list.list:
                gui.add_oval(int(person.loc[0]) - person.r,
                             int(person.loc[1]) - person.r, int(person.loc[0]) + person.r,
                             int(person.loc[1]) + person.r, person.id)
                gui.add_line(person, people_list.list, person.group_id)

        # Increment simulation time
        time += delta_time
        if gui_display:
            gui.update_time(str(round(time, 3)))
            gui.update_gui()

    if gui_display:
        gui.start()
        
    return time, time_perct if recorded_time else None



def one_simulation_without_gui(barrier_set, percent_threshold, 
                   r_1, r_2, r_3, A_p, A_o, B, beta1, beta2,
                   N = 135, delta_time=0.005, seed=41):
    """
    Simulation function for global sensitive analysis.
    When time reach the threshold, simulation stops and return the threshold time
    """

    initial_people_count = None
    recorded_time = False

    # Build PeopleList saving each people object
    people_list = PeopleList(seed)
    # Build direction matrix
    people_list.direction_matrix(barrier_set)
    # Assign groups
    (_,n_2,n_3) = group_num_calculator(N, r_1, r_2, r_3)
    group_split = {3: n_3, 2: n_2}
    people_list.assign_groups(group_split,seed)

    time = 0

    # Initial each people
    initial_people_count = len(people_list.list)

    # Move
    while people_list.list:
        i = 0
        while i < len(people_list.list):
            
            if people_list.list[i].loc[0] > 1040 and people_list.list[i].loc[1] >300 and people_list.list[i].loc[1] <380:  # delete people if it goes out of room
                people_list.list.pop(i)
                continue
            i += 1

        # Check the percentage of people who have moved out
        remaining_people_count = len(people_list.list)
        moved_out_percentage = ((initial_people_count - remaining_people_count) / initial_people_count) * 100

        if not recorded_time and moved_out_percentage >= percent_threshold:
            threshold_time = round(time, 3)
            print(f"Time when {percent_threshold}% of people moved out: {threshold_time} seconds") 
            recorded_time = True
            break

        people_list.move(barrier_set, delta_time=delta_time,  A_p= A_p, A_o= A_o, B=B,
                         threshold_ped=1.2, threshold_obs=120,
                        threshold_group=2, q_A=1, beta1=beta1, beta2=beta2)  # ped movement


        # update time
        time += delta_time
    
    return threshold_time


def plot_with_ci(output_list, param_list, obstacle_type, value_noobs, xlabel='Distance', ylabel='Average Evacuation Time'):
    """
    Plots average evacuation times with confidence intervals over different parameters.

    Args:
    output_list (list of lists): Each sublist contains multiple simulation results for a specific parameter.
    param_list: Parameters corresponding to each sublist in output_list.
    obstacle_type: Name of the obstacle.
    value_noobs: Reference value for scenarios without obstacles.
    """
    mean_values = []
    conf_intervals = []

    # Calculate mean and 95% confidence interval for each set of outputs
    for outputs in output_list:
        mean_val = np.mean(outputs)
        std_dev = np.std(outputs)
        conf_int = stats.norm.interval(0.95, loc=mean_val, scale=std_dev / np.sqrt(len(outputs)))  # Calculate confidence interval
        mean_values.append(mean_val)
        conf_intervals.append(conf_int)

    mean_values = np.array(mean_values)
    conf_lower = mean_values - np.array([ci[0] for ci in conf_intervals])
    conf_upper = np.array([ci[1] for ci in conf_intervals]) - mean_values


    plt.figure(figsize=(10, 5))
    plt.errorbar(param_list, mean_values, yerr=[conf_lower, conf_upper], 
                 fmt='o', color='b', ecolor='lightgray', elinewidth=3, capsize=0,
                 label=obstacle_type)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(f'Average Evacuation Time ({obstacle_type})')
    plt.xticks(param_list)

    # Draw a horizontal line for the no obstacle scenario for comparison
    if value_noobs:
        plt.axhline(y=value_noobs, color='r', linestyle='--', label='No Obstacle')

    plt.legend()
    plt.savefig(f"Avg_time_{obstacle_type}.png", dpi=150)

    

def plot_with_two_ci_bar(no_obstacle_output_list, obstacle_output_list, param_list, xlabel='Group', ylabel='Average Evacuation Time', scenario='for 80% People'):
    """
    Plots bar charts with CI.

    Args:
    no_obstacle_output_list (list of lists): Each sublist contains multiple simulation results for a specific parameter. Scenarios without obstacles.
    obstacle_output_list (list of lists):  Scenarios with obstacles.
    """

    plt.figure(figsize=(10, 6))

    bar_width = 0.3
    index = np.arange(len(param_list))
    colors = ['#d2a17b', '#aab54f']
    labels = ['No obstacle', 'Obstacle']

    for i, output_list in enumerate([no_obstacle_output_list, obstacle_output_list]):
        mean_values = []
        conf_intervals = []

        for outputs in output_list:
            mean_val = np.mean(outputs)
            std_dev = np.std(outputs)
            conf_int = stats.norm.interval(0.95, loc=mean_val, scale=std_dev / np.sqrt(len(outputs)))
            mean_values.append(mean_val)
            conf_intervals.append(conf_int)
        
        mean_values = np.array(mean_values)
        conf_err = [(mean_val - ci[0], ci[1] - mean_val) for mean_val, ci in zip(mean_values, conf_intervals)]

        plt.bar(index + i * bar_width, mean_values, yerr=np.transpose(conf_err), color=colors[i], width=bar_width, label=labels[i], capsize=5, error_kw={'elinewidth': 2, 'ecolor':'gray'}, alpha=0.7)

    plt.xlabel(xlabel, fontsize=16)
    plt.ylabel(ylabel, fontsize=16)
    plt.title(f'Average Evacuation Time {scenario}', fontsize=20)
    plt.xticks(index + bar_width / 2, param_list)
    plt.legend()
    plt.savefig("bar.png", dpi=150)