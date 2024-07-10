"""
Simulation and plot functions.

Key Features:
1. Determines the distribution of agents into groups.
2. Conducts the simulation either visually via a GUI or simply for data collection.
3. Plots evacuation times with confidence intervals.
"""

import numpy as np
from gui import GUI
from people import PeopleList, People
import scipy.stats as stats
import matplotlib.pyplot as plt
import seaborn as sns

def group_num_calculator(N, r1, r2, r3):
    """Calculation of the number of groups according to the corresponding ratio """
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
    barrier_set = [(start_boundary), (end_boundary), (door), (start_obstacle1), (end_obstacle1), (start_obstacle2), (end_obstacle2)...]
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
    
    # Build GUI
    if gui_display==True:
        gui = GUI()
        gui.add_barrier(barrier_set)
        gui.update_gui()
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
    if gui_display==True:
        for people in people_list.list:
            gui.add_line(people, people_list.list, people.group_id)
            gui.add_oval(people.loc[0] - people.r, people.loc[1] - people.r,
                         people.loc[0] + people.r, people.loc[1] + people.r, people.id)
        gui.update_gui()
    initial_people_count = len(people_list.list)
    # Move
    while people_list.list:
        i = 0
        while i < len(people_list.list):
            if gui_display==True:
                gui.del_line(people_list.list[i].group_id)
                gui.del_oval(people_list.list[i].id)
            if people_list.list[i].loc[0] > 1040 and people_list.list[i].loc[1] >300 and people_list.list[i].loc[1] <380:  # delete people if it goes out of room
                people_list.list.pop(i)
                continue
            i += 1
        # Check the percentage of people who have moved out
        remaining_people_count = len(people_list.list)
        moved_out_percentage = ((initial_people_count - remaining_people_count) / initial_people_count) * 100

        if not recorded_time and moved_out_percentage >= percent_threshold:
    
            time_perct = round(time, 3)
            recorded_time = True

        people_list.move(barrier_set, delta_time=delta_time,  A_p= A_p, A_o= A_o, B=B,
                         threshold_ped=1.2, threshold_obs=120,
                        threshold_group=2, q_A=1, beta1=beta1, beta2=beta2)  # ped movement
        if gui_display==True:
            for people in people_list.list:  # update position
                gui.add_oval(int(people.loc[0]) - people.r,
                             int(people.loc[1]) - people.r, int(people.loc[0]) + people.r,
                             int(people.loc[1]) + people.r, people.id)
                gui.add_line(people, people_list.list, people.group_id)
        # update time
        time += delta_time
        if gui_display==True:
            gui.update_time(str(round(time, 3)))
            gui.update_gui()
    if gui_display==True:
        gui.start()

    return time, time_perct


def one_simulation_for_heatmap(barrier_set, percent_threshold, 
                   r_1, r_2, r_3, A_p, A_o, B, beta1, beta2,
                   N = 135, delta_time=0.005, seed=41, gui_display=False):
    """
    Conducts a single simulation and tracks agents' positions throughout the evacuation
        for generating heat-maps that show the density and flow of agents over time.

    Returns:
    tuple: Returns the total evacuation time, time at the percentage threshold, and positions of agents
    """

    initial_people_count = None
    recorded_time = False
    
    # Build GUI
    if gui_display==True:
        gui = GUI()
        gui.add_barrier(barrier_set)
        gui.update_gui()

    # Build PeopleList saving each people object
    people_list = PeopleList(seed)
    # Build direction matrix
    people_list.direction_matrix(barrier_set)
    # Assign groups
    (_,n_2,n_3) = group_num_calculator(N, r_1, r_2, r_3)
    group_split = {3: n_3, 2: n_2}
    people_list.assign_groups(group_split,seed)

    time = 0
    positions = []

    # Initial each people
    if gui_display==True:
        for people in people_list.list:
            gui.add_line(people, people_list.list, people.group_id)
            gui.add_oval(people.loc[0] - people.r, people.loc[1] - people.r,
                         people.loc[0] + people.r, people.loc[1] + people.r, people.id)
        gui.update_gui()

    initial_people_count = len(people_list.list)

    # Move
    while people_list.list:
        i = 0
        current_positions = []
        while i < len(people_list.list):
            if gui_display==True:
                gui.del_line(people_list.list[i].group_id)
                gui.del_oval(people_list.list[i].id)

            if people_list.list[i].loc[0] > 1040 and people_list.list[i].loc[1] >300 and people_list.list[i].loc[1] <380:  # delete people if it goes out of room
                people_list.list.pop(i)
                continue
            current_positions.append(people_list.list[i].loc)
            i += 1

        positions.append(current_positions)

        # Check the percentage of people who have moved out
        remaining_people_count = len(people_list.list)
        moved_out_percentage = ((initial_people_count - remaining_people_count) / initial_people_count) * 100

        if not recorded_time and moved_out_percentage >= percent_threshold:
            time_perct = round(time, 3)
            recorded_time = True

        people_list.move(barrier_set, delta_time=delta_time,  A_p= A_p, A_o= A_o, B=B,
                         threshold_ped=1.2, threshold_obs=120,
                        threshold_group=2, q_A=1, beta1=beta1, beta2=beta2)  # ped movement

        if gui_display==True:
            for people in people_list.list:  # update position
                gui.add_oval(int(people.loc[0]) - people.r,
                             int(people.loc[1]) - people.r, int(people.loc[0]) + people.r,
                             int(people.loc[1]) + people.r, people.id)
                gui.add_line(people, people_list.list, people.group_id)

        # update time
        time += delta_time
        if gui_display==True:
            gui.update_time(str(round(time, 3)))
            gui.update_gui()
    if gui_display==True:
        gui.start()
        
    return time, time_perct, positions



def one_simulation_without_gui(barrier_set, percent_threshold, 
                   r_1, r_2, r_3, A_p, A_o, B, beta1, beta2,
                   N = 135, delta_time=0.005, seed=41, gui_display=False):
    """
    Simulation function for global sensitive analysis.
    When time reach the threshold, simulation stops and return the threshold time.
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




def batch_simulation(barrier_set, percent_threshold, 
                   r_1, A_p, A_o, B, beta1, beta2,
                   N = 135, delta_time=0.005, sim_run=20,gui_display=False):
    """
    Performs a batch of simulations to analyze evacuation times under varying group configurations.

    Args:
    barrier_set: Specifications of boundaries and obstacles as tuples.
    percent_threshold: Percentage to record the time once this fraction of the population exited.
    r_1, A_p, A_o, B, beta1, beta2: Constant coefficient.
    sim_run: Number of simulations to run.

    Returns:
    tuple: Mean evacuation time, lower confidence interval, upper confidence interval.
    """
    time_list = []
    sim_count = 0

    while sim_count < sim_run:

        seed=np.random.randint(0, 1000)

        initial_people_count = None
        recorded_time = False
        
        r_1 =r_1 
        r_2= np.random.uniform(0, 1 - r_1)
        r_3 = 1 - r_1 - r_2
        
        # Build GUI
        if gui_display==True:
            gui = GUI()
            gui.add_barrier(barrier_set)
            gui.update_gui()

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
        if gui_display==True:
            for people in people_list.list:
                gui.add_line(people, people_list.list, people.group_id)
                gui.add_oval(people.loc[0] - people.r, people.loc[1] - people.r,
                            people.loc[0] + people.r, people.loc[1] + people.r, people.id)
            gui.update_gui()

        initial_people_count = len(people_list.list)

        # Move
        while people_list.list:
            i = 0
            while i < len(people_list.list):
                if gui_display==True:
                    gui.del_line(people_list.list[i].group_id)
                    gui.del_oval(people_list.list[i].id)

                if people_list.list[i].loc[0] > 1040 and people_list.list[i].loc[1] >300 and people_list.list[i].loc[1] <380:  # delete people if it goes out of room
                    people_list.list.pop(i)
                    continue
                i += 1

            # Check the percentage of people who have moved out
            remaining_people_count = len(people_list.list)
            moved_out_percentage = ((initial_people_count - remaining_people_count) / initial_people_count) * 100

            if not recorded_time and moved_out_percentage >= percent_threshold:
                print(f"Time when {percent_threshold}% of people moved out: {round(time, 3)} seconds") 
                recorded_time = True
                break

            people_list.move(barrier_set, delta_time=delta_time,  A_p= A_p, A_o= A_o, B=B,
                            threshold_ped=1.2, threshold_obs=120,
                            threshold_group=2, q_A=1, beta1=beta1, beta2=beta2)  # ped movement

            if gui_display==True:
                for people in people_list.list:  # update position
                    gui.add_oval(int(people.loc[0]) - people.r,
                                int(people.loc[1]) - people.r, int(people.loc[0]) + people.r,
                                int(people.loc[1]) + people.r, people.id)
                    gui.add_line(people, people_list.list, people.group_id)

            # update time
            time += delta_time
            if gui_display==True:
                gui.update_time(str(round(time, 3)))
                gui.update_gui()

            
        
        time_list.append(time)
        sim_count += 1

        if gui_display==True:
            gui.start()

    mean_time = np.mean(time_list)
    std_time = np.std(time_list,ddof=1)
    n = len(time_list)
    t_critical = stats.t.ppf(0.975, df=n-1)

    margin_error = t_critical * (std_time / np.sqrt(n))
    ci_lower = mean_time - margin_error
    ci_upper = mean_time + margin_error

    return mean_time, ci_lower, ci_upper         



def intense_simulation(barrier_set, percent_threshold, 
                   r_1, r_2, r_3, A_p, A_o, B, beta1, beta2,
                   N = 135, delta_time=0.005, sim_run=20,gui_display=False):
    """
    Conducts multiple simulations to analyze evacuation efficiency under various configurations.
    Each simulation run considers different random seed values and potentially different group
        configurations if r_2 and r_3 are set to vary.
    """
    time_list = []
    sim_count = 0

    while sim_count < sim_run:
        seed=np.random.randint(0, 1000)

        initial_people_count = None
        recorded_time = False
        
        
        # Build GUI
        if gui_display==True:
            gui = GUI()
            gui.add_barrier(barrier_set)
            gui.update_gui()

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
        if gui_display==True:
            for people in people_list.list:
                gui.add_line(people, people_list.list, people.group_id)
                gui.add_oval(people.loc[0] - people.r, people.loc[1] - people.r,
                            people.loc[0] + people.r, people.loc[1] + people.r, people.id)
            gui.update_gui()

        initial_people_count = len(people_list.list)

        # Move
        while people_list.list:
            i = 0
            while i < len(people_list.list):
                if gui_display==True:
                    gui.del_line(people_list.list[i].group_id)
                    gui.del_oval(people_list.list[i].id)

                if people_list.list[i].loc[0] > 1040 and people_list.list[i].loc[1] >300 and people_list.list[i].loc[1] <380:  # delete people if it goes out of room
                    people_list.list.pop(i)
                    continue
                i += 1

            # Check the percentage of people who have moved out
            remaining_people_count = len(people_list.list)
            moved_out_percentage = ((initial_people_count - remaining_people_count) / initial_people_count) * 100

            if not recorded_time and moved_out_percentage >= percent_threshold:
                print(f"Time when {percent_threshold}% of people moved out: {round(time, 3)} seconds") 
                recorded_time = True
                break 

            people_list.move(barrier_set, delta_time=delta_time,  A_p= A_p, A_o= A_o, B=B,
                            threshold_ped=1.2, threshold_obs=120,
                            threshold_group=2, q_A=1, beta1=beta1, beta2=beta2)  # ped movement

            if gui_display==True:
                for people in people_list.list:  # update position
                    gui.add_oval(int(people.loc[0]) - people.r,
                                int(people.loc[1]) - people.r, int(people.loc[0]) + people.r,
                                int(people.loc[1]) + people.r, people.id)
                    gui.add_line(people, people_list.list, people.group_id)

            # update time
            time += delta_time
            if gui_display==True:
                gui.update_time(str(round(time, 3)))
                gui.update_gui()

            if time >300:
                break
        
        time_list.append(time)
        sim_count += 1

        if gui_display==True:
            gui.start()

    mean_time = np.mean(time_list)
    std_time = np.std(time_list,ddof=1)
    n = len(time_list)
    t_critical = stats.t.ppf(0.975, df=n-1)

    margin_error = t_critical * (std_time / np.sqrt(n))
    ci_lower = mean_time - margin_error
    ci_upper = mean_time + margin_error

    return mean_time, ci_lower, ci_upper 
        

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

    

def plot_with_two_ci_bar(no_obstacle_output_list, obstacle_output_list, param_list, xlabel='Group', ylabel='Average Evacuation Time', scenario=None):
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


def plot_heatmap(positions, room_size=(27, 17), scale=40):
    heatmap = np.zeros(room_size)
    
    for timestep in positions:
        for (x, y) in timestep:
            grid_x = int(x // scale)
            grid_y = int(y // scale)
            if 0 <= grid_x < room_size[0] and 0 <= grid_y < room_size[1]:
                heatmap[grid_x, grid_y] += 1

    plt.figure(figsize=(12, 8))
    sns.heatmap(np.flipud(heatmap.T), cmap="YlOrRd", cbar=False)
    plt.title("Pedestrian Density Heatmap")
    plt.xlabel("Room Width")
    plt.ylabel("Room Height")
    plt.xticks([])
    plt.yticks([])
    plt.show()


def plot_index(s, params, i, title=''):
    """
    Creates a plot for Sobol sensitivity analysis that shows the contributions
    of each parameter to the global sensitivity.

    Args:
        s (dict): dictionary {'S#': dict, 'S#_conf': dict} of dicts that hold
            the values for a set of parameters
        params (list): the parameters taken from s
        i (str): string that indicates what order the sensitivity is.
        title (str): title for the plot
    """

    if i == '2':
        p = len(params)
        params = list(combinations(params, 2))
        indices = s['S' + i].reshape((p ** 2))
        indices = indices[~np.isnan(indices)]
        errors = s['S' + i + '_conf'].reshape((p ** 2))
        errors = errors[~np.isnan(errors)]
    else:
        indices = s['S' + i]
        errors = s['S' + i + '_conf']
        plt.figure()

    l = len(indices)

    plt.title(title)
    plt.ylim([-0.2, len(indices) - 1 + 0.2])
    plt.yticks(range(l), params)
    plt.errorbar(indices, range(l), xerr=errors, linestyle='None', marker='o')
    plt.axvline(0, c='k')