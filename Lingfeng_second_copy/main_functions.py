"""Functions for basic calculation, sensitive analysis and experiment"""
import numpy as np
from gui import GUI
from people import PeopleList, People
import numpy as np
from scipy import stats
import pandas as pd

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
    barrier_set = [(start_boundary), (end_boundary), (door), (start_obstacle1), (end_obstacle1), (start_obstacle2), (end_obstacle2)...]
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
            print(f"Time when {percent_threshold}% of people moved out: {round(time, 3)} seconds") 
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
        
    return time






def batch_simulation(barrier_set, percent_threshold, 
                   r_1, A_p, A_o, B, beta1, beta2,
                   N = 135, delta_time=0.005, sim_run=20,gui_display=False):
    """
    barrier_set = [(start_boundary), (end_boundary), (door), (start_obstacle1), (end_obstacle1), (start_obstacle2), (end_obstacle2)...]
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
    barrier_set = [(start_boundary), (end_boundary), (door), (start_obstacle1), (end_obstacle1), (start_obstacle2), (end_obstacle2)...]
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
        




