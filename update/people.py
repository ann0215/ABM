"""People setting & movement"""

import random
import math
from astar import AStar
from force import self_driven_force, ped_repulsive_force, ped_obstacle_rep_force, group_vis_force,  group_att_force, group_rep_force

class People:
    def __init__(self, _id, _loc_x, _loc_y,):
        random.seed(41)
        self.id = _id  # ped id
        self.m = 50 + random.randint(0, 20)  # ped weight/kg
        self.r = (35 + random.randint(0, 5))/2  # ped radius(shoulder length/2)/!!cm!!
        self.d_v = (60 + random.randint(0, 20)) / 100  # expected vel (/m/s)
        self.loc = (_loc_x, _loc_y)  # current position
        self.v = (0, 0)  # current vel
        self.a = (0, 0)  # current acceleration
        self.group_id = None # group id
        self.group_size = 1 # size of group it belongs to 

class PeopleList:
    def __init__(self):
        self.list = []
        
        self.matrix = [[0 for i in range(17)] for i in range(27)]
        count = 0
        #  add ped into self.list
        for i in range(0, 15):
            self.list.append(People("o"+str(count), 60, 60 + i * 40))
            count = count + 1
            self.list.append(People("o"+str(count), 100, 60 + i * 40))
            count = count + 1
            self.list.append(People("o"+str(count), 140, 60 + i * 40))
            count = count + 1
            self.list.append(People("o"+str(count), 180, 60 + i * 40))
            count = count + 1
            self.list.append(People("o"+str(count), 220, 60 + i * 40))
            count = count + 1
    
    def assign_groups(self, group_split, max_distance=150):
        """Assigns pedestrians to groups with members within a specified distance."""
        random.seed(41)
        ped_num_list = list(range(len(self.list)))
        random.shuffle(ped_num_list)

        group_id = 1
        for size, count in group_split.items():
            for _ in range(count):
                if len(ped_num_list) >= size:
                    selected_indices = [ped_num_list.pop(0)]
                    for _ in range(1, size):
                        for index in ped_num_list:
                            if self._within_distance(self.list[selected_indices[0]], self.list[index], max_distance):
                                selected_indices.append(index)
                                ped_num_list.remove(index)
                                break

                    for index in selected_indices:
                        self.list[index].group_id = group_id
                        self.list[index].group_size = size

                    group_id += 1
                else:
                    break

        for index in ped_num_list:
            self.list[index].group_id = group_id
            self.list[index].group_size = 1
            group_id += 1

    def _within_distance(self, person1, person2, max_distance):
        """Checks if person2 is within max_distance of person1."""
        distance = math.sqrt((person1.loc[0] - person2.loc[0]) ** 2 + (person1.loc[1] - person2.loc[1]) ** 2)
        return distance <= max_distance

    def direction_matrix(self,barrier_list, k=5):
        """matrix saving next directions
           barrier_list saving all starpoints&endpoints of barrier"""
        k=5
        # Obstacle
        t=3
        while t<len(barrier_list):
            (left_bottom0, left_bottom1) = barrier_list[t]
            (right_top0, right_top1) = barrier_list[t+1]
            for i in range(left_bottom0+1, right_top0-1):  
                self.matrix[i][left_bottom1] = (0, -k) # under the bottom boundray
                self.matrix[i][right_top1-1] = (0, k) # over the top boundary
            for j in range(left_bottom1+1, right_top1-1):  
                self.matrix[left_bottom0][j] = (-k, 0) # left of the left boundray
                self.matrix[right_top0-1][j] = (k, 0) # close to the right boarder
            self.matrix[left_bottom0][left_bottom1] = (-k, -k)
            self.matrix[left_bottom0][right_top1-1] = (-k, k)
            self.matrix[right_top0-1][right_top1-1] = (k, k)
            self.matrix[right_top0-1][left_bottom1] = (k, -k)
            # Points inside obstacle
            if right_top0-left_bottom0>2 and right_top1-left_bottom1>2: #only big squre cases have inner points
                for i in range(left_bottom0+1, right_top0):
                    for j in range(left_bottom1+1,right_top1):
                        # print(f"({i},{j})")
                        self.matrix[i][j]=(0,k)
            t+=2
        # Boundray
        (left_bottom0, left_bottom1) = barrier_list[0]
        (right_top0, right_top1) = barrier_list[1]
        (door0, door1) = barrier_list[2]
        for i in range(left_bottom0, right_top0):  
            self.matrix[i][left_bottom1] = (0, k) # close to the bottom boarder
            self.matrix[i][right_top1-1] = (0, -k) # close to the top boarder
        for j in range(left_bottom1, right_top1):  
            self.matrix[left_bottom0][j] = (k, 0) # close to the left boarder
            self.matrix[right_top0-1][j] = (-k, 0) # close to the right boarder
        self.matrix[door0][door1] = (1, 0) # door (target position)                
        # Other Position
        for i in range(0, right_top0):
            for j in range(0, right_top1):
                if self.matrix[i][j]==0:
                    self.matrix[i][j] = AStar.next_loc(i, j,barrier_list)    
    
    
    def move(self,barrier_set, delta_time = 0.005, A=2000, B=-0.08, threshold_ped=1.4, threshold_obs=120,threshold_group=2, q_A=1, q_R=1, beta1=-4, beta2=2, beta3=2):
        delta_time = 0.005
        #  Calculate combined force
        for i in range(0, len(self.list)):
            now = self.list[i]
            
            # (1) F_i self-driven force
            F_i = self_driven_force(self.matrix, now, tau=0.5)
            
            # (2) F_ij ped-ped repulsive force
            F_ij = ped_repulsive_force(self.list, i, A, B, threshold_ped)
            
            # (3) F_iw ped-obstacle repulsive force
            F_iw = ped_obstacle_rep_force(self.list[i], barrier_set, threshold_obs, A, B,scale=40)
            
            # (4) F_vis group vision of view force
            F_vis = group_vis_force(now, self.list, self.matrix, beta1)
            
            # (5) F_att group attractive force
            F_att = group_att_force(now, self.list, q_A, beta2, threshold_group)
            
            # (6) F_rep group repulsive force
            # F_rep = group_rep_force(now, self.list, q_R, beta3, threshold_ped)
            F_rep = (0,0) # here i think we don't need to re-plus the repulsive force
            
            # Calculate acceleration by combined forces
            a_x= (F_i[0]+F_ij[0]+F_iw[0]+F_vis[0]+F_att[0]+F_rep[0])/now.m
            a_y= (F_i[1]+F_ij[1]+F_iw[1]+F_vis[1]+F_att[1]+F_rep[1])/now.m
            self.list[i].a = (a_x, a_y)

        #  new vel -> next position -> update
        for i in range(0, len(self.list)):
            now = self.list[i]
            a_x = now.a[0]
            a_y = now.a[1]
            v0_x = now.v[0]
            v0_y = now.v[1]
            v_x = v0_x + a_x * delta_time  
            v_y = v0_y + a_y * delta_time
            self.list[i].v = (v_x, v_y)
            # l_x = (v0_x * delta_time + delta_time * a_x * delta_time * delta_time)*100 + now.loc[0]  
            # l_y = (v0_y * delta_time + delta_time * a_y * delta_time * delta_time)*100 + now.loc[1]
            l_x = (v0_x * delta_time + 0.5 * a_x * delta_time * delta_time)*100 + now.loc[0]  
            l_y = (v0_y * delta_time + 0.5 * a_y * delta_time * delta_time)*100 + now.loc[1]
            self.list[i].loc = (l_x, l_y)


