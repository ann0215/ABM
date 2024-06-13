import random
import math
from astar import AStar
import numpy as np
import random

class People:
    def __init__(self, _id, _loc_x, _loc_y):
        self.id = _id  # ped id
        self.m = 50 + random.randint(0, 20)  # ped mass(kg)
        self.r = (35 + random.randint(0, 5))/200  # ped radius(shoulder/2)(m)
        self.d_v = (60 + random.randint(0, 20)) / 100  # desired vel(m/s)
        self.loc = (_loc_x, _loc_y)  # current location
        self.v = (0, 0)  # current vel
        self.a = (0, 0)  # current acceleration
        self.group = 0 # group_id
        self.group_size = 1
       
class PeopleList:
    def __init__(self, row):
        self.ped_list = [] # save each people as an object
        count = 0
        #  add peds to self.list
        for i in range(0, row): 
            # 3 columns peds
            # 3*15 = 45 peds
            self.ped_list.append(People("o"+str(count), 60, 60 + i * 40))
            count = count + 1
            self.ped_list.append(People("o"+str(count), 100, 60 + i * 40))
            count = count + 1
            self.ped_list.append(People("o"+str(count), 140, 60 + i * 40))
            count = count + 1 
            
    def find_group(self, group_size):
        random.shuffle(self.ped_list)  # Shuffle to start at a random point
        for person in self.ped_list:
            if person.group is None:  # Only consider ungrouped people
                # Check for available group members
                available_group = self.get_nearby_people(person, group_size)
                if available_group and all(p.group is None for p in available_group):
                    group_id = max((p.group for p in self.ped_list if p.group is not None), default=0) + 1
                    for member in available_group:
                        member.group = group_id
                    return True
        return False

    def get_nearby_people(self, start_person, group_size):
        start_index = self.ped_list.index(start_person)
        end_index = start_index + group_size
        if end_index <= len(self.ped_list):
            potential_group = self.ped_list[start_index:end_index]
            if all(p.group is None for p in potential_group):
                return potential_group
        return None

    def form_groups(self, size_list):
        for size in size_list:
            while self.find_group(size):
                pass
        # Assign remaining people to individual groups
        group_id = max((p.group for p in self.ped_list if p.group is not None), default=0) + 1
        for person in self.ped_list:
            if person.group is None:
                person.group = group_id
                group_id += 1

            
    # def group_split(self, group_sizes, count):
    #     """group_sizes={4:num4, 3:num3, 2:num2}"""
    #     random.seed(41)
    #     ped_num_list = list(range(0,count))
    #     random.shuffle(ped_num_list)
            
    #     id_group = 0
    #     i=0
    #     while i < len(ped_num_list):
    #         num = ped_num_list[i]
    #         if 15-num%15+1 >= 4 and group_sizes[4]!=0:
    #             for i in range(0,4):
    #                 self.ped_list[num+i].group = id_group
    #                 self.ped_list[num+i].group_size = 4
    #                 ped_num_list.remove(num+i)
    #             group_sizes[4]-=1
                    
    #         if 15-num%15+1 >= 3 & group_sizes[3]!=0:
    #             for i in range(0,3):
    #                 self.ped_list[num+i].group = id_group
    #                 self.ped_list[num+i].group_size = 3
    #                 ped_num_list.remove(self.ped_list[num+i])
    #             group_sizes[3]-=1 
                    
    #         if 15-num%15+1 >= 2 & group_sizes[2]!=0:
    #             for i in range(0,2):
    #                 self.ped_list[num+i].group = id_group
    #                 self.ped_list[num+i].group_size = 2
    #                 ped_num_list.remove(self.ped_list[num+i])
    #             group_sizes[2]-=1
   
    #         else:
    #             self.ped_list[num].group = id_group
    #         id_group +=1
                
        # Matrix saving all desired directions of each ped (by Astar Algorithm)
        # self.matrix = [[0 for i in range(17)] for i in range(27)]
        # for i in range(0, 27):
        #     for j in range(0, 17):
        #         self.matrix[i][j] = AStar.next_loc(i, j)


class Forces:
    def __init__(self):
        self.self_force=(0,0)
        self.ped_rep_force=(0,0)
        self.obs_rep_force=(0,0)
        self.group_vis_force=(0,0)
        self.group_att_force=(0,0)
        self.group_rep_force=(0,0)
        self.all_forces = (0,0)
        
    def self_driven_force(self, ped, barrier_list, tau=0.5):
        """ped is a People object"""
        next_desired = AStar.next_loc(ped.loc[0], ped.loc[1],barrier_list) # Astar -> next desired location
        self.self_force = (ped.m/tau) * ((next_desired-ped.loc)*ped.d_v - ped.v)
    
    def ped_repulsive_force(self, ped, Peoplelist, A=2000, B=0.08, threshold=1.4, k=819.62, kappa = 510.49):
        """ped is a People object"""
        
        for j in range(0, len(Peoplelist.ped_list)):
            temp = Peoplelist.ped_list[j]
            d = (((ped.loc[0]-temp.loc[0])/100)**2 + ((ped.loc[1]-temp.loc[1])/100)**2)**0.5

            if d<threshold or d!=0:
                n_ij = ped.loc-temp.loc
                n_ij = n_ij/np.linalg.norm(n_ij) # normalized vector pointing from j to i
                t_ij = (-n_ij[1], n_ij[0])
                vel_tang = ped.v * t_ij - temp.v * t_ij
                r_d = ped.r+temp.r - d
                g = lambda r_d: r_d if r_d > 0 else 0
                f_ij = (A * math.exp(r_d/B)+k * g) * n_ij + kappa * g * vel_tang * t_ij
                self.ped_rep_force += f_ij
    
    def ped_obstacle_force(self, ped, Barrier, A=2000, B=0.08, threshold=1.4, k=819.62, kappa = 510.49):
        """ped is a People object"""
        
        for j in range(0, len(Barrier.list)):
            temp = Barrier.list[j]
            d = (((ped.loc[0]-temp.loc[0])/100)**2 + ((ped.loc[1]-temp.loc[1])/100)**2)**0.5
            d_min = 100000
            if d<threshold:
                if d<d_min:
                    d_min = d
                    n_ij = ped.loc-temp.loc
                    n_ij = n_ij/np.linalg.norm(n_ij) # normalized vector pointing from obstacle to i
                    t_ij = (-n_ij[1], n_ij[0])
                    r_d = ped.r+temp.r - d
                    g = lambda r_d: r_d if r_d > 0 else 0
                    self.obs_rep_force = (A * math.exp(r_d/B)+k * g) * n_ij + kappa * g * (-ped.v * t_ij) * t_ij
    
    def group_vis_force(self, ped, Peoplelist,beta1=-4):
        """ped is a People object; Vision of Scene force"""
        for j in range(0, len(Peoplelist.ped_list)):
            temp = Peoplelist.ped_list[j]
            # centroid of other group members
            centroid = (0,0)
            if temp.group==ped.group:
                centroid[0] += temp.m * temp.loc[0]
                centroid[1] += temp.m * temp.loc[1]
                mass_total += temp.m
            centroid = centroid/mass_total
    
        next_desired = AStar.next_loc(ped.loc[0], ped.loc[1])
        d = centroid - ped.loc
        norm_d = (-d[1],d[0])
        cos_alpha = np.abs(np.dot(norm_d*next_desired) / (np.linalg.norm(norm_d)*np.linalg.norm(next_desired)))
        alpha = np.arccos(cos_alpha)
        alpha = np.degrees(alpha)
        self.group_vis_force = -beta1*alpha*ped.v
    
    def group_att_force(self, ped, Peoplelist, q_A, beta2=2, threshold=2):
        """ped is a People object; only works when ped is too far from the centroid"""
        
        centroid = (ped.m * ped.loc[0], ped.m * ped.loc[1])
        mass_total = ped.m
        for j in range(0, len(Peoplelist.ped_list)):
            temp = Peoplelist.ped_list[j]
            if temp.group==ped.group:
                centroid[0] += temp.m * temp.loc[0]
                centroid[1] += temp.m * temp.loc[1]
                mass_total += temp.m
            centroid = centroid/mass_total
        
            d = centroid - ped.loc # vector pointing form ped to mass
            if np.linalg.norm(d)>threshold:
                d = d/np.linalg.norm(d)
                self.group_att_force += q_A * beta2 * d
            
    def group_rep_force(self, ped, Peoplelist, q_R, beta3=2):
        """ped is a People object; Group inter repulsive force"""
        for j in range(0, len(Peoplelist.ped_list)):
            temp = Peoplelist.ped_list[j]
            if temp.group==ped.group:
                w_ik = temp.loc - ped.loc # vector pointing from ped to groupfriends
                w_ik = w_ik/np.linalg.norm(w_ik)
                self.group_rep_force += q_R * beta3 * w_ik
    
    def all_forces(self, ped):
        self.all_forces = self.self_force+self.ped_rep_force+self.obs_rep_force
        if ped.group_size>1:
            self.all_forces += self.group_vis_force+self.group_att_force+self.group_rep_force
            
    def move(self, delta_time=0.005):
        
        a = self.all_forces/self.m
        distance = self.v * delta_time + a**2 * delta_time /2
        self.loc = self.loc + distance