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
        self.group = None # group_id
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

class Forces:
    def __init__(self):
        self.self_force=(0,0)
        self.ped_rep_force=(0,0)
        self.obs_rep_force=(0,0)
        self.group_vis_force=(0,0)
        self.group_att_force=(0,0)
        self.group_rep_force=(0,0)
        self.all_forces = (0,0)
        
    def self_driven_force(self, ped, barrier_list, tau=0.05):
        """ped is a People object"""
        next_desired = AStar.next_loc(int(ped.loc[0]//40), int(ped.loc[1]//40), barrier_list) # next desired position
        print(next_desired)
        desired_v = ((next_desired[0]-ped.loc[0])*ped.d_v, (next_desired[1]-ped.loc[1])*ped.d_v)  #💢here divided by 100
    
        self.self_force = ((ped.m/tau) * (desired_v[0] - ped.v[0]),(ped.m/tau) * (desired_v[1] - ped.v[1]))

    
    def ped_repulsive_force(self, ped, Peoplelist, A=2000, B=-0.08, threshold=1.4, k=81962, kappa = 510490):
        """ped is a People object"""
        
        for j in range(0, len(Peoplelist.ped_list)):
            temp = Peoplelist.ped_list[j]
            d = (((ped.loc[0]-temp.loc[0])/100)**2 + ((ped.loc[1]-temp.loc[1])/100)**2)**0.5

            if d<threshold or d!=0:
                n_ij = (ped.loc[0]-temp.loc[0], ped.loc[1]-temp.loc[1])
                n_ij = n_ij/np.linalg.norm(n_ij) # normalized vector pointing from j to i
                t_ij = (-n_ij[1], n_ij[0])
                vel_tang = ((ped.v[0]-temp.v[0]) * t_ij[0], (ped.v[1] - temp.v[1]) * t_ij[1])
                r_d = ped.r+temp.r - d
                g=max(0, r_d/100)
                # g = lambda r_d: r_d if r_d > 0 else 0
                f_ij = ((A * math.exp(r_d/B)+k * g) * n_ij[0] + kappa * g * vel_tang[0] * t_ij[0],
                        (A * math.exp(r_d/B)+k * g) * n_ij[1] + kappa * g * vel_tang[1] * t_ij[1])
                # f_ij = ((A * math.exp(r_d/B)) * n_ij[0] ,
                #         (A * math.exp(r_d/B)) * n_ij[1])
                self.ped_rep_force += f_ij
    
    def ped_obstacle_force(self, ped, Barrier, A=2000, B=-0.08, threshold=1, k=81962, kappa = 510490):
        """ped is a People object"""
        
        for j in range(0, len(Barrier.barrier_list)):
            temp = Barrier.barrier_list[j]
            
            d = (((ped.loc[0]-temp[0])/100)**2 + ((ped.loc[1]-temp[1])/100)**2)**0.5
            d_min = 100000
            if d<threshold:
                if d<d_min:
                    d_min = d
                    n_ij = (ped.loc[0]-temp[0], ped.loc[1]-temp[1])
                    n_ij = n_ij/np.linalg.norm(n_ij) # normalized vector pointing from obstacle to i
                    t_ij = (-n_ij[1], n_ij[0])
                    r_d = ped.r - d
                    g=max(0, r_d/100)
                    self.obs_rep_force = ((A * math.exp(r_d/B)+k * g) * n_ij[0] + kappa * g * (-np.dot(ped.v, t_ij)) * t_ij[0],
                                          (A * math.exp(r_d/B)+k * g) * n_ij[1] + kappa * g * (-np.dot(ped.v, t_ij)) * t_ij[1])
                    # self.obs_rep_force = ((A * math.exp(r_d/B)) * n_ij[0],
                    #                       (A * math.exp(r_d/B)) * n_ij[1])
    
    def group_vis_force(self, ped, Peoplelist,beta1=-4):
        """ped is a People object; Vision of Scene force"""
        
        if ped.group_size==1:return 0
        # for j in range(0, len(Peoplelist.ped_list)):
        #     temp = Peoplelist.ped_list[j]
        #     # centroid of other group members
        #     centroid = (0,0)
        #     if temp.group==ped.group:
        #         centroid[0] += temp.m * temp.loc[0]
        #         centroid[1] += temp.m * temp.loc[1]
        #         mass_total += temp.m
        #     centroid = centroid/mass_total
    
        # next_desired = AStar.next_loc(ped.loc[0], ped.loc[1])
        # d = centroid - ped.loc
        # norm_d = (-d[1],d[0])
        # cos_alpha = np.abs(np.dot(norm_d*next_desired) / (np.linalg.norm(norm_d)*np.linalg.norm(next_desired)))
        # alpha = np.arccos(cos_alpha)
        # alpha = np.degrees(alpha)
        # self.group_vis_force = -beta1*alpha*ped.v
    
    def group_att_force(self, ped, Peoplelist, q_A, beta2=2, threshold=2):
        """ped is a People object; only works when ped is too far from the centroid"""
        
        if ped.group_size==1:return 
        # centroid = (ped.m * ped.loc[0], ped.m * ped.loc[1])
        # mass_total = ped.m
        # for j in range(0, len(Peoplelist.ped_list)):
        #     temp = Peoplelist.ped_list[j]
        #     if temp.group==ped.group:
        #         centroid[0] += temp.m * temp.loc[0]
        #         centroid[1] += temp.m * temp.loc[1]
        #         mass_total += temp.m
        #     centroid = centroid/mass_total
        
        #     d = centroid - ped.loc # vector pointing form ped to mass
        #     if np.linalg.norm(d)>threshold:
        #         d = d/np.linalg.norm(d)
        #         self.group_att_force += q_A * beta2 * d
            
    def group_rep_force(self, ped, Peoplelist, q_R, beta3=2):
        """ped is a People object; Group inter repulsive force"""
        
        if ped.group_size==1:return 
        # for j in range(0, len(Peoplelist.ped_list)):
        #     temp = Peoplelist.ped_list[j]
        #     if temp.group==ped.group:
        #         w_ik = temp.loc - ped.loc # vector pointing from ped to groupfriends
        #         w_ik = w_ik/np.linalg.norm(w_ik)
        #         self.group_rep_force += q_R * beta3 * w_ik
    
    def sum_forces(self):
        self.all_forces = self.self_force+self.ped_rep_force+self.obs_rep_force+self.group_vis_force+self.group_att_force+self.group_rep_force
        # if ped.group_size>1:
        #     self.all_forces += self.group_vis_force+self.group_att_force+self.group_rep_force
            
    def move(self, ped, delta_time=0.005):
        
        a = (self.all_forces[0]/ped.m, self.all_forces[1]/ped.m)
        distance = (ped.v[0] * delta_time + a[0] * delta_time**2 /2,
                    ped.v[1] * delta_time + a[1] * delta_time**2 /2)
        ped.loc = (ped.loc[0] + distance[0]*100, ped.loc[1] + distance[1]*100)