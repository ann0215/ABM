"""Different forces calculator"""

import math
import numpy as np
from astar import AStar

def self_driven_force(matrix, ped, tau=0.5):
    """ped is a object: PeopleList.list[i]
       matrix: PeopleList.matrix"""
    next_desired = matrix[int(ped.loc[0]//40)][int(ped.loc[1]//40)] 
    desired_v = (ped.d_v*next_desired[0], ped.d_v*next_desired[1])
    fi = (ped.m * (desired_v[0] - ped.v[0]) / tau, ped.m * (desired_v[1] - ped.v[1]) / tau)
    return fi

def ped_repulsive_force(ped_list,i, A=2000, B=-0.08, threshold=1.4):
    """ped_list: PeopleList.list"""
    sum_of_fij = (0,0)
    ped = ped_list[i]
    for j in range(0, len(ped_list)):
        if i==j:
            continue
        temp = ped_list[j]
        d = (((ped_list[i].loc[0] - temp.loc[0])/100)**2 + ((ped_list[i].loc[1] - temp.loc[1])/100)**2)**0.5
        if d<threshold: #Only works when two peds are closer than 1.4m
            fij =A * math.exp((d-ped_list[i].r/100-temp.r/100)/B)
            sum_of_fij = (sum_of_fij[0] + fij * (ped.loc[0] - temp.loc[0])/100,
                          sum_of_fij[1] + fij * (ped.loc[1] - temp.loc[1])/100)
    return sum_of_fij

def ped_obstacle_rep_force(ped, barrier_list, threshold=120, A=2000, B=-0.08, scale=40):
    """ped is a object: PeopleList.list[i]
       threshold (cm)"""
    sum_of_fiw = (0,0)
    # Boundray - 4 walls
    (left_bottom0, left_bottom1) = barrier_list[0]
    (right_top0, right_top1) = barrier_list[1]
    d = ped.loc[0] - (left_bottom0+1)*scale #left line
    if d<threshold:
        fiw = A * math.exp((d - ped.r) / 100 / B)
        sum_of_fiw = (fiw * d / 100 + sum_of_fiw[0], sum_of_fiw[1])
    d = (right_top0-1)*scale - ped.loc[0] #right line
    if (d < threshold) and (ped.loc[1] > 380 or ped.loc[1] < 300):
        fiw = A * math.exp((d - ped.r) / 100 / B)
        sum_of_fiw = (fiw * (-d) / 100 + sum_of_fiw[0], sum_of_fiw[1])
    d = ped.loc[1] - (left_bottom1+1)*scale #bottom line
    if d<threshold:
        fiw = A * math.exp((d - ped.r) / 100 / B)
        sum_of_fiw = (sum_of_fiw[0], fiw * d / 100 + sum_of_fiw[1])
    d = (right_top1-1)*scale - ped.loc[1] #top line
    if d<threshold:
        fiw = A * math.exp((d - ped.r) / 100 / B)
        sum_of_fiw = (sum_of_fiw[0], fiw * (-d) / 100 + sum_of_fiw[1])
        
    # obstacles
    t=3
    while t<len(barrier_list):
        (left_bottom0, left_bottom1) = barrier_list[t]
        (right_top0, right_top1) = barrier_list[t+1]
        d = left_bottom0 * scale - ped.loc[0] #left boundary
        if (d < threshold) and (d > 0) and (ped.loc[1] > left_bottom1*scale) and (ped.loc[1] < right_top1*scale):
            fiw = A * math.exp((d - ped.r) / 100 / B)
            sum_of_fiw = (fiw * (-d) / 100 + sum_of_fiw[0], sum_of_fiw[1])
        d = ped.loc[0] - right_top0*scale
        if (d < threshold) and (d > 0) and (ped.loc[1] > left_bottom1*scale) and (ped.loc[1] < right_top1*scale):
            fiw = A * math.exp((d - ped.r) / 100 / B)
            sum_of_fiw = (fiw * d / 100 + sum_of_fiw[0], sum_of_fiw[1])
        d = left_bottom1*scale - ped.loc[1]
        if (d < threshold) and (d > 0) and (ped.loc[0] > left_bottom0*scale) and (ped.loc[0] < right_top0*scale):
            fiw = A * math.exp((d - ped.r) / 100 / B)
            sum_of_fiw = (sum_of_fiw[0], fiw * (-1) * d / 100 + sum_of_fiw[1])   
        d = ped.loc[1] - right_top1*scale     
        if (d < threshold) and (d > 0) and (ped.loc[0] > left_bottom0*scale) and (ped.loc[0] < right_top0*scale):
            fiw = A * math.exp((d - ped.r) / 100 / B)
            sum_of_fiw = (sum_of_fiw[0], fiw * d / 100 + sum_of_fiw[1])
        t+=2
    return sum_of_fiw

def group_vis_force(ped, Peoplelist, matrix,beta1=-4):
    """ped is a People object; Vision of Scene force"""
        
    f_vis = (0,0)
    centroid = [0,0] # centroid of other group members
    d = (0,0)
    mass_total = 0 
    if ped.group_size>1:
        for j in range(0, len(Peoplelist)):
            temp = Peoplelist[j]
            if temp.group_id==ped.group_id and temp.id!=ped.id:
                centroid[0] = temp.m * temp.loc[0]
                centroid[1] += temp.m * temp.loc[1]
                mass_total += temp.m
        if mass_total>0: # All other group members are inside the room 
            centroid = [centroid[0]/mass_total, centroid[1]/mass_total]
            next_desired = matrix[int(ped.loc[0]//40)][int(ped.loc[1]//40)] 
            d= (centroid[0] - ped.loc[0],centroid[1] - ped.loc[1])
            norm_d = (-d[1],d[0])
            if norm_d is None:
                print("Error: norm_d is None")
            if next_desired is None:
                print("Error: next_desired is None")
            norm_d = np.array(norm_d)  
            next_desired = np.array(next_desired)
            cos_alpha = np.abs(np.dot(norm_d, next_desired) / (np.linalg.norm(norm_d)*np.linalg.norm(next_desired)))
            alpha = np.arccos(cos_alpha)
            alpha = np.degrees(alpha)
            f_vis = (-beta1*alpha*ped.v[0], -beta1*alpha*ped.v[1])
    return f_vis
   
 
def group_att_force(ped, Peoplelist, q_A, beta2=2, threshold=2):
    """ped is a People object; only works when ped is too far from the centroid"""
        
    f_att=(0,0)
    centroid = [0,0] # centroid of all group members
    mass_total = 0
    d = (0,0)
    if ped.group_size>1:
        for j in range(0, len(Peoplelist)):
            temp = Peoplelist[j]
            centroid[0] += temp.m * temp.loc[0]
            centroid[1] += temp.m * temp.loc[1]
            mass_total += temp.m
        if mass_total>0: # every group memember is inside the room
            centroid = [centroid[0]/mass_total, centroid[1]/mass_total]
            d= (centroid[0] - ped.loc[0],centroid[1] - ped.loc[1]) # vector pointing form ped to mass
            distance = np.linalg.norm(np.array(d))
            if distance/100>threshold:
                d = (d[0]/distance, d[1]/distance)
                f_att = (q_A * beta2 * d[0], q_A * beta2 * d[1])
    return f_att
        
            
def group_rep_force(ped, Peoplelist, q_R, beta3=2, threshold=1.4):
    """Here we drop this force since we've already consider the repulsive forces between ped-ped."""
    """ped is a People object; Group inter repulsive force"""
    w_ik = (0,0)
    f_rep0 =0
    f_rep1 =0
    if ped.group_size>1:
        for j in range(0, len(Peoplelist)):
            temp = Peoplelist[j]
            if temp.group_id==ped.group_id and temp.group_id!=ped.group_id:
                w_ik = (temp.loc[0] - ped.loc[0], temp.loc[1] - ped.loc[1]) # vector pointing from ped to groupfriends
                w = np.arrary(w_ik)
                if w<threshold:
                    w_ik = (w_ik[0]/np.linalg.norm(w), w_ik[1]/np.linalg.norm(w))
                    f_rep0 += q_R * beta3 * w_ik[0]
                    f_rep1 += q_R * beta3 * w_ik[1]
    f_rep = (f_rep0,f_rep1)
    
    return f_rep
    