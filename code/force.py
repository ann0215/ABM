'''
Different Forces Calculator, calculates different types of forces that influence the movement of agents

Functions:
1. self_driven_force:
    Calculates the force that drives an agent towards the desired direction.
2. ped_repulsive_force:
    Calculates the repulsive force exerted by other nearby agents to prevent collisions and maintain personal space.
3. ped_obstacle_rep_force:
    Calculates the repulsive forces between an agent and obstacles.
4. group_vis_force:
    Calculates the force related to the agent's field of vision and the group's direction.
    This force aims to adjust the agent's direction to maintain visual contact with the group.
5. group_att_force:
    Calculates an attractive force that acts on an agent if she is far from the centroid of their group.
    This force pulls the agent towards the group's center when the agent is beyond a certain threshold distance from the centroid.
'''

import math
import numpy as np
from astar import AStar

def self_driven_force(matrix, ped, tau=0.5):
    """
    Computes the self-driven force for an agent based on the desired velocity towards a target.

    Args:
    matrix: A grid representing navigational directions.
    ped: The pedestrian object, with properties like location (loc), desired velocity (d_v), and mass (m).
    tau: Relaxation time

    Returns:
    Tuple of force components (x, y).
    """
    next_desired = matrix[int(ped.loc[0]//40)][int(ped.loc[1]//40)] 
    desired_v = (ped.d_v * next_desired[0], ped.d_v * next_desired[1])
    fi = (ped.m * (desired_v[0] - ped.v[0]) / tau, ped.m * (desired_v[1] - ped.v[1]) / tau)
    return fi

def ped_repulsive_force(ped_list, i, A=998.97, B=-0.08, threshold=0.5, k=819.62, kappa=510490):
    """
    Calculates the repulsive force by other agents when they are within a certain threshold distance.
    This force helps maintain personal space between pedestrians.

    Args:
    ped_list: List of all pedestrians.
    i: Index of the current pedestrian in ped_list.
    A, B, k, kappa: Constant coefficients.
    threshold: Distance threshold for the force to be active.

    Returns:
    Tuple of force components (x, y).
    """
    sum_of_fij = (0,0)
    ped = ped_list[i]
    for j in range(len(ped_list)):
        if i == j:
            continue
        temp = ped_list[j]
        d = math.sqrt((ped.loc[0] - temp.loc[0])**2 + (ped.loc[1] - temp.loc[1])**2) / 100
        if d < threshold:
            n0 = (ped.loc[0] - temp.loc[0]) / 100
            n1 = (ped.loc[1] - temp.loc[1]) / 100
            r_d = ped.r/100 + temp.r/100 - d
            g = max(0, r_d)
            fij_1 = A * math.exp(-r_d/B) + k * g
            v_diff_tang = (temp.v[0] - ped.v[0]) * (-n1) + (temp.v[1] - ped.v[1]) * (n0)
            fij_2 = kappa * g * v_diff_tang
            sum_of_fij = (sum_of_fij[0] + fij_1 * n0 - fij_2 * n1,
                          sum_of_fij[1] + fij_1 * n1 + fij_2 * n0)
    return sum_of_fij

def ped_obstacle_rep_force(ped, barrier_list, threshold=120, A=998.97, B=-0.08, scale=40):
    """
    Calculates the repulsive force between an agent and nearby obstacles.

    Args:
    ped: The pedestrian object.
    barrier_list: Coordinates defining obstacles and boundaries.
    threshold: Maximum distance at which obstacles exert force.
    A, B: Constant coefficients.
    scale: Scaling factor.

    Returns:
    Tuple of force components (x, y).
    """
    sum_of_fiw = (0,0)
    # Process each boundary specified in barrier_list
    for t in range(0, len(barrier_list), 2):
        (left_bottom0, left_bottom1) = barrier_list[t]
        (right_top0, right_top1) = barrier_list[t + 1]
        # Calculate forces for each boundary around the pedestrian
        # Forces are computed based on proximity to each boundary edge
        # Example for left boundary:
        d = ped.loc[0] - left_bottom0 * scale
        if 0 < d < threshold and left_bottom1 * scale < ped.loc[1] < right_top1 * scale:
            fiw = A * math.exp((d - ped.r) / 100 / B)
            sum_of_fiw = (sum_of_fiw[0] - fiw * d / 100, sum_of_fiw[1])
        # Similar calculations are repeated for other boundaries
        
    return sum_of_fiw


def group_vis_force(ped, Peoplelist, matrix, beta1=4):
    """
    Calculates the force related to the agent's field of vision and the group's direction.

    Args:
    ped: The pedestrian object.
    Peoplelist: List containing all pedestrian objects.
    matrix: Grid representing navigation directions.
    beta1: Constant coefficient.

    Returns:
    Tuple of force components (x, y).
    """
    f_vis = (0,0)
    centroid = [0, 0]
    mass_total = 0
    # Calculate the centroid of the group
    if ped.group_size > 1:
        for temp in Peoplelist:
            if temp.group_id == ped.group_id and temp.id != ped.id:
                centroid[0] += temp.m * temp.loc[0]
                centroid[1] += temp.m * temp.loc[1]
                mass_total += temp.m
        if mass_total > 0:
            centroid = [x / mass_total for x in centroid]
            next_desired = matrix[int(ped.loc[0]//40)][int(ped.loc[1]//40)]
            d = (centroid[0] - ped.loc[0], centroid[1] - ped.loc[1])
            norm_d = np.array([-d[1], d[0]])
            next_desired = np.array(next_desired)
            # Calculate the angle between the pedestrian's orientation and the direction to the centroid
            cos_alpha = np.dot(norm_d, next_desired) / (np.linalg.norm(norm_d) * np.linalg.norm(next_desired))
            alpha = np.degrees(np.arccos(np.clip(cos_alpha, -1, 1)))
            f_vis = (-beta1 * alpha * ped.v[0] * ped.m, -beta1 * alpha * ped.v[1] * ped.m)
    return f_vis

def group_att_force(ped, Peoplelist, q_A=1, beta2=2, threshold=2):
    """
    Calculates an attractive force that pulls an agent towards the centroid of their group.

    Args:
    ped: The pedestrian object.
    Peoplelist: List of all pedestrians.
    q_A, beta2: Constant coefficient.
    threshold: Distance threshold to activate the force.

    Returns:
    Tuple of force components (x, y).
    """
    f_att = (0,0)
    centroid = [0, 0]
    mass_total = 0
    if ped.group_size > 1:
        for temp in Peoplelist:
            centroid[0] += temp.m * temp.loc[0]
            centroid[1] += temp.m * temp.loc[1]
            mass_total += temp.m
        if mass_total > 0:
            centroid = [x / mass_total for x in centroid]
            d = (centroid[0] - ped.loc[0], centroid[1] - ped.loc[1])
            distance = np.linalg.norm(d)
            if distance / 100 > threshold:
                normalized_d = (d[0] / distance, d[1] / distance)
                f_att = (q_A * beta2 * normalized_d[0] * ped.m, q_A * beta2 * normalized_d[1] * ped.m)
    return f_att
