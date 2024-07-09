'''
The movement and interactions of agents.

Key Features:
1. Agents are grouped to simulate social groups moving together.
2. Using A-star for path-finding, using a directional matrix to guide agent movement. 
3. Calculates and updates agent positions based on a combination of forces.
'''


import random
import math
from astar import AStar
from force import self_driven_force, ped_repulsive_force, ped_obstacle_rep_force, group_vis_force, group_att_force

class People:
    def __init__(self, _id, _loc_x, _loc_y, seed):
        """Initializes pedestrian attributes with random values for simulation."""
        random.seed(seed)
        self.id = _id  # Unique identifier for each pedestrian
        self.m = 50 + random.randint(0, 20)  # Mass of the pedestrian
        self.r = (35 + random.randint(0, 5))/2  # Radius to represent pedestrian's body (half of shoulder width)
        self.d_v = (200 + random.randint(0, 20)) / 100  # Desired velocity in m/s
        self.loc = (_loc_x, _loc_y)  # Current position (x, y)
        self.v = (0, 0)  # Current velocity (x, y)
        self.a = (0, 0)  # Current acceleration (x, y)
        self.group_id = None  # Identifier for the group the pedestrian belongs to
        self.group_size = 1  # Size of the group

class PeopleList:
    def __init__(self, seed):
        """Initializes list of pedestrians and their starting positions."""
        self.list = []
        self.matrix = [[0 for i in range(17)] for i in range(27)]  # Navigation matrix for direction guidance
        count = 0
        # Create multiple pedestrians
        for i in range(0, 15):
            for j in [60, 100, 140, 180, 220, 260, 300, 340, 380, 420, 460, 500, 540]:
                self.list.append(People("o"+str(count), j, 60 + i * 40, seed))
                count += 1

    def assign_groups(self, group_split, seed, max_distance=150):
        """Assigns pedestrians to groups based on proximity."""
        random.seed(seed)
        ped_num_list = list(range(len(self.list)))
        random.shuffle(ped_num_list)  # Shuffle to randomize group assignment

        group_id = 1
        for size, count in group_split.items():
            for _ in range(count):
                if len(ped_num_list) >= size:
                    selected_indices = [ped_num_list.pop(0)]
                    # Find additional group members within the specified distance
                    for _ in range(1, size):
                        for index in ped_num_list:
                            if self._within_distance(self.list[selected_indices[0]], self.list[index], max_distance):
                                selected_indices.append(index)
                                ped_num_list.remove(index)
                                break
                    # Assign group ID and size
                    for index in selected_indices:
                        self.list[index].group_id = group_id
                        self.list[index].group_size = size
                    group_id += 1
                else:
                    break
        # Assign remaining pedestrians to individual groups
        for index in ped_num_list:
            self.list[index].group_id = group_id
            self.list[index].group_size = 1
            group_id += 1

    def _within_distance(self, person1, person2, max_distance):
        """Helper function to determine if two pedestrians are within a specified distance."""
        distance = math.sqrt((person1.loc[0] - person2.loc[0]) ** 2 + (person1.loc[1] - person2.loc[1]) ** 2)
        return distance <= max_distance

    def direction_matrix(self, barrier_list, k=5):
        """Calculates direction matrix for navigation avoiding barriers."""
        # Define repulsion points around barriers
        for (left_bottom0, left_bottom1), (right_top0, right_top1) in zip(barrier_list[3::2], barrier_list[4::2]):
            for i in range(left_bottom0+1, right_top0-1):
                self.matrix[i][left_bottom1] = (0, -k)
                self.matrix[i][right_top1-1] = (0, k)
            for j in range(left_bottom1+1, right_top1-1):
                self.matrix[left_bottom0][j] = (-k, 0)
                self.matrix[right_top0-1][j] = (k, 0)
            # Handle inner points for larger obstacles
            if right_top0 - left_bottom0 > 2 and right_top1 - left_bottom1 > 2:
                for i in range(left_bottom0+1, right_top0):
                    for j in range(left_bottom1+1, right_top1):
                        self.matrix[i][j] = (0, k)
        # Set directions for general movement towards exit
        for i in range(27):
            for j in range(17):
                if self.matrix[i][j] == 0:
                    self.matrix[i][j] = AStar.next_loc(i, j, barrier_list)

    def move(self, barrier_set, delta_time=0.005):
        """Calculates and updates positions of all pedestrians based on various forces."""
        # Iteratively compute forces and update pedestrian states
        for ped in self.list:
            F_i = self_driven_force(self.matrix, ped)  # Self-driven force
            F_ij = ped_repulsive_force(self.list, ped.id)  # Pedestrian-pedestrian repulsive force
            F_iw = ped_obstacle_rep_force(ped, barrier_set)  # Pedestrian-obstacle repulsive force
            F_vis = group_vis_force(ped, self.list, self.matrix)  # Group visibility force
            F_att = group_att_force(ped, self.list)  # Group attraction force

            # Sum all forces to determine net force and calculate acceleration
            total_force_x = sum([f[0] for f in [F_i, F_ij, F_iw, F_vis, F_att]])
            total_force_y = sum([f[1] for f in [F_i, F_ij, F_iw, F_vis, F_att]])
            ped.a = (total_force_x / ped.m, total_force_y / ped.m)

            # Update velocity and position
            ped.v = (ped.v[0] + ped.a[0] * delta_time, ped.v[1] + ped.a[1] * delta_time)
            ped.loc = (ped.loc[0] + ped.v[0] * delta_time, ped.loc[1] + ped.v[1] * delta_time)

            # Check boundary conditions and adjust positions if necessary
            if ped.loc[0] >= 1080 or ped.loc[1] >= 680:
                ped.loc = (min(ped.loc[0], 1080), min(ped.loc[1], 680))
