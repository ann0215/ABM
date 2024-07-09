'''
A-star Path-finding Algorithm, designed to calculate the most efficient path 
from a starting point to a predefined goal on a grid.

Two primary classes:
`Node`, represents individual points on the grid with properties to track path costs and parent nodes;
`AStar`, encapsulates the pathfinding logic.

Key features:
1. Construct barriers and obstacles on the grid
2. The use of Manhattan distance as a heuristic to estimate the cost from any node on the grid to the goal
3. Provides the next directional step from the start location towards the goal for step-by-step navigation.
'''

class Node:
    def __init__(self):
        self.g = 0  # Cost from start node
        self.h = 0  # Estimated cost to goal node (heuristic)
        self.f = 0  # Total cost (f = g + h)
        self.father = (0, 0)  # Parent node in the path

def add_barrier(barrier_set):
    """
    Construct barriers and obstacles.
    :param barrier_set: List of tuples defining barrier and obstacle coordinates
    :return: List of tuples representing barrier points
    """
    barrier_list = []
    (left_bottom0, left_bottom1) = barrier_set[0]
    (right_top0, right_top1) = barrier_set[1]
    door = barrier_set[2]  # initially assumed to be a barrier, removed later

    # Define boundaries except for an opening that acts as a door
    for i in range(left_bottom0, right_top0):
        barrier_list.append((i, left_bottom1))
        barrier_list.append((i, right_top1 - 1))
    for j in range(left_bottom1 + 1, right_top1 - 1):
        barrier_list.append((left_bottom0, j))
        barrier_list.append((right_top0 - 1, j))
    barrier_list.remove(door)

    # Add custom obstacles defined in barrier_set
    t = 3
    while t < len(barrier_set):
        (left_bottom0, left_bottom1) = barrier_set[t]
        (right_top0, right_top1) = barrier_set[t + 1]
        for i in range(left_bottom0, right_top0):
            for j in range(left_bottom1, right_top1):
                barrier_list.append((i, j))
        t += 2
    return barrier_list
    

class AStar:
    @staticmethod
    def next_loc(x, y, barrier_set):
        """
        Calculate the next move direction using the A-star algorithm.
        :param x: Starting x-coordinate
        :param y: Starting y-coordinate
        :param barrier_set: Set of barriers and obstacles
        :return: Tuple indicating the direction to move towards the target
        """
        start_loc = (x, y)
        aim_loc = [(26, 8)]  # Goal point is predefined
        open_list = []
        close_list = []
        barrier_list = add_barrier(barrier_set)  # Add barriers to the grid

        if start_loc in barrier_list:
            return None  # No path if start is in a barrier

        # Initialize nodes in a grid
        node_matrix = [[Node() for _ in range(17)] for _ in range(27)]

        open_list.append(start_loc)
        
        while open_list:
            now_loc = min(open_list, key=lambda loc: node_matrix[loc[0]][loc[1]].f)
            open_list.remove(now_loc)
            close_list.append(now_loc)
            
            # Explore neighbors
            for dx, dy in [(-1, 0), (0, -1), (0, 1), (1, 0), (-1, 1), (1, -1), (1, 1), (-1, -1)]:
                temp_loc = (now_loc[0] + dx, now_loc[1] + dy)
                if not (0 <= temp_loc[0] <= 26 and 0 <= temp_loc[1] <= 16) or temp_loc in barrier_list or temp_loc in close_list:
                    continue  # Skip invalid or blocked locations

                new_g = node_matrix[now_loc[0]][now_loc[1]].g + int(((dx**2 + dy**2) * 100) ** 0.5)
                if temp_loc not in open_list or new_g < node_matrix[temp_loc[0]][temp_loc[1]].g:
                    open_list.append(temp_loc)
                    node_matrix[temp_loc[0]][temp_loc[1]].g = new_g
                    node_matrix[temp_loc[0]][temp_loc[1]].h = (abs(aim_loc[0][0] - temp_loc[0]) + abs(aim_loc[0][1] - temp_loc[1])) * 10
                    node_matrix[temp_loc[0]][temp_loc[1]].f = node_matrix[temp_loc[0]][temp_loc[1]].g + node_matrix[temp_loc[0]][temp_loc[1]].h
                    node_matrix[temp_loc[0]][temp_loc[1]].father = now_loc

            if aim_loc[0] in close_list:
                break

        # Trace back from goal to start to find the path
        next_step = aim_loc[0]
        while node_matrix[next_step[0]][next_step[1]].father != start_loc:
            next_step = node_matrix[next_step[0]][next_step[1]].father

        return (next_step[0] - start_loc[0], next_step[1] - start_loc[1])  # Direction to the next step
