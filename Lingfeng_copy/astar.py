"""Astar algorithm to find the desired direction"""

class Node:
    def __init__(self):
        self.g = 0
        self.h = 0
        self.f = 0
        self.father = (0, 0)


def add_barrier(barrier_set):  #input barrier_set is a list of tuples where is the key corner of the barrier
    # add boundray (borader)
    barrier_list = []
    (left_bottom0, left_bottom1) = barrier_set[0]
    (right_top0, right_top1) = barrier_set[1]
    door = barrier_set[2]   # change the door to barrier
    for i in range(left_bottom0, right_top0):  #(0 to 26) 
        barrier_list.append((i, left_bottom1))  #(0, 0) (1, 0) (2, 0) ... (26, 0)
        barrier_list.append((i, right_top1-1))  #(0, 16) (1, 16) (2, 16) ... (26, 16)
    for j in range(left_bottom1+1, right_top1-1):  #(1 to 15)
        barrier_list.append((left_bottom0, j))  #(0, 1) (0, 2) (0, 3) ... (0, 15)
        barrier_list.append((right_top0-1, j))   #(26, 1) (26, 2) (26, 3) ... (26, 15)
    barrier_list.remove(door)
    
    # add obstacles  (对中间障碍物的改动在这里进行)
    t=3
    while t<len(barrier_set):  #前三个都安排好了，从barrier set的第四个开始 set the obstacles 
        (left_bottom0, left_bottom1) = barrier_set[t]
        (right_top0, right_top1) = barrier_set[t+1]
        for i in range(left_bottom0, right_top0):
            for j in range(left_bottom1, right_top1):
                barrier_list.append((i, j))
        t+=2
    return barrier_list
    

class AStar:
    @staticmethod
    def next_loc(x, y, barrier_set):
        start_loc = (x, y)  # start point
        # aim_loc = [barrier_set[2]]
        aim_loc = [(26, 8)]  # target point
        open_list = []  
        close_list = []  
        barrier_list = [] 
        
        # add barrier
        barrier_list = add_barrier(barrier_set)


        if start_loc in barrier_list:
            return
    
        node_matrix = [[0 for i in range(17)] for i in range(27)]
        for i in range(0, 27):
            for j in range(0, 17):
                node_matrix[i][j] = Node()

        open_list.append(start_loc)  # add start point to openlist
        
        while True:
            now_loc = open_list[0]
            for i in range(1, len(open_list)):  # （1）obtain the point with min f
                if node_matrix[open_list[i][0]][open_list[i][1]].f < node_matrix[now_loc[0]][now_loc[1]].f:
                    now_loc = open_list[i]
            #   （2）move current location to close list
            open_list.remove(now_loc)
            close_list.append(now_loc)
            #  （3）corresponding to each node in the neighbourhood
            list_offset = [(-1, 0), (0, -1), (0, 1), (1, 0), (-1, 1), (1, -1), (1, 1), (-1, -1)]
            for temp in list_offset:
                temp_loc = (now_loc[0] + temp[0], now_loc[1] + temp[1])
                if temp_loc[0] < 0 or temp_loc[0] > 26 or temp_loc[1] < 0 or temp_loc[1] > 16:
                    continue
                if temp_loc in barrier_list:  # skip if in barrier list
                    continue
                if temp_loc in close_list:  # skip if in close list
                    continue

                #  add to open list if it's not in it, and calculate f,g,h
                if temp_loc not in open_list:
                    open_list.append(temp_loc)
                    node_matrix[temp_loc[0]][temp_loc[1]].g = (node_matrix[now_loc[0]][now_loc[1]].g +
                                                             int(((temp[0]**2+temp[1]**2)*100)**0.5))
                    node_matrix[temp_loc[0]][temp_loc[1]].h = (abs(aim_loc[0][0]-temp_loc[0])
                                                               + abs(aim_loc[0][1]-temp_loc[1]))*10
                    node_matrix[temp_loc[0]][temp_loc[1]].f = (node_matrix[temp_loc[0]][temp_loc[1]].g +
                                                               node_matrix[temp_loc[0]][temp_loc[1]].h)
                    node_matrix[temp_loc[0]][temp_loc[1]].father = now_loc
                    continue

                #  compare and recalculate if it's in open list
                if node_matrix[temp_loc[0]][temp_loc[1]].g > (node_matrix[now_loc[0]][now_loc[1]].g +
                                                             int(((temp[0]**2+temp[1]**2)*100)**0.5)):
                    node_matrix[temp_loc[0]][temp_loc[1]].g = (node_matrix[now_loc[0]][now_loc[1]].g +
                                                             int(((temp[0]**2+temp[1]**2)*100)**0.5))
                    node_matrix[temp_loc[0]][temp_loc[1]].father = now_loc
                    node_matrix[temp_loc[0]][temp_loc[1]].f = (node_matrix[temp_loc[0]][temp_loc[1]].g +
                                                               node_matrix[temp_loc[0]][temp_loc[1]].h)

            if aim_loc[0] in close_list:
                break

        #  Iterate over the parent node to find the next position
        temp = aim_loc[0]
        while node_matrix[temp[0]][temp[1]].father != start_loc:
            temp = node_matrix[temp[0]][temp[1]].father
        #  return the new direction vector:（-1,0），（-1,1）......
        re = (temp[0] - start_loc[0], temp[1] - start_loc[1])
        return re
