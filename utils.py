import networkx as nx
from scipy import *
import numpy as np
import random
from itertools import permutations
from time import *
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
from pprint import pprint
import timeit


# Dijkstra's algorithm
# replace 0 on infinite for edges that don't exist
def dist_inf(n, matrix):
    for i in range(n):
        for j in range(n):
            if i != j and matrix[i][j] == 0:
                matrix[i][j] = 1000000
    return matrix


# n - dimension of matrix
# start - first vertice that is considered
# matrix is an adjacency matrix
def Dijkstra(n, start, matrix):
    matrix = dist_inf(n, matrix)
    INF = 1000000
    # list of distances between start vertice and other vertices
    dist = [INF] * n
    dist[start] = 0
    is_visited = [False] * n
    min_dist = 0
    min_vertex = start
    while min_dist < INF:
        i = min_vertex
        is_visited[i] = True
        for j in range(n):
            if dist[i] + matrix[i][j] < dist[j]:
                dist[j] = dist[i] + matrix[i][j]
        min_dist = INF
        for j in range(n):
            if not is_visited[j] and dist[j] < min_dist:
                min_dist = dist[j]
                min_vertex = j
    return dist


# Bellman Ford algorithm
def Bellman_Ford(n, start, matrix):
    matrix = dist_inf(n, matrix)
    INF = 1000000
    # list of distances between start vertice and other vertices
    dist = [INF] * n
    dist[start] = 0
    for k in range(1, n):
        for i in range(n):
            for j in range(n):
                if dist[j] + matrix[j][i] < dist[i]:
                    dist[i] = dist[j] + matrix[j][i]
    for i in range(n):
        for j in range(n):
            assert dist[j] + matrix[j][i] >= dist[i], "Negative weight cycle."
    return dist

def path_analysis(num_iter, dimension, start, matrix):
    i=0
    time_D = []
    time_BF = []
    delta_dist =[]
    while i < num_iter:
    #calculate time required for the paths search in Dijkstra algorithm
        start_time_D = perf_counter()
        dist_D = Dijkstra(dimension, start, matrix)
        end_time_D=perf_counter()
        time_D.append(end_time_D-start_time_D)
        #calculate time required for the paths search in Bellman-Ford algorithm
        start_time_BF = perf_counter()
        dist_BF = Bellman_Ford(dimension, start, matrix)
        end_time_BF=perf_counter()
        time_BF.append(end_time_BF-start_time_BF)
        #calculate the difference between results of two algorithms
        d_dist = [i-j for i,j in zip(Dijkstra(100, 0, matrix),
        Bellman_Ford(100, 0, matrix))]
        delta_dist.append(sum(d_dist))
        i=i+1

    print('The average runtime of the Dijkstra algorithm - %f sec' %
          np.mean(time_D))
    print('The average runtime of the Bellman-Ford algorithm - %f sec' %
          np.mean(time_BF))
    print('Distance difference between two algorithms - %f' % sum(delta_dist))
    fig, axes = plt.subplots(figsize=(5, 3), dpi=100)
    axes.plot(time_D, label="Dijkstra's algorithm")
    axes.plot(time_BF, label="Bellman-Ford algorithm")
    axes.set_xlabel('i')
    axes.set_ylabel('sec')
    plt.legend()
    plt.show()

# use existing implementation of the A* algorithm
class Node:
 """
A node class for A* Pathfinding
"""
def __init__(self, parent=None, position=None):
    self.parent = parent
    self.position = position
    self.g = 0
    self.h = 0
    self.f = 0
def __eq__(self, other):
    return self.position == other.position

def return_path(current_node):
    path = []
    current = current_node

    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1] # Return reversed path

def astar(maze, start, end, allow_diagonal_movement = False):

# Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0
# Initialize both open and closed list
    open_list = []
    closed_list = []
    # Add the start node
    open_list.append(start_node)
    # Adding a stop condition
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 2
    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)

    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),
                            (-1, -1), (-1, 1), (1, -1), (1, 1),)

# Loop until you find the end
    while len(open_list) > 0:
        outer_iterations += 1
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        if outer_iterations > max_iterations:
        # if we hit this point return the path such as it is
        # it will not contain the destination
            warn("giving up on pathfinding too many iterations")
            return return_path(current_node)

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        # Found the goal
        if current_node == end_node:
            return return_path(current_node)
        # Generate children
        children = []
        for new_position in adjacent_squares: # Adjacent squares
            # Get node position
            node_position = (current_node.position[0] + new_position[0],
            current_node.position[1] + new_position[1])
            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (
                len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue
            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue
            # Create new node
            new_node = Node(current_node, node_position)
            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if
                    closed_child == child]) > 0:
                continue
            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
            (child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h
            # Child is already in the open list
            if len([open_node for open_node in open_list if
            child == open_node and child.g > open_node.g]) > 0:
                continue
            # Add the child to the open list
            open_list.append(child)
