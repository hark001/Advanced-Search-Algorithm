# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

from hashlib import new
from this import d
import matplotlib.pyplot as plt
import numpy as np
import math
import networkx as nx
from scipy import spatial
from bresenham import bresenham

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        self.goal_vicinity = 5
        self.step_size = 7

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices_pts = []
        self.vertices.append(self.start)
        self.vertices_pts.append((self.start.row, self.start.col))

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        dist = math.sqrt((node2.col-node1.col)**2 + (node2.row-node1.row)**2)
        # distance = np.sqrt(np.sum(np.square(point2 - point1)))
        return dist


    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        line_pts = bresenham(node1.col,node1.row,node2.col,node2.row)
        for (x,y) in line_pts:
            if self.map_array[y][x] == 0:
                return False
        return True


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        if np.random.random() <= goal_bias:
           return self.goal
        else:
            row = np.random.random_integers(0,self.size_row-1)
            col = np.random.random_integers(0,self.size_col-1)
            random_pt = Node(row, col)
            return random_pt

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        # print("self.vertices_pts: ",self.vertices_pts)
        kdtree = spatial.KDTree(self.vertices_pts)
        # array_nodes = np.asarray(self.vertices)
        _, nearest_index = kdtree.query((point.row,point.col),k=[1])
        return self.vertices[nearest_index[0]]

    def extend_node(self, nearest_node, random_pt, step_size):
        step_size_total_distance_ratio = step_size/self.dis(nearest_node,random_pt)
        if step_size_total_distance_ratio < 1:
            new_node_col = int((1-step_size_total_distance_ratio)*nearest_node.col + step_size_total_distance_ratio*random_pt.col)
            new_node_row = int((1-step_size_total_distance_ratio)*nearest_node.row + step_size_total_distance_ratio*random_pt.row)
        else:
            new_node_col = random_pt.col
            new_node_row = random_pt.row

        new_node = Node(new_node_row,new_node_col)
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + self.dis(nearest_node,new_node)
        return new_node

    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbors = []
        kdtree = spatial.KDTree(self.vertices_pts)
        pairs = kdtree.query_ball_point((new_node.row,new_node.col),neighbor_size)
        for pair_id in pairs:
            neighbors.append(pair_id)
        return neighbors


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        for neighbor_id in neighbors:
            neighbor_node = self.vertices[neighbor_id]
            update_cost = neighbor_node.cost + self.dis(neighbor_node,new_node)
            if new_node.cost > update_cost:
                if self.check_collision(neighbor_node,new_node):
                    new_node.parent = neighbor_node
                    new_node.cost = update_cost
        
        for neighbor_id in neighbors:
            neighbor_node = self.vertices[neighbor_id]
            update_cost = new_node.cost + self.dis(neighbor_node,new_node)
            if update_cost < neighbor_node.cost:
                if self.check_collision(neighbor_node,new_node):
                    neighbor_node.parent = new_node
                    neighbor_node.cost = update_cost
            
    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.
        count = 0
        while((self.found == False) and count<n_pts):
            count += 1
            random_pt = self.get_new_point(0.1)
            if (random_pt.row,random_pt.col) not in self.vertices_pts:
                nearest_node = self.get_nearest_node(random_pt)
                new_node = self.extend_node(nearest_node, random_pt,self.step_size)
                if self.check_collision(nearest_node,new_node):
                    if self.map_array[new_node.row,new_node.col] == 1:
                        self.vertices.append(new_node)
                        self.vertices_pts.append((new_node.row, new_node.col))
                        # self.get_neighbors(new_node,10)
                        distance = self.dis(new_node,self.goal)
                        if distance <= self.goal_vicinity:
                            if self.check_collision(new_node,self.goal):
                                self.found = True
                                self.goal.parent = new_node
                                self.goal.cost = new_node.cost + distance        
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.
        count = 0
        while((self.found == False) and count<n_pts):
            count += 1
            random_pt = self.get_new_point(0.1)
            if (random_pt.row, random_pt.col) not in self.vertices_pts:
                nearest_node = self.get_nearest_node(random_pt)
                new_node = self.extend_node(nearest_node, random_pt, self.step_size)
                if self.map_array[new_node.row,new_node.col] == 1:
                    if self.check_collision(nearest_node,new_node):
                        self.vertices.append(new_node)
                        self.vertices_pts.append((new_node.row, new_node.col))
                        # self.get_neighbors(new_node,10)
                        neighbors = self.get_neighbors(new_node, neighbor_size)
                        self.rewire(new_node, neighbors)
                        distance = self.dis(new_node, self.goal)
                        if distance <= self.goal_vicinity:
                            if self.check_collision(new_node, self.goal):
                                self.found = True
                                self.goal.parent = new_node
                                self.goal.cost = new_node.cost + distance
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
