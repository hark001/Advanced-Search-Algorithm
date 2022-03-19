# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

from turtle import distance
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial
from bresenham import bresenham
import math
# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
        line_pts = bresenham(p1[1],p1[0],p2[1],p2[0])
        # print("p1: ",p1," p2: ",p2)
        for (x,y) in line_pts:
            # print("x: ",x," y: ",y)
            if self.map_array[y][x] == 0:
                return False
        return True


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        dist = math.sqrt((point2[1]-point1[1])**2 + (point2[0]-point1[0])**2)
        # distance = np.sqrt(np.sum(np.square(point2 - point1)))
        return dist


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        self.samples.append((0, 0))
        div_ratio = math.floor(math.sqrt(n_pts))
        for i in range(0,self.size_row,round(self.size_row/div_ratio)):
            for j in range(0,self.size_col,round(self.size_row/div_ratio)):
                row = i
                col = j
                
                if self.map_array[row,col] == 1:
                    if (row,col) not in self.samples:
                        self.samples.append((row, col))
        kdtree = spatial.KDTree(self.samples)
        pairs = kdtree.query_pairs(15)
        valid_pairs = []
        for pair in pairs:
            if self.check_collision(self.samples[pair[0]],self.samples[pair[1]]):
                valid_pairs.append(pair)
        # print("pair size :",len(pairs))
        # print("pairs :",pairs)
        self.graph.add_nodes_from(range(len(self.samples)))
        self.graph.add_edges_from(valid_pairs)

    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        self.samples.append((0, 0))
        for i in range(n_pts):
        # while(len(self.samples) <= n_pts):
            row = np.random.random_integers(0,self.size_row-1)
            col = np.random.random_integers(0,self.size_col-1)
            
            if self.map_array[row,col] == 1:
                if (row,col) not in self.samples:
                    self.samples.append((row, col))
        kdtree = spatial.KDTree(self.samples)
        pairs = kdtree.query_pairs(30)
        valid_pairs = []
        for pair in pairs:
            if self.check_collision(self.samples[pair[0]],self.samples[pair[1]]):
                valid_pairs.append(pair)
        # print("pair size :",len(pairs))
        # print("pairs :",pairs)
        self.graph.add_nodes_from(range(len(self.samples)))
        self.graph.add_edges_from(valid_pairs)

    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        self.samples.append((0, 0))
        
        for i in range(int(n_pts/2)):
            # while(len(self.samples) <= n_pts):
            row = np.random.random_integers(0,self.size_row-1)
            col = np.random.random_integers(0,self.size_col-1)
            
            # if self.map_array[row,col] == 1:
            # if (row,col) not in self.samples:
            guass_dist = int(np.random.normal(0,10,1))
            gauss_pt_row = guass_dist + row
            gauss_pt_col = guass_dist + col
            if (0 <= gauss_pt_row < self.size_row) and (0 <= gauss_pt_col < self.size_col):
                if (gauss_pt_row, gauss_pt_col) not in self.samples:
                    if self.map_array[row,col] ^ self.map_array[gauss_pt_row,gauss_pt_col]:
                        if self.map_array[row,col] == 1:
                            self.samples.append((row, col))
                        else:
                            self.samples.append((gauss_pt_row, gauss_pt_col))
                    else:
                        continue
        
        kdtree = spatial.KDTree(self.samples)
        pairs = kdtree.query_pairs(55)
        valid_pairs = []
        for pair in pairs:
            if self.check_collision(self.samples[pair[0]],self.samples[pair[1]]):
                valid_pairs.append(pair)
        # print("pair size :",len(pairs))
        # print("pairs :",pairs)
        self.graph.add_nodes_from(range(len(self.samples)))
        self.graph.add_edges_from(valid_pairs)

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        self.samples.append((0, 0))
        
        for i in range(int(n_pts)):
            # while(len(self.samples) <= n_pts):
            row = np.random.random_integers(0,self.size_row-1)
            col = np.random.random_integers(0,self.size_col-1)
            
            if self.map_array[row,col] == 0:
                # if (row,col) not in self.samples:
                guass_dist = int(np.random.normal(0,20,1))
                gauss_pt_row = guass_dist + row
                gauss_pt_col = guass_dist + col

                if (0 <= gauss_pt_row < self.size_row) and (0 <= gauss_pt_col < self.size_col):
                    if (gauss_pt_row, gauss_pt_col) not in self.samples:
                            if self.map_array[gauss_pt_row,gauss_pt_col] == 0:
                                new_pt_row = int((gauss_pt_row + row)/2)
                                new_pt_col = int((gauss_pt_col + col)/2)
                                if self.map_array[new_pt_row,new_pt_col] == 1:
                                    self.samples.append((new_pt_row,new_pt_col))
        
        kdtree = spatial.KDTree(self.samples)
        pairs = kdtree.query_pairs(50)
        valid_pairs = []
        for pair in pairs:
            if self.check_collision(self.samples[pair[0]],self.samples[pair[1]]):
                valid_pairs.append(pair)

        self.graph.add_nodes_from(range(len(self.samples)))
        self.graph.add_edges_from(valid_pairs)


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        pairs = []
        for pair in self.graph.edges():
            weight = self.dis(self.samples[pair[0]],self.samples[pair[1]])
            pairs.append((pair[0],pair[1],weight))
                
        # for pair in :
            
        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from([])
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []


        # spatial.KDTree()
        # pairs = kdtree.query_pairs(0.15)
        # self.graph.add_edges_from(list(pairs))
        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        
        start_pairs = []
        goal_pairs = []

        kdtree = spatial.KDTree(self.samples)
        
        pairs = []
        radius = 15
        while(len(pairs)<=4):
            pairs = kdtree.query_ball_point(start,radius)
            radius += 5
        
        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])
        
        for pair in pairs:
            if self.check_collision(start,self.samples[pair]):
                start_pairs.append(('start',pair,self.dis(start,self.samples[pair])))
        
        pairs = []
        radius = 15
        while(len(pairs)<=4):
            pairs = kdtree.query_ball_point(goal,radius)
            radius += 5

        for pair in pairs:
            if self.check_collision(goal,self.samples[pair]):
                goal_pairs.append(('goal',pair,self.dis(goal,self.samples[pair])))
        # goal_pairs = kdtree.query_ball_point(start,15)
        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        