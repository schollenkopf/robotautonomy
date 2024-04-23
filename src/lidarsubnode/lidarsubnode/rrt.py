"""
RRT_2D
@author: huiming zhou
"""



import math
import numpy as np
from lidarsubnode.raycast import *






class NodeRrt:
    def __init__(self, n, id):
        self.id = id
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, s_start,  step_len, num_nodes, map_size, occupancy_grid, cell_size):
        self.s_start = NodeRrt(s_start,0)
        self.step_len = step_len
        
        self.iter_max = num_nodes
        self.occupancy_grid = occupancy_grid
        self.cell_size = cell_size
        self.vertex = [self.s_start]

        self.x_range = map_size
        self.y_range = map_size
        

    def planning(self):
        self.gain = {}
        self.gain[0] = 0
        g_best = 0
        n_best = self.s_start
        for i in range(self.iter_max):
            node_rand = self.generate_random_node()
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.path_collides_obstacle(node_near, node_new):
                self.vertex.append(node_new)
                g_new = self.calc_gain(node_new)
                if g_new > g_best:
                    n_best = node_new
                    g_best = g_new
        print("number nodes:",len(self.vertex))
        print("best gain:", g_best)
        return self.extract_path(n_best)
    
    def calc_gain(self,node):
        
        prev_gain = self.gain[node.parent.id] 
        visible =  ray_cast_gain(self.occupancy_grid,node.x,node.y,self.cell_size)
        y = 0.7
        edge_length = math.sqrt((node.parent.y-node.y)**2+(node.parent.x-node.x)**2)
        gain =  prev_gain + visible * math.e**(-y*edge_length)
        self.gain[node.id] = gain
        return gain

    
    def path_collides_obstacle(self, node1, node2):
        a = math.atan2(node2.y - node1.y, node2.x - node1.x)
        r = math.sqrt((node2.y-node1.y)**2+(node2.x-node1.x)**2)
        cells = ray_cast(r,a,node1.x/self.cell_size+100,node1.y/self.cell_size+100,self.cell_size)
        for map_x,map_y in cells:
            if self.occupancy_grid[map_x,map_y] != 0 and self.occupancy_grid[map_x,map_y] <= 100:
                return True
        return False

    def generate_random_node(self):
        delta = 0.5

        return NodeRrt((np.random.uniform(-self.x_range/2 , self.x_range/2),
                         np.random.uniform(-self.y_range/2, self.y_range/2)),-1)


    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = NodeRrt((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)),len(self.vertex))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = []
        node_now = node_end
        distance = 0
        while node_now.parent is not None and distance < 3.5:
            node_now = node_now.parent
            distance = math.sqrt((node_now.y-self.s_start.y)**2+(node_now.x-self.s_start.x)**2)
            path.append((node_now.x, node_now.y))
        if distance>3.5:
            print("distance > 3.5")
            return (node_now.x, node_now.y)
        return path[-3]

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


