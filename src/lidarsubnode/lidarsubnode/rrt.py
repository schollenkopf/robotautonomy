"""
RRT_2D
@author: huiming zhou
"""



import math
import numpy as np
from lidarsubnode.raycast import *
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point


class NodeRrt:
    def __init__(self, n, id, path_len = 0, gain = 0):
        self.id = id
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.path_len = path_len
        self.gain = gain


class Rrt:
    def __init__(self, robot_node,  step_len, iter_max, map_size, occupancy_grid, cell_size,path_publisher):
        self.robot_node = NodeRrt(robot_node,0)
        self.step_len = step_len
        self.iter_max = iter_max
        self.occupancy_grid = occupancy_grid
        self.cell_size = cell_size
        self.nodes = [self.robot_node]
        self.path_publisher = path_publisher
        self.pose_sequence = []

        self.x_range = map_size
        self.y_range = map_size
    
    def plot_path(self,path_id,n_best):
        end_node = self.nodes[-1]
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        scale = Vector3()
        scale.x = 0.01
        scale.y = 0.01
        scale.z = 0.0
        marker.scale = scale
        marker.id = path_id
        marker.header.frame_id = "map"
        marker.color.r = 1.0
        marker.color.a = 1.0

        #color chosen bath green
        if end_node.id == n_best.id:
            marker.color.r = 0.0
            marker.color.g = 1.0

        #iterate from end of path until hitting a path already plotted
        while end_node is not None and end_node in self.nodes:
            self.nodes.remove(end_node)
            point = Point()
            point.x = end_node.x
            point.y = end_node.y
            marker.points.append(point)
            end_node = end_node.parent
        
        #plot connection to path already plotted
        if end_node is not None:
            point = Point()
            point.x = end_node.x
            point.y = end_node.y
            marker.points.append(point)
            
        return marker
    
    def plot_paths(self,n_best):
        marker_array = MarkerArray()
        self.nodes
        path_id = 0
        #plot paths from leaf to robot until all nodes are plotted
        while self.nodes is not None and len(self.nodes) > 0:
            marker_array.markers.append(self.plot_path(path_id,n_best))
            path_id += 1

        #publish path plot
        self.path_publisher.publish(marker_array)

    def planning(self):

        g_best = 0 #best found gain value
        n_best = self.robot_node #node with best gain
        g_max = (3.5/self.cell_size/2) ** 2 * math.pi * math.exp(-0.7 * self.step_len) * 3  #gain value at which search stops because it corresponds to aprox a full unknow circle scan
        print("gmx",g_max) 
        
        
        for i in range(self.iter_max):
            node_rand = self.generate_random_node()
            node_near = self.nearest_neighbor(self.nodes, node_rand) #find node closest to new node
            node_new = self.new_state(node_near, node_rand) #move in direction of new node from closest node according to step_len

            #check if new node and path to that node is not hitting any occupied cell
            if node_new and not self.path_collides_obstacle(node_near, node_new):
                self.nodes.append(node_new)
                g_new = self.calc_gain(node_new)
                if g_new > g_best:
                    n_best = node_new
                    g_best = g_new
                if g_best >= g_max: 
                    break
        print("number nodes:",len(self.nodes))
        
        print("best gain:", g_best)
        best_path = self.extract_path(n_best)
        self.plot_paths(n_best)
        return best_path
    
    def calc_gain(self,node):
        prev_gain = node.parent.gain
        visible =  ray_cast_gain(self.occupancy_grid,node.x,node.y,self.cell_size)
        y = 0.01
        edge_length = math.sqrt((node.parent.y-node.y)**2+(node.parent.x-node.x)**2)
        path_length = edge_length + node.parent.path_len
        node.path_len = path_length
        gain =  prev_gain + visible * math.e**(-y*edge_length)
        node.gain = gain
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
        return NodeRrt((np.random.uniform(-self.x_range/2 , self.x_range/2),
                         np.random.uniform(-self.y_range/2, self.y_range/2)),-1)


    def nearest_neighbor(self,node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = NodeRrt((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)),len(self.nodes))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = []
        node_now = node_end
        distance = 0
        while node_now.parent is not None:
            node_now = node_now.parent
            distance = math.sqrt((node_now.y-self.robot_node.y)**2+(node_now.x-self.robot_node.x)**2)
            path.append((node_now.x, node_now.y))
            if distance<3.5:
                print("distance < 3.5")
                return (node_now.x, node_now.y)
        return path[-2]

    def get_distance_and_angle(self,node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


