import numpy as np
import math
from collections import defaultdict
from lidarsubnode.raycast import *

def find_k_nearest_neighbors(nodes, k):
    """Findes the indicies of the K nearest neighbors of to any give node

    Args:
        nodes (List[Tuple[float, float]]): List of the nodes positions
        k (int): Number of nearest nodes to connect to

    Returns:
        dict: dictionary where the key-value pairs are:  node_idx: [neighbor_1_idx, neighbor_2_idx, ..., neighbor_k_idx]
    """
    node_neighbors = defaultdict(list)

    for i, node1 in enumerate(nodes):
        distances = []
        for j, node2 in enumerate(nodes):
            if i != j:
                (x1, y1), (x2, y2) = node1, node2
                dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                distances.append((dist, j))
        k_nearest = sorted(distances)[:k]
        for _, neighbor_idx in k_nearest:
            node_neighbors[i].append(neighbor_idx)

    return node_neighbors

class PRM:
    def __init__(self, num_nodes, map_size, num_neighbors, occupancy_grid, cell_size):
        self.num_nodes = num_nodes
        self.map_size = map_size
        self.num_neighbors = num_neighbors
        self.nodes = []
        self.path = []
        self.node_neighbors = None
        self.edges = defaultdict(list)
        self.occupancy_grid = occupancy_grid
        self.cell_size = cell_size
        self.start = None
        self.start_idx = None

        self.goal = None
        self.goal_idx = None

    def generate_random_nodes(self):
        for _ in range(self.num_nodes):
            collides = True
            while collides:
              x = np.random.uniform(-self.map_size/2, self.map_size/2)
              y = np.random.uniform(-self.map_size/2, self.map_size/2)
              # Check if nodes collides with obstacles
              node = (x, y)
              collides =  self.collides_with_obstacle(node)
            self.nodes.append(node)


    def compute_edges(self):
        # Calculate the k-nearest neighbors, and store them for future use.
        self.node_neighbors = find_k_nearest_neighbors(self.nodes, self.num_neighbors)

        # For each neighbor, check if the direct path is obstacle free.

        for i, node in enumerate(self.nodes):
            
            for neighbor_idx in self.node_neighbors[i]:
                neighbor = self.nodes[neighbor_idx]
                # If path is free, calculate distance and create the edge
                if neighbor_idx != self.start_idx:
                    if not self.path_collides_obstacle(node, neighbor):
                        dist = math.sqrt(
                            (node[0] - neighbor[0]) ** 2 + (node[1] - neighbor[1]) ** 2
                        )
                        
                        self.edges[i].append((neighbor_idx, dist))
        print(self.edges)

    def collides_with_obstacle(self, node1):
        x, y = node1
        map_x = int(100+x/self.cell_size)
        map_y = int(100+y/self.cell_size)
        if self.occupancy_grid[map_x,map_y] == 0 or self.occupancy_grid[map_x,map_y] > 100:
            return False
        return True

    def path_collides_obstacle(self, node1, node2):
        x2,y2 = node2
        x1, y1 = node1
        a = math.atan2(y2 - y1, x2 - x1)
        r = math.sqrt((y2-y1)**2+(x2-x1)**2)
        cells = ray_cast(r,a,x1/self.cell_size+100,y1/self.cell_size+100,self.cell_size)
        for map_x,map_y in cells:
            if self.occupancy_grid[map_x,map_y] != 0 and self.occupancy_grid[map_x,map_y] <= 100:
                return True
        return False

    def add_robot(self, map_x, map_y):
        # Set start and goal positions, and add them to the nodes
        self.start = ((map_x-100)*self.cell_size,(map_y-100)*self.cell_size)

        self.start_idx = len(self.nodes)
        self.nodes.append(self.start)
        print(self.nodes)

    def calc_gain(self,edge_length,from_node_idx,node):
        x,y = node
        prev_gain = self.gain[from_node_idx] 
        visible =  ray_cast_gain(self.occupancy_grid,x,y,self.cell_size)
        y = 0.7
        return prev_gain + visible * math.e**(-y*edge_length)
    
    def extract_path(self,best_path_end):
        path = []
        u = best_path_end
        while u != self.start_idx:
            path.append(self.previous[u])
            u = self.previous[u]
        return path
    


    def next_best_view(self):

        g_best = 0
        n_best = self.start_idx
        n_T = 1

        self.gain = {}
        self.previous = {}
        pq = PQ()
        for idx, v in enumerate(self.nodes):
            self.gain[idx] = 0
            self.previous[idx] = None
            pq.add_task(idx, self.gain[idx])
        self.gain[self.start_idx] = 0
        pq.update_task(self.start_idx, 0.1)
       
        while len(pq.pq) != 0:
            p,q = pq.pop_task()
            print(q)
            if p == 0:
                break
            
            for (n,w) in self.edges[q]:
                if not n in self.extract_path(q):
                    g_new = self.calc_gain(w,q,self.nodes[n])
                    if self.gain[n] < g_new:
                        self.gain[n] = g_new
                        self.previous[n] = q
                        pq.update_task(n, g_new)
                        if g_new > g_best:
                            n_best = n
        path = self.extract_path(n_best)
        print(path)
        return self.nodes[path[-2]]
        








class PQ:


    pq = []                         # list of entries arranged in a heap


    def add_task(self,task, prio):
        self.pq.append((prio, task))
        self.pq.sort(reverse = True)
    def pop_task(self):
        return self.pq.pop(0)
    def update_task(self,task,prio):
        for i,(p,t) in enumerate(self.pq):
            if t == task:
                self.pq[i] = (prio,task)
                self.pq.sort(reverse = True)




