import numpy as np
import math
from collections import defaultdict
from shapely.geometry import LineString, Point


def circle_line_intersection(node1, node2, obstacle) -> bool:
    """Check if line intersects with circle

    Args:
        node1 (_type_): The (x, y) location of the first node
        node2 (_type_): The (x, y) location of the second node
        obstacle (_type_): The (x, y, r) location and radius of the obstacle

    Returns:
        bool: True if line intersectecs with circle, else False
    """

    from shapely.geometry import LineString, Point

    x1, y1 = node1
    x2, y2 = node2
    cx, cy, r = obstacle

    p_o = Point((cx, cy))
    c = p_o.buffer(r).boundary
    l = LineString([(x1, y1), (x2, y2)])
    i = c.intersection(l)

    return False if i.is_empty else True

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
        self.obstacles = []
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
              x = np.random.uniform(0, self.map_size)
              y = np.random.uniform(0, self.map_size)
              # Check if nodes collides with obstacles
              node = (x, y)
              if not self.collides_with_obstacle(node):
                collides = False
            self.nodes.append(node)

    def compute_edges(self):
        # Calculate the k-nearest neighbors, and store them for future use.
        self.node_neighbors = find_k_nearest_neighbors(self.nodes, self.num_neighbors)

        # For each neighbor, check if the direct path is obstacle free.

        for i, node in enumerate(self.nodes):
            for neighbor_idx in self.node_neighbors[i]:
                neighbor = self.nodes[neighbor_idx]
                # If path is free, calculate distance and create the edge
                if not self.path_collides_obstacle(node, neighbor):
                    dist = math.sqrt(
                        (node[0] - neighbor[0]) ** 2 + (node[1] - neighbor[1]) ** 2
                    )
                    self.edges[i].append((neighbor_idx, dist))

    def collides_with_obstacle(self, node1):


        x, y = node1
        map_x = int(self.center[0]+x/self.cell_size)
        map_y = int(self.center[1]+y/self.cell_size)



        if self.occupancy_grid(map_x,map_y) == 0:
            return True
        return False

    def path_collides_obstacle(self, node1, node2):
        for obstacle in self.obstacles:
            if circle_line_intersection(node1, node2, obstacle):
                return True
        return False

    def add_start_and_goal(self, start, goal):
        if self.collides_with_obstacle(start):
            print("Error: Starting point is not valid")
            exit(-1)

        if self.collides_with_obstacle(goal):
            print("Error: Goal point is not valid")
            exit(-1)

        # Set start and goal positions, and add them to the nodes
        self.start, self.goal = start, goal

        self.start_idx = len(self.nodes)
        self.nodes.append(start)

        self.goal_idx = len(self.nodes)
        self.nodes.append(goal)

        # recompute the edges to attach the start and goal nodes
        self.compute_edges()

    def add_obstacle(self, x, y, radius):
        self.obstacles.append((x, y, radius))

    
    def add_path(self,previous):
        self.path = []
        u = self.goal_idx
        while u is not None and u != self.start_idx:
            self.path.append(previous[u])
            u = previous[u]
        print(self.path)

