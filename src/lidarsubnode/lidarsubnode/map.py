import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from lidarsubnode.raycast import *


class Map():
    previous_cloud = []
    mean = []
    map_size = (200,200)
    cell_size=0.1
    robot_cell = np.array([100,100])
    last_rotation_timestamp = -10
    
    angle_offset = 0

    def __init__(self):
        self.map = np.full(self.map_size,-1,dtype=np.int8) #one cell = 0.1*0.1

    def numpy_to_occupancy_grid(self,arr):
        #used to publish map to topic
        grid = OccupancyGrid()
        grid.data = arr.flatten().astype(np.int8).tolist()
        grid.info = MapMetaData()
        grid.info.height = arr.shape[0]
        grid.info.width = arr.shape[1]
        grid.info.resolution = 0.1
        grid.info.origin.orientation.x = 0.7071068
        grid.info.origin.orientation.y = 0.7071068
        grid.info.origin.orientation.z = 0.
        grid.info.origin.orientation.w =  0.
        grid.info.origin.position.x = -10.0
        grid.info.origin.position.y = -10.0
        grid.info.origin.position.z = 0.0
        grid.header.frame_id = "map"

        return grid

    def draw_point(self,r, a):
        #fill out cells of map according to the lidar measurement
        if r>3.5:
            r=3.5
        cells = ray_cast(r,a,self.robot_cell[0],self.robot_cell[1],self.cell_size)
        for map_x,map_y in cells:
            self.map[map_x,map_y] = max(self.map[map_x,map_y]-1,0)
        if r<3.5:
            self.map[map_x,map_y] = 100
        
    def update_map(self,msg):
        #draw points for all measurement in a lidar scan
        ranges = msg.ranges
        inc = msg.angle_increment
        angles = [inc*i+self.angle_offset for i in range(len(ranges))]
        vdraw_point = np.vectorize(self.draw_point)
        vdraw_point(ranges,angles)


    def update_odom(self,msg):
        #get the robots pose from the odometry
        self.robot_coords = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])
        self.robot_cell = np.round(self.robot_coords/self.cell_size + 100)
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        new_angle_offset = math.atan2(t3, t4)
        if abs(new_angle_offset-self.angle_offset)>0.05:
            self.last_rotation_timestamp = msg.header.stamp.sec
        self.angle_offset = new_angle_offset

    def update(self,msg,icp=False):
        #update the map only if the robot has not rotated for at least 1 sec to avoid artifacts in the map
        if self.last_rotation_timestamp < msg.header.stamp.sec - 1:
            self.update_map(msg)
        grid = self.numpy_to_occupancy_grid(self.map)
        grid.header.stamp = msg.header.stamp
        return grid
    


        
        

    

