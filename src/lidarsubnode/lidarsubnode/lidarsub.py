import rclpy
from rclpy.node import Node
# from rclpy import time
import numpy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

import laser_geometry.laser_geometry as lg


class Map():
    previous_cloud = []
    mean = []
    map_size = (100,100)
    cell_size=0.1
    step_size = min(0.1,0.1)/2
    center_x = 50
    center_y = 50

    def numpy_to_occupancy_grid(self,arr):
        grid = OccupancyGrid()
        grid.data = arr.flatten().astype(np.int8).tolist()
        grid.info = MapMetaData()
        grid.info.height = arr.shape[0]
        grid.info.width = arr.shape[1]
        grid.info.resolution = 0.1
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.7071068
        grid.info.origin.orientation.w =  0.7071068
        grid.info.origin.position.x = 7.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.position.z = 0.0
        grid.header.frame_id = "map"

        return grid



    def get_point(self,r,a):
        x = r*np.cos(a)
        y = r*np.sin(a)
        if abs(x) > 1000 or abs(y) > 1000:
            x = 0
            y = 0
        return (x,y)

    def draw_point(self,r, a):
        if r>300:
            r=5
        for t in range(math.ceil(r/self.step_size)):
            t = t * self.step_size

            x = t*np.cos(a)
            y = t*np.sin(a)
            map_x = int(50+x/self.cell_size)
            map_y = int(50+y/self.cell_size)
            self.map[map_x,map_y] = 0
        self.map[map_x,map_y] = 50

        
    
    def create_map(self,msg):
        self.map = np.full(self.map_size,-1,dtype=np.int8) #one cell = 0.1*0.1
        
        ranges = msg.ranges
        inc = msg.angle_increment
        angles = [inc*i for i in range(len(ranges))]
        
        vdraw_point = np.vectorize(self.draw_point)
        vdraw_point(ranges,angles)
        grid = self.numpy_to_occupancy_grid(self.map)
        grid.header.stamp = msg.header.stamp
        return grid




    def update(self,msg):

        ranges = msg.ranges
        inc = msg.angle_increment
        angles = [inc*i for i in range(len(ranges))]
        
        vget_point = np.vectorize(self.get_point)
        points_x, points_y = vget_point(ranges,angles)


        x_mean = np.mean(points_x)
        y_mean = np.mean(points_y)



        points_x = points_x - x_mean 
        points_y = points_y - y_mean
        mean = np.array([x_mean,y_mean])

        points = np.array(list(zip(points_x,points_y)))
        # print(points.shape)
        if len(self.previous_cloud) == 0:
            self.previous_cloud = points
            self.mean = mean
            return points
        
        w = np.matmul(self.previous_cloud,points)
        c =  np.cov(w)
        print(w.shape)
        print(self.previous_cloud.shape)
        print(c.shape)
        u,d,vh = np.linalg.svd(c)
        print(u.shape)
        print(vh.shape)

        

        r = np.dot(u,vh.T)


        
        t = self.mean  - np.dot(r,mean)


        self.previous_cloud = points
        self.x_mean = x_mean
        self.y_mean = y_mean




        return (r,t)

        

    




class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map2', 10)
        
        self.subscription  # prevent unused variable warning
        self.map = Map()

    def listener_callback(self, msg):
        # l = len(msg.ranges)
        # self.get_logger().info(len(msg.ranges))
        # self.get_logger().info("angles:")
        # self.get_logger().info(msg.angle_increment)
        
        grid = self.map.create_map(msg)
        
        # self.get_logger().info('ranges: "%s"' % ranges)
        # self.get_logger().info('len: "%s"' % len(points))
        self.map_publisher.publish(grid)
        self.get_logger().info('points: "%s"' % grid)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()