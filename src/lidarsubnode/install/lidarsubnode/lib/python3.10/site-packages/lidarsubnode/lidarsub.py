import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import LaserScan
import math
import numpy as np

import laser_geometry.laser_geometry as lg


class Map():
    previous_cloud = []
    mean = []

    def get_point(self,r, a):
        x = r*np.cos(a)
        y = r*np.sin(a)
        if abs(x) > 1000 or abs(y) > 1000:
            x = 0
            y = 0
        return (x,y)

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
        

        c =  np.cov(self.previous_cloud.T,points.T)
        print(points.T.shape)
        print(self.previous_cloud.T.shape)
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
        self.subscription  # prevent unused variable warning
        self.map = Map()

    def listener_callback(self, msg):
        # l = len(msg.ranges)
        # self.get_logger().info(len(msg.ranges))
        # self.get_logger().info("angles:")
        # self.get_logger().info(msg.angle_increment)
        
        points = self.map.update(msg)
        
        # self.get_logger().info('ranges: "%s"' % ranges)
        # self.get_logger().info('len: "%s"' % len(points))
        self.get_logger().info('points: "%s"' % points)


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