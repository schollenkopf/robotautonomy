import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import  NavigateToPose
from lidarsubnode.map import *
from visualization_msgs.msg import MarkerArray

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.lidar_subscription = self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map2', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom',self.odom_callback,10)
        
        self.lidar_subscription  # prevent unused variable warning
        self.odom_subscription # prevent unused variable warning
        self.map = Map()

    def lidar_callback(self, msg):
        grid = self.map.update(msg)
        self.map_publisher.publish(grid)


    def odom_callback(self,msg):
        self.map.update_odom(msg)
        





def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

if __name__ == '__main__':
    main()