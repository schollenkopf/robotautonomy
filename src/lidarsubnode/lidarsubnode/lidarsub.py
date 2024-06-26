import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from rclpy import time
import numpy
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData, Odometry
from nav2_msgs.msg import ParticleCloud
from nav2_msgs.action import  NavigateToPose
from lidarsubnode.rrt import *
from lidarsubnode.raycast import *

import laser_geometry.laser_geometry as lg


class Map():
    previous_cloud = []
    mean = []
    map_size = (200,200)
    cell_size=0.1
    center = np.array([100,100])
    last_rotation_timestamp = -10
    
    angle_offset = 0

    def __init__(self):
        self.map = np.full(self.map_size,-1,dtype=np.int8) #one cell = 0.1*0.1

    def numpy_to_occupancy_grid(self,arr):
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



    def get_point(self,r,a):
        x = r*np.cos(a)
        y = r*np.sin(a)
        if abs(x) > 1000 or abs(y) > 1000:
            x = 0
            y = 0
        return (x,y)

    def draw_point(self,r, a,color=False):
        if r>3.5:
            r=3.5
        cells = ray_cast(r,a,self.center[0],self.center[1],self.cell_size)
        for map_x,map_y in cells:
            self.map[map_x,map_y] = max(self.map[map_x,map_y]-1,0)
            if color:
                self.map[map_x,map_y] = 220
        if r<3.5 and not color:
            self.map[map_x,map_y] = max(self.map[map_x,map_y]+1,100)
        
    
    def update_map(self,msg):
        
        ranges = msg.ranges
        
        inc = msg.angle_increment
        angles = [inc*i+self.angle_offset for i in range(len(ranges))]
        
        vdraw_point = np.vectorize(self.draw_point)
        vdraw_point(ranges,angles)


    def icp(self,points,mean):
        w = self.previous_cloud @ points.T
        c =  np.cov(w)
        u,d,vh = np.linalg.svd(c)
        r = u @ vh
        t = self.previous_mean  - r @ mean
        return (r,t)
    
    def run_icp(self,msg):
        ranges = msg.ranges
        inc = msg.angle_increment
        angles = [inc*i for i in range(len(ranges))]
        vget_point = np.vectorize(self.get_point)
        points_x, points_y = vget_point(ranges,angles)
        points = np.array(list(zip(points_x,points_y))).T


        # print(points.shape)
        if len(self.previous_cloud) == 0:
            self.previous_cloud = points
            return None,None
        
        mean = np.mean(points, axis=1).reshape(2, 1)

        points = points - mean
        
        self.previous_mean = np.mean(self.previous_cloud, axis=1).reshape(2, 1)
        self.previous_cloud = self.previous_cloud - self.previous_mean


        max_iter = 50
        tolerance = 1e-4
        for i in range(max_iter):
            # Iterate through the ICP algorithm
            R, t = self.icp(points, mean)

            # Apply transformation to align the old point cloud
            self.previous_cloud = R @ self.previous_cloud + t  # (2,2) @ (2,n) + (2,1) = (2,n)

            # Compute the change in the computed transformation
            change = np.linalg.norm(t)


            #Check for convergence
            if change < tolerance:
                # print(
                #     f"Converged after {i} iterations, change: {change}"
                # )
                break

        # if i == max_iter-1:
        #     print(f"Failed to converge after {max_iter} iterations")
        # print(f'R:\n"{R}"')
        # print(f't:\n"{t}"')

        self.previous_cloud = points + mean
        return R,t


    def update_odom(self,msg):
        self.center = np.array([msg.pose.pose.position.x/self.cell_size+ 100,msg.pose.pose.position.y/self.cell_size + 100])
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
        if icp:
            R,t = self.run_icp(msg)
            if t is not None:
                self.center = self.center + t.flatten()/self.cell_size
                self.angle_offset = math.acos(max(-1,min(R[0,0],1)))
                self.update_map(msg)
        elif self.last_rotation_timestamp < msg.header.stamp.sec - 1:
            self.update_map(msg)
        # self.draw_point(0.5,self.angle_offset,True)
        # self.map[int(self.center[0]),int(self.center[1])] = 200

        grid = self.numpy_to_occupancy_grid(self.map)
        grid.header.stamp = msg.header.stamp
        return grid
    
    def explore_next_step(self):
        rrt = Rrt(((self.center[0]-100)*self.cell_size,(self.center[1]-100)*self.cell_size),step_len=1.5,num_nodes=1000,map_size=20,occupancy_grid=self.map,cell_size=self.cell_size)
        return rrt.planning()


        
        

    




class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map2', 10)

        self.particles = self.create_subscription(
        ParticleCloud,
        '/particle_cloud',
        self.particle_callback,
        qos_profile=QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1

        ))

        self.navigate_to = ActionClient(self,NavigateToPose,'/navigate_to_pose')

        self.odom_subscription = self.create_subscription(Odometry, '/odom',self.odom_callback,10)

        self.particles
        
        self.subscription  # prevent unused variable warning
        self.delay = 15
        self.map = Map()
        self.pose_sent = False




    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected :(')
        else:
            print('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print('Result: {0}'.format(result))
        self.send_pose()

    

    def send_pose(self):
        print("Calculating pose")
        x,y = self.map.explore_next_step()
        print("x,y:",x,y)
        pose_goal = NavigateToPose.Goal()
        pose_goal.pose.header.frame_id = 'map'
        pose_goal.pose.pose.position.x = x
        pose_goal.pose.pose.position.y = y
        self._send_goal_future = self.navigate_to.send_goal_async(pose_goal)

        self._send_goal_future.add_done_callback(self.goal_response_callback)



    def listener_callback(self, msg):
        grid = self.map.update(msg)

        self.map_publisher.publish(grid)
        if not self.pose_sent:
            self.send_pose()
            self.pose_sent = True

    def odom_callback(self,msg):
        self.map.update_odom(msg)
        

    def particle_callback(self,msg):
        print(len(msg.particles))



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