import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import  NavigateToPose
from lidarsubnode.map import *
from visualization_msgs.msg import MarkerArray

class NextBestViewNode(Node):

    def __init__(self):
        super().__init__('next_best_view')
        self.map2_subscription = self.create_subscription(OccupancyGrid, '/map2', self.map2_callback, 10)
        self.rrt_publisher = self.create_publisher(MarkerArray, '/plan2', 10)
        self.navigate_to = ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.odom_subscription = self.create_subscription(Odometry, '/odom',self.odom_callback,10)
        
        self.odom_subscription # prevent unused variable warning
        self.pose_sent = False
        self.robot_coords = None
        self.map2 = None

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
        if self.robot_coords is not None and self.map2 is not None:
            self.pose_sent = True
            print("Calculating pose")
            rrt = Rrt(self.robot_coords,step_len=1.5,iter_max=1000,map_size=20,occupancy_grid=self.map2,cell_size=0.1,path_publisher = self.rrt_publisher)
            x,y = rrt.planning()
            print("x,y:",x,y)
            pose_goal = NavigateToPose.Goal()
            pose_goal.pose.header.frame_id = 'map'
            pose_goal.pose.pose.position.x = x
            pose_goal.pose.pose.position.y = y
            self._send_goal_future = self.navigate_to.send_goal_async(pose_goal)
            self._send_goal_future.add_done_callback(self.goal_response_callback)


    def map2_callback(self, msg):
        self.map2 = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        if not self.pose_sent:
            self.send_pose()
            

    def odom_callback(self,msg):
        self.robot_coords = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])
        





def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = NextBestViewNode()

    rclpy.spin(minimal_subscriber)

if __name__ == '__main__':
    main()