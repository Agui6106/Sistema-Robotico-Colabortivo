import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import transforms3d
import signal  
import sys 

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function

        # Declare parameters
        self.declare_parameter('kw', 1.0)
        self.declare_parameter('kv', 1.0)

        # Retrieve parameters
        self.kw = self.get_parameter('kw').value
        self.kv = self.get_parameter('kv').value

        # Initial Conditions
        #self.coordinates = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
        self.start = False
        self.xr = 0.0
        self.yr = 0.0
        self.theta_r = 0.0
        self.xg = 0.0
        self.yg = 0.0 
        self.i = 0
        self.state = 'stop'
        self.v_max = 0.4
        self.w_max = 1.0
        self.d_safety = 0.3 # Distance to keeep from the closest object [m].
        self.d_start_a = 0.50 # Distance to start avoiding the closest object [m].


        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10) 
        self.set_point_sub = self.create_subscription(Point, "set_point", self.set_point_cb, 10)
        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10) 
        self.leave_sub = self.create_subscription(Bool, "leave", self.leave_cb, 10) 

        #Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.next_point_pub = self.create_publisher(Bool, "next_point", 10) 
        self.hit_pub = self.create_publisher(Bool, "hit", 10)

        # Create Message objects
        self.cmd_vel = Twist()
        self.odom = Odometry()
        self.cmd_test = Twist()
        self.lidar = LaserScan() # Data from lidar will be stored here. 
        self.robot_vel = Twist() # Velocity command to be published will be stored here.
        self.leave = Bool() # Message to indicate whether the robot is leaving the wall will be stored here.
        self.leave.data = False # Initialize leave message to False

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)


    #Timer Callback
    def timer_cb(self):

        if self.odom:
            q = [self.odom.pose.pose.orientation.w,
                 self.odom.pose.pose.orientation.x,
                 self.odom.pose.pose.orientation.y,
                 self.odom.pose.pose.orientation.z]
            _, _, yaw = transforms3d.euler.quat2euler(q)  #input quat2euler(q=[w, x, y, z]) , output roll, pitch, yaw
            
            self.xr = self.odom.pose.pose.position.x
            self.yr = self.odom.pose.pose.position.y
            self.theta_r = yaw
        
            self.navigation() 
        else:
            print("Waiting for odometry data...")

    def odom_cb(self, odom_msg):
        self.odom = odom_msg
    
    def set_point_cb(self, set_point_msg):
        self.xg = set_point_msg.x
        self.yg = set_point_msg.y
        print(f"Received new set point: ({set_point_msg.x}, {set_point_msg.y})")
        self.start = True
        self.state = 'turn'

    def lidar_cb(self, lidar_msg): 
        ## This function receives the ROS LaserScan message 
        self.lidar =  lidar_msg  

    def leave_cb(self, leave_msg):
        self.leave = leave_msg

    def navigation(self):
        if self.start:

            ed, etheta = self.get_errors()
            print(f"ed = {ed}")
            print(f"etheta = {etheta}")

            if self.state == 'stop':
                print("Stop")
                self.cmd_vel.linear.x = 0.0 
                self.cmd_vel.angular.z = 0.0
                self.start = False
                self.i = 0
                self.next_point_pub.publish(Bool(data=True))  # Signal to Point_generator to send the next point

                self.cmd_vel_pub.publish(self.cmd_vel)

            elif self.state == 'turn':
                if (np.abs(etheta) >= 0.08):
                    print("Turning")
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = np.clip(self.kw * etheta, -self.w_max, self.w_max)
                else:
                    self.state = 'move'

                self.cmd_vel_pub.publish(self.cmd_vel)

            elif self.state == 'move':

                if (ed >= 0.025):
                    print("Moving")
                    self.cmd_vel.linear.x = min(self.kv * ed, self.v_max)
                    self.cmd_vel.angular.z = np.clip(self.kw * etheta, -self.w_max, self.w_max)
                    self.wall_detected(ed, etheta) # Call wall following behavior while moving towards the goal
                    
                else:
                    print(f"Goal reached: ({self.xg}, {self.yg})")
                    self.i += 1
                    self.state = 'stop'

                self.cmd_vel_pub.publish(self.cmd_vel)
                
            elif self.state == 'wall_following':
                print("Wall following")
                if self.leave.data == True:
                    self.state = 'turn'

                else:
                    #self.get_logger().info("Waiting for leave flag")
                    pass
                    
        else:
            print("Waiting for start signal...")

    def get_errors(self):
        ed = np.sqrt((self.xg - self.xr)**2 + (self.yg - self.yr)**2)

        etheta = np.arctan2(self.yg - self.yr, self.xg - self.xr) - self.theta_r
        # Normalize etheta to the range [-pi, pi]
        etheta = np.arctan2(np.sin(etheta), np.cos(etheta))

        return ed, etheta
    
    def get_closest_object(self):
        closest_range = min(self.lidar.ranges)
        closest_index = self.lidar.ranges.index(closest_range)
        theta_closest = self.lidar.angle_min + closest_index * self.lidar.angle_increment
        theta_closest = np.arctan2(np.sin(theta_closest), np.cos(theta_closest))

        return closest_range, theta_closest
    
    def wall_detected(self, ed, etheta):
        if self.lidar.ranges:
            closest_range, theta_closest = self.get_closest_object()
            print(f"Closest range: {closest_range}")
            print(f"Closest angle: {theta_closest}")

            if np.isinf(closest_range):
                print("There are no objects nearby.")
                self.cmd_vel.linear.x = min(self.kv * ed, self.v_max)
                self.cmd_vel.angular.z = np.clip(self.kw * etheta, -self.w_max, self.w_max)
                
            else:

                if (theta_closest > np.pi/2 or theta_closest < -np.pi/2) or closest_range > self.d_start_a:
                    print("Object is behind the robot or far away. Ignoring.")
                    self.cmd_vel.linear.x = min(self.kv * ed, self.v_max)
                    self.cmd_vel.angular.z = np.clip(self.kw * etheta, -self.w_max, self.w_max)
                else:
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                    self.state = 'wall_following'
                    self.leave.data = False
                    self.hit_pub.publish(Bool(data=True))
                    print("Object detected in front. Starting wall following behavior.")

    def shutdown_function(self, signum, frame): 
        # Handle shutdown gracefully 
        # This function will be called when Ctrl+C is pressed 
        # It will stop the robot and shutdown the node 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.cmd_vel_pub.publish(stop_twist) # publish it to stop the robot before shutting down 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program 


def main(args=None):
    rclpy.init(args=args)

    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()