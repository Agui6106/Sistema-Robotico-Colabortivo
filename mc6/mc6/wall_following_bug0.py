import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import transforms3d
import numpy as np
import signal  
import sys 

class LaserScanSub(Node): 
    def __init__(self): 
        super().__init__('laser_scan_subscriber') 

        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10) 
        self.hit_sub = self.create_subscription(Bool, "hit", self.hit_cb, 10) 
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10) 
        self.set_point_sub = self.create_subscription(Point, "set_point", self.set_point_cb, 10) 

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.leave_pub = self.create_publisher(Bool, 'leave', 10)


        # Message objects
        self.lidar = LaserScan() # Data from lidar will be stored here. 
        self.robot_vel = Twist() # Velocity command to be published will be stored here.
        self.hit = Bool() # Flag to store time when we detect a wall 
        self.hit.data = False
        self.odom = Odometry() 
        self.leave = Bool() # Flag to indicate when to leave the wall following behavior
        self.leave.data = True

        self.d_start_a = 0.60 # Distance to start avoiding the closest object [m].
        self.d_wall = 0.3 # Desired distance to the wall [m].
        self.kv = 0.5 # Proportional gain for linear velocity.

        self.vel_v = 0.2 # Linear velocity when following the wall.

        self.kw = 0.8 # Proportional gain for angular velocity when avoiding the closest object.
        self.kw_2 = 1.0 # Proportional gain for angular velocity when following the wall.

        self.xg = 0.0 # Goal x position
        self.yg = 0.0 # Goal y position
        self.xr = 0.0 # Robot's x position
        self.yr = 0.0 # Robot's y position
        self.theta_r = 0.0 # Robot's orientation
        self.d_hit = 0.0 # Distance to the wall when we first detect it
        self.d_leave = 0.0 # Distance to the wall when we decide to leave the wall following behavior

        timer_period = 0.05

        # Lidar regions
        self.back_left = 0.0
        self.left = 0.0
        self.front_left = 0.0
        self.front = 0.0
        self.front_right = 0.0
        self.right = 0.0
        self.back_right = 0.0

        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 

    def timer_callback(self): 
        if self.lidar.ranges:

            q = [self.odom.pose.pose.orientation.w,
                self.odom.pose.pose.orientation.x,
                self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z]
            _, _, yaw = transforms3d.euler.quat2euler(q)  #input quat2euler(q=[w, x, y, z]) , output roll, pitch, yaw
            
            self.xr = self.odom.pose.pose.position.x
            self.yr = self.odom.pose.pose.position.y
            self.theta_r = yaw

            v, w, theta_avoid = self.wall_following()

            theta_gtg = np.arctan2(self.yg - self.yr, self.xg - self.xr) - self.theta_r
            self.d_leave = np.sqrt((self.xr - self.xg)**2 + (self.yr - self.yg)**2)

            if self.hit.data:
                self.hit.data = False
                self.d_hit = np.sqrt((self.xr - self.xg)**2 + (self.yr - self.yg)**2)

            print(f"Distance to goal: {self.d_leave}, Angle to goal: {theta_gtg}")
            print(f"Distance to wall when hit: {self.d_hit}")

            if self.d_leave < abs(self.d_hit - 0.15) and abs(theta_avoid - theta_gtg) < np.pi / 2:
                print("Leaving wall following behavior. Approaching goal.")
                self.leave.data = True
                self.leave_pub.publish(self.leave)

            if not self.leave.data:
                self.robot_vel.linear.x = v
                self.robot_vel.angular.z = w
                self.cmd_vel_pub.publish(self.robot_vel)
 

    def lidar_cb(self, lidar_msg): 
        ## This function receives the ROS LaserScan message 
        self.lidar =  lidar_msg  

    def hit_cb(self, hit_msg): 
        ## This function receives the ROS Bool message indicating whether we have detected a wall or not
        self.hit =  hit_msg  
        self.leave.data = False # Reset leave flag when we hit a wall again

    def odom_cb(self, odom_msg):
        ## This function receives the ROS Odometry message 
        self.odom =  odom_msg

    def set_point_cb(self, set_point_msg):
        self.xg = set_point_msg.x
        self.yg = set_point_msg.y
        print(f"Received new set point: ({set_point_msg.x}, {set_point_msg.y})")    

    def get_index(self, angle):
        index = int(angle / self.lidar.angle_increment)
        return index

    def get_closest_object(self):
        closest_range = min(self.lidar.ranges)
        closes_index = self.lidar.ranges.index(closest_range)
        theta_closest = self.lidar.angle_min + closes_index * self.lidar.angle_increment
        theta_closest = np.arctan2(np.sin(theta_closest), np.cos(theta_closest))

        return closest_range, theta_closest
    
    def shutdown_function(self, signum, frame): 
        # Handle shutdown gracefully 
        # This function will be called when Ctrl+C is pressed 
        # It will stop the robot and shutdown the node 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.cmd_vel_pub.publish(stop_twist) # publish it to stop the robot before shutting down 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program 

    def wall_following(self):

        # BACK LEFT: 90 to 126 degrees
        self.back_left = min(self.lidar.ranges[self.get_index(np.pi / 2):self.get_index(7 * np.pi / 10)])
        # LEFT: 54 to 90 degrees
        self.left = min(self.lidar.ranges[self.get_index(3 * np.pi / 10):self.get_index(np.pi / 2)])
        # FRONT LEFT: 18 to 54 degrees
        self.front_left = min(self.lidar.ranges[self.get_index(np.pi / 10):self.get_index(3 * np.pi / 10)])
        # FRONT: -18 to 18 degrees (wrap around)
        front_a = self.lidar.ranges[self.get_index(-np.pi / 10):] + self.lidar.ranges[:self.get_index(np.pi / 10)]
        self.front = min(front_a)
        # FRONT RIGHT: -54 to -18 degrees
        self.front_right = min(self.lidar.ranges[self.get_index(-3 * np.pi / 10):self.get_index(-np.pi / 10)])
        # RIGHT: -90 to -54 degrees
        self.right = min(self.lidar.ranges[self.get_index(-np.pi / 2):self.get_index(-3 * np.pi / 10)])
        # BACK RIGHT: -126 to -90 degrees
        self.back_right = min(self.lidar.ranges[self.get_index(-7 * np.pi / 10):self.get_index(-np.pi / 2)])
        

        closest_range, theta_closest = self.get_closest_object()

        theta_avoid = theta_closest - np.pi
        theta_avoid = np.arctan2(np.sin(theta_avoid), np.cos(theta_avoid))

        print(f"Closest range: {closest_range}")

        if np.isinf(closest_range):
            print("There are no objects nearby.")
            v = self.vel_v
            w = 0.0
            
        else:
            if theta_closest > 7* np.pi/10 or theta_closest < -7 * np.pi/10:
                print("Object is behind the robot. Ignoring.")
                v = self.vel_v
                w = 0.0 
            else:
                if closest_range < self.d_start_a:
                    if self.left < self.right and self.left < 1.5 * self.d_wall:
                        print("Object is on the left. Avoiding by turning right.")

                        # INNER CORNER CASE
                        if self.front < 2 * self.d_wall:
                            if self.front < self.d_wall:
                                v = 0.0
                                w = -0.5

                            else:
                                theta_avoid = theta_closest - np.pi
                                theta_avoid = np.arctan2(np.sin(theta_avoid), np.cos(theta_avoid))

                                theta_fwc = theta_avoid + np.pi / 2.0
                                theta_fwc = np.arctan2(np.sin(theta_fwc), np.cos(theta_fwc))

                                print(f'Left distance: {self.left}')
                                ed_ccw = self.left - self.d_wall

                                v = self.vel_v * 0.5 # Slow down when avoiding
                                w = self.kw * theta_fwc + self.kw_2 * ed_ccw
                                #w = self.kw * theta_fwc

                        else:
                            theta_avoid = theta_closest - np.pi
                            theta_avoid = np.arctan2(np.sin(theta_avoid), np.cos(theta_avoid))

                            theta_fwc = theta_avoid + np.pi / 2.0
                            theta_fwc = np.arctan2(np.sin(theta_fwc), np.cos(theta_fwc))

                            print(f'Left distance: {self.left}')
                            ed_ccw = self.left - self.d_wall

                            v = self.vel_v # Slow down when avoiding
                            w = self.kw * theta_fwc + self.kw_2 * ed_ccw
                            #w = self.kw * theta_fwc


                    elif self.right < self.left and self.right < 1.5 * self.d_wall:
                        print("Object is on the right. Avoiding by turning left.")

                        # INNER CORNER CASE
                        if self.front < 2 * self.d_wall:
                            if self.front < self.d_wall:
                                v = 0.0
                                w = 0.5

                            else:
                                theta_avoid = theta_closest - np.pi
                                theta_avoid = np.arctan2(np.sin(theta_avoid), np.cos(theta_avoid))

                                theta_fwc = theta_avoid - np.pi / 2.0
                                theta_fwc = np.arctan2(np.sin(theta_fwc), np.cos(theta_fwc))

                                print(f'Right distance: {self.right}')
                                ed_cw = self.d_wall - self.right

                                v = self.vel_v * 0.5 # Slow down when avoiding
                                w = self.kw * theta_fwc + self.kw_2 * ed_cw
                                #w = self.kw * theta_fwc
                        else:
                            theta_avoid = theta_closest - np.pi
                            theta_avoid = np.arctan2(np.sin(theta_avoid), np.cos(theta_avoid))

                            theta_fwc = theta_avoid - np.pi / 2.0
                            theta_fwc = np.arctan2(np.sin(theta_fwc), np.cos(theta_fwc))

                            print(f'Right distance: {self.right}')
                            ed_cw = self.d_wall - self.right

                            v = self.vel_v # Slow down when avoiding
                            w = self.kw * theta_fwc + self.kw_2 * ed_cw
                            #w = self.kw * theta_fwc

                    else:

                        if self.back_left < 2 * self.d_wall:
                            print("OUTTER CORNER LEFT")
                            v = 0.3
                            w = 1.5

                        elif self.back_right < 2 * self.d_wall:
                            print("OUTTER CORNER RIGHT")
                            v = 0.3
                            w = -1.5

                        else:
                            print("Object is in front.")
                            v = 0.0
                            w = 1.0
                    
                else:
                    print("Object is in front but far. Approaching.")
                    v = self.vel_v
                    w = 0.0

        return v, w, theta_avoid
        

def main(args=None): 
    rclpy.init(args=args) 
    m_p=LaserScanSub() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 

    main() 