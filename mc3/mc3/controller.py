import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import transforms3d
import numpy as np

class Joint_Pub(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        # Draw a square
        self.coordinates = [[1.0, 0.0],
                            [1.0, 1.0],
                            [0.0, 1.0],
                            [0.0, 0.0]]

        # Flag for state machine
        self.start = False
        self.state = "Stop"
        # Iterator for target points
        self.i = 0

        # Controller Gains
        self.kv = 1.0
        self.kw = 1.0

        # Initial conditions
        self.xr = 0.0
        self.yr = 0.0
        self.theta_r = 0.0
        # Goal position
        self.xg = 0.0
        self.yg = 0.0

        # Subscribers
        self.create_subscription(Odometry, "odom", self.odom_cb, 10) 
        self.create_subscription(Point, "set_point", self.set_point_cb, 10)

        #Publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_finish_pt = self.create_publisher(Bool, 'finish', 10)

        # Create objects
        self.cmd_vel = Twist()  # Topico de velocidad
        self.odom = Odometry()  # Topico de odometria
        self.point = Point()    # Topico de punto objetivo

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.get_logger().info("Controller Node Initialized!!!")
        
    # Suscriber Callback
    def odom_cb(self, odom_msg):
        self.odom = odom_msg

    def set_point_cb(self, point_msg):
        '''self.xg = point_msg.data.x
        self.yg = point_msg.data.y'''
        self.point = point_msg
        self.start = True
        self.get_logger().info("New point received: ({:.2f}, {:.2f})".format(point_msg.x, point_msg.y))

    #Timer Callback
    def timer_cb(self):
        if self.odom:
            # Get the angles from quaternion
            q = [self.odom.pose.pose.orientation.w,
                 self.odom.pose.pose.orientation.x,
                 self.odom.pose.pose.orientation.y,
                 self.odom.pose.pose.orientation.z]
            
            _, _, yaw = transforms3d.euler.quat2euler(q)

            # Updtate the positions
            self.xr = self.odom.pose.pose.position.x
            self.yr = self.odom.pose.pose.position.y
            self.theta_r = yaw
            
            self.navigation()

    # Function to calculate error between current position and target position
    def get_error(self):
        '''
        Get the distance and angle error between the current position and the target position.
        '''
        # Calculate the distance and angle error
        ed = np.sqrt((self.xg - self.xr)**2 + (self.yg - self.yr)**2)

        # Calculate the angle to the goal and limit it to the range [-pi, pi]
        etheta = np.arctan2(self.yg - self.yr, self.xg - self.xr) - self.theta_r
        etheta = np.arctan2(np.sin(etheta), np.cos(etheta))

        return ed, etheta
    
    def navigation(self):
        '''
        Calculate the linear and angular velocity commands based on the distance and angle error.
        '''
        if self.start:
            points = len(self.coordinates)  # Get number of points

            # State machine to navigate through the points
            if self.i < points:
                self.xg, self.yg = self.coordinates[self.i] # Get current target point
            else:
                self.state = "stop"

            ed, etheta = self.get_error()   # Get errors

            # Final state: Robot stopped
            if self.state == "stop":
                self.get_logger().info("State: STOP")
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.start = False
                self.i = 0
                self.pub_finish_pt.publish(Bool(data=True))  # Publish finish signal
            
            # State: Robot turning towards the target point
            elif self.state == "turn":
                if (abs(etheta) >= 0.8):
                    self.get_logger().info("State: TURN")
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = self.kw * etheta
                else:
                    self.state = "move"

            # State: Robot moving towards the target point
            elif self.state == "move":
                if ed >= 0.05: # Adjust this tolerance if necessary
                    self.get_logger().info("State: MOVE")
                    self.cmd_vel.linear.x = self.kv * ed
                    self.cmd_vel.angular.z = self.kw * etheta
                else:
                    self.get_logger().info(f'point reached: {self.i +1}: ({self.xg}, {self.yg})')
                    self.i += 1     # Move to the next point
                    self.state = "turn"

            # Publish the velocity commands
            self.publisher.publish(self.cmd_vel)
        
        else:
            self.get_logger().warn("Waiting for start command...")

def main(args=None):
    rclpy.init(args=args)

    node = Joint_Pub()

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