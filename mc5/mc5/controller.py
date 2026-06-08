import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import transforms3d


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

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

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10) 
        self.set_point_sub = self.create_subscription(Point, "set_point", self.set_point_cb, 10)

        #Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.next_point_pub = self.create_publisher(Bool, "next_point", 10) 

        # Create Message objects
        self.cmd_vel = Twist()
        self.odom = Odometry()
        self.cmd_test = Twist()

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

    def navigation(self):
        if self.start:

            ed, etheta = self.get_errors()

            if self.state == 'stop':
                print("Stop")
                self.cmd_vel.linear.x = 0.0 
                self.cmd_vel.angular.z = 0.0
                self.start = False
                self.i = 0
                self.next_point_pub.publish(Bool(data=True))  # Signal to Point_generator to send the next point

            elif self.state == 'turn':
                if (np.abs(etheta) >= 0.08):
                    print("Turning")
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = self.kw * etheta
                else:
                    self.state = 'move'

            elif self.state == 'move':
                if (ed >= 0.025):
                    print("Moving")
                    self.cmd_vel.linear.x = self.kv * ed
                    self.cmd_vel.angular.z = self.kw * etheta
                else:
                    print(f"Goal reached: ({self.xg}, {self.yg})")
                    self.i += 1
                    self.state = 'stop'
            
            self.cmd_vel_pub.publish(self.cmd_vel)

        else:
            print("Waiting for start signal...")

    def get_errors(self):
        ed = np.sqrt((self.xg - self.xr)**2 + (self.yg - self.yr)**2)

        etheta = np.arctan2(self.yg - self.yr, self.xg - self.xr) - self.theta_r
        # Normalize etheta to the range [-pi, pi]
        etheta = np.arctan2(np.sin(etheta), np.cos(etheta))

        return ed, etheta


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