import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import transforms3d

class Localisation(Node):
    def __init__(self):
        super().__init__('localisation')

        # Declare parameters
        self.declare_parameter('r', 0.05)
        self.declare_parameter('L', 0.19)

        # Retrieve parameters
        self.r = self.get_parameter('r').value
        self.L = self.get_parameter('L').value

        self.namespace = self.get_namespace().rstrip('/')

        # Subscribers
        self.wr_sub = self.create_subscription(Float32, "wr", self.wr_cb, 10) 
        self.wl_sub = self.create_subscription(Float32, "wl", self.wl_cb, 10) 

        # Publishers
        self.pose_sim_pub = self.create_publisher(Odometry, 'odom', 10)

        # Message objects
        self.wr = Float32()
        self.wl = Float32()

        # Initial Conditions
        self.v = 0.0
        self.w = 0.0
        self.prev_time = self.get_clock().now().nanoseconds/1e9
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.get_logger().info("Node initialized!!!") 


    #Timer Callback
    def timer_cb(self):
        if self.wr and self.wl:
            self.estimate_velocities()
            self.update_pose()
            self.create_odom_msg()

            print(f"v ={self.v}")
            print(f"w ={self.w}")
            print(f"x ={self.x}")
            print(f"y ={self.y}")
            print(f"theta ={self.theta}")

    def wr_cb(self, wr_msg):
        self.wr = wr_msg

    def wl_cb(self, wl_msg):
        self.wl = wl_msg

    def estimate_velocities(self):
        #self.v = self.r * (self.wr.data + self.wl.data) / 2.0
        #self.w = self.r * (self.wr.data - self.wl.data) / self.L

        mat_a = np.array([[self.r/2, self.r/2], [self.r/self.L, -self.r/self.L]])
        mat_b = np.array([[self.wr.data], [self.wl.data]])

        result = mat_a @ mat_b
        self.v = result[0, 0]
        self.w = result[1, 0]

    def update_pose(self):
        # Update the robot's pose based on the current velocities and time step
        current_time = self.get_clock().now().nanoseconds/1e9
        dt = current_time - self.prev_time
        self.theta += self.w * dt
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt

        self.prev_time = self.get_clock().now().nanoseconds/1e9

    def create_odom_msg(self):
        # Create a new Odometry message 
        odom_msg = Odometry() 
        # Fill the message with the robot's pose 
        odom_msg.header.stamp = self.get_clock().now().to_msg() # Get the current time 
        odom_msg.header.frame_id = 'odom' # Set the frame id 
        odom_msg.child_frame_id = f'{self.namespace}/base_footprint'
        odom_msg.pose.pose.position.x = self.x # x position [m] 
        odom_msg.pose.pose.position.y = self.y # y position [m] 
        odom_msg.pose.pose.position.z = 0.0 # z position [m] 
        
        # Set the orientation using quaternion
        # Convert the yaw angle to a quaternion 
        quat = transforms3d.euler.euler2quat(0, 0, self.theta) #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z]
        odom_msg.pose.pose.orientation.w = quat[0] 
        odom_msg.pose.pose.orientation.x = quat[1] 
        odom_msg.pose.pose.orientation.y = quat[2] 
        odom_msg.pose.pose.orientation.z = quat[3]

        # Fill the message with the robot's velocities
        odom_msg.twist.twist.linear.x = self.v # Linear velocity in x [m/s] 
        odom_msg.twist.twist.linear.y = 0.0 # Linear velocity in y [m/s] 
        odom_msg.twist.twist.linear.z = 0.0 # Linear velocity in z [m/s] 
        odom_msg.twist.twist.angular.x = 0.0 # Angular velocity in x [rad/s]    
        odom_msg.twist.twist.angular.y = 0.0 # Angular velocity in y [rad/s] 
        odom_msg.twist.twist.angular.z = self.w # Angular velocity in z [rad/s]

        # Publish the message
        self.pose_sim_pub.publish(odom_msg)

    
def main(args=None):
    rclpy.init(args=args)

    node = Localisation()

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