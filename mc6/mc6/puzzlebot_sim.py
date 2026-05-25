import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
import numpy as np
import transforms3d

class Puzzlebot_sim(Node):

    def __init__(self):
        super().__init__('puzzlebot_sim')

        # Declare parameters
        self.declare_parameter('r', 0.05)
        self.declare_parameter('L', 0.19)

        # Retrieve parameters
        self.r = self.get_parameter('r').value
        self.L = self.get_parameter('L').value

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10) 

        # Publishers
        self.pose_sim_pub = self.create_publisher(PoseStamped, 'pose_sim', 10)
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)

        # Message objects
        self.cmd_vel = Twist()
        self.wr = Float32()
        self.wl = Float32()
        self.pose_sim = PoseStamped()

        # Initial Conditions
        self.prev_time = self.get_clock().now().nanoseconds/1e9
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.get_logger().info("Node initialized!!!") 


    #Timer Callback
    def timer_cb(self):
        if self.cmd_vel:
            self.update_pose()

            print(f"v ={self.cmd_vel.linear.x}")
            print(f"w ={self.cmd_vel.angular.z}")
            print(f"x ={self.x}")
            print(f"y ={self.y}")
            print(f"theta ={self.theta}")
            print(f"wl ={self.wl.data}")
            print(f"wr ={self.wr.data}")
        
        else:
            print("No cmd_vel received yet.")
        
        self.publish_msgs()


    def cmd_vel_cb(self, cmd_msg):
        ## This function receives the ROS Twist message 
        self.cmd_vel =  cmd_msg 

    def update_pose(self):
        # Update the robot's pose based on the current velocities and time step
        current_time = self.get_clock().now().nanoseconds/1e9
        dt = current_time - self.prev_time
        self.theta += self.cmd_vel.angular.z * dt
        self.x += self.cmd_vel.linear.x * np.cos(self.theta) * dt
        self.y += self.cmd_vel.linear.x * np.sin(self.theta) * dt

        self.prev_time = self.get_clock().now().nanoseconds/1e9
        
    def update_wheel_velocities(self):
        if self.cmd_vel:
            v = self.cmd_vel.linear.x
            w = self.cmd_vel.angular.z

            wr = (v / self.r) + (self.L * w) / (2.0 * self.r)
            wl = (v / self.r) - (self.L * w) / (2.0 * self.r)

            return wl, wr
        
        else:
            return 0.0, 0.0
    
    def publish_msgs(self):
        self.pose_sim.header.stamp = self.get_clock().now().to_msg()
        self.pose_sim.header.frame_id = "odom"

        self.pose_sim.pose.position.x = self.x
        self.pose_sim.pose.position.y = self.y
        
        q = transforms3d.euler.euler2quat(0, 0, self.theta) #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z] 
        self.pose_sim.pose.orientation.x = q[1]
        self.pose_sim.pose.orientation.y = q[2]
        self.pose_sim.pose.orientation.z = q[3]
        self.pose_sim.pose.orientation.w = q[0]

        wl, wr = self.update_wheel_velocities()

        self.wr.data = wr
        self.wl.data = wl

        self.pose_sim_pub.publish(self.pose_sim)
        self.wr_pub.publish(self.wr)
        self.wl_pub.publish(self.wl)

    
def main(args=None):
    rclpy.init(args=args)

    node = Puzzlebot_sim()

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