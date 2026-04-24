import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class Joint_Pub(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        #Frame Initial Pose
        self.intial_pos_x = 0.0
        self.intial_pos_y = 0.0
        self.intial_pos_z = 0.0
        self.intial_pos_yaw = 0.0
        self.intial_pos_pitch = 0.0
        self.intial_pos_roll = 0.0
        self.wr = 0.0
        self.wl = 0.0
        self.r = 0.05 # Wheel radius
        self.L = 0.19 # Distance between wheels
        self.prev_time = self.get_clock().now().nanoseconds/1e9

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10) 

        #Publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        #Define Transformations
        # Define the dynamic transform message
        self.base_footprint_joint = self.create_transform('odom', 
                                                'base_footprint', 
                                                0.0, 0.0, 0.05, 
                                                0.0, 0.0, 0.0)

        # Message objects
        self.ctrlJoints = JointState()
        self.odom = Odometry()

        #initialise Message to be published
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ["base_wheel_l", "base_wheel_r"]
        self.ctrlJoints.position = [0.0] * 2
        self.ctrlJoints.velocity = [0.0] * 2
        self.ctrlJoints.effort = [0.0] * 2

        #Create Transform Boradcasters
        self.tf_br_base_footprint = TransformBroadcaster(self)

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.get_logger().info("Joint_Pub Node Initialized!!!")


    #Timer Callback
    def timer_cb(self):
        if self.odom:
            current_time = self.get_clock().now().nanoseconds/1e9
            elapsed_time = current_time - self.prev_time

            # Transform the quaternion to euler angles from odom message
            q = [self.odom.pose.pose.orientation.w,
                 self.odom.pose.pose.orientation.x,
                 self.odom.pose.pose.orientation.y,
                 self.odom.pose.pose.orientation.z]
            
            roll, pitch, yaw = transforms3d.euler.quat2euler(q)  #input quat2euler(q=[w, x, y, z]) , output roll, pitch, yaw

            # Create the dynamic transform message
            self.base_footprint_joint = self.create_transform('odom', 
                                                    'base_footprint', 
                                                    self.odom.pose.pose.position.x, 
                                                    self.odom.pose.pose.position.y, 
                                                    self.odom.pose.pose.position.z, 
                                                    roll,
                                                    pitch,
                                                    yaw + self.odom.twist.twist.angular.z * elapsed_time)

            self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()

            wl, wr = self.update_wheel_velocities()

            self.wl += wl * elapsed_time
            self.wr += wr * elapsed_time

            self.ctrlJoints.position[0] = self.wl
            self.ctrlJoints.position[1] = self.wr

            self.tf_br_base_footprint.sendTransform(self.base_footprint_joint)

            self.publisher.publish(self.ctrlJoints)

            self.prev_time = current_time


    def create_transform(self, frame_id, child_frame, tx, ty, tz, roll, pitch, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        q = transforms3d.euler.euler2quat(roll, pitch, yaw)      #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z] 
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        return t
    
    def odom_cb(self, odom_msg):
        self.odom = odom_msg

    def update_wheel_velocities(self):
        if self.odom:
            v = self.odom.twist.twist.linear.x
            w = self.odom.twist.twist.angular.z

            wr = (v / self.r) + (self.L * w) / (2.0 * self.r)
            wl = (v / self.r) - (self.L * w) / (2.0 * self.r)

            return wl, wr
        
        else:
            return 0.0, 0.0


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