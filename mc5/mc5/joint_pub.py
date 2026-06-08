import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class Joint_Publisher(Node):

    def __init__(self):
        super().__init__('joint_publisher')

        self.namespace = self.get_namespace().rstrip('/')

        # Declare the parameter with a default value
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_z', 0.0)
        self.declare_parameter('init_pose_yaw', 0.0)
        self.declare_parameter('init_pose_pitch', 0.0)
        self.declare_parameter('init_pose_roll', 0.0)
        self.declare_parameter('r', 0.05)
        self.declare_parameter('L', 0.19)
        self.declare_parameter('odom_frame', 'odom')

        # Retrieve the parameter values
        self.initial_pos_x = self.get_parameter('init_pose_x').value
        self.initial_pos_y = self.get_parameter('init_pose_y').value
        self.initial_pos_z = self.get_parameter('init_pose_z').value
        self.initial_pos_yaw = self.get_parameter('init_pose_yaw').value
        self.initial_pos_pitch = self.get_parameter('init_pose_pitch').value
        self.initial_pos_roll = self.get_parameter('init_pose_roll').value
        self.r = self.get_parameter('r').value
        self.L = self.get_parameter('L').value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value.strip('/')

        self.wr = 0.0
        self.wl = 0.0

        self.prev_time = self.get_clock().now().nanoseconds/1e9

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_cb, 10) 

        #Publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        #Define Transformations
        # Define the dynamic transform message
        self.base_footprint_joint = TransformStamped()
        self.base_footprint_joint.header.frame_id = self.odom_frame
        self.base_footprint_joint.child_frame_id = f'{self.namespace}/base_footprint'
        self.update_transform(self.base_footprint_joint,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0)  # Initial transform with zero translation and rotation

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


    #Timer Callback
    def timer_cb(self):
        if self.odom:
            current_time = self.get_clock().now().nanoseconds/1e9
            elapsed_time = current_time - self.prev_time

            q = [self.odom.pose.pose.orientation.w,
                 self.odom.pose.pose.orientation.x,
                 self.odom.pose.pose.orientation.y,
                 self.odom.pose.pose.orientation.z]
            
            roll, pitch, yaw = transforms3d.euler.quat2euler(q)  #input quat2euler(q=[w, x, y, z]) , output roll, pitch, yaw

            # Create the dynamic transform message
            self.update_transform(self.base_footprint_joint,
                                self.odom.pose.pose.position.x, 
                                self.odom.pose.pose.position.y, 
                                self.odom.pose.pose.position.z, 
                                roll,
                                pitch,
                                yaw)

            self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()

            wl, wr = self.update_wheel_velocities()

            self.wl += wl * elapsed_time
            self.wr += wr * elapsed_time

            self.ctrlJoints.position[0] = self.wl
            self.ctrlJoints.position[1] = self.wr

            self.tf_br_base_footprint.sendTransform(self.base_footprint_joint)

            self.publisher.publish(self.ctrlJoints)

            self.prev_time = current_time

    def update_transform(self, tf, tx, ty, tz, roll, pitch, yaw):
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.transform.translation.x = self.initial_pos_x + tx
        tf.transform.translation.y = self.initial_pos_y + ty
        tf.transform.translation.z = self.initial_pos_z + tz
        q = transforms3d.euler.euler2quat(self.initial_pos_roll + roll, 
                                          self.initial_pos_pitch + pitch, 
                                          self.initial_pos_yaw + yaw)      #input euler2quat(roll, pitch, yaw) , output q=[w, x, y, z] 
        tf.transform.rotation.x = q[1]
        tf.transform.rotation.y = q[2]
        tf.transform.rotation.z = q[3]
        tf.transform.rotation.w = q[0]

    
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

    node = Joint_Publisher()

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