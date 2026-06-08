import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import numpy as np

class Point_generator(Node):

    def __init__(self):
        super().__init__('point_generator')

        # Parameters
        self.declare_parameter('figure', 'square')
        self.current_figure = self.get_parameter('figure').get_parameter_value().string_value
        
        # Initial Conditions
        self.figures = {
            'square': [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)],
            'pentagon': [(1.0, 0.0), (1.5, 1.0), (0.0, 2.0), (-1.5, 1.0), (-1.0, 0.0)],
            'star': [(-1.0, -2.0), (0.0, 2.0), (1.0, -2.0), (-2.0, 1.0), (2.0, 1.0)]
        }

        if self.current_figure not in self.figures:
            self.get_logger().warn(f"Figure '{self.current_figure}' not found. Using 'square'.")
            self.current_figure = 'square'

        self.send_coordinates = True
        self.i = 0

        # Subscribers
        self.next_point_sub = self.create_subscription(Bool, "next_point", self.next_point_cb, 10) 

        #Publisher
        self.set_point_pub = self.create_publisher(Point, 'set_point', 10)

        # Create Message objects
        self.next_point = Bool()
        self.set_point = Point()

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

    #Timer Callback
    def timer_cb(self):
        if self.send_coordinates:
            points = len(self.figures[self.current_figure])
                
            if self.i < points:
                self.set_point.x = self.figures[self.current_figure][self.i][0]
                self.set_point.y = self.figures[self.current_figure][self.i][1]
                self.set_point_pub.publish(self.set_point)
                print(f"Current goal: ({self.set_point.x}, {self.set_point.y})")
                self.send_coordinates = False
                self.i += 1
            else:
                print("All points sent. Stopping.")
                self.send_coordinates = False

    def next_point_cb(self, next_point_msg):
        self.send_coordinates = next_point_msg.data


def main(args=None):
    rclpy.init(args=args)

    node = Point_generator()

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