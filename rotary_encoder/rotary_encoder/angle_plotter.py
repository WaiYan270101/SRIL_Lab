#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class angle_plotter(Node):

    def __init__(self, topic):
        super().__init__("angle_plotter_node")

        # subscriber object to get angle data from the give topic and call plotting function
        self.angle_data_subscriber = self.create_subscription(Float64, topic, self.plot_angle, 10)
        self.get_logger().info("Angle Plotter Node initialized.")

    def plot_angle(self, msg:Float64):
        # print("Angle in Degrees = ", msg.data)
        # self.get_logger().info(str(msg.data))
        self.get_logger().info(f'Angle in Degrees : "{msg.data}"')
        

def main(args=None):
    rclpy.init(args=args)

    node1 = angle_plotter("/MRI_robot/sensors/rotary_encoder1_angle")
    rclpy.spin(node1)

    rclpy.shutdown()

if __name__ == '__main__':
    main()