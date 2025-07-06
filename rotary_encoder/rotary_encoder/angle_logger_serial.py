#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import pandas
import atexit

# node to subscribe to given topics and log data to a given file
class SubAndLog(Node):

    def __init__(self, topic_1_, topic_2_, filepath_):
        super().__init__("SubAndLog_node")

        # subscriber objects to get angle data from the given topics and update given file
        self.angle_data_subscriber1 = self.create_subscription(Float64, topic_1_, self.update_data1, 10)
        self.up_to_date_angle1 = 0.0
        self.angle_data_subscriber2 = self.create_subscription(Float64, topic_2_, self.update_data2, 10)
        self.up_to_date_angle2 = 0.0

        # dictionary to store time series data
        self.angle_data = {'Time (ms)':[], 'Angle1 (radians)':[], 'Angle2 (radians)':[]}
        # file path to save data
        self.filepath = filepath_

        self.timer = self.create_timer(0.1, self.save_file)
        atexit.register(self.save_file)

        self.get_logger().info("Angle Logger Node initialized.")
        self.start_time = time.time()

    def update_data1(self, msg:Float64):
        self.up_to_date_angle1 = msg.data
        self.angle_data['Time (ms)'].append((time.time()-self.start_time)*1000)
        self.angle_data['Angle1 (radians)'].append(self.up_to_date_angle1)
        self.angle_data['Angle2 (radians)'].append(self.up_to_date_angle2)

        # print(self.angle_data)
        # self.get_logger().info(f'Angle in Radians : "{msg.data}", Time : "{time.time()-self.start_time}"')
        
    def update_data2(self, msg:Float64):
        self.up_to_date_angle2 = msg.data
        self.angle_data['Time (ms)'].append((time.time()-self.start_time)*1000)
        self.angle_data['Angle1 (radians)'].append(self.up_to_date_angle1)
        self.angle_data['Angle2 (radians)'].append(self.up_to_date_angle2)

    def save_file(self):
        df = pandas.DataFrame(self.angle_data)
        df.to_csv(self.filepath, index=False)

def main(args=None):
    rclpy.init(args=args)

    motor_anlge_topic = "/MRI_robot/motor_angle"
    encoder_angle_topic = "/MRI_robot/encoder_angle"
    filepath = "~/angle_logger_serial_test1.csv"

    node1 = SubAndLog(motor_anlge_topic, encoder_angle_topic, filepath)
    rclpy.spin(node1)

    rclpy.shutdown()

if __name__ == '__main__':
    main()