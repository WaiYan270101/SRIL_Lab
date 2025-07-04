#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float64
import atexit
import time
import math

class angle_publisher_serial(Node):

    def __init__(self, port_, baudrate_, motor_angle_topic_, encoder_angle_topic_, publishing_period_):
        super().__init__("angle_publisher_serial_node")

        # initialize serial object to communicate with STM32
        self.ser = serial.Serial(port=port_, baudrate=baudrate_, timeout=1)

        # publisher object to publish motor angle values as Float64 to given topic
        self.motor_angle_publisher = self.create_publisher(Float64, motor_angle_topic_, 10)
        # publisher object to publish actuator angle values as Float64 to given topic
        self.actuator_angle_publisher = self.create_publisher(Float64, encoder_angle_topic_, 10)
        # timer for publishing periodically
        self.publishing_period = publishing_period_
        self.timer = self.create_timer(self.publishing_period, self.publish_angles)

        # flag to show if the serial data is the first received one
        self.first_data = True

        self.get_logger().info("Angle Publisher Serial Node initialized.")
        atexit.register(self.on_shutdown)

    def publish_angles(self):
        angle_in_degrees = Float64()

        # read current angle value
        return_msg = self.ser.readline()

        # process return_msg if it is not the first one, because the first one could be out of format

        if not self.first_data:
            # extract time_stamp, tag and angle value from return_msg
            # tag shows whether the angle value is that of the motor or actuator
            index = return_msg.find(b' ')
            time_stamp = return_msg[:index]
            time_removed_msg = return_msg[index+4:]
            index = time_removed_msg.find(b' ')
            tag = time_removed_msg[:index]
            value_part = time_removed_msg[index+1:]
            index = value_part.find(b' ')
            angle_reading = value_part[:index]
	
            # convert the angle reading from return_msg into a float
            reading_float = float(angle_reading.decode(encoding="ascii"))
	        # convert to angle value in degrees
            # angle_in_degrees.data = (reading_float/math.pi) * 180
            angle_in_degrees.data = reading_float

            # publish angle value to corresponding topic
            if(tag == b'MO'):
                self.motor_angle_publisher.publish(angle_in_degrees)
            elif(tag == b'EN'):
                self.actuator_angle_publisher.publish(angle_in_degrees)
        else:
            self.first_data = False

    def on_shutdown(self):
        # shut down the serial object on exiting the node
        self.ser.close()


def main(args=None):
    rclpy.init(args = args)

    # Serial communication parameters
    port = '/dev/ttyACM0'
    baudrate = 115200

    # angle values will be published to following topics
    motor_angle_topic = "/MRI_robot/motor_angle"
    encoder_angle_topic = "/MRI_robot/encoder_angle"

    # rate of publishing
    publishing_period = 0.001

    node1 = angle_publisher_serial(port, baudrate, motor_angle_topic, encoder_angle_topic, publishing_period)

    try:
        rclpy.spin(node1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()