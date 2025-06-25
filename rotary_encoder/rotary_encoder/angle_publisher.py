#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
from std_msgs.msg import Float64
import atexit
import time

class angle_publisher(Node):

    def __init__(self, interface_, channel_, bitrate_, encoder_id, encoder_resolution_, reset_cmd_, request_cmd_, topic):
        super().__init__("angle_publisher_node")

        # initialize Bus object to communicate with the encoder using CAN protocol
        self.bus = can.interface.Bus(channel=channel_, interface=interface_, bitrate=bitrate_)

        # encoder_resolution, and CAN messages for resetting and requesting angle value
        self.encoder_resolution = encoder_resolution_
        self.reset_cmd = can.Message(arbitration_id=encoder_id, data = reset_cmd_, is_extended_id=False)
        self.request_cmd = can.Message(arbitration_id=encoder_id, data=request_cmd_, is_extended_id=False)

        # publisher object to publish angle readings as Float64 to given topic
        self.angle_value_publisher = self.create_publisher(Float64, topic, 10)
        # timer for publishing periodically
        # period = 0.1 second
        self.publishing_period = 0.1 
        self.timer = self.create_timer(self.publishing_period, self.publish_angle)

        self.get_logger().info("Angle Publisher Node initialized.")
        atexit.register(self.on_shutdown)

    def publish_angle(self):
        angle_in_degrees = Float64()

        # request and receive current angle reading
        self.bus.send(self.request_cmd)
        return_msg = self.bus.recv()
	
        # convert the angle reading from return_msg into an unsigned integer
        reading_int = int.from_bytes(return_msg.data[3:], byteorder='little')
	    # convert to angle value in degrees
        angle_in_degrees.data = (reading_int/self.encoder_resolution) * 360

        # publish angle value in degrees
        self.angle_value_publisher.publish(angle_in_degrees)

    def reset_angle(self):
        self.bus.send(self.reset_cmd)
        return_msg = self.bus.recv()
        print(return_msg)

        # wait for 1 second
        wait_time = 1 
        time.sleep(wait_time)

        # check if the angle value has been reset to 0
        self.bus.send(self.request_cmd)
        return_msg = self.bus.recv()
        print(return_msg)

        self.get_logger().info("Angle Value has been reset to zero.")

    def on_shutdown(self):
        # shut down the CAN bus on exiting the node
        self.bus.shutdown()


def main(args=None):
    rclpy.init(args = args)

    # parameters of the rotary encoder
    encoder_id = 1
    encoder_resolution_bits = 17
    encoder_resoltuion = (2**encoder_resolution_bits)-1
    angle_reset_cmd = [4,1,6,0]
    angle_request_cmd = [4,1,1,0]

    # CANable parameters
    firmware = 'slcan'
    port = '/dev/ttyACM0'
    bitrate = 500000

    # angle values will be published to following topic
    topic = "/MRI_robot/sensors/rotary_encoder1_angle"

    node1 = angle_publisher(firmware, port, bitrate, encoder_id, encoder_resoltuion, angle_reset_cmd, angle_request_cmd, topic)
    node1.reset_angle()
    try:
        rclpy.spin(node1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()