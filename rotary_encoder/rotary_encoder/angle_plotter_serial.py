#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# node to subscribe to given topics and plot the data in real time
class SubAndPlot(Node):

    def __init__(self, topic_1_, topic_2_, time_window_, data_update_period_):
        super().__init__("SubAndPlot_node")

        # subscriber objects to get angle data from the given topics and update data 
        self.angle_data_subscriber1 = self.create_subscription(Float64, topic_1_, self.update_data1, 10)
        self.angle_data_subscriber2 = self.create_subscription(Float64, topic_2_, self.update_data2, 10)

        # dictionary to store time series data
        self.angle_data = {'Time1 (s)':[], 'Angle1 (radians)':[], 'Time2 (s)':[], 'Angle2 (radians)':[]}

        # lock used for multi-threading
        self.data_lock = threading.Lock()

        # currently not used
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')

        # length of the displayed time interval, and number of data samples within the interval
        self.time_window = time_window_
        self.data_update_period = data_update_period_
        self.number_of_samples_in_window = int(self.time_window/self.data_update_period)
        
        self.get_logger().info("Angle Plotter Serial Node initialized.")
        self.start_time = time.time()

    def update_data1(self, msg:Float64):
        with self.data_lock:
            self.angle_data['Time1 (s)'].append(time.time()-self.start_time)
            self.angle_data['Angle1 (radians)'].append(msg.data)
        # print(f'message : "{msg}')
        # self.get_logger().info(f'Angle in Radians : "{msg.data}", Time : "{time.time()-self.start_time}"')
        
    def update_data2(self, msg:Float64):
        with self.data_lock:
            self.angle_data['Time2 (s)'].append(time.time()-self.start_time)
            self.angle_data['Angle2 (radians)'].append(msg.data)

    # currently not used
    def animate_plot(self, frame):
        with self.data_lock:
            self.line.set_data(self.angle_data['Time1 (s)'], self.angle_data['Angle1 (radians)'])
        
        self.ax.relim()
        self.ax.autoscale()
        return self.line,

    def animate_plot2(self, frame):
        with self.data_lock:
            x1 = self.angle_data['Time1 (s)']
            y1 = self.angle_data['Angle1 (radians)']
            if len(x1) > self.number_of_samples_in_window:
                x1 = x1[-self.number_of_samples_in_window:]
                y1 = y1[-self.number_of_samples_in_window:]

            x2 = self.angle_data['Time2 (s)']
            y2 = self.angle_data['Angle2 (radians)']
            if len(x2) > self.number_of_samples_in_window:
                x2 = x2[-self.number_of_samples_in_window:]
                y2 = y2[-self.number_of_samples_in_window:]

        plt.cla()

        plt.plot(x1, y1, label='Motor Angle')
        plt.plot(x2, y2, label='Encoder Angle')
            
        plt.title("Realtime Plot of Motor and Encoder Angles")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (radian)")
        plt.legend(loc='upper left')
        plt.tight_layout()
        plt.grid()

        return plt.gca(),

def main(args=None):
    rclpy.init(args=args)

    motor_anlge_topic = "/MRI_robot/motor_angle"
    encoder_angle_topic = "/MRI_robot/encoder_angle"

    # settings for the real-time plot
    # length of the time interval displayed on the plot (in seconds)
    time_window = 20
    # average period of publishing of data to the topics
    data_publishing_period = 0.05

    node1 = SubAndPlot(motor_anlge_topic, encoder_angle_topic, time_window, data_publishing_period)

    # spin the node in a separate thread
    node_spin_thread = threading.Thread(target=rclpy.spin, args=(node1,))
    node_spin_thread.start()

    # object to animate the plot
    ani = FuncAnimation(plt.gcf(), node1.animate_plot2, interval=100)
    plt.show()

    node1.destroy_node()
    rclpy.shutdown()
    node_spin_thread.join()

if __name__ == '__main__':
    main()
