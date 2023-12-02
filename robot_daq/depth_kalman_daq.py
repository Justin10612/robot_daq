import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import time
import csv

from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String, Float64


class DepthKalmanDaq(Node):

    depth_raw_list = []
    depth_list = []
    time_list  = []
    depth_data = 0.0
    depth_raw = 0.0
    mode = ''
    rec_flag = False

    def __init__(self):
        super().__init__('depth_kalman_daq')
        self.pose_sub = self.create_subscription(Float64, 'depth_raw', self.depth_raw_callback, 10)
        self.pose_sub  # prevent unused variable warning
        self.cmd_sub = self.create_subscription(Float64, 'depth', self.depth_callback, 10)
        self.cmd_sub  # prevent unused variable warning
        self.mode_sub = self.create_subscription(String, 'robot_mode', self.mode_callback, 10)
        self.mode_sub  # prevent unused variable warning
        self.start_time = 0

    def mode_callback(self, msg):
        self.mode = msg.data
        # Enter Follow Mode, Start Recording.
        if self.mode == 'FOLLOW':
            # The First Time Enter Follow Mode
            if self.rec_flag == False:
                self.start_time = round(time.time(), 2)
                self.get_logger().info('Start')
                self.rec_flag = True
            # Record Data and Time
            current_time = round(time.time(), 2) - self.start_time
            self.time_list.append(current_time)
            self.depth_list.append(self.depth_data)
            self.depth_raw_list.append(self.depth_raw)
        # Exit Follow Mode
        else:
            if  self.rec_flag == True:
                self.get_logger().info('Stop')
                # Plot
                plt.plot(self.time_list, self.depth_raw_list, label='Depth_raw', color='red')
                plt.plot(self.time_list, self.depth_list, label='Depth_out', color='blue')
                plt.xlabel('Time')
                plt.ylabel('Output')
                plt.title('Depth Kalman Filter')
                plt.legend(loc='upper left')
                plt.show()
                self.get_logger().info('Plot Closed')
                # # Create CSV file
                # with open('output.csv', 'w', newline='') as csvfile:
                #     writer = csv.writer(csvfile, delimiter=',') 
                #     writer.writerow(self.depth)
                # # Cal
                
                # Clear
                self.depth_raw_list= []
                self.depth_list = []
                self.time_list = []
                self.rec_flag = False

    def depth_callback(self, msg):
        if self.mode == 'FOLLOW':
            #FOLLOW
            self.depth_data = msg.data

    def depth_raw_callback(self, msg):
        if self.mode == 'FOLLOW':
            self.depth_raw = msg.data

def main(args=None):
    rclpy.init(args=args)
    pid_daq = DepthKalmanDaq()
    rclpy.spin(pid_daq)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_daq.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()