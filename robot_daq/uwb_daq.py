import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String, Float64


class KalmanDaq(Node):

    raw_list = []
    filter_list = []
    time_list  = []
    mode = ''
    num = 0
    break_flag = False
    rec_flag = False

    def __init__(self):
        super().__init__('uwb_anchor_daq')
        # Subscriber
        self.mode_sub_ = self.create_subscription(String, 'robot_mode', self.mode_callback, 10)
        self.mode_sub_
        self.pose_sub = self.create_subscription(Twist, 'uwb_log_data', self.data_callback, 10)
        self.pose_sub  # prevent unused variable warning
        self.start_time = 0
        plt.ion()

    def mode_callback(self, msg):
        self.mode = msg.data
        # Enter Follow Mode, Start Recording.

    def data_callback(self, msg):
        if self.mode == 'TELEOP' and self.break_flag == False:
            k = 0
            # The First Time Enter Follow Mode
            if self.rec_flag == False and self.break_flag == False:
                self.num = 0
                self.rec_flag = True
                self.get_logger().info('Start')
            # Record Data and Time
            self.time_list.append(self.num)
            if k==0:
                self.raw_list.append(msg.linear.x)
                self.filter_list.append(msg.angular.x)
            if k==1:
                self.raw_list.append(msg.linear.y)
                self.filter_list.append(msg.angular.y)
            if k==2:
                self.raw_list.append(msg.linear.z)
                self.filter_list.append(msg.angular.z)
            self.num =  self.num + 1
            if self.num >100:
                self.break_flag = True
                plt.ioff()
                plt.show()
            print(self.num)
            # # Plot
            plt.clf()
            # plt.ylim(0, 200)
            plt.plot(self.time_list, self.raw_list, label='Raw_Data', color='red')
            plt.plot(self.time_list, self.filter_list, label='Filtered_Data', color='blue')
            plt.pause(0.001)
        # Exit Follow Mode
        else:
            if  self.rec_flag == True:
                self.get_logger().info('Stop')
                # # Plot
                # plt.ylim(0, 200)
                # plt.plot(self.time_list, self.raw_list, label='Raw_Data', color='red')
                # plt.plot(self.time_list, self.filter_list, label='Filtered_Data', color='blue')
                # plt.xlabel('Time(k)')
                # plt.ylabel('Output(m)')
                # plt.title('Anchor Kalman Filter')
                # plt.legend(loc='upper left')
                # plt.show()
                # self.get_logger().info('Plot Closed')
                # Clear
                self.raw_list= []
                self.filter_list = []
                self.time_list = []
                self.rec_flag = False
                self.break_flag == False

def main(args=None):
    rclpy.init(args=args)
    pid_daq = KalmanDaq()
    rclpy.spin(pid_daq)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_daq.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()