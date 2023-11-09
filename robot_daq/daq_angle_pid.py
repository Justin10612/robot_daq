import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import time

from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String


class ANGLE_PID_DAQ(Node):

    angle = []
    angular_z = []
    time_list  = []
    angle_data = 0.0
    output_z = 0.0
    mode = ''
    rec_flag = False

    def __init__(self):
        super().__init__('ANGLE_PID_DAQ')
        self.pose_sub = self.create_subscription(Vector3, 'human_pose', self.pose_callback, 10)
        self.pose_sub  # prevent unused variable warning
        self.cmd_sub = self.create_subscription(Twist, 'follow_cmd_vel', self.cmd_callback, 10)
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
            # Record Data and Time
            current_time = round(time.time(), 2)
            self.time_list.append(current_time-self.start_time)
            self.angle.append(self.angle_data)
            self.angular_z.append(self.output_z)
            self.rec_flag = True
            plt.clf()
        # Exit Follow Mode
        else:
            if  self.rec_flag == True:
                self.get_logger().info('Stop')
                plt.plot(self.time_list, self.angle, label='Angle', color='red')
                plt.plot(self.time_list, self.angular_z, label='Output_z', color='blue')
                plt.xlabel('Time')
                plt.ylabel('Output')
                plt.title('Angle PID Respone')
                plt.legend(loc='upper left')
                plt.show()
                self.get_logger().info('Plot Closed')
                self.angle = []
                self.angular_z = []
                self.time_list = []
                self.rec_flag = False

    def pose_callback(self, msg):
        if self.mode == 'FOLLOW':
            #FOLLOW
            self.angle_data = (msg.x-640)*0.07

    def cmd_callback(self, msg):
        if self.mode == 'FOLLOW':
            self.output_z = msg.angular.z

def main(args=None):
    rclpy.init(args=args)
    pid_daq = ANGLE_PID_DAQ()
    rclpy.spin(pid_daq)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_daq.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()