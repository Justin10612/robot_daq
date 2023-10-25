import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import time

from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String


class PidDAQ(Node):

    depth = []
    linear_x = []
    time_list  = []
    mode = ''
    last_mode = ''
    

    def __init__(self):
        super().__init__('PID_DAQ')
        self.pose_sub = self.create_subscription(Vector3, 'human_pose', self.pose_callback, 10)
        self.pose_sub  # prevent unused variable warning
        self.cmd_sub = self.create_subscription(Twist, 'follow_cmd_vel', self.cmd_callback, 10)
        self.cmd_sub  # prevent unused variable warning
        self.mode_sub = self.create_subscription(String, 'robot_mode', self.mode_callback, 10)
        self.mode_sub  # prevent unused variable warning
        self.start_time = 0

    def mode_callback(self, msg):
        self.mode = msg.data
        if self.mode == 'FOLLOW':             
            if self.mode != self.last_mode:
                self.start_time = round(time.time(), 2)
                self.get_logger().info('Start')
            current_time = round(time.time(), 2)
            self.time_list.append(current_time-self.start_time)

        if self.mode != self.last_mode and self.mode != 'FOLLOW':
            # plt.plot(self.time_list, self.depth, 'red')
            plt.plot(self.time_list, self.linear_x, 'blue')
            plt.xlabel('Time')
            plt.ylabel('Output')
            plt.title('Depth PID Respone')
            plt.show()
            self.depth = []
            self.linear_x = []
            self.time_list = []
            self.get_logger.info('Stop')
        
        self.last_mode = msg.data

    def pose_callback(self, msg):
        if self.mode == 'FOLLOW':
            self.depth.append(float(msg.y))

    def cmd_callback(self, msg):
        if self.mode == 'FOLLOW':
            self.linear_x.append(float(msg.linear.x))

def main(args=None):
    rclpy.init(args=args)
    pid_daq = PidDAQ()
    rclpy.spin(pid_daq)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_daq.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()