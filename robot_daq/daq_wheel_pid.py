import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import time

from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String


class WHEEL_PID_DAQ(Node):

    l_target_vel = [] #rad/s
    l_actual_vel = []
    l_cmd_vel = 0.0
    l_wheel_vel = 0.0

    r_target_vel = [] #rad/s
    r_actual_vel = []
    r_cmd_vel = 0.0
    r_wheel_vel = 0.0

    time_list  = []
    mode = ''
    rec_flag = False
    pi_constant = 6.283

    wheel_select = 2
    # select 0:left 1:right 2:both

    def __init__(self):
        super().__init__('DEPTH_PID_DAQ')
        self.cmd_vel_sub = self.create_subscription(Vector3, 'wheel_cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_sub  # prevent unused variable warning
        self.actual_vel_sub = self.create_subscription(Vector3, 'wheel_vel', self.actual_vel_callback, 10)
        self.actual_vel_sub  # prevent unused variable warning
        self.mode_sub = self.create_subscription(String, 'robot_mode', self.mode_callback, 10)
        self.mode_sub  # prevent unused variable warning
        self.start_time = 0

    def mode_callback(self, msg):
        self.mode = msg.data
        # Enter Follow Mode, Start Recording.
        if self.mode == 'TELEOP':
            # The First Time Enter Follow Mode
            if self.rec_flag == False:
                self.start_time = round(time.time(), 3)
                self.get_logger().info('Start')
            # Record Data and Time
            current_time = round(time.time(), 3)
            self.time_list.append(current_time-self.start_time)
            if self.wheel_select == 0 or self.wheel_select==2:
                self.l_target_vel.append(self.l_cmd_vel)
                self.l_actual_vel.append(self.l_wheel_vel)
            if self.wheel_select == 1 or self.wheel_select==2:
                self.r_target_vel.append(self.r_cmd_vel)
                self.r_actual_vel.append(self.r_wheel_vel)
            self.rec_flag = True
        # Exit Follow Mode
        else:
            if  self.rec_flag == True:
                self.get_logger().info('Stop')
                # left wheel
                if self.wheel_select == 0 or self.wheel_select==2:
                    l_cmd_vel_rps = []
                    for i in self.l_target_vel:
                        l_cmd_vel_rps.append(-i/self.pi_constant)
                    plt.plot(self.time_list, l_cmd_vel_rps, label='l_cmd_vel', color='black')
                    plt.plot(self.time_list, self.l_actual_vel, label='l_wheel_vel', color='orange')
                if self.wheel_select == 1 or self.wheel_select==2:
                    r_cmd_vel_rps = []
                    for i in self.r_target_vel:
                        r_cmd_vel_rps.append(i/self.pi_constant)
                    plt.plot(self.time_list, r_cmd_vel_rps, label='r_cmd_vel', color='blue')
                    plt.plot(self.time_list, self.r_actual_vel, label='r_wheel_vel', color='green')
                plt.xlabel('Time')
                plt.ylabel('In / Output')
                plt.title('Motor PID Response')
                plt.legend(loc='upper left')
                plt.show()
                self.get_logger().info('Plot Closed')
                self.l_actual_vel = []
                self.linear_x = []
                self.r_actual_vel = []
                self.rinear_x = []
                self.time_list = []
                self.rec_flag = False

    def cmd_vel_callback(self, msg):
        if self.mode == 'TELEOP':
            self.l_cmd_vel = msg.x
            self.r_cmd_vel = msg.y

    def actual_vel_callback(self, msg):
        if self.mode == 'TELEOP':
            self.l_wheel_vel = round(msg.x, 2)
            self.r_wheel_vel = round(msg.y, 2)

def main(args=None):
    rclpy.init(args=args)
    pid_daq = WHEEL_PID_DAQ()
    rclpy.spin(pid_daq)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_daq.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()