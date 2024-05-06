import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import time

from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String, Float64


class DEPTH_PID_DAQ(Node):

  depth = []
  linear_x = []
  time_list  = []
  depth_data = 0.0
  output_x = 0.0
  mode = ''
  rec_flag = False
  i = 0
  
  def __init__(self):
    super().__init__('DEPTH_PID_DAQ')
    self.pose_sub = self.create_subscription(Vector3, 'human_pose', self.pose_callback, 10)
    self.pose_sub  # prevent unused variable warning
    self.cmd_sub = self.create_subscription(Twist, 'follow_cmd_vel', self.cmd_callback, 10)
    self.cmd_sub  # prevent unused variable warning
    self.mode_sub = self.create_subscription(String, 'robot_mode', self.mode_callback, 10)
    self.mode_sub  # prevent unused variable warning

  def mode_callback(self, msg):
    self.mode = msg.data
    # Enter Follow Mode, Start Recording.
    if self.mode == 'FOLLOW':
      # The First Time Enter Follow Mode
      if self.rec_flag == False:
        self.get_logger().info('Start')
        self.start_time = round(time.time(), 2)
        self.rec_flag = True
      # Record Data and Time
      self.time_list.append(self.i)
      self.depth.append(self.depth_data)
      self.linear_x.append(self.output_x)
      self.i = self.i+1
    # Exit Follow Mode
    else:
      if  self.rec_flag == True:
        self.get_logger().info('Stop')
        # Plot
        plt.plot(self.time_list, self.depth, label='Depth', color='red')
        plt.plot(self.time_list, self.linear_x, label='Output_x', color='blue')
        plt.xlabel('Time')
        plt.ylabel('Output')
        plt.title('Depth PID Respone')
        plt.legend(loc='upper left')
        plt.show()
        self.get_logger().info('Plot Closed')
        # Clear
        i=0
        self.depth = []
        self.linear_x = []
        self.time_list = []
        self.rec_flag = False
      else:
        self.start_time = 0

  def pose_callback(self, msg):
      if self.mode == 'FOLLOW':
        #FOLLOW
        self.depth_data = msg.y

  def cmd_callback(self, msg):
    if self.mode == 'FOLLOW':
      self.output_x = msg.linear.x

def main(args=None):
  rclpy.init(args=args)
  pid_daq = DEPTH_PID_DAQ()
  rclpy.spin(pid_daq)
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  pid_daq.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()