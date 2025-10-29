import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import math

class CmdVelNode(Node):
  def __init__(self):
    super().__init__('cmd_vel_node')

    # Read parameters
    self.declare_parameter('wheel_radius', 0.1)
    self.declare_parameter('wheel_base', 0.5)
    self.declare_parameter('ticks_per_revolution', 1075)

    # Get parameter values
    self.wheel_radius = self.get_parameter('wheel_radius').value
    self.wheel_base = self.get_parameter('wheel_base').value
    self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').value

    self.get_logger().info(f'Wheel radius: {self.wheel_radius}')
    self.get_logger().info(f'Distance between wheels: {self.wheel_base}')
    self.get_logger().info(f'Sensor revolution: {self.ticks_per_revolution}')

    self.cmd_vel_subscriber = self.create_subscription(
        Twist,
        'cmd_vel_safe',
        self.cmd_vel_callback,
        10
    )

    self.mc_publisher = self.create_publisher(
        String,
        'motor_command',
        10
    )

  # Convert m/s to motor controller spd value
  def mps_to_spd(self,mps):
    pid_freq = 10 # Motor controller updates at 10Hz

    # angular velocity (rad/s)
    radps = mps / self.wheel_radius

    # RPM
    rpm = radps * ( 60 / ( 2 * math.pi ) ) # RPM = 60*v/(2*pi*r)

    spd = rpm * (self.ticks_per_revolution / 60 / pid_freq)
    return spd

  # Callback khi nhận được lệnh cmd_vel (loi dau cua van toc trai phai)
  def cmd_vel_callback(self, msg):
    #Inverse kinematics for differential drive
    # mps_l = left wheel m/s
    # mps_r = right wheel m/s
    mps_l = +(msg.linear.x - (msg.angular.z * self.wheel_base / 2.0))
    mps_r = -(msg.linear.x + (msg.angular.z * self.wheel_base / 2.0))

    # mps_l = msg.angualar.z * (self.wheel_base / 2.0)   
    string_msg = String()

    # co the truyen gia tri la SPD(du lieu duoc xu li qua PID), PWM (truc tiep set gia tri PWM)
    string_msg.data = f"SPD;{self.mps_to_spd(mps_l)};{self.mps_to_spd(mps_r)};"

    self.mc_publisher.publish(string_msg)

def main(args=None):
  rclpy.init(args=args)

  cmdvel_node = CmdVelNode()

  try:
    rclpy.spin(cmdvel_node)
  except KeyboardInterrupt:
    pass
  finally:
    cmdvel_node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()