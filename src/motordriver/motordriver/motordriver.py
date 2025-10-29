import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32

import serial
import time

from motordriver_msgs.msg import MotordriverMessage

try:
  from .simserial import SimSerial
except:
  from simserial import SimSerial
  
class MotordriverNode(Node):
  def __init__(self):
    super().__init__('motordriver_node')

    self.msg = "x\n"
    self.timercount = 0

    # Declare parameters and get their values
    self.declare_parameter('simulation', False)
    self.simulation = self.get_parameter('simulation').value
    self.get_logger().info(f'Starting motor_controller in simulation: {self.simulation}')
    if self.simulation:
      self.arduino = SimSerial()
    else:
      self.arduino = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
      if not self.arduino.isOpen():
        raise Exception("No connection to motor controller")

    self.arduino.write(("ALIVE;1;\n").encode())

    self.subscriber = self.create_subscription(
        String,
        'motor_command', # relative reference. If you write a slash in front, for example '/motor_command', it becomes absolute, and the namespace setting no longer affects it.
        self.motor_command_callback,
        10
    )

    self.publisher = self.create_publisher(
        MotordriverMessage,
        'motor_data',
        10
    )

    self.distance_publisher = self.create_publisher(
        Int32,
        'ultrasonic_distance', # publish distance
        10
    )
    
    timer_period = 0.01  # Seconds <=> 100Hz
    self.timer = self.create_timer(timer_period, self.timer_callback)


  # use:  
  def timer_callback(self):
    # Create message for Arduino
    if self.msg != "x\n": # only send if there is a new command
        self.arduino.write(self.msg.encode()) #change string into bytes, then send to arduino

    if self.timercount == 11: # every 0.1s <=> 10Hz (inversely proportional to timer period) => after 11 cycles
        if self.msg == "x\n":
            self.arduino.write(self.msg.encode())
            print(self.msg)

        self.timercount = 0
        # Read from Arduino (inWaiting return the number of bytes in buffer(temporary storage area))
        if self.arduino.inWaiting()>0:
          while self.arduino.inWaiting()>0:
            answer=self.arduino.readline().decode("utf8").split(";") # return a list of strings of the values separated by ";" (reduce the number of bytes in buffer)
            print(answer)
          msg = MotordriverMessage()

          try:
            msg.encoder1 = int(answer[0])
            msg.encoder2 = int(answer[1])
            msg.speed1 = int(answer[2])
            msg.speed2 = int(answer[3])
            msg.pwm1 = int(answer[4])
            msg.pwm2 = int(answer[5])

            # Read distance value from the ultrasonic sensor
            distance = int(answer[6]) 
            distance_msg = Int32()
            distance_msg.data = distance

            # Publish message on topic
            self.publisher.publish(msg)
            self.distance_publisher.publish(distance_msg) 

          except Exception as err:
            pass

    self.msg = "x\n"
    self.timercount += 1

  def motor_command_callback(self, message):
    self.msg = f"{message.data}\n"

def main(args=None):
  rclpy.init(args=args)
  motordriver_node = MotordriverNode()
  try:
    rclpy.spin(motordriver_node)
  except KeyboardInterrupt:
    pass
  finally:
    motordriver_node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()