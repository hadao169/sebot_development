import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Int32, String
from enum import Enum

SAFETY_DISTANCE_CM = 20  # Khoảng cách an toàn (cm)
AVOID_TURN_TIME = 5.0    # Thời gian quay khi tránh vật cản (s)
REVERSE_TIME = 2.0       # Thời gian lùi lại khi vật cản ở tất cả các hướng
isRevese = False

class State(Enum):
    DRIVING = 1
    WAITING_FOR_SCAN = 2
    AVOIDING = 3
    REVERSING = 4  # Trạng thái lùi lại

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid_node')
        self.last_teleop_cmd = Twist()
        self.distance = 1000
        self.scan_data = [1000, 1000]  
        self.isNewScanData = False
        self.state = State.DRIVING
        self.state_start_time = 0.0
        self.avoid_angular = 0.0
        self.avoid_linear = 0.0

        self.create_subscription(Twist, 'cmd_vel', self.teleop_callback, 10)
        self.create_subscription(Int32, 'ultrasonic_distance', self.distance_callback, 10)
        self.create_subscription(Int32MultiArray, 'scan_data', self.scan_callback, 10)

        self.safe_cmd_pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)
        self.arduino_cmd_pub = self.create_publisher(String, 'motor_command', 10)
        self.timer = self.create_timer(0.1, self.decision_callback)

    def teleop_callback(self, msg):
        self.last_teleop_cmd = msg

    def distance_callback(self, msg):
        self.distance = msg.data

    def scan_callback(self, msg):
        self.scan_data = msg.data
        self.isNewScanData = True
        self.get_logger().info(f" Received SCAN data: L={self.scan_data[0]}, R={self.scan_data[1]}, F={self.distance}")

    def decision_callback(self):
        final_cmd = Twist()

        if self.state == State.DRIVING:
            if self.distance < SAFETY_DISTANCE_CM and self.last_teleop_cmd.linear.x > 0:
                self.get_logger().info(f' Obstacle detected at {self.distance} cm — requesting SCAN...')
                self.arduino_cmd_pub.publish(String(data="SCAN;"))
                self.state = State.WAITING_FOR_SCAN
                return
            else:
                final_cmd = self.last_teleop_cmd

        elif self.state == State.WAITING_FOR_SCAN:
            if self.isNewScanData:
                left = self.scan_data[0]
                right = self.scan_data[1]
                front = self.distance
                self.get_logger().info(f" SCAN received: L={left}, R={right}, F={front}")

                if left < SAFETY_DISTANCE_CM and right < SAFETY_DISTANCE_CM and front < SAFETY_DISTANCE_CM:
                    final_cmd.linear.x = -0.5  
                    final_cmd.angular.z = 0.0
                    self.avoid_linear = final_cmd.linear.x
                    self.avoid_angular = final_cmd.angular.z
                    self.get_logger().info("Obstacle detected in all directions. Reversing.")
                    self.state = State.REVERSING
                    self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                    self.isNewScanData = False
                    return

                elif left <= right:  
                    final_cmd.linear.x = 0.0
                    final_cmd.angular.z = -1.0  
                    self.avoid_linear = final_cmd.linear.x
                    self.avoid_angular = final_cmd.angular.z
                    self.get_logger().info("Turning RIGHT to avoid obstacle.")
                else:  
                    final_cmd.linear.x = 0.0
                    final_cmd.angular.z = 1.0  
                    self.avoid_linear = final_cmd.linear.x
                    self.avoid_angular = final_cmd.angular.z
                    self.get_logger().info("Turning LEFT to avoid obstacle.")

                self.isNewScanData = False
                self.state = State.AVOIDING
                self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                return

            else:
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 0.0

        elif self.state == State.REVERSING:
            final_cmd.linear.x = self.avoid_linear
            final_cmd.angular.z = self.avoid_angular
            self.safe_cmd_pub.publish(final_cmd)

            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.state_start_time > REVERSE_TIME:
                self.get_logger().info("Reversal complete. Moving forward to avoid obstacle.")
                self.arduino_cmd_pub.publish(String(data="SCAN;"))
                self.state = State.WAITING_FOR_SCAN
                self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]
            return

        elif self.state == State.AVOIDING:
            # Quay trong một thời gian ngắn rồi quay lại trạng thái DRIVING
            final_cmd.linear.x = self.avoid_linear
            final_cmd.angular.z = self.avoid_angular
            self.safe_cmd_pub.publish(final_cmd)

            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.state_start_time > AVOID_TURN_TIME:
                self.get_logger().info("Avoid complete. Back to DRIVING.")
                self.state = State.DRIVING
                self.avoid_angular = 0.0  # Đặt lại giá trị tránh
                self.avoid_linear = self.last_teleop_cmd.linear.x  # Trở lại lệnh trước đó
            return

        self.safe_cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
