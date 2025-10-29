import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

SAFETY_DISTANCE_CM = 20 

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid_node')
        self.last_teleop_cmd = Twist()
        self.distance = 1000  # Initialize with a large distance

        # Subscriber lắng nghe lệnh từ bàn phím
        self.create_subscription(Twist, 'cmd_vel', self.teleop_callback, 10)
        
        self.create_subscription(Int32, 'ultrasonic_distance', self.distance_callback, 10)

        # Publisher để xuất bản lệnh an toàn
        self.safe_cmd_pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)

        self.create_timer(0.5, self.decision_callback)

    def teleop_callback(self, msg):
        self.last_teleop_cmd = msg

    def distance_callback(self, msg):
        self.distance = msg.data

    def decision_callback(self):
        final_cmd = Twist()
        # Nếu có vật cản phía trước VÀ người dùng đang muốn đi tới
        if self.distance < SAFETY_DISTANCE_CM:
            final_cmd.linear.x = 0.0
            final_cmd.angular.z = 0.0
            #final_cmd.angular.z = 0.5 # z > 0: turn left and z < 0: turn right
            self.get_logger().info(f'Obstacle detected at distance {self.distance}! Stopping and turning')
            
        else:
            final_cmd = self.last_teleop_cmd
        
        self.safe_cmd_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()