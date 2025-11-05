# Material 9 – Obstacle Avoidance System

## Part I. Hardware Documentation

### 1. Hardware Overview

This project integrates two additional components — an **ultrasonic distance sensor ([HC-SR04](https://www.aliexpress.com/item/1005007849944952.html))** and a **servo motor ([SG90](https://www.aliexpress.com/item/1005006283358420.html))** — with the existing motor control system to enable obstacle detection and directional scanning.

---

### 2. Ultrasonic Sensor (HC-SR04)

The HC-SR04 sensor measures the distance between the robot and obstacles ahead.  
It has four pins:

| Pin | Description | Arduino Pin |
|-----|--------------|--------------|
| VCC | Power Supply (+5V) | 5V |
| GND | Ground | GND |
| TRIG | Trigger Signal | A3 |
| ECHO | Echo Signal | A2 |

The sensor emits ultrasonic pulses through the **TRIG** pin and receives the reflected signal via the **ECHO** pin to calculate distance.

---

### 3. Servo Motor (SG90)

A small SG90 servo motor rotates the ultrasonic sensor for left and right scanning.  
The servo is connected as follows:

| Pin | Description | Arduino Pin |
|------|--------------|--------------|
| VCC | Power Supply (+5V) | 5V |
| GND | Ground | GND |
| Signal | Control Signal | A4 |

This setup allows the robot to measure distances in multiple directions for better navigation decisions.

---

### 4. DC Motors and Motor Driver (L293D)

Two DC motors drive the robot, controlled through an H-bridge motor driver (**L298N** or **L293D**).  
The driver controls motor direction and speed for forward, reverse, and turning motions.

**Initial Motor Pin Configuration:**
```cpp
Motor motor1(6, 8, 5, 3, 4);
Motor motor2(11, 12, 10, 2, 9);
```

During testing, a **timer conflict** occurred between the servo (A4) and **motor2** due to shared internal Arduino timer resources.  
To resolve this, **motor2** pin mapping was modified:

**Updated Motor Pin Configuration:**
```cpp
Motor motor2(7, 12, 11, 2, 9);
```

**Changes made:**
- Wire from **D10 (Enable)** moved to **D11**.
- Wire from **D11 (Control)** moved to **D7**.

---

## Part II. Software Flow Documentation

### 1. Node Architecture Overview

The obstacle avoidance system is implemented in **ROS 2**, where multiple nodes cooperate to achieve autonomous navigation and obstacle detection.  
The core nodes include:

- **motordriver_node** → Interfaces with Arduino for motor control and sensor data.
- **obstacle_avoid_node** → Makes decisions to avoid obstacles based on distance and scan data.
- **cmd_vel_node** → Converts safe velocity commands to motor driver instructions.
- **robot_state_publisher** → Publishes robot model transformations (TFs).

---

### 2. System Flow Diagram


### 3. Node Implementations
**~/ros2_ws/src/motordriver/motordriver/motordriver.py**
#### 3.1 motordriver_node (motordriver.py)

This node now publishes both **ultrasonic distance** and **scan data** along with existing motor data.

- **Publishes:**
  - `motor_data` → Encoder and speed readings.
  - `ultrasonic_distance` → Distance from ultrasonic sensor.
  - `scan_data` → Left and right distances from servo scans.
- **Subscribes:**
  - `motor_command` → Motor control commands from higher nodes.

```python
# (Code unchanged)
```

---

#### 3.2 obstacle_avoid_node (obstacle_avoid.py)
Let's create a motordriver python package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
ros2 pkg create --build-type ament_python obstacle_avoid
cd ~
```
Create a `obstacle_avoid.py` file to 

**~/ros2_ws/src/obstacle_avoid/obstacle_avoid/obstacle_avoid.py**
The **ObstacleAvoiderNode** functions as the robot’s main decision unit.  
It constantly reads movement commands (`cmd_vel`) and distance data from the **ultrasonic sensor** and **servo scan** (`ultrasonic_distance`, `scan_data`) to control motion through a **Finite State Machine (FSM)**.

- In **DRIVING**, the robot moves forward as long as the front distance is greater than the **safety threshold** (around **20 cm**).  
  When an obstacle is detected closer than this limit, it switches to **WAITING_FOR_SCAN**.  

- In **WAITING_FOR_SCAN**, the servo rotates left and right to measure side distances.  
  - If one side has a **larger distance**, meaning more free space, the robot **turns toward that side**.  
  - If both sides are **too close**, the robot **moves backward** to gain space before scanning again.  

- In **AVOIDING**, the robot keeps turning for a short, fixed time defined by `AVOID_TURN_TIME` (e.g., 5 seconds).  
- In **REVERSING**, it moves backward for `REVERSE_TIME` (e.g., 2 seconds), then requests a new scan.  

The node uses **non-blocking timers (`create_timer()`)** and timestamp checks (`get_clock().now()`) to manage how long each action lasts.  
This ensures smooth transitions between states without blocking other ROS2 processes, allowing continuous and real-time reactions to changing obstacle distances.

##### Topics

- **Subscribes:** `cmd_vel`, `ultrasonic_distance`, `scan_data`  
- **Publishes:** `cmd_vel_safe`, `motor_command`


```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Int32, String
from enum import Enum

SAFETY_DISTANCE_CM = 20  # Minimum safe distance
AVOID_TURN_TIME = 5.0    # Duration of turn (s)
REVERSE_TIME = 2.0       # Reverse duration (s)

class State(Enum):
    DRIVING = 1
    WAITING_FOR_SCAN = 2
    AVOIDING = 3
    REVERSING = 4

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
        self.get_logger().info(f"📡 SCAN Data Received: L={self.scan_data[0]}, R={self.scan_data[1]}, F={self.distance}")

    def decision_callback(self):
        final_cmd = Twist()

        if self.state == State.DRIVING:
            if self.distance < SAFETY_DISTANCE_CM and self.last_teleop_cmd.linear.x > 0:
                self.get_logger().info(f'Obstacle detected {self.distance} cm ahead — requesting SCAN')
                self.arduino_cmd_pub.publish(String(data="SCAN;"))
                self.state = State.WAITING_FOR_SCAN
                return
            else:
                final_cmd = self.last_teleop_cmd

        elif self.state == State.WAITING_FOR_SCAN:
            if self.isNewScanData:
                left, right, front = self.scan_data[0], self.scan_data[1], self.distance
                if left < SAFETY_DISTANCE_CM and right < SAFETY_DISTANCE_CM and front < SAFETY_DISTANCE_CM:
                    final_cmd.linear.x = -0.5
                    final_cmd.angular.z = 0.0
                    self.state = State.REVERSING
                    self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                    self.get_logger().info("All directions blocked — reversing.")
                    self.isNewScanData = False
                    return
                elif left <= right:
                    final_cmd.angular.z = -1.0
                    self.get_logger().info("Turning RIGHT to avoid obstacle.")
                else:
                    final_cmd.angular.z = 1.0
                    self.get_logger().info("Turning LEFT to avoid obstacle.")
                self.state = State.AVOIDING
                self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.isNewScanData = False
                return

        elif self.state == State.REVERSING:
            final_cmd.linear.x = -0.5
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.state_start_time > REVERSE_TIME:
                self.get_logger().info("Reversal complete — requesting SCAN.")
                self.arduino_cmd_pub.publish(String(data="SCAN;"))
                self.state = State.WAITING_FOR_SCAN
                self.state_start_time = current_time
                return

        elif self.state == State.AVOIDING:
            final_cmd.angular.z = 1.0 if self.avoid_angular == 0.0 else self.avoid_angular
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.state_start_time > AVOID_TURN_TIME:
                self.get_logger().info("Avoidance complete — resuming driving.")
                self.state = State.DRIVING
                self.avoid_angular = 0.0
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
```
---

#### 3.3 cmd_vel_node (cmd_vel.py)

- **Updated Subscription:** from `cmd_vel` ➞ `cmd_vel_safe`  
- Ensures all motion commands are validated by the obstacle avoidance logic.

---

#### 3.4 Launch File Update

Added the obstacle avoidance node to the main launch configuration:

```python
Node(
    package='obstacle_avoid',
    executable='obstacle_avoid_node',
    name='obstacle_avoid_node',
    output='screen',
    parameters=[os.path.join(colcon_prefix_path, 'config', 'params.yaml')]
),
```

---

### 4. System Behavior Summary

| Node | Subscribed Topics | Published Topics | Description |
|------|--------------------|------------------|--------------|
| `motordriver_node` | motor_command | motor_data, ultrasonic_distance, scan_data | Interfaces with Arduino for motor and sensor control |
| `obstacle_avoid_node` | cmd_vel, ultrasonic_distance, scan_data | cmd_vel_safe, motor_command | Makes autonomous obstacle avoidance decisions |
| `cmd_vel_node` | cmd_vel_safe | motor_command | Converts safe velocity commands to motor control signals |

