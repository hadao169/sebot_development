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
  import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32, Int32MultiArray

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

    self.scan_publisher = self.create_publisher(
        Int32MultiArray,
        'scan_data', 
        10
    )
    
    timer_period = 0.01  # Seconds <=> 100Hz
    self.timer = self.create_timer(timer_period, self.timer_callback)


  def timer_callback(self):
    if self.msg != "x\n":
        self.arduino.write(self.msg.encode()) 

    if self.timercount == 11: # every 0.1s <=> 10Hz (inversely proportional to timer period) => after 11 cycles
        if self.msg == "x\n":
            self.arduino.write(self.msg.encode())

        self.timercount = 0
        # Read from Arduino (inWaiting return the number of bytes in buffer(temporary storage area))
        while self.arduino.inWaiting()>0:
            response = ""
            try: 
              response = self.arduino.readline().decode("utf8").strip()
              if not response:
                continue

              answer = response.split(";") # return a list of strings of the values separated by ";" 

              if len(answer) == 3 and answer[0] == "SCAN":
                
                leftDistance = int(answer[1])
                rightDistance = int(answer[2])
                scan_msg = Int32MultiArray()
                scan_msg.data = [leftDistance, rightDistance]
                self.scan_publisher.publish(scan_msg)
                self.get_logger().info(f'>>> Published SCAN data: [L:{leftDistance}, R:{rightDistance}]')

              elif len(answer) == 7:
                msg = MotordriverMessage()
                msg.encoder1 = int(answer[0])
                msg.encoder2 = int(answer[1])
                msg.speed1 = int(answer[2])
                msg.speed2 = int(answer[3])
                msg.pwm1 = int(answer[4])
                msg.pwm2 = int(answer[5])
                distance = int(answer[6]) 
                distance_msg = Int32()
                distance_msg.data = distance
                self.publisher.publish(msg)
                self.distance_publisher.publish(distance_msg)
                self.get_logger().debug(f'Published 7-field motor data (Distance: {distance})') # Dùng debug để đỡ rối

            except Exception as err:
              self.get_logger().warn(f"Failed to parse line '{response}': {err}")
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
```

---

#### 3.2 Arduino (~/ros2_ws/arduino/motorcontroller/motorcontrller.ino)
```cpp
#include "Motor.h"
#include <Servo.h>

const int trigPin = A3;
const int echoPin = A2;
const int servoPin = A4;

Servo servo;
Motor *Motor::instances[2] = {nullptr, nullptr};
int Motor::instanceCount = 0;

// Motor motor1(4,5,9,2,3);
Motor motor1(6, 8, 5, 3, 4);
// Motor motor2(11, 12, 10, 2, 9);
Motor motor2(7, 12, 11, 2, 9);

int mode = 0; // 0=PWM 1=SPEED
int alive = 0;
int aliveSignal = 10 * 3; // 3s
int printDelay = 0;

enum ScanState {
  DRIVING,
  SWEEPING
};
ScanState scanState = DRIVING;
unsigned long lastScanTime = 0;
const int SWEEP_TIME = 800; // 800ms per action
int sweepIndex = 0; // 0: start, 1: left, 2: middle, 3: right, 4: report
int leftDistance = 1000;
int rightDistance = 1000;

unsigned long startTime = 0;
String msg;

void setup()
{
	motor1.begin();
	motor2.begin();
	servo.write(90);
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	servo.attach(servoPin);
	Serial.begin(115200);
}

void loop()
{
	handleObstacleScan(millis());
	readSerialPort();

	if (millis() - startTime >= 1)
	{
		startTime++;
		if (printDelay == 0)
		{
			printDelay = 100; // 100ms

			if (alive > 0)
			{
				alive--;
			}

			if (aliveSignal == 0)
			{
				alive = 1;
			}

			motor1.run(mode, alive);
			motor2.run(mode, alive);

			sendData();
		}
		printDelay--;
	}
}

void readSerialPort()
{
	msg = "";
	String sa[4];
	int r = 0, t = 0;

	if (Serial.available())
	{
		msg = Serial.readStringUntil('\n');

		for (int i = 0; i < msg.length(); i++)
		{
			if (msg.charAt(i) == ';')
			{
				sa[t] = msg.substring(r, i);
				r = (i + 1);
				t++;
			}
		}

		if (sa[0] == "PWM")
		{
			mode = 0;
			motor1.setPWM(sa[1].toInt());
			motor2.setPWM(sa[2].toInt());
		}
		else if (sa[0] == "SPD")
		{
			mode = 1;
			alive = aliveSignal;
			motor1.setSPD(sa[1].toInt());
			motor2.setSPD(sa[2].toInt());
		}
		else if (sa[0] == "PID")
		{
			motor1.setPID(sa[1].toFloat(), sa[2].toFloat(), sa[3].toFloat());
			motor2.setPID(sa[1].toFloat(), sa[2].toFloat(), sa[3].toFloat());
		}
		else if (sa[0] == "ZERO")
		{
			motor1.zeroEncoder();
			motor2.zeroEncoder();
		}
		else if (sa[0] == "ALIVE")
		{
			aliveSignal = sa[1].toInt() * 10;
		}
		else if (sa[0] == "SCAN")
		{
			scanState = SWEEPING;
			sweepIndex = 0;
			lastScanTime = millis();
		}

		Serial.flush();
	}
}

void sendData()
{
	Serial.print(motor1.getEncoder());
	Serial.print(";");
	Serial.print(motor2.getEncoder());
	Serial.print(";");
	Serial.print(motor1.getSpeed());
	Serial.print(";");
	Serial.print(motor2.getSpeed());
	Serial.print(";");
	Serial.print(motor1.getMotorSpeed());
	Serial.print(";");
	Serial.print(motor2.getMotorSpeed());
	Serial.print(";");
	Serial.print(readDistance());
	Serial.print("\n");
}

void sendScanData()
{
	Serial.print("SCAN;");
	Serial.print(leftDistance);
	Serial.print(";");
	Serial.print(rightDistance);
	Serial.print("\n");
}

int readDistance()
{
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);
	long duration = pulseIn(echoPin, HIGH);
	return (duration / 2 / 29.41);
}

void handleObstacleScan(unsigned long currentTime)
{
	if (scanState == DRIVING)
	{
		return;
	}

	if (scanState == SWEEPING)
	{
		if (currentTime - lastScanTime < SWEEP_TIME)
		{
			return;
		}
		lastScanTime = currentTime;

		switch (sweepIndex)
		{
		case 0:
			servo.write(30);
			sweepIndex = 1;
			break;
		case 1:
			rightDistance = readDistance();
			servo.write(90);
			sweepIndex = 2;
			break;
		case 2:
			servo.write(150);
			sweepIndex = 3;
			break;
		case 3:
			leftDistance = readDistance();
			servo.write(90);
			sweepIndex = 4;
			break;
		case 4:
			sendScanData();
			sweepIndex = 0;
			scanState = DRIVING;
			break;
		}
	}
}
```
---

#### 3.3 obstacle_avoid_node (obstacle_avoid.py)
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

All good, update `setup.py` and compile the package as part of the system:

**~/ros2_ws/src/obstacle_avoid/setup.py**
```python
from setuptools import find_packages, setup

package_name = 'obstacle_avoid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoid_node = obstacle_avoid.obstacle_avoid:main',
        ],
    },
)
```
---

#### 3.4 cmd_vel_node (cmd_vel.py)
**~/ros2_ws/src/diffdrive/diffdrive/cmd_vel.py**
- **Updated Subscription:** from `cmd_vel` ➞ `cmd_vel_safe`  
- This node has been updated to subscribe to the cmd_vel_safe topic instead of the original cmd_vel.
By doing so, it ensures that all motion commands are first processed and verified by the ObstacleAvoiderNode before being sent to the motor driver.

```python
self.cmd_vel_subscriber = self.create_subscription(
  Twist,
  'cmd_vel_safe',
  self.cmd_vel_callback,
  10
)
```
---

#### 3.5 Launch File Update
~/ros2_ws/src/diffdrive/launch/diffdrive.launch.py
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

