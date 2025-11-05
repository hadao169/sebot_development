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

### 4. DC Motors and Motor Driver (L298N / L293D)

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

```mermaid
graph TD

A[cmd_vel (Teleoperation)] -->|User Commands| B[obstacle_avoid_node]
B -->|Filtered Safe Velocity| C[cmd_vel_safe]
C --> D[cmd_vel_node]
D -->|Speed Commands| E[motordriver_node]
E -->|Encoder, Speed, Distance| F[motor_data]
E -->|Ultrasonic Distance| G[ultrasonic_distance]
E -->|Scan Data (Left, Right)| H[scan_data]

subgraph Arduino
  E
end

subgraph ROS2_Nodes
  B
  C
  D
  F
  G
  H
end

subgraph Decision Logic
  G -->|Obstacle Detected| B
  H -->|Directional Scan| B
end
```

---

### 3. Node Implementations

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

The **ObstacleAvoiderNode** determines robot behavior using a **Finite State Machine (FSM)** that transitions between driving, scanning, avoiding, and reversing.

##### FSM States

- **DRIVING:** Moves forward unless an obstacle is detected.
- **WAITING_FOR_SCAN:** Requests scan data when an obstacle is close.
- **AVOIDING:** Turns in the safer direction for a fixed duration.
- **REVERSING:** Moves backward when all sides are blocked.

Timers are implemented via ROS2's `create_timer()` for non-blocking transitions.

##### Topics

- **Subscribes:** `cmd_vel`, `ultrasonic_distance`, `scan_data`  
- **Publishes:** `cmd_vel_safe`, `motor_command`

```python
# (Code unchanged)
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

