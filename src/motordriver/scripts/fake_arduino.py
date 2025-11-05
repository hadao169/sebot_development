import serial
import time
import random

# --- CÁC THAM SỐ GIẢ LẬP (SAO CHÉP TỪ ARDUINO) ---
# THAY ĐỔI CỔNG NÀY CHO PHÙ HỢP VỚI LỆNH `socat` CỦA BẠN
SERIAL_PORT = '/dev/pts/5'
BAUDRATE = 115200
UPDATE_RATE_HZ = 10  # Tương đương printDelay=100
SWEEP_TIME_SEC = 0.8 # Tương đương SWEEP_TIME = 800

# --- CÁC BIẾN TRẠNG THÁI (SAO CHÉP TỪ ARDUINO) ---
motor1_speed_target = 0
motor2_speed_target = 0
motor1_encoder = 0
motor2_encoder = 0

scanState = "DRIVING" # Trạng thái ban đầu là DRIVING
sweepIndex = 0
lastScanTime = 0

ahead_distance = 100
left_distance = 15  # Giá trị khởi tạo từ Arduino
right_distance = 25 # Giá trị khởi tạo từ Arduino

# Biến để tự động giả lập vật cản
obstacle_simulation_triggered = False
drive_start_time = time.time()

def get_distance():
    """Mô phỏng hàm readDistance() của Arduino."""
    return random.randint(10, 200)

def handle_obstacle_scan(current_time):
    """Mô phỏng chính xác hàm handleObstacleScan() của Arduino."""
    global scanState, sweepIndex, lastScanTime, left_distance, right_distance, current_servo_angle

    if scanState == "DRIVING":
        return

    if scanState == "SWEEPING":
        if current_time - lastScanTime < SWEEP_TIME_SEC:
            return
        lastScanTime = current_time

        if sweepIndex == 0:
            print("--- [Servo Sim] Writing 30 deg ---")
            sweepIndex = 1
        elif sweepIndex == 1:
            left_distance = get_distance()
            print(f"--- [Servo Sim] Measured Left: {left_distance} cm. Writing 90 deg ---")
            sweepIndex = 2
        elif sweepIndex == 2:
            print("--- [Servo Sim] Writing 150 deg ---")
            sweepIndex = 3
        elif sweepIndex == 3:
            right_distance = get_distance()
            print(f"--- [Servo Sim] Measured Right: {right_distance} cm. Writing 90 deg ---")
            sweepIndex = 4
        elif sweepIndex == 4:
            print("--- [Servo Sim] Scan finished. Returning to DRIVING state. ---")
            sweepIndex = 0
            scanState = "DRIVING"

def main():
    global motor1_speed_target, motor2_speed_target, motor1_encoder, motor2_encoder
    global ahead_distance, scanState, sweepIndex, lastScanTime, drive_start_time, obstacle_simulation_triggered

    print(f"Connecting to virtual serial port {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
    except Exception as e:
        print(f"Error: Could not connect to {SERIAL_PORT}. Is socat running?")
        return

    print("Fake Arduino (1:1 Simulation) started...")
    last_update_time = time.time()

    while True:
        current_time = time.time()

        # --- Mô phỏng hàm readSerialPort() ---
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(f"[RX] from ROS 2: {line}")
                parts = line.split(';')
                cmd = parts[0]
                
                if cmd == "SPD" and len(parts) >= 3:
                    spd1 = int(float(parts[1]))
                    spd2 = int(float(parts[2]))
                    motor1_speed_target = spd1
                    motor2_speed_target = spd2
                    
                    if spd1 == 0 and spd2 == 0:
                        scanState = "SWEEPING"
                        sweepIndex = 0
                        lastScanTime = current_time
                    
                    if spd1 > 0: # Nếu bắt đầu lái, reset vật cản
                        drive_start_time = current_time
                        obstacle_simulation_triggered = False

        # --- Mô phỏng hàm loop() ---
        handle_obstacle_scan(current_time)

        if current_time - last_update_time >= (1.0 / UPDATE_RATE_HZ):
            last_update_time = current_time

            # Mô phỏng vật cản sau 3s lái xe
            if motor1_speed_target > 0 and (current_time - drive_start_time) > 3.0 and not obstacle_simulation_triggered:
                print("\n!!! SIMULATING OBSTACLE !!!")
                ahead_distance = 15
                obstacle_simulation_triggered = True
            
            # Mô phỏng motor.run() (chỉ encoder)
            motor1_encoder += int(motor1_speed_target * 0.1)
            motor2_encoder += int(motor2_speed_target * 0.1)
            
            # Mô phỏng sendData()
            if scanState == "DRIVING":
                ahead_distance = get_distance()

            motor1_pwm = int(abs(motor1_speed_target * 2.5))
            motor2_pwm = int(abs(motor2_speed_target * 2.5))
            
            # Chuỗi 9 trường dữ liệu
            data_to_send = (
                f"{motor1_encoder};{motor2_encoder};"
                f"{motor1_speed_target};{motor2_speed_target};"
                f"{motor1_pwm};{motor2_pwm};"
                f"{ahead_distance};{left_distance};{right_distance}\n"
            )
            print(f"[TX] to ROS 2: {data_to_send.strip()}")
            ser.write(data_to_send.encode('utf-8'))
            
        time.sleep(0.001)

if __name__ == '__main__':
    main()