import serial
import time

# --- CẤU HÌNH CỔNG ARDUINO ---
# Thay đổi tên cổng này thành cổng thực tế của Arduino (ví dụ: /dev/ttyACM0)
SERIAL_PORT = '/dev/ttyACM0' 
BAUDRATE = 115200

def send_and_receive(ser, command):
    """Gửi lệnh và chờ phản hồi từ Arduino."""
    
    full_command = f"{command}\n"
    print(f"-> Gửi: {full_command.strip()}")
    
    try:
        # Gửi lệnh
        ser.write(full_command.encode('utf-8'))
        
        # Đợi một chút để Arduino xử lý và phản hồi
        time.sleep(0.1)
        
        # Đọc phản hồi
        if ser.in_waiting > 0:
            response = ser.read_all().decode('utf-8').strip()
            print(f"<- Nhận (Phản hồi): \n{response} 2")
        else:
            print("<- Nhận: KHÔNG CÓ PHẢN HỒI")
            
    except Exception as e:
        print(f"Lỗi giao tiếp: {e}")

def main():
    try:
        # 1. Khởi tạo kết nối Serial
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(2) # Chờ Arduino reset sau khi mở cổng
        print(f"Kết nối thành công với {SERIAL_PORT}")
        
    except serial.SerialException as e:
        print(f"Lỗi: Không thể mở cổng {SERIAL_PORT}. Kiểm tra quyền truy cập hoặc tên cổng.")
        print(f"Chi tiết: {e}")
        return

    # --- CHU TRÌNH KIỂM TRA ---

    print("\n--- BẮT ĐẦU KIỂM TRA LOGIC MOTOR ---")
    
    # 1. Kiểm tra Lệnh Di chuyển (Ví dụ: SPD 100/-100)
    send_and_receive(ser, "SPD;100;-100;")
    
    print("\n--- KIỂM TRA LỆNH DỪNG VÀ QUÉT SERVO ---")
    
    # **SỬA LỖI:** Không dùng send_and_receive, gửi lệnh trực tiếp
    ser.write("SPD;0;0;\n".encode('utf-8'))
    print("-> Gửi: SPD;0;0;")

    # 3. Vòng lặp chờ (MONITORING LOOP)
    print("Đang chờ quá trình quét (3.5 giây)...")
    start_wait_time = time.time()
    
    while (time.time() - start_wait_time) < 4.0: # Chờ 4 giây
        if ser.in_waiting > 0:
            # Đọc từng dòng một
            response_line = ser.readline().decode('utf-8').strip()
            print(f"<- Nhận: {response_line}")
            if response_line:
                if response_line.startswith("SCAN;"):
                    print(f"\n<<< ĐÃ TÌM THẤY KẾT QUẢ QUÉT: {response_line} >>>\n")
                else:
                    # Ghi log các dòng motor data (tần suất cao)
                    pass 
        time.sleep(0.05) # Giảm tải CPU

    # 4. Kiểm tra Lệnh Di chuyển lại (Ra khỏi trạng thái quét)
    print("\n--- KIỂM TRA KẾT THÚC QUÉT (GỬI LỆNH MỚI) ---")
    send_and_receive(ser, "SPD;50;50;") 

    ser.close()

if __name__ == '__main__':
    main()