import serial
import threading
import time

# --- CONFIGURATION: SET YOUR PORTS AND FILENAMES HERE ---
PORT_BNO055 = "COM14"  # Replace with your BNO055 Arduino port
PORT_BNO085 = "COM10"  # Replace with your BNO085 Arduino port

FILE_BNO055 = "test_doing nothing_bno055_center.csv"
FILE_BNO085 = "test_doing_nothing_bno085_center.csv"

BAUD_RATE = 115200
# --------------------------------------------------------

stop_event = threading.Event()

def log_serial(port, filename):
    print(f"[START] Thread initialized for {port} -> {filename}")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)  # Allow Arduino reboot window to clear
        ser.reset_input_buffer()
        
        with open(filename, 'w') as f:
            f.write("timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz\n")
            
            while not stop_event.is_set():
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line and len(line.split(',')) == 10:
                        f.write(line + "\n")
                        f.flush()
    except Exception as e:
        print(f"\n[ERROR] Problem on port {port}: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print(f"[STOP] Port {port} closed safely.")

if __name__ == "__main__":
    # Initialize separate threads for simultaneous tracking
    thread_055 = threading.Thread(target=log_serial, args=(PORT_BNO055, FILE_BNO055))
    thread_085 = threading.Thread(target=log_serial, args=(PORT_BNO085, FILE_BNO085))
    
    thread_055.start()
    thread_085.start()
    
    print("\n>>> DUAL LOGGING ACTIVE. Press Ctrl+C to terminate safely. <<<\n")
    
    try:
        while thread_055.is_alive() or thread_085.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[SIGNAL] Shutdown signal received. Wrapping up log files...")
        stop_event.set()
        
    thread_055.join()
    thread_085.join()
    print("All logging threads completed successfully.")