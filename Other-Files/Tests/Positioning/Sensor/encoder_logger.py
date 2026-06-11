import serial
import time

# --- CONFIGURATION SECTION ---
PORT = "COM10"              # Change to your specific Arduino port (e.g., '/dev/ttyACM0' on Linux)
BAUD_RATE = 460800       # Must match firmware output speed
OUTPUT_FILE = "spin_drive_test.csv"  # Target output name for processing
# -----------------------------

def run_logger():
    print(f"[INIT] Opening connection to {PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Give Arduino a moment to stabilize after boot connection
        ser.reset_input_buffer()
        print(f"[READY] Successfully opened. Saving data streams to '{OUTPUT_FILE}'")
        
        with open(OUTPUT_FILE, 'w') as csv_file:
            # Write a clean, predictable header line for pandas ingestion
            csv_file.write("time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg\n")
            
            print(">>> LOGGING ACTIVE. Move joystick and push wheelchair now. Press Ctrl+C to terminate test. <<<")
            while True:
                if ser.in_waiting:
                    raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Split and parse to ensure it's a valid data line
                    payload = raw_line.split(',')
                    if payload[0] == "DATA" and len(payload) == 9:
                        # Rejoin data fields omitting the 'DATA' prefix tag
                        clean_csv_row = ",".join(payload[1:])
                        csv_file.write(clean_csv_row + "\n")
                        csv_file.flush() # Force write to physical storage drive disk immediately
                        
    except KeyboardInterrupt:
        print("\n[STOP] Logging stopped via user input request. File finalized safely.")
    except Exception as e:
        print(f"\n[CRITICAL ERROR] Serial exception occurred: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print("[EXIT] Thread cleaned up and closed cleanly.")

if __name__ == "__main__":
    run_logger()