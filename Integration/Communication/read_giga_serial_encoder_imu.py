import serial
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    # Give Arduino time to reset after serial connection
    time.sleep(2)

    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")

    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if not line:
            continue

        print(line)


if __name__ == "__main__":
    main()