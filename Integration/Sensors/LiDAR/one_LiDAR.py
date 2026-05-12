#In Windows command prompt, run: usbipd list 
#usbipd attach --wsl --busid 2-4

#In WSL, run: lsusb
#ls /dev/ttyUSB*


from rplidar import RPLidar, RPLidarException
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 460800


def run_once():
    lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=5)

    try:
        print("Connecting to RPLIDAR C1...")
        print("Info:", lidar.get_info())
        print("Health:", lidar.get_health())

        try:
            lidar._serial.reset_input_buffer()
            lidar._serial.reset_output_buffer()
        except Exception:
            pass

        time.sleep(1)

        print("\nReading scans. Press Ctrl+C to stop.\n")

        for scan_number, scan in enumerate(
            lidar.iter_scans(scan_type="normal", max_buf_meas=20000)
        ):
            print(f"Scan {scan_number}: {len(scan)} points")

            if scan_number % 10 == 0:
                for quality, angle, distance in scan[:10]:
                    print(
                        f"Angle: {angle:7.2f} deg | "
                        f"Distance: {distance:8.1f} mm | "
                        f"Quality: {quality}"
                    )
                print("-" * 50)

    finally:
        print("Stopping LiDAR...")
        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass
        print("Disconnected.")


def main():
    while True:
        try:
            run_once()

        except KeyboardInterrupt:
            print("\nStopped by user.")
            break

        except RPLidarException as e:
            print(f"RPLidar error: {e}")
            print("Restarting LiDAR connection...\n")
            time.sleep(2)


if __name__ == "__main__":
    main()