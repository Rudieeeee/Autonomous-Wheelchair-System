#In Windows command prompt, run: usbipd list 
#usbipd attach --wsl --busid 2-4
#usbipd attach --wsl --busid 2-1

#In WSL, run: lsusb
#ls /dev/ttyUSB*


from rplidar import RPLidar, RPLidarException
import time
import threading

BAUDRATE = 460800

LIDARS = {
    "front_left": "/dev/ttyUSB0",
    "front_right": "/dev/ttyUSB1",
}


def stop_lidar(lidar):
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


def read_lidar(name, port):
    while True:
        lidar = None

        try:
            print(f"[{name}] Connecting on {port}...")
            lidar = RPLidar(port, baudrate=BAUDRATE, timeout=5)

            print(f"[{name}] Info:", lidar.get_info())
            print(f"[{name}] Health:", lidar.get_health())

            try:
                lidar._serial.reset_input_buffer()
                lidar._serial.reset_output_buffer()
            except Exception:
                pass

            time.sleep(1)

            print(f"[{name}] Reading scans. Press Ctrl+C to stop.")

            for scan_number, scan in enumerate(
                lidar.iter_scans(scan_type="normal", max_buf_meas=20000)
            ):
                if scan_number % 10 == 0:
                    print(f"[{name}] Scan {scan_number}: {len(scan)} points")

                    for quality, angle, distance in scan[:5]:
                        print(
                            f"[{name}] "
                            f"Angle: {angle:7.2f} deg | "
                            f"Distance: {distance:8.1f} mm | "
                            f"Quality: {quality}"
                        )

                    print("-" * 50)

        except RPLidarException as e:
            print(f"[{name}] RPLidar error: {e}")
            print(f"[{name}] Restarting connection...")
            time.sleep(2)

        except Exception as e:
            print(f"[{name}] Unexpected error: {e}")
            time.sleep(2)

        finally:
            if lidar is not None:
                print(f"[{name}] Stopping LiDAR...")
                stop_lidar(lidar)
                print(f"[{name}] Disconnected.")


def main():
    threads = []

    for name, port in LIDARS.items():
        thread = threading.Thread(
            target=read_lidar,
            args=(name, port),
            daemon=True
        )
        thread.start()
        threads.append(thread)

    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":
    main()