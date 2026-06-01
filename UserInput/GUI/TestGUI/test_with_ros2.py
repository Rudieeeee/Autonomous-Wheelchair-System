#!/usr/bin/env python3

import tkinter as tk
import subprocess
import signal
import os
import time

process = None

USB_ATTACH_COMMAND = """
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "
usbipd attach --wsl --busid 1-1;
usbipd attach --wsl --busid 1-3;
usbipd attach --wsl --busid 1-4
"
"""

ROS_MAPPING_COMMAND = """
cd /home/rudrh/Autonomous-Wheelchair-System/Positioning/MapGeneration/Mapping &&
source /opt/ros/jazzy/setup.bash &&
source install/setup.bash &&
ros2 launch map_generator mapping.launch.py \
left_lidar_port:=/dev/left_lidar \
right_lidar_port:=/dev/right_lidar \
arduino_port:=/dev/arduino_wheelchair \
auto_save_map:=true \
auto_save_delay:=5.0 \
save_map:=/home/rudrh/Autonomous-Wheelchair-System/Other-Files/GeneralData/Maps/test_map
"""

def attach_usb_devices():
    status_label.config(text="Attaching USB devices...")
    root.update_idletasks()

    result = subprocess.run(
        ["bash", "-c", USB_ATTACH_COMMAND],
        capture_output=True,
        text=True
    )

    print("USB attach stdout:")
    print(result.stdout)

    print("USB attach stderr:")
    print(result.stderr)

    if result.returncode == 0:
        status_label.config(text="USB devices attached")
        return True
    else:
        status_label.config(text="USB attach failed. Check terminal.")
        return False

def start_mapping():
    global process

    if process is not None and process.poll() is None:
        status_label.config(text="Mapping is already running")
        return

    usb_ok = attach_usb_devices()

    if not usb_ok:
        return

    time.sleep(2)

    status_label.config(text="Starting mapping...")
    root.update_idletasks()

    process = subprocess.Popen(
        ["bash", "-c", ROS_MAPPING_COMMAND],
        preexec_fn=os.setsid
    )

    status_label.config(text="Mapping started")

def stop_mapping():
    global process

    if process is not None and process.poll() is None:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        status_label.config(text="Stopping mapping...")
    else:
        status_label.config(text="Mapping is not running")

root = tk.Tk()
root.title("Wheelchair Mapping Launcher")
root.geometry("400x220")

title_label = tk.Label(root, text="ROS2 Mapping Control", font=("Arial", 16))
title_label.pack(pady=15)

start_button = tk.Button(
    root,
    text="Attach USB + Start Mapping",
    font=("Arial", 13),
    width=25,
    command=start_mapping
)
start_button.pack(pady=5)

stop_button = tk.Button(
    root,
    text="Stop Mapping",
    font=("Arial", 13),
    width=25,
    command=stop_mapping
)
stop_button.pack(pady=5)

status_label = tk.Label(root, text="Idle", font=("Arial", 11))
status_label.pack(pady=10)

root.mainloop()