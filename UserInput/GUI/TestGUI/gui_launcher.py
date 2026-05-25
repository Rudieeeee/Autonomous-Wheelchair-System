import os
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import scrolledtext
from pathlib import Path


BASE_DIR = "/home/rudrh/Autonomous-Wheelchair-System"
MAPPING_DIR = (
    "/home/rudrh/Autonomous-Wheelchair-System/"
    "Positioning/MapGeneration/MappingTest"
)

LOCALIZATION_DIR = (
    "/home/rudrh/Autonomous-Wheelchair-System/"
    "Positioning/Localization/LocalizationTest"
)

NAVIGATION_DIR = (
    "/home/rudrh/Autonomous-Wheelchair-System/"
    "Navigation/Pathfinding/NavigationTest"
)

LOC_SETUP = (
    "source /opt/ros/jazzy/setup.bash && "
    f"source {MAPPING_DIR}/install/setup.bash && "
    f"source {LOCALIZATION_DIR}/install/setup.bash"
)

NAV_SETUP = (
    "source /opt/ros/jazzy/setup.bash && "
    f"source {MAPPING_DIR}/install/setup.bash && "
    f"source {LOCALIZATION_DIR}/install/setup.bash && "
    f"source {NAVIGATION_DIR}/install/setup.bash"
)

# Change these coordinates to real positions from your map.
# Format: "name": (x, y, yaw)
LOCATIONS = {
    "Mid": (-18.3722, -0.153244, 0.0),
    "Start": (0, 0, 0),
    "End": (-37.6, 20.0, 0),
}


class WheelchairLauncherGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Autonomous Wheelchair ROS2 Launcher")
        self.root.geometry("1000x720")

        self.processes = {}

        title = tk.Label(
            root,
            text="Autonomous Wheelchair ROS2 Launcher",
            font=("Arial", 18, "bold"),
        )
        title.pack(pady=10)

        button_frame = tk.Frame(root)
        button_frame.pack(pady=10)

        self.add_button(button_frame, "Build All", self.build_all, 0, 0)
        self.add_button(button_frame, "Clear All Builds", self.clear_all_builds, 0, 1)
        self.add_button(button_frame, "Start Mapping", self.start_mapping, 0, 2)
        self.add_button(button_frame, "Start Localization", self.start_localization, 0, 3)

        self.add_button(button_frame, "Start Navigation", self.start_navigation, 1, 0)
        self.add_button(button_frame, "Open Navigation RViz", self.open_navigation_rviz, 1, 1)
        self.add_button(button_frame, "Stop Mapping", lambda: self.stop_process("mapping"), 1, 2)
        self.add_button(button_frame, "Stop Localization", lambda: self.stop_process("localization"), 1, 3)

        self.add_button(button_frame, "Stop Navigation", lambda: self.stop_process("navigation"), 2, 0)
        self.add_button(button_frame, "Stop RViz", lambda: self.stop_process("rviz"), 2, 1)
        self.add_button(button_frame, "Stop All", self.stop_all, 2, 2)

        goal_label = tk.Label(
            root,
            text="Predefined Navigation Goals",
            font=("Arial", 14, "bold"),
        )
        goal_label.pack(pady=(10, 0))

        goal_frame = tk.Frame(root)
        goal_frame.pack(pady=10)

        self.add_button(
            goal_frame,
            "Go to Mid",
            lambda: self.send_navigation_goal("Mid"),
            0,
            0,
        )
        self.add_button(
            goal_frame,
            "Go to Start",
            lambda: self.send_navigation_goal("Start"),
            0,
            1,
        )
        self.add_button(
            goal_frame,
            "Go to End",
            lambda: self.send_navigation_goal("End"),
            0,
            2,
        )
        self.add_button(
            goal_frame,
            "Cancel Goal",
            self.cancel_navigation_goal,
            0,
            3,
        )

        self.log_box = scrolledtext.ScrolledText(root, wrap=tk.WORD, height=26)
        self.log_box.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.log("GUI ready.")

    def add_button(self, frame, text, command, row, column):
        button = tk.Button(
            frame,
            text=text,
            width=22,
            height=2,
            command=command,
        )
        button.grid(row=row, column=column, padx=5, pady=5)

    def log(self, message):
        self.log_box.insert(tk.END, message + "\n")
        self.log_box.see(tk.END)

    def safe_log(self, message):
        self.root.after(0, lambda: self.log(message))

    def run_command(self, name, command):
        process = self.processes.get(name)

        if process is not None and process.poll() is None:
            self.log(f"{name} is already running.")
            return

        self.log(f"Starting {name}...")

        process = subprocess.Popen(
            ["bash", "-c", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )

        self.processes[name] = process

        thread = threading.Thread(
            target=self.read_output_thread,
            args=(name, process),
            daemon=True,
        )
        thread.start()

    def read_output_thread(self, name, process):
        try:
            for line in process.stdout:
                self.safe_log(f"[{name}] {line.rstrip()}")
        except Exception as error:
            self.safe_log(f"[{name}] output error: {error}")

        process.wait()
        self.safe_log(f"{name} stopped with code {process.returncode}.")

    def stop_process(self, name):
        process = self.processes.get(name)

        if process is None or process.poll() is not None:
            self.log(f"{name} is not running.")
            return

        self.log(f"Stopping {name}...")

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except ProcessLookupError:
            self.log(f"{name} process already stopped.")
            return

        self.root.after(3000, lambda: self.force_stop_process(name))

    def force_stop_process(self, name):
        process = self.processes.get(name)

        if process is None or process.poll() is not None:
            return

        self.log(f"{name} did not stop after SIGINT. Sending SIGTERM...")

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            return

        self.root.after(3000, lambda: self.kill_process(name))

    def kill_process(self, name):
        process = self.processes.get(name)

        if process is None or process.poll() is not None:
            return

        self.log(f"{name} did not stop after SIGTERM. Sending SIGKILL...")

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            return

    def stop_all(self):
        for name in list(self.processes.keys()):
            self.stop_process(name)

    def build_all(self):
        command = f"""
        source /opt/ros/jazzy/setup.bash

        cd {MAPPING_DIR}
        colcon build --packages-select mapping_test --build-base build --install-base install --symlink-install

        source {MAPPING_DIR}/install/setup.bash

        cd {LOCALIZATION_DIR}
        colcon build --packages-select localization_test --build-base build --install-base install --symlink-install

        source {LOCALIZATION_DIR}/install/setup.bash

        cd {NAVIGATION_DIR}
        colcon build --packages-select navigation_test --build-base build --install-base install --symlink-install
        """

        self.run_command("build_all", command)

    def clear_all_builds(self):
        command = f"""
        echo "Removing build, install, and log folders..."
        rm -rf {MAPPING_DIR}/build {MAPPING_DIR}/install {MAPPING_DIR}/log
        rm -rf {LOCALIZATION_DIR}/build {LOCALIZATION_DIR}/install {LOCALIZATION_DIR}/log
        rm -rf {NAVIGATION_DIR}/build {NAVIGATION_DIR}/install {NAVIGATION_DIR}/log
        echo "All build, install, and log folders removed."
        """

        self.run_command("clear_all_builds", command)

    def start_mapping(self):
        command = f"""
        cd {MAPPING_DIR}
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        ros2 launch mapping_test mapping_test.launch.py
        """

        self.run_command("mapping", command)

    def start_localization(self):
        command = f"""
        cd {LOCALIZATION_DIR}
        {LOC_SETUP}
        ros2 launch localization_test localization_test.launch.py
        """

        self.run_command("localization", command)

    def start_navigation(self):
        command = f"""
        cd {NAVIGATION_DIR}
        {NAV_SETUP}
        ros2 launch navigation_test navigation_test.launch.py
        """

        self.run_command("navigation", command)

    def open_navigation_rviz(self):
        command = f"""
        {NAV_SETUP}
        rviz2 -d {NAVIGATION_DIR}/install/navigation_test/share/navigation_test/rviz/navigation.rviz
        """

        self.run_command("rviz", command)

    def send_navigation_goal(self, location_name):
        if location_name not in LOCATIONS:
            self.log(f"Unknown location: {location_name}")
            return

        x, y, yaw = LOCATIONS[location_name]

        command = f"""
        {NAV_SETUP}
        ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{{
          pose: {{
            header: {{
              frame_id: 'map'
            }},
            pose: {{
              position: {{
                x: {x},
                y: {y},
                z: 0.0
              }},
              orientation: {{
                z: {self.yaw_to_quaternion_z(yaw)},
                w: {self.yaw_to_quaternion_w(yaw)}
              }}
            }}
          }}
        }}"
        """

        self.run_command(f"goal_{location_name}", command)

    def cancel_navigation_goal(self):
        self.log("Cancel Goal: stopping navigation launch.")
        self.stop_process("navigation")

    def yaw_to_quaternion_z(self, yaw):
        import math
        return math.sin(yaw / 2.0)

    def yaw_to_quaternion_w(self, yaw):
        import math
        return math.cos(yaw / 2.0)

    def on_close(self):
        self.stop_all()
        self.root.after(500, self.root.destroy)


if __name__ == "__main__":
    root = tk.Tk()
    app = WheelchairLauncherGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()