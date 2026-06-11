#!/usr/bin/env python3

import csv
import math
import os
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray


@dataclass
class CalibrationStep:
    axis: str
    joystick_x: int
    joystick_y: int


class WheelchairCalibrationCollector(Node):
    """
    Publishes fixed joystick commands and records the measured odometry response.

    Run this with the Arduino sensor node active, because that node subscribes to
    /joystick_cmd and writes J,x,y to the wheelchair controller.

    Do not run Nav2 or cmd_vel_to_joystick while this script is running.
    """

    def __init__(self):
        super().__init__("wheelchair_calibration_collector")

        self.declare_parameter("joystick_topic", "/joystick_cmd")
        self.declare_parameter("odom_topic", "/odom")

        self.declare_parameter("output_directory", ".")
        self.declare_parameter("raw_output_file", "wheelchair_calibration_raw.csv")
        self.declare_parameter("summary_output_file", "wheelchair_calibration_step_summary.csv")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("settle_time_s", 1.5)
        self.declare_parameter("sample_time_s", 2.0)
        self.declare_parameter("stop_time_s", 1.0)

        self.declare_parameter("test_forward_y", [25, 30, 35, 40, 45, 50, 60])
        self.declare_parameter("test_backward_y", [40, 45, 50, 60])
        self.declare_parameter("test_left_x", [25, 30, 35, 40, 45, 50, 60])
        self.declare_parameter("test_right_x", [25, 30, 35, 40, 45, 50, 60])

        self.declare_parameter("include_forward", True)
        self.declare_parameter("include_backward", True)
        self.declare_parameter("include_left", True)
        self.declare_parameter("include_right", True)

        self.joystick_topic = self.get_parameter("joystick_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.output_directory = self.get_parameter("output_directory").value
        self.raw_output_file = self.get_parameter("raw_output_file").value
        self.summary_output_file = self.get_parameter("summary_output_file").value

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.settle_time_s = float(self.get_parameter("settle_time_s").value)
        self.sample_time_s = float(self.get_parameter("sample_time_s").value)
        self.stop_time_s = float(self.get_parameter("stop_time_s").value)

        self.test_forward_y = [int(v) for v in self.get_parameter("test_forward_y").value]
        self.test_backward_y = [int(v) for v in self.get_parameter("test_backward_y").value]
        self.test_left_x = [int(v) for v in self.get_parameter("test_left_x").value]
        self.test_right_x = [int(v) for v in self.get_parameter("test_right_x").value]

        self.include_forward = bool(self.get_parameter("include_forward").value)
        self.include_backward = bool(self.get_parameter("include_backward").value)
        self.include_left = bool(self.get_parameter("include_left").value)
        self.include_right = bool(self.get_parameter("include_right").value)

        os.makedirs(self.output_directory, exist_ok=True)
        self.raw_path = os.path.join(self.output_directory, self.raw_output_file)
        self.summary_path = os.path.join(self.output_directory, self.summary_output_file)

        self.publisher = self.create_publisher(Int16MultiArray, self.joystick_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20,
        )

        self.steps: List[CalibrationStep] = self.build_steps()
        self.current_step_index = -1
        self.current_step: Optional[CalibrationStep] = None
        self.step_start_time = self.get_clock().now()
        self.finished = False

        self.latest_v = 0.0
        self.latest_w = 0.0
        self.has_odom = False
        self.samples = []

        self.raw_file_handle = open(self.raw_path, "w", newline="")
        self.summary_file_handle = open(self.summary_path, "w", newline="")

        self.raw_writer = csv.DictWriter(
            self.raw_file_handle,
            fieldnames=[
                "step_index",
                "axis",
                "joystick_x",
                "joystick_y",
                "t_step_s",
                "phase",
                "measured_v_mps",
                "measured_w_radps",
            ],
        )
        self.summary_writer = csv.DictWriter(
            self.summary_file_handle,
            fieldnames=[
                "step_index",
                "axis",
                "joystick_x",
                "joystick_y",
                "n_samples",
                "mean_v_mps",
                "median_v_mps",
                "mean_w_radps",
                "median_w_radps",
                "abs_mean_v_mps",
                "abs_median_v_mps",
                "abs_mean_w_radps",
                "abs_median_w_radps",
            ],
        )
        self.raw_writer.writeheader()
        self.summary_writer.writeheader()

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

        self.get_logger().info("Wheelchair calibration collector started")
        self.get_logger().info(f"Publishing joystick commands to {self.joystick_topic}")
        self.get_logger().info(f"Reading measured velocity from {self.odom_topic}")
        self.get_logger().info(f"Raw output: {self.raw_path}")
        self.get_logger().info(f"Summary output: {self.summary_path}")
        self.get_logger().warn("Make sure the wheelchair is lifted or in a large clear area.")
        self.get_logger().warn("Make sure Nav2 and cmd_vel_to_joystick are not publishing joystick commands.")

        self.start_next_step()

    def build_steps(self) -> List[CalibrationStep]:
        steps = []

        if self.include_forward:
            for y in self.test_forward_y:
                steps.append(CalibrationStep("linear_positive", 0, abs(int(y))))

        if self.include_backward:
            for y in self.test_backward_y:
                steps.append(CalibrationStep("linear_negative", 0, -abs(int(y))))

        if self.include_left:
            for x in self.test_left_x:
                steps.append(CalibrationStep("angular_positive", abs(int(x)), 0))

        if self.include_right:
            for x in self.test_right_x:
                steps.append(CalibrationStep("angular_negative", -abs(int(x)), 0))

        return steps

    def odom_callback(self, msg: Odometry):
        self.latest_v = float(msg.twist.twist.linear.x)
        self.latest_w = float(msg.twist.twist.angular.z)
        self.has_odom = True

    def publish_joystick(self, x: int, y: int):
        msg = Int16MultiArray()
        msg.data = [self.clamp_int(x), self.clamp_int(y)]
        self.publisher.publish(msg)

    @staticmethod
    def clamp_int(value: int, low: int = -100, high: int = 100) -> int:
        return max(low, min(high, int(value)))

    @staticmethod
    def median(values):
        if not values:
            return 0.0
        sorted_values = sorted(values)
        n = len(sorted_values)
        middle = n // 2
        if n % 2 == 1:
            return sorted_values[middle]
        return 0.5 * (sorted_values[middle - 1] + sorted_values[middle])

    def start_next_step(self):
        self.publish_joystick(0, 0)
        self.samples = []
        self.current_step_index += 1

        if self.current_step_index >= len(self.steps):
            self.finished = True
            self.get_logger().info("Calibration sequence finished. Sending stop command.")
            self.publish_joystick(0, 0)
            self.raw_file_handle.flush()
            self.summary_file_handle.flush()
            return

        self.current_step = self.steps[self.current_step_index]
        self.step_start_time = self.get_clock().now()
        self.get_logger().info(
            f"Step {self.current_step_index + 1}/{len(self.steps)}: "
            f"axis={self.current_step.axis}, "
            f"x={self.current_step.joystick_x}, y={self.current_step.joystick_y}"
        )

    def write_summary_for_current_step(self):
        if self.current_step is None:
            return

        v_values = [sample[0] for sample in self.samples]
        w_values = [sample[1] for sample in self.samples]

        n = len(self.samples)
        mean_v = sum(v_values) / n if n > 0 else 0.0
        mean_w = sum(w_values) / n if n > 0 else 0.0
        median_v = self.median(v_values)
        median_w = self.median(w_values)

        row = {
            "step_index": self.current_step_index,
            "axis": self.current_step.axis,
            "joystick_x": self.current_step.joystick_x,
            "joystick_y": self.current_step.joystick_y,
            "n_samples": n,
            "mean_v_mps": mean_v,
            "median_v_mps": median_v,
            "mean_w_radps": mean_w,
            "median_w_radps": median_w,
            "abs_mean_v_mps": abs(mean_v),
            "abs_median_v_mps": abs(median_v),
            "abs_mean_w_radps": abs(mean_w),
            "abs_median_w_radps": abs(median_w),
        }
        self.summary_writer.writerow(row)
        self.summary_file_handle.flush()

        self.get_logger().info(
            f"Summary axis={self.current_step.axis}, "
            f"x={self.current_step.joystick_x}, y={self.current_step.joystick_y}, "
            f"median_v={median_v:.4f} m/s, median_w={median_w:.4f} rad/s, n={n}"
        )

    def timer_callback(self):
        if self.finished:
            self.publish_joystick(0, 0)
            return

        if self.current_step is None:
            self.start_next_step()
            return

        now = self.get_clock().now()
        t_step_s = (now - self.step_start_time).nanoseconds / 1e9
        total_step_time_s = self.stop_time_s + self.settle_time_s + self.sample_time_s

        if not self.has_odom:
            self.publish_joystick(0, 0)
            self.get_logger().warn("No odometry received yet. Waiting.")
            return

        if t_step_s < self.stop_time_s:
            self.publish_joystick(0, 0)
            phase = "stop_before_step"
        else:
            self.publish_joystick(self.current_step.joystick_x, self.current_step.joystick_y)
            time_after_stop = t_step_s - self.stop_time_s
            phase = "settle" if time_after_stop < self.settle_time_s else "sample"

        if phase == "sample":
            self.samples.append((self.latest_v, self.latest_w))

        self.raw_writer.writerow(
            {
                "step_index": self.current_step_index,
                "axis": self.current_step.axis,
                "joystick_x": self.current_step.joystick_x,
                "joystick_y": self.current_step.joystick_y,
                "t_step_s": t_step_s,
                "phase": phase,
                "measured_v_mps": self.latest_v,
                "measured_w_radps": self.latest_w,
            }
        )

        if t_step_s >= total_step_time_s:
            self.write_summary_for_current_step()
            self.start_next_step()

    def destroy_node(self):
        try:
            self.publish_joystick(0, 0)
        except Exception:
            pass

        try:
            self.raw_file_handle.close()
        except Exception:
            pass

        try:
            self.summary_file_handle.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelchairCalibrationCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
