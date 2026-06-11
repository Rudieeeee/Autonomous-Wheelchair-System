#!/usr/bin/env python3

import argparse
import csv
import json
import math
from typing import Dict, List, Tuple


def read_rows(path: str) -> List[dict]:
    with open(path, "r", newline="") as handle:
        return list(csv.DictReader(handle))


def safe_float(row: dict, key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default))
    except ValueError:
        return default


def linear_regression(xs: List[float], ys: List[float]) -> Tuple[float, float, float]:
    """
    Fits y = slope*x + intercept and returns slope, intercept, r_squared.
    """
    n = len(xs)
    if n < 2:
        raise ValueError("Need at least two points for a fit")

    mean_x = sum(xs) / n
    mean_y = sum(ys) / n

    sxx = sum((x - mean_x) ** 2 for x in xs)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))

    if abs(sxx) < 1e-12:
        raise ValueError("All command values are the same")

    slope = sxy / sxx
    intercept = mean_y - slope * mean_x

    predictions = [slope * x + intercept for x in xs]
    ss_res = sum((y - y_hat) ** 2 for y, y_hat in zip(ys, predictions))
    ss_tot = sum((y - mean_y) ** 2 for y in ys)

    r_squared = 1.0 if ss_tot < 1e-12 else 1.0 - ss_res / ss_tot
    return slope, intercept, r_squared


def select_axis_data(rows: List[dict], axis: str, min_abs_velocity: float) -> Tuple[List[float], List[float]]:
    command_abs_values = []
    velocity_abs_values = []

    for row in rows:
        if row.get("axis") != axis:
            continue

        joystick_x = safe_float(row, "joystick_x")
        joystick_y = safe_float(row, "joystick_y")

        if axis.startswith("linear"):
            command_abs = abs(joystick_y)
            velocity_abs = abs(safe_float(row, "median_v_mps"))
        else:
            command_abs = abs(joystick_x)
            velocity_abs = abs(safe_float(row, "median_w_radps"))

        if command_abs <= 0.0:
            continue

        if velocity_abs < min_abs_velocity:
            continue

        command_abs_values.append(command_abs)
        velocity_abs_values.append(velocity_abs)

    return command_abs_values, velocity_abs_values


def fit_axis(rows: List[dict], axis: str, min_abs_velocity: float) -> Dict[str, float]:
    commands, velocities = select_axis_data(rows, axis, min_abs_velocity)

    if len(commands) < 2:
        return {
            "valid": False,
            "reason": f"Not enough moving data for {axis}",
            "samples": len(commands),
        }

    slope, intercept, r_squared = linear_regression(commands, velocities)

    if slope <= 0.0:
        return {
            "valid": False,
            "reason": f"Non positive slope for {axis}",
            "samples": len(commands),
            "slope": slope,
            "intercept": intercept,
            "r_squared": r_squared,
        }

    deadband_estimate = max(0.0, -intercept / slope)

    moving_commands = [cmd for cmd, vel in zip(commands, velocities) if vel >= min_abs_velocity]
    min_output_command = min(moving_commands) if moving_commands else min(commands)
    max_output_command = max(commands)

    return {
        "valid": True,
        "model": "velocity_abs = slope * joystick_abs + intercept",
        "slope": slope,
        "intercept": intercept,
        "r_squared": r_squared,
        "deadband_estimate": deadband_estimate,
        "min_output_command": min_output_command,
        "max_output_command": max_output_command,
        "min_measured_velocity_abs": min(velocities),
        "max_measured_velocity_abs": max(velocities),
        "samples": len(commands),
    }


def build_calibration(rows: List[dict], min_linear_velocity: float, min_angular_velocity: float) -> Dict[str, object]:
    return {
        "format": "wheelchair_joystick_calibration_v1",
        "description": "Feedforward inverse uses joystick_abs = (velocity_abs - intercept) / slope.",
        "linear_positive": fit_axis(rows, "linear_positive", min_linear_velocity),
        "linear_negative": fit_axis(rows, "linear_negative", min_linear_velocity),
        "angular_positive": fit_axis(rows, "angular_positive", min_angular_velocity),
        "angular_negative": fit_axis(rows, "angular_negative", min_angular_velocity),
    }


def print_report(calibration: Dict[str, object]):
    for key in ["linear_positive", "linear_negative", "angular_positive", "angular_negative"]:
        data = calibration[key]
        print()
        print(key)
        print("=" * len(key))

        if not data.get("valid", False):
            print(f"invalid: {data.get('reason')}")
            continue

        slope = data["slope"]
        intercept = data["intercept"]
        deadband = data["deadband_estimate"]
        r_squared = data["r_squared"]
        min_output = data["min_output_command"]
        max_output = data["max_output_command"]
        max_speed = data["max_measured_velocity_abs"]

        print(f"velocity_abs = {slope:.6f} * joystick_abs + {intercept:.6f}")
        print(f"estimated deadband command = {deadband:.2f}")
        print(f"minimum moving command = {min_output:.1f}")
        print(f"maximum tested command = {max_output:.1f}")
        print(f"maximum measured speed = {max_speed:.4f}")
        print(f"r_squared = {r_squared:.4f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv", help="Path to wheelchair_calibration_step_summary.csv")
    parser.add_argument("output_json", help="Path to write joystick_calibration.json")
    parser.add_argument("--min_linear_velocity", type=float, default=0.02)
    parser.add_argument("--min_angular_velocity", type=float, default=0.02)
    args = parser.parse_args()

    rows = read_rows(args.input_csv)
    calibration = build_calibration(rows, args.min_linear_velocity, args.min_angular_velocity)

    with open(args.output_json, "w") as handle:
        json.dump(calibration, handle, indent=2)

    print_report(calibration)
    print()
    print(f"Wrote {args.output_json}")


if __name__ == "__main__":
    main()
