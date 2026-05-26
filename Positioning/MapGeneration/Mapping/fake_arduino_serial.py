#!/usr/bin/env python3

import math
import time
import serial

PORT = "/tmp/ttyFAKE_WRITER"
BAUD = 460800
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ

# Must match your ROS parameters
WHEEL_DIAMETER_M = 0.35
MAGNETS_PER_WHEEL = 12

# Target speed
TARGET_SPEED_KMH = 0.1
TARGET_SPEED_MPS = TARGET_SPEED_KMH / 3.6

WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M
DISTANCE_PER_TICK_M = WHEEL_CIRCUMFERENCE_M / MAGNETS_PER_WHEEL
TICKS_PER_SECOND = TARGET_SPEED_MPS / DISTANCE_PER_TICK_M

ser = serial.Serial(PORT, BAUD, timeout=0.1)

start = time.time()
last_time = start

left_ticks = 0
right_ticks = 0

left_tick_accumulator = 0.0
right_tick_accumulator = 0.0

print(f"Writing fake Arduino DATA to {PORT} at {RATE_HZ} Hz")
print(f"Target speed: {TARGET_SPEED_KMH} km/h")
print(f"Ticks per second: {TICKS_PER_SECOND:.4f}")

while True:
    now = time.time()
    elapsed = now - start
    time_ms = int(elapsed * 1000)

    real_dt = now - last_time
    last_time = now

    # Very slow straight driving
    left_tick_accumulator += TICKS_PER_SECOND * real_dt
    right_tick_accumulator += TICKS_PER_SECOND * real_dt

    if left_tick_accumulator >= 1.0:
        new_ticks = int(left_tick_accumulator)
        left_ticks += new_ticks
        left_tick_accumulator -= new_ticks

    if right_tick_accumulator >= 1.0:
        new_ticks = int(right_tick_accumulator)
        right_ticks += new_ticks
        right_tick_accumulator -= new_ticks

    yaw_deg = 0.0
    gyro_z_radps = 0.0

    gyro_x_radps = 0.0
    gyro_y_radps = 0.0

    accel_x = 0.0
    accel_y = 0.0
    accel_z = 9.81

    pitch_deg = 0.0
    roll_deg = 0.0

    cal_sys = 3
    cal_gyro = 3
    cal_accel = 3
    cal_mag = 0

    line = (
        f"DATA,{time_ms},{left_ticks},{right_ticks},"
        f"{gyro_x_radps:.6f},{gyro_y_radps:.6f},{gyro_z_radps:.6f},"
        f"{accel_x:.6f},{accel_y:.6f},{accel_z:.6f},"
        f"{yaw_deg:.3f},{pitch_deg:.3f},{roll_deg:.3f},"
        f"{cal_sys},{cal_gyro},{cal_accel},{cal_mag}\n"
    )

    ser.write(line.encode("utf-8"))
    ser.flush()

    time.sleep(DT)