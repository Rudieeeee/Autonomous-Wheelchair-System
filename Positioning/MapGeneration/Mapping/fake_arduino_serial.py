#!/usr/bin/env python3

import math
import time
import serial

PORT = "/tmp/ttyFAKE_WRITER"
BAUD = 460800
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ

# Must match your ROS parameters.
WHEEL_DIAMETER_M = 0.35
MAGNETS_PER_WHEEL = 12

# Target speed.
# Use 1.0 or 2.0 km/h for visible odometry during testing.
TARGET_SPEED_KMH = 1.0
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

    left_state = 1
    right_state = 1

    yaw_deg = 0.0
    pitch_deg = 0.0
    roll_deg = 0.0

    # Short format expected by the old sensor node:
    # DATA,time_ms,left_ticks,right_ticks,left_state,right_state,yaw_deg,pitch_deg,roll_deg
    line = (
        f"DATA,{time_ms},{left_ticks},{right_ticks},"
        f"{left_state},{right_state},"
        f"{yaw_deg:.3f},{pitch_deg:.3f},{roll_deg:.3f}\n"
    )

    ser.write(line.encode("utf-8"))
    ser.flush()

    time.sleep(DT)
