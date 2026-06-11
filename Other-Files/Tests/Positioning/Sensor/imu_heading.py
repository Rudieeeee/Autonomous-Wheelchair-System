#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Simulation settings
# -----------------------------

dt = 0.1                 # time step [s]
total_time = 10.0        # simulation duration [s]
v = 0.5                  # forward speed [m/s]
omega = 0.5              # angular velocity [rad/s]

time = np.arange(0.0, total_time + dt, dt)

# -----------------------------
# State variables
# -----------------------------

x_normal = 0.0
y_normal = 0.0
yaw_normal = 0.0

x_mid = 0.0
y_mid = 0.0
yaw_mid = 0.0

normal_x_list = []
normal_y_list = []

mid_x_list = []
mid_y_list = []

diff_list = []

# -----------------------------
# Integration loop
# -----------------------------

for t in time:
    normal_x_list.append(x_normal)
    normal_y_list.append(y_normal)

    mid_x_list.append(x_mid)
    mid_y_list.append(y_mid)

    diff = np.sqrt((x_mid - x_normal)**2 + (y_mid - y_normal)**2)
    diff_list.append(diff)

    distance = v * dt
    yaw_delta = omega * dt

    # -----------------------------
    # Normal Euler integration
    # Uses the old heading
    # -----------------------------
    x_normal += distance * np.cos(yaw_normal)
    y_normal += distance * np.sin(yaw_normal)
    yaw_normal += yaw_delta

    # -----------------------------
    # Midpoint heading integration
    # Uses the heading halfway during the step
    # -----------------------------
    yaw_old = yaw_mid
    yaw_new = yaw_mid + yaw_delta
    yaw_half = yaw_old + yaw_delta / 2.0

    x_mid += distance * np.cos(yaw_half)
    y_mid += distance * np.sin(yaw_half)
    yaw_mid = yaw_new

# Convert to arrays
normal_x = np.array(normal_x_list)
normal_y = np.array(normal_y_list)

mid_x = np.array(mid_x_list)
mid_y = np.array(mid_y_list)

diff = np.array(diff_list)

# -----------------------------
# Plot trajectory comparison
# -----------------------------

plt.figure()
plt.plot(normal_x, normal_y, label="Normal Euler")
plt.plot(mid_x, mid_y, label="Midpoint heading")
plt.xlabel("x position [m]")
plt.ylabel("y position [m]")
plt.title("Normal Euler vs midpoint heading integration")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("normal_vs_midpoint_trajectory.png", dpi=150)
plt.show()

# -----------------------------
# Plot difference over time
# -----------------------------

plt.figure()
plt.plot(time, diff)
plt.xlabel("Time [s]")
plt.ylabel("Position difference [m]")
plt.title("Difference between normal and midpoint integration")
plt.grid(True)
plt.tight_layout()
plt.savefig("normal_vs_midpoint_difference.png", dpi=150)
plt.show()

print("Final normal position:")
print(f"  x = {normal_x[-1]:.3f} m")
print(f"  y = {normal_y[-1]:.3f} m")

print("Final midpoint position:")
print(f"  x = {mid_x[-1]:.3f} m")
print(f"  y = {mid_y[-1]:.3f} m")

print("Final difference:")
print(f"  {diff[-1]:.3f} m")

print()
print("Saved:")
print("  normal_vs_midpoint_trajectory.png")
print("  normal_vs_midpoint_difference.png")