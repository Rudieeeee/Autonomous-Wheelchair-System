import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


"""
Level 1 differential-drive wheelchair simulation.

This file simulates a wheelchair using a pure kinematic differential-drive model.
It assumes:
- no slip
- no dynamics
- motion on a flat 2D plane

INPUT: linear velocity v(t) [m/s] and angular velocity omega(t) [rad/s]
OUTPUT: 2D trajectory, velocity plots, derived wheel speeds, and animation
"""


# PARAMETERS
# Simulation parameters
simulation_time = 10.0   # total simulation time [s]
dt = 0.05                # time step [s]

# Initial state
x0 = 0.0                 # initial x position [m]
y0 = 0.0                 # initial y position [m]
theta0 = 0.0             # initial heading [rad]


# INPUT FUNCTIONS
def v_function(t):
    """
    Linear (forward) velocity of the wheelchair as a function of time [m/s].
    Positive = forward, negative = backward.
    """
    return 0.3  # constant forward speed


def omega_function(t):
    """
    Angular velocity of the wheelchair as a function of time [rad/s].
    Positive = counter-clockwise (turn left), negative = clockwise (turn right).
    """
    return 0.3 * np.sin(0.5 * t)  # gentle sinusoidal steering


# SIMULATION SETUP
time = np.arange(0.0, simulation_time + dt, dt)
num_steps = len(time)

# Current state
x = x0
y = y0
theta = theta0

# History storage
x_history     = [x]
y_history     = [y]
theta_history = [theta]

v_history          = []
omega_body_history = []


# SIMULATION LOOP
for i in range(num_steps - 1):
    t_current = time[i]

    # Evaluate body velocities at current time
    v     = v_function(t_current)
    omega = omega_function(t_current)

    # Kinematic update (Euler integration)
    x_dot     = v * np.cos(theta)
    y_dot     = v * np.sin(theta)
    theta_dot = omega

    x     = x     + x_dot     * dt
    y     = y     + y_dot     * dt
    theta = theta + theta_dot * dt

    # Store history
    x_history.append(x)
    y_history.append(y)
    theta_history.append(theta)

    v_history.append(v)
    omega_body_history.append(omega)

# Convert to arrays
x_history           = np.array(x_history)
y_history           = np.array(y_history)
theta_history       = np.array(theta_history)
v_history          = np.array(v_history)
omega_body_history = np.array(omega_body_history)

# Time array aligned with velocity history (one step shorter)
time_v = time[:-1]


# PLOT 1: Trajectory
plt.figure(figsize=(8, 6))
plt.plot(x_history, y_history, label="Wheelchair trajectory", linewidth=2)
plt.plot(x_history[0], y_history[0], "go", markersize=8, label="Start")
plt.plot(x_history[-1], y_history[-1], "ro", markersize=8, label="End")
plt.title("Differential-Drive Wheelchair Simulation")
plt.xlabel("X Position [m]")
plt.ylabel("Y Position [m]")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# PLOT 2: Input velocities (v and omega)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

ax1.plot(time_v, v_history, label="Linear velocity v(t)", color="tab:blue")
ax1.set_ylabel("Linear Velocity [m/s]")
ax1.set_title("Input Velocities vs Time")
ax1.grid(True)
ax1.legend()

ax2.plot(time_v, omega_body_history, label="Angular velocity ω(t)", color="tab:orange")
ax2.set_ylabel("Angular Velocity [rad/s]")
ax2.set_xlabel("Time [s]")
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()


# ANIMATION SETUP
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_title("Differential-Drive Wheelchair Animation")
ax.set_xlabel("X Position [m]")
ax.set_ylabel("Y Position [m]")
ax.grid(True)
ax.set_aspect("equal", adjustable="box")

x_min, x_max = np.min(x_history), np.max(x_history)
y_min, y_max = np.min(y_history), np.max(y_history)
x_margin = max(0.5, 0.1 * (x_max - x_min + 1e-6))
y_margin = max(0.5, 0.1 * (y_max - y_min + 1e-6))
ax.set_xlim(x_min - x_margin, x_max + x_margin)
ax.set_ylim(y_min - y_margin, y_max + y_margin)

trajectory_line,  = ax.plot([], [], "b-", linewidth=2, label="Trajectory")
wheelchair_point, = ax.plot([], [], "ro", markersize=8, label="Wheelchair")
heading_line,     = ax.plot([], [], "r-", linewidth=2, label="Heading")
ax.legend()

heading_length = 0.3


def init():
    trajectory_line.set_data([], [])
    wheelchair_point.set_data([], [])
    heading_line.set_data([], [])
    return trajectory_line, wheelchair_point, heading_line


def animate(i):
    trajectory_line.set_data(x_history[:i + 1], y_history[:i + 1])

    x_cur     = x_history[i]
    y_cur     = y_history[i]
    theta_cur = theta_history[i]

    wheelchair_point.set_data([x_cur], [y_cur])

    x_head = x_cur + heading_length * np.cos(theta_cur)
    y_head = y_cur + heading_length * np.sin(theta_cur)
    heading_line.set_data([x_cur, x_head], [y_cur, y_head])

    return trajectory_line, wheelchair_point, heading_line


ani = animation.FuncAnimation(
    fig,
    animate,
    init_func=init,
    frames=len(x_history),
    interval=30,
    blit=True,
    repeat=True
)

plt.tight_layout()
plt.show()