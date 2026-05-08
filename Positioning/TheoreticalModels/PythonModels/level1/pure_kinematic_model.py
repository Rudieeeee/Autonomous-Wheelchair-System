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
- wheel angular velocities can be constant or time-varying
"""


# PARAMETERS
# Wheelchair physical parameters
r = 0.1778   # wheel radius [m]
b = 0.615   # distance between left and right wheels [m]

# Simulation parameters
simulation_time = 10.0   # total simulation time [s]
dt = 0.05                # time step [s]

# Initial state
x0 = 0.0                 # initial x position [m]
y0 = 0.0                 # initial y position [m]
theta0 = 0.0             # initial heading [rad]


# WHEEL INPUT FUNCTIONS
def omega_left_function(t):
    """
    Left wheel angular velocity as a function of time.
    Change this however you want.
    """
    return 1.6 + 0.5 * np.sin(0.5 * t)


def omega_right_function(t):
    """
    Right wheel angular velocity as a function of time.
    Example: slightly varying right wheel speed.
    """
    return 1.6 + 0.5 * np.cos(0.5 * t)


# MODEL FUNCTIONS
def compute_body_velocities(omega_left, omega_right, r, b):
    """
    Convert wheel angular velocities into body linear and angular velocity.
    """
    v = r * (omega_left + omega_right) / 2.0
    omega = r * (omega_right - omega_left) / b
    return v, omega


# SIMULATION SETUP
time = np.arange(0.0, simulation_time + dt, dt)
num_steps = len(time)

# Current state
x = x0
y = y0
theta = theta0

# History storage
x_history = [x]
y_history = [y]
theta_history = [theta]

omega_left_history = [omega_left_function(time[0])]
omega_right_history = [omega_right_function(time[0])]
v_history = []
omega_history = []


# SIMULATION LOOP
for i in range(num_steps - 1):
    t_current = time[i]

    # Evaluate wheel speeds at current time
    omega_left = omega_left_function(t_current)
    omega_right = omega_right_function(t_current)

    # Compute body velocities
    v, omega = compute_body_velocities(omega_left, omega_right, r, b)

    # Kinematic update
    x_dot = v * np.cos(theta)
    y_dot = v * np.sin(theta)
    theta_dot = omega

    x = x + x_dot * dt
    y = y + y_dot * dt
    theta = theta + theta_dot * dt

    # Store state history
    x_history.append(x)
    y_history.append(y)
    theta_history.append(theta)

    # Store input and velocity history
    omega_left_history.append(omega_left_function(time[i + 1]))
    omega_right_history.append(omega_right_function(time[i + 1]))
    v_history.append(v)
    omega_history.append(omega)

# Convert to arrays
x_history = np.array(x_history)
y_history = np.array(y_history)
theta_history = np.array(theta_history)

omega_left_history = np.array(omega_left_history)
omega_right_history = np.array(omega_right_history)
v_history = np.array(v_history)
omega_history = np.array(omega_history)


# STATIC TRAJECTORY PLOT
plt.figure(figsize=(8, 6))
plt.plot(x_history, y_history, label="Wheelchair trajectory", linewidth=2)
plt.plot(x_history[0], y_history[0], "go", label="Start")
plt.plot(x_history[-1], y_history[-1], "ro", label="End")
plt.title("Differential-Drive Wheelchair Simulation")
plt.xlabel("X Position [m]")
plt.ylabel("Y Position [m]")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# OPTIONAL INPUT PLOTS
plt.figure(figsize=(10, 6))
plt.plot(time, omega_left_history, label="Left wheel angular velocity")
plt.plot(time, omega_right_history, label="Right wheel angular velocity")
plt.title("Wheel Angular Velocities vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [rad/s]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# ANIMATION SETUP
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_title("Differential-Drive Wheelchair Animation")
ax.set_xlabel("X Position [m]")
ax.set_ylabel("Y Position [m]")
ax.grid(True)
ax.set_aspect("equal", adjustable="box")

# Automatic axis limits with margin
x_min, x_max = np.min(x_history), np.max(x_history)
y_min, y_max = np.min(y_history), np.max(y_history)

x_margin = max(0.5, 0.1 * (x_max - x_min + 1e-6))
y_margin = max(0.5, 0.1 * (y_max - y_min + 1e-6))

ax.set_xlim(x_min - x_margin, x_max + x_margin)
ax.set_ylim(y_min - y_margin, y_max + y_margin)

# Plot elements
trajectory_line, = ax.plot([], [], "b-", linewidth=2, label="Trajectory")
wheelchair_point, = ax.plot([], [], "ro", markersize=8, label="Wheelchair")
heading_line, = ax.plot([], [], "r-", linewidth=2, label="Heading")

ax.legend()

# Arrow length for heading visualization
heading_length = 0.5


# ANIMATION FUNCTIONS
def init():
    trajectory_line.set_data([], [])
    wheelchair_point.set_data([], [])
    heading_line.set_data([], [])
    return trajectory_line, wheelchair_point, heading_line


def animate(i):
    # Path so far
    trajectory_line.set_data(x_history[:i + 1], y_history[:i + 1])

    # Current wheelchair position
    x_current = x_history[i]
    y_current = y_history[i]
    theta_current = theta_history[i]

    wheelchair_point.set_data([x_current], [y_current])

    # Heading line
    x_head = x_current + heading_length * np.cos(theta_current)
    y_head = y_current + heading_length * np.sin(theta_current)
    heading_line.set_data([x_current, x_head], [y_current, y_head])

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