import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


"""
Level 2 differential-drive wheelchair simulation.

This file simulates a wheelchair using:
- differential-drive kinematics
- time-varying wheel angular velocities
- first-order velocity dynamics
- speed saturation
- acceleration limits
"""


# PARAMETERS
# Wheelchair physical parameters
r = 0.1778   # wheel radius [m]
b = 0.615   # distance between left and right wheels [m]

# Simulation parameters
simulation_time = 10.0   # total simulation time [s]
dt = 0.05                # time step [s]

# Dynamic parameters
tau_v = 0.8              # linear velocity time constant [s]
tau_omega = 0.5          # angular velocity time constant [s]

# Saturation limits
v_max = 2.78             # maximum linear velocity [m/s]
omega_max = 2.0         # maximum angular velocity [rad/s]

# Acceleration limits
a_max = 1             # maximum linear acceleration [m/s^2]
alpha_max = 2.0          # maximum angular acceleration [rad/s^2]

v_deadband = 0.05        # [m/s]
omega_deadband = 0.08    # [rad/s]

# Initial state
x0 = 0.0                 # initial x position [m]
y0 = 0.0                 # initial y position [m]
theta0 = 0.0             # initial heading [rad]
v0 = 0.0                 # initial actual linear velocity [m/s]
omega0 = 0.0             # initial actual angular velocity [rad/s]


# WHEEL INPUT FUNCTIONS
def omega_left_function(t):
    return 1.6 + 0.5 * np.sin(0.5 * t)


def omega_right_function(t):
    return 1.6 + 0.5 * np.cos(0.5 * t)

def apply_deadband(value, threshold):
    if abs(value) < threshold:
        return 0.0
    return value


# MODEL FUNCTIONS
def compute_commanded_body_velocities(omega_left, omega_right, r, b):
    v_cmd = r * (omega_left + omega_right) / 2.0
    omega_cmd = r * (omega_right - omega_left) / b
    return v_cmd, omega_cmd


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


# SIMULATION SETUP
time = np.arange(0.0, simulation_time + dt, dt)
num_steps = len(time)

# Current state
x = x0
y = y0
theta = theta0
v = v0
omega = omega0

# History storage
x_history = [x]
y_history = [y]
theta_history = [theta]

v_history = [v]
omega_history = [omega]

omega_left_history = [omega_left_function(time[0])]
omega_right_history = [omega_right_function(time[0])]

v_cmd_history = [0.0]
omega_cmd_history = [0.0]

v_dot_history = [0.0]
omega_dot_history = [0.0]


# SIMULATION LOOP
for i in range(num_steps - 1):
    t_current = time[i]

    # Evaluate wheel speeds at current time
    omega_left = omega_left_function(t_current)
    omega_right = omega_right_function(t_current)

    # Compute commanded body velocities
    v_cmd, omega_cmd = compute_commanded_body_velocities(omega_left, omega_right, r, b)

    # Apply speed saturation to commanded velocities
    v_cmd = clamp(v_cmd, -v_max, v_max)
    omega_cmd = clamp(omega_cmd, -omega_max, omega_max)

    v_cmd = apply_deadband(v_cmd, v_deadband)
    omega_cmd = apply_deadband(omega_cmd, omega_deadband)

    # First-order velocity dynamics
    v_dot = (v_cmd - v) / tau_v
    omega_dot = (omega_cmd - omega) / tau_omega

    # Apply acceleration limits
    v_dot = clamp(v_dot, -a_max, a_max)
    omega_dot = clamp(omega_dot, -alpha_max, alpha_max)

    # Update actual velocities
    v = v + v_dot * dt
    omega = omega + omega_dot * dt

    # Optional: also clamp actual velocities for extra safety
    v = clamp(v, -v_max, v_max)
    omega = clamp(omega, -omega_max, omega_max)

    # Update pose using actual velocities
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

    v_history.append(v)
    omega_history.append(omega)

    # Store wheel input history
    omega_left_history.append(omega_left_function(time[i + 1]))
    omega_right_history.append(omega_right_function(time[i + 1]))

    # Store commanded velocities and accelerations
    v_cmd_history.append(v_cmd)
    omega_cmd_history.append(omega_cmd)

    v_dot_history.append(v_dot)
    omega_dot_history.append(omega_dot)

# Convert to arrays
x_history = np.array(x_history)
y_history = np.array(y_history)
theta_history = np.array(theta_history)

v_history = np.array(v_history)
omega_history = np.array(omega_history)

omega_left_history = np.array(omega_left_history)
omega_right_history = np.array(omega_right_history)

v_cmd_history = np.array(v_cmd_history)
omega_cmd_history = np.array(omega_cmd_history)

v_dot_history = np.array(v_dot_history)
omega_dot_history = np.array(omega_dot_history)


# STATIC TRAJECTORY PLOT
plt.figure(figsize=(8, 6))
plt.plot(x_history, y_history, label="Wheelchair trajectory", linewidth=2)
plt.plot(x_history[0], y_history[0], "go", label="Start")
plt.plot(x_history[-1], y_history[-1], "ro", label="End")
plt.title("Differential-Drive Wheelchair Trajectory")
plt.xlabel("X Position [m]")
plt.ylabel("Y Position [m]")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# WHEEL INPUT PLOT
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


# COMMAND VS ACTUAL LINEAR VELOCITY
plt.figure(figsize=(10, 6))
plt.plot(time, v_cmd_history, label="Commanded linear velocity")
plt.plot(time, v_history, label="Actual linear velocity", linestyle="--")
plt.title("Commanded vs Actual Linear Velocity")
plt.xlabel("Time [s]")
plt.ylabel("Linear velocity [m/s]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# COMMAND VS ACTUAL ANGULAR VELOCITY
plt.figure(figsize=(10, 6))
plt.plot(time, omega_cmd_history, label="Commanded angular velocity")
plt.plot(time, omega_history, label="Actual angular velocity", linestyle="--")
plt.title("Commanded vs Actual Angular Velocity")
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [rad/s]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# ACCELERATION PLOTS
plt.figure(figsize=(10, 6))
plt.plot(time, v_dot_history, label="Linear acceleration")
plt.plot(time, omega_dot_history, label="Angular acceleration")
plt.title("Acceleration Histories")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration")
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
heading_length = 0.5


# ANIMATION FUNCTIONS
def init():
    trajectory_line.set_data([], [])
    wheelchair_point.set_data([], [])
    heading_line.set_data([], [])
    return trajectory_line, wheelchair_point, heading_line


def animate(i):
    trajectory_line.set_data(x_history[:i + 1], y_history[:i + 1])

    x_current = x_history[i]
    y_current = y_history[i]
    theta_current = theta_history[i]

    wheelchair_point.set_data([x_current], [y_current])

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