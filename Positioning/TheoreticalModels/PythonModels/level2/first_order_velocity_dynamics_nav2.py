import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

"""
Level 2 NAV2-style differential-drive wheelchair simulation.

Input interface:
    v_cmd(t), omega_cmd(t)

Extra effects compared with pure kinematics:
- first-order velocity response
- speed saturation
- acceleration limits
- deadband
"""

r = 0.1778
b = 0.615
simulation_time = 10.0
dt = 0.05

tau_v = 0.8
tau_omega = 0.5
v_max = 2.78
omega_max = 2.0
a_max = 1.0
alpha_max = 2.0
v_deadband = 0.05
omega_deadband = 0.08

x0 = 0.0
y0 = 0.0
theta0 = 0.0
v0 = 0.0
omega0 = 0.0

# NAV2-LIKE INPUT FUNCTIONS
# These are the ONLY high-level inputs for this file.
# v_cmd is the commanded forward velocity [m/s].
# omega_cmd is the commanded yaw rate [rad/s].
def v_cmd_function(t):
    if t < 2.0:
        return 0.35
    if t < 5.5:
        return 0.45
    if t < 7.5:
        return 0.25
    return 0.0


def omega_cmd_function(t):
    if t < 2.0:
        return 0.0
    if t < 5.5:
        return -0.45  # negative = turn right in the plot convention
    if t < 7.5:
        return 0.45
    return 0.0


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def apply_deadband(value, threshold):
    if abs(value) < threshold:
        return 0.0
    return value


def smooth_sign(value, epsilon=1e-3):
    return value / (abs(value) + epsilon)


def body_to_wheel_speeds(v, omega, r, b):
    omega_left = (v - (b / 2.0) * omega) / r
    omega_right = (v + (b / 2.0) * omega) / r
    return omega_left, omega_right


def plot_results(time, x_history, y_history, theta_history, histories, title):
    plt.figure(figsize=(8, 6))
    plt.plot(x_history, y_history, label="Wheelchair trajectory", linewidth=2)
    plt.plot(x_history[0], y_history[0], "go", label="Start")
    plt.plot(x_history[-1], y_history[-1], "ro", label="End")
    plt.title(title + " Trajectory")
    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    for plot_title, ylabel, series in histories:
        plt.figure(figsize=(10, 6))
        for label, values, style in series:
            plt.plot(time, values, style, label=label)
        plt.title(plot_title)
        plt.xlabel("Time [s]")
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()


def animate_trajectory(x_history, y_history, theta_history, title, interval=30):
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_title(title + " Animation")
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

    trajectory_line, = ax.plot([], [], "b-", linewidth=2, label="Trajectory")
    wheelchair_point, = ax.plot([], [], "ro", markersize=8, label="Wheelchair")
    heading_line, = ax.plot([], [], "r-", linewidth=2, label="Heading")
    ax.legend()
    heading_length = 0.5

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
        fig, animate, init_func=init, frames=len(x_history),
        interval=interval, blit=True, repeat=True
    )
    plt.tight_layout()
    plt.show()


time = np.arange(0.0, simulation_time + dt, dt)
x, y, theta = x0, y0, theta0
v, omega = v0, omega0

x_history = [x]
y_history = [y]
theta_history = [theta]
v_history = [v]
omega_history = [omega]
v_cmd_history = [0.0]
omega_cmd_history = [0.0]
v_dot_history = [0.0]
omega_dot_history = [0.0]
omega_left_history = [0.0]
omega_right_history = [0.0]

for i in range(len(time) - 1):
    t = time[i]
    v_cmd = clamp(v_cmd_function(t), -v_max, v_max)
    omega_cmd = clamp(omega_cmd_function(t), -omega_max, omega_max)
    v_cmd = apply_deadband(v_cmd, v_deadband)
    omega_cmd = apply_deadband(omega_cmd, omega_deadband)

    omega_left, omega_right = body_to_wheel_speeds(v_cmd, omega_cmd, r, b)

    v_dot = clamp((v_cmd - v) / tau_v, -a_max, a_max)
    omega_dot = clamp((omega_cmd - omega) / tau_omega, -alpha_max, alpha_max)

    v = clamp(v + v_dot * dt, -v_max, v_max)
    omega = clamp(omega + omega_dot * dt, -omega_max, omega_max)

    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt

    x_history.append(x); y_history.append(y); theta_history.append(theta)
    v_history.append(v); omega_history.append(omega)
    v_cmd_history.append(v_cmd); omega_cmd_history.append(omega_cmd)
    v_dot_history.append(v_dot); omega_dot_history.append(omega_dot)
    omega_left_history.append(omega_left); omega_right_history.append(omega_right)

x_history = np.array(x_history); y_history = np.array(y_history); theta_history = np.array(theta_history)
v_history = np.array(v_history); omega_history = np.array(omega_history)
v_cmd_history = np.array(v_cmd_history); omega_cmd_history = np.array(omega_cmd_history)
v_dot_history = np.array(v_dot_history); omega_dot_history = np.array(omega_dot_history)
omega_left_history = np.array(omega_left_history); omega_right_history = np.array(omega_right_history)

plot_results(time, x_history, y_history, theta_history, [
    ("NAV2 Commanded vs Actual Linear Velocity", "m/s", [("v_cmd", v_cmd_history, "-"), ("v_actual", v_history, "--")]),
    ("NAV2 Commanded vs Actual Angular Velocity", "rad/s", [("omega_cmd", omega_cmd_history, "-"), ("omega_actual", omega_history, "--")]),
    ("Derived Wheel Speeds", "rad/s", [("omega_left", omega_left_history, "-"), ("omega_right", omega_right_history, "-")]),
    ("Acceleration Histories", "acceleration", [("v_dot", v_dot_history, "-"), ("omega_dot", omega_dot_history, "-")]),
], "Level 2 NAV2")
animate_trajectory(x_history, y_history, theta_history, "Level 2 NAV2")
