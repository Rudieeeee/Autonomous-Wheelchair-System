import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

"""
Level 4 NAV2-style differential-drive wheelchair simulation.

Input interface:
    v_cmd(t), omega_cmd(t)

Extra effects beyond Level 4:
- torque-equivalent wheel drive
- left/right traction loss factors
- Coulomb-like resistance
- additional turn efficiency loss
"""

r = 0.3
b = 0.5
m = 120.0
J = 25.0
simulation_time = 10.0
dt = 0.05
v_max = 1.2
omega_max = 1.5
a_max = 1.0
alpha_max = 2.5
tau_left_max = 45.0
tau_right_max = 45.0
c_v = 18.0
c_omega = 12.0
F_c = 8.0
tau_c = 4.0
mu_left = 0.92
mu_right = 0.88
eta_turn = 0.90
torque_gain = 45.0
x0 = y0 = theta0 = 0.0
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
x_history = [x]; y_history = [y]; theta_history = [theta]
v_history = [v]; omega_history = [omega]
v_cmd_history = [0.0]; omega_cmd_history = [0.0]
tau_left_history = [0.0]; tau_right_history = [0.0]
F_left_eff_history = [0.0]; F_right_eff_history = [0.0]
v_dot_history = [0.0]; omega_dot_history = [0.0]
force_net_history = [0.0]; torque_net_history = [0.0]; F_resist_history = [0.0]; tau_resist_history = [0.0]

for i in range(len(time) - 1):
    t = time[i]
    v_cmd = clamp(v_cmd_function(t), -v_max, v_max)
    omega_cmd = clamp(omega_cmd_function(t), -omega_max, omega_max)
    omega_left_cmd, omega_right_cmd = body_to_wheel_speeds(v_cmd, omega_cmd, r, b)
    tau_left = clamp(torque_gain * omega_left_cmd / 4.0, -tau_left_max, tau_left_max)
    tau_right = clamp(torque_gain * omega_right_cmd / 4.0, -tau_right_max, tau_right_max)

    F_left_eff = mu_left * tau_left / r
    F_right_eff = mu_right * tau_right / r
    F_resist = c_v * v + F_c * smooth_sign(v)
    tau_resist = c_omega * omega + tau_c * smooth_sign(omega)
    force_net = F_left_eff + F_right_eff - F_resist
    torque_drive = eta_turn * (b / 2.0) * (F_right_eff - F_left_eff)
    torque_net = torque_drive - tau_resist

    v_dot = clamp(force_net / m, -a_max, a_max)
    omega_dot = clamp(torque_net / J, -alpha_max, alpha_max)
    v = clamp(v + v_dot * dt, -v_max, v_max)
    omega = clamp(omega + omega_dot * dt, -omega_max, omega_max)

    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt

    x_history.append(x); y_history.append(y); theta_history.append(theta)
    v_history.append(v); omega_history.append(omega)
    v_cmd_history.append(v_cmd); omega_cmd_history.append(omega_cmd)
    tau_left_history.append(tau_left); tau_right_history.append(tau_right)
    F_left_eff_history.append(F_left_eff); F_right_eff_history.append(F_right_eff)
    v_dot_history.append(v_dot); omega_dot_history.append(omega_dot)
    force_net_history.append(force_net); torque_net_history.append(torque_net)
    F_resist_history.append(F_resist); tau_resist_history.append(tau_resist)

for name in list(globals()):
    if name.endswith('_history'):
        globals()[name] = np.array(globals()[name])

plot_results(time, x_history, y_history, theta_history, [
    ("NAV2 Commanded vs Actual Body Velocities", "velocity", [("v_cmd", v_cmd_history, "-"), ("v_actual", v_history, "--"), ("omega_cmd", omega_cmd_history, "-"), ("omega_actual", omega_history, "--")]),
    ("Wheel Torque Commands", "N m", [("tau_left", tau_left_history, "-"), ("tau_right", tau_right_history, "-")]),
    ("Effective Wheel Forces", "N", [("F_left_eff", F_left_eff_history, "-"), ("F_right_eff", F_right_eff_history, "-")]),
    ("Resistance and Net Balance", "force / torque", [("force_net", force_net_history, "-"), ("torque_net", torque_net_history, "-"), ("F_resist", F_resist_history, "--"), ("tau_resist", tau_resist_history, "--")]),
], "Level 4 NAV2")
animate_trajectory(x_history, y_history, theta_history, "Level 4 NAV2")
