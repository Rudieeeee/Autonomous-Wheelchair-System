import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

"""
Advanced NAV2-style differential-drive wheelchair simulation.

Input interface:
    v_cmd(t), omega_cmd(t)

Extra effects:
- body translational and rotational dynamics
- left/right wheel rotational dynamics
- motor torque-speed relationship
- slip-based traction model
- traction limited by friction and motor torque
- nonlinear resistance and caster turning resistance
"""

r = 0.3
b = 0.5
m = 120.0
J_body = 25.0
J_wheel = 2.0
simulation_time = 10.0
dt = 0.01
v_max = 1.5
omega_body_max = 2.0
omega_wheel_max = 15.0
a_max = 2.0
alpha_body_max = 4.0
alpha_wheel_max = 20.0
tau_stall = 25.0
omega_no_load = 12.0
motor_damping = 0.6
mu = 0.85
g = 9.81
slip_gain = 4.0
slip_epsilon = 1e-3
c_v = 18.0
F_c = 10.0
c_v2 = 6.0
c_omega = 12.0
tau_c = 4.0
c_omega2 = 4.0
caster_turn_resistance = 3.0
x0 = y0 = theta0 = 0.0
v0 = 0.0
omega_body0 = 0.0
omega_left0 = 0.0
omega_right0 = 0.0

# NAV2-LIKE INPUT FUNCTIONS
# These are the ONLY high-level inputs for this file.
# v_cmd is the commanded forward velocity [m/s].
# omega_cmd is the commanded yaw rate [rad/s].
def v_cmd_function(t):
    if t < 2.0:
        return 0.2
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


def motor_torque_from_command(u_cmd, omega_wheel):
    speed_factor = max(0.0, 1.0 - abs(omega_wheel) / omega_no_load)
    return u_cmd * tau_stall * speed_factor


def slip_ratio(omega_wheel, v_wheel_side):
    rim_speed = r * omega_wheel
    denom = max(abs(v_wheel_side), abs(rim_speed), slip_epsilon)
    return (rim_speed - v_wheel_side) / denom


def traction_force_from_slip(slip):
    return mu * 0.5 * m * g * np.tanh(slip_gain * slip)


def limited_traction_force(F_slip, tau_motor):
    return clamp(F_slip, -abs(tau_motor) / r, abs(tau_motor) / r)


def body_resistance(v, omega_body):
    F_resist = c_v * v + F_c * smooth_sign(v) + c_v2 * v * abs(v)
    tau_resist = (
        c_omega * omega_body + tau_c * smooth_sign(omega_body)
        + c_omega2 * omega_body * abs(omega_body)
        + caster_turn_resistance * smooth_sign(omega_body) * abs(v)
    )
    return F_resist, tau_resist


time = np.arange(0.0, simulation_time + dt, dt)
x, y, theta = x0, y0, theta0
v = v0
omega_body = omega_body0
omega_left = omega_left0
omega_right = omega_right0

x_history = [x]; y_history = [y]; theta_history = [theta]
v_history = [v]; omega_body_history = [omega_body]
omega_left_history = [omega_left]; omega_right_history = [omega_right]
v_cmd_history = [0.0]; omega_cmd_history = [0.0]
u_left_history = [0.0]; u_right_history = [0.0]
tau_motor_left_history = [0.0]; tau_motor_right_history = [0.0]
slip_left_history = [0.0]; slip_right_history = [0.0]
F_traction_left_history = [0.0]; F_traction_right_history = [0.0]
force_net_history = [0.0]; torque_net_history = [0.0]

for i in range(len(time) - 1):
    t = time[i]
    v_cmd = clamp(v_cmd_function(t), -v_max, v_max)
    omega_cmd = clamp(omega_cmd_function(t), -omega_body_max, omega_body_max)
    omega_left_target, omega_right_target = body_to_wheel_speeds(v_cmd, omega_cmd, r, b)
    u_left = clamp(omega_left_target / omega_no_load, -1.0, 1.0)
    u_right = clamp(omega_right_target / omega_no_load, -1.0, 1.0)

    tau_motor_left = motor_torque_from_command(u_left, omega_left)
    tau_motor_right = motor_torque_from_command(u_right, omega_right)

    v_left_contact = v - (b / 2.0) * omega_body
    v_right_contact = v + (b / 2.0) * omega_body
    s_left = slip_ratio(omega_left, v_left_contact)
    s_right = slip_ratio(omega_right, v_right_contact)

    F_slip_left = traction_force_from_slip(s_left)
    F_slip_right = traction_force_from_slip(s_right)
    F_traction_left = limited_traction_force(F_slip_left, tau_motor_left)
    F_traction_right = limited_traction_force(F_slip_right, tau_motor_right)

    omega_left_dot = clamp((tau_motor_left - r * F_traction_left - motor_damping * omega_left) / J_wheel, -alpha_wheel_max, alpha_wheel_max)
    omega_right_dot = clamp((tau_motor_right - r * F_traction_right - motor_damping * omega_right) / J_wheel, -alpha_wheel_max, alpha_wheel_max)
    omega_left = clamp(omega_left + omega_left_dot * dt, -omega_wheel_max, omega_wheel_max)
    omega_right = clamp(omega_right + omega_right_dot * dt, -omega_wheel_max, omega_wheel_max)

    F_resist, tau_resist = body_resistance(v, omega_body)
    force_net = F_traction_left + F_traction_right - F_resist
    torque_net = (b / 2.0) * (F_traction_right - F_traction_left) - tau_resist
    v_dot = clamp(force_net / m, -a_max, a_max)
    omega_body_dot = clamp(torque_net / J_body, -alpha_body_max, alpha_body_max)
    v = clamp(v + v_dot * dt, -v_max, v_max)
    omega_body = clamp(omega_body + omega_body_dot * dt, -omega_body_max, omega_body_max)

    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega_body * dt

    x_history.append(x); y_history.append(y); theta_history.append(theta)
    v_history.append(v); omega_body_history.append(omega_body)
    omega_left_history.append(omega_left); omega_right_history.append(omega_right)
    v_cmd_history.append(v_cmd); omega_cmd_history.append(omega_cmd)
    u_left_history.append(u_left); u_right_history.append(u_right)
    tau_motor_left_history.append(tau_motor_left); tau_motor_right_history.append(tau_motor_right)
    slip_left_history.append(s_left); slip_right_history.append(s_right)
    F_traction_left_history.append(F_traction_left); F_traction_right_history.append(F_traction_right)
    force_net_history.append(force_net); torque_net_history.append(torque_net)

for name in list(globals()):
    if name.endswith('_history'):
        globals()[name] = np.array(globals()[name])

plot_results(time, x_history, y_history, theta_history, [
    ("NAV2 Commanded vs Actual Body Velocities", "velocity", [("v_cmd", v_cmd_history, "-"), ("v_actual", v_history, "--"), ("omega_cmd", omega_cmd_history, "-"), ("omega_actual", omega_body_history, "--")]),
    ("Motor Commands", "normalized", [("u_left", u_left_history, "-"), ("u_right", u_right_history, "-")]),
    ("Motor Torques", "N m", [("tau_left", tau_motor_left_history, "-"), ("tau_right", tau_motor_right_history, "-")]),
    ("Slip Ratios", "slip", [("slip_left", slip_left_history, "-"), ("slip_right", slip_right_history, "-")]),
    ("Actual Traction Forces", "N", [("F_left", F_traction_left_history, "-"), ("F_right", F_traction_right_history, "-")]),
], "Advanced NAV2")
animate_trajectory(x_history, y_history, theta_history, "Advanced NAV2", interval=20)
