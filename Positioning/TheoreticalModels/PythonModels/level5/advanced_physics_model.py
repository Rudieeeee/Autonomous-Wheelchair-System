import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


"""
Advanced differential-drive wheelchair simulation.

Included physics:
- differential-drive planar kinematics
- body translational and rotational dynamics
- left/right wheel rotational dynamics
- motor torque-speed relationship
- slip-based traction model
- traction limited by BOTH friction and motor torque
- nonlinear resistance
- simple extra turning resistance
"""


# Parameters
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

u_left_deadband = 0.02
u_right_deadband = 0.02

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

x0 = 0.0
y0 = 0.0
theta0 = 0.0
v0 = 0.0
omega_body0 = 0.0
omega_left0 = 0.0
omega_right0 = 0.0


# Input functions
def u_left_function(t):
    return 0.7 + 0.15 * np.sin(0.6 * t)


def u_right_function(t):
    return 0


# Helper functions
def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def apply_deadband(value, threshold):
    if abs(value) < threshold:
        return 0.0
    return value


def smooth_sign(value, epsilon=1e-3):
    return value / (abs(value) + epsilon)


def motor_torque_from_command(u_cmd, omega_wheel, tau_stall, omega_no_load):
    speed_factor = max(0.0, 1.0 - abs(omega_wheel) / omega_no_load)
    return u_cmd * tau_stall * speed_factor


def wheel_normal_forces(m, g):
    N_left = 0.5 * m * g
    N_right = 0.5 * m * g
    return N_left, N_right


def slip_ratio(omega_wheel, v_wheel_side, r, epsilon=1e-3):
    rim_speed = r * omega_wheel
    denom = max(abs(v_wheel_side), abs(rim_speed), epsilon)
    return (rim_speed - v_wheel_side) / denom


def traction_force_from_slip(slip, mu, N, slip_gain):
    return mu * N * np.tanh(slip_gain * slip)


def limited_traction_force(F_slip, tau_motor, r):
    """
    Limit traction force by the motor torque capability:
    |F| <= |tau_motor| / r
    """
    F_torque_max = abs(tau_motor) / r
    return clamp(F_slip, -F_torque_max, F_torque_max)


def body_resistance(v, omega_body, c_v, F_c, c_v2, c_omega, tau_c, c_omega2, caster_turn_resistance):
    F_resist = c_v * v + F_c * smooth_sign(v) + c_v2 * v * abs(v)

    tau_resist = (
        c_omega * omega_body
        + tau_c * smooth_sign(omega_body)
        + c_omega2 * omega_body * abs(omega_body)
        + caster_turn_resistance * smooth_sign(omega_body) * abs(v)
    )

    return F_resist, tau_resist


# Simulation setup
time = np.arange(0.0, simulation_time + dt, dt)
num_steps = len(time)

x = x0
y = y0
theta = theta0
v = v0
omega_body = omega_body0
omega_left = omega_left0
omega_right = omega_right0

x_history = [x]
y_history = [y]
theta_history = [theta]

v_history = [v]
omega_body_history = [omega_body]
omega_left_history = [omega_left]
omega_right_history = [omega_right]

u_left_history = [0.0]
u_right_history = [0.0]

tau_motor_left_history = [0.0]
tau_motor_right_history = [0.0]

slip_left_history = [0.0]
slip_right_history = [0.0]

F_slip_left_history = [0.0]
F_slip_right_history = [0.0]
F_traction_left_history = [0.0]
F_traction_right_history = [0.0]

v_dot_history = [0.0]
omega_body_dot_history = [0.0]
omega_left_dot_history = [0.0]
omega_right_dot_history = [0.0]

F_resist_history = [0.0]
tau_resist_history = [0.0]
force_net_history = [0.0]
torque_net_history = [0.0]


# Simulation loop
for i in range(num_steps - 1):
    t_current = time[i]

    u_left = clamp(apply_deadband(u_left_function(t_current), u_left_deadband), -1.0, 1.0)
    u_right = clamp(apply_deadband(u_right_function(t_current), u_right_deadband), -1.0, 1.0)

    tau_motor_left = motor_torque_from_command(u_left, omega_left, tau_stall, omega_no_load)
    tau_motor_right = motor_torque_from_command(u_right, omega_right, tau_stall, omega_no_load)

    v_left_contact = v - (b / 2.0) * omega_body
    v_right_contact = v + (b / 2.0) * omega_body

    s_left = slip_ratio(omega_left, v_left_contact, r, slip_epsilon)
    s_right = slip_ratio(omega_right, v_right_contact, r, slip_epsilon)

    N_left, N_right = wheel_normal_forces(m, g)

    F_slip_left = traction_force_from_slip(s_left, mu, N_left, slip_gain)
    F_slip_right = traction_force_from_slip(s_right, mu, N_right, slip_gain)

    F_traction_left = limited_traction_force(F_slip_left, tau_motor_left, r)
    F_traction_right = limited_traction_force(F_slip_right, tau_motor_right, r)

    omega_left_dot = (tau_motor_left - r * F_traction_left - motor_damping * omega_left) / J_wheel
    omega_right_dot = (tau_motor_right - r * F_traction_right - motor_damping * omega_right) / J_wheel

    omega_left_dot = clamp(omega_left_dot, -alpha_wheel_max, alpha_wheel_max)
    omega_right_dot = clamp(omega_right_dot, -alpha_wheel_max, alpha_wheel_max)

    omega_left = omega_left + omega_left_dot * dt
    omega_right = omega_right + omega_right_dot * dt

    omega_left = clamp(omega_left, -omega_wheel_max, omega_wheel_max)
    omega_right = clamp(omega_right, -omega_wheel_max, omega_wheel_max)

    F_resist, tau_resist = body_resistance(
        v, omega_body, c_v, F_c, c_v2, c_omega, tau_c, c_omega2, caster_turn_resistance
    )

    force_net = F_traction_left + F_traction_right - F_resist
    torque_net = (b / 2.0) * (F_traction_right - F_traction_left) - tau_resist

    v_dot = force_net / m
    omega_body_dot = torque_net / J_body

    v_dot = clamp(v_dot, -a_max, a_max)
    omega_body_dot = clamp(omega_body_dot, -alpha_body_max, alpha_body_max)

    v = v + v_dot * dt
    omega_body = omega_body + omega_body_dot * dt

    v = clamp(v, -v_max, v_max)
    omega_body = clamp(omega_body, -omega_body_max, omega_body_max)

    x_dot = v * np.cos(theta)
    y_dot = v * np.sin(theta)
    theta_dot = omega_body

    x = x + x_dot * dt
    y = y + y_dot * dt
    theta = theta + theta_dot * dt

    x_history.append(x)
    y_history.append(y)
    theta_history.append(theta)

    v_history.append(v)
    omega_body_history.append(omega_body)
    omega_left_history.append(omega_left)
    omega_right_history.append(omega_right)

    u_left_history.append(u_left)
    u_right_history.append(u_right)

    tau_motor_left_history.append(tau_motor_left)
    tau_motor_right_history.append(tau_motor_right)

    slip_left_history.append(s_left)
    slip_right_history.append(s_right)

    F_slip_left_history.append(F_slip_left)
    F_slip_right_history.append(F_slip_right)
    F_traction_left_history.append(F_traction_left)
    F_traction_right_history.append(F_traction_right)

    v_dot_history.append(v_dot)
    omega_body_dot_history.append(omega_body_dot)
    omega_left_dot_history.append(omega_left_dot)
    omega_right_dot_history.append(omega_right_dot)

    F_resist_history.append(F_resist)
    tau_resist_history.append(tau_resist)
    force_net_history.append(force_net)
    torque_net_history.append(torque_net)


# Convert to arrays
x_history = np.array(x_history)
y_history = np.array(y_history)
theta_history = np.array(theta_history)

v_history = np.array(v_history)
omega_body_history = np.array(omega_body_history)
omega_left_history = np.array(omega_left_history)
omega_right_history = np.array(omega_right_history)

u_left_history = np.array(u_left_history)
u_right_history = np.array(u_right_history)

tau_motor_left_history = np.array(tau_motor_left_history)
tau_motor_right_history = np.array(tau_motor_right_history)

slip_left_history = np.array(slip_left_history)
slip_right_history = np.array(slip_right_history)

F_slip_left_history = np.array(F_slip_left_history)
F_slip_right_history = np.array(F_slip_right_history)
F_traction_left_history = np.array(F_traction_left_history)
F_traction_right_history = np.array(F_traction_right_history)

v_dot_history = np.array(v_dot_history)
omega_body_dot_history = np.array(omega_body_dot_history)
omega_left_dot_history = np.array(omega_left_dot_history)
omega_right_dot_history = np.array(omega_right_dot_history)

F_resist_history = np.array(F_resist_history)
tau_resist_history = np.array(tau_resist_history)
force_net_history = np.array(force_net_history)
torque_net_history = np.array(torque_net_history)


# Plots
plt.figure(figsize=(8, 6))
plt.plot(x_history, y_history, label="Wheelchair trajectory", linewidth=2)
plt.plot(x_history[0], y_history[0], "go", label="Start")
plt.plot(x_history[-1], y_history[-1], "ro", label="End")
plt.title("Advanced Wheelchair Trajectory")
plt.xlabel("X Position [m]")
plt.ylabel("Y Position [m]")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time, tau_motor_left_history, label="Left motor torque")
plt.plot(time, tau_motor_right_history, label="Right motor torque")
plt.title("Motor Torques")
plt.xlabel("Time [s]")
plt.ylabel("Torque [N m]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time, omega_left_history, label="Left wheel speed")
plt.plot(time, omega_right_history, label="Right wheel speed")
plt.plot(time, omega_body_history, label="Body yaw rate")
plt.title("Wheel and Body Angular Velocities")
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [rad/s]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time, slip_left_history, label="Left slip ratio")
plt.plot(time, slip_right_history, label="Right slip ratio")
plt.title("Slip Ratios")
plt.xlabel("Time [s]")
plt.ylabel("Slip ratio")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time, F_slip_left_history, label="Left desired slip force")
plt.plot(time, F_slip_right_history, label="Right desired slip force")
plt.plot(time, F_traction_left_history, "--", label="Left actual traction force")
plt.plot(time, F_traction_right_history, "--", label="Right actual traction force")
plt.title("Slip Force vs Actual Traction Force")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time, v_history, label="Linear velocity")
plt.plot(time, omega_body_history, label="Angular velocity")
plt.title("Body Velocities")
plt.xlabel("Time [s]")
plt.ylabel("Velocity")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time, force_net_history, label="Net forward force")
plt.plot(time, torque_net_history, label="Net yaw torque")
plt.title("Net Force and Torque")
plt.xlabel("Time [s]")
plt.ylabel("Force / Torque")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# Animation
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_title("Advanced Wheelchair Animation")
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
    fig,
    animate,
    init_func=init,
    frames=len(x_history),
    interval=20,
    blit=True,
    repeat=True
)

plt.tight_layout()
plt.show()