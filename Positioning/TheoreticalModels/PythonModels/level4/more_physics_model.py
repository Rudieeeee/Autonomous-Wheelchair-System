import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


"""
Level 4 differential-drive wheelchair simulation.

This file simulates a wheelchair using:
- differential-drive kinematics
- wheel torque inputs
- force/torque-based body dynamics
- linear damping + Coulomb-like resistance
- simple slip / traction-loss model
- speed saturation
- acceleration limits

Model chain:
torque inputs -> wheel forces -> effective traction forces -> body accelerations -> velocities -> pose
"""


# Parameters
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

tau_left_deadband = 0.5
tau_right_deadband = 0.5

x0 = 0.0
y0 = 0.0
theta0 = 0.0
v0 = 0.0
omega0 = 0.0


# Input functions
def tau_left_function(t):
    return 28.0 + 8.0 * np.sin(0.5 * t)


def tau_right_function(t):
    return 28.0 + 8.0 * np.cos(0.5 * t)


# Helper functions
def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def apply_deadband(value, threshold):
    if abs(value) < threshold:
        return 0.0
    return value


def smooth_sign(value, epsilon=1e-3):
    return value / (abs(value) + epsilon)


def compute_wheel_forces_from_torque(tau_left, tau_right, r):
    F_left = tau_left / r
    F_right = tau_right / r
    return F_left, F_right


def apply_slip_model(F_left, F_right, mu_left, mu_right):
    F_left_eff = mu_left * F_left
    F_right_eff = mu_right * F_right
    return F_left_eff, F_right_eff


def compute_resistance(v, omega, c_v, c_omega, F_c, tau_c):
    F_resist = c_v * v + F_c * smooth_sign(v)
    tau_resist = c_omega * omega + tau_c * smooth_sign(omega)
    return F_resist, tau_resist


def compute_body_accelerations(F_left_eff, F_right_eff, v, omega, b, m, J, c_v, c_omega, F_c, tau_c, eta_turn):
    F_resist, tau_resist = compute_resistance(v, omega, c_v, c_omega, F_c, tau_c)

    force_net = F_left_eff + F_right_eff - F_resist
    torque_drive = eta_turn * (b / 2.0) * (F_right_eff - F_left_eff)
    torque_net = torque_drive - tau_resist

    v_dot = force_net / m
    omega_dot = torque_net / J

    return v_dot, omega_dot, force_net, torque_drive, torque_net, F_resist, tau_resist


# Simulation setup
time = np.arange(0.0, simulation_time + dt, dt)
num_steps = len(time)

x = x0
y = y0
theta = theta0
v = v0
omega = omega0

x_history = [x]
y_history = [y]
theta_history = [theta]

v_history = [v]
omega_history = [omega]

tau_left_history = [0.0]
tau_right_history = [0.0]

F_left_history = [0.0]
F_right_history = [0.0]
F_left_eff_history = [0.0]
F_right_eff_history = [0.0]

v_dot_history = [0.0]
omega_dot_history = [0.0]

force_net_history = [0.0]
torque_drive_history = [0.0]
torque_net_history = [0.0]

F_resist_history = [0.0]
tau_resist_history = [0.0]


# Simulation loop
for i in range(num_steps - 1):
    t_current = time[i]

    tau_left = tau_left_function(t_current)
    tau_right = tau_right_function(t_current)

    tau_left = clamp(tau_left, -tau_left_max, tau_left_max)
    tau_right = clamp(tau_right, -tau_right_max, tau_right_max)

    tau_left = apply_deadband(tau_left, tau_left_deadband)
    tau_right = apply_deadband(tau_right, tau_right_deadband)

    F_left, F_right = compute_wheel_forces_from_torque(tau_left, tau_right, r)
    F_left_eff, F_right_eff = apply_slip_model(F_left, F_right, mu_left, mu_right)

    v_dot, omega_dot, force_net, torque_drive, torque_net, F_resist, tau_resist = compute_body_accelerations(
        F_left_eff, F_right_eff, v, omega, b, m, J, c_v, c_omega, F_c, tau_c, eta_turn
    )

    v_dot = clamp(v_dot, -a_max, a_max)
    omega_dot = clamp(omega_dot, -alpha_max, alpha_max)

    v = v + v_dot * dt
    omega = omega + omega_dot * dt

    v = clamp(v, -v_max, v_max)
    omega = clamp(omega, -omega_max, omega_max)

    x_dot = v * np.cos(theta)
    y_dot = v * np.sin(theta)
    theta_dot = omega

    x = x + x_dot * dt
    y = y + y_dot * dt
    theta = theta + theta_dot * dt

    x_history.append(x)
    y_history.append(y)
    theta_history.append(theta)

    v_history.append(v)
    omega_history.append(omega)

    tau_left_history.append(tau_left)
    tau_right_history.append(tau_right)

    F_left_history.append(F_left)
    F_right_history.append(F_right)
    F_left_eff_history.append(F_left_eff)
    F_right_eff_history.append(F_right_eff)

    v_dot_history.append(v_dot)
    omega_dot_history.append(omega_dot)

    force_net_history.append(force_net)
    torque_drive_history.append(torque_drive)
    torque_net_history.append(torque_net)

    F_resist_history.append(F_resist)
    tau_resist_history.append(tau_resist)


# Convert to arrays
x_history = np.array(x_history)
y_history = np.array(y_history)
theta_history = np.array(theta_history)

v_history = np.array(v_history)
omega_history = np.array(omega_history)

tau_left_history = np.array(tau_left_history)
tau_right_history = np.array(tau_right_history)

F_left_history = np.array(F_left_history)
F_right_history = np.array(F_right_history)
F_left_eff_history = np.array(F_left_eff_history)
F_right_eff_history = np.array(F_right_eff_history)

v_dot_history = np.array(v_dot_history)
omega_dot_history = np.array(omega_dot_history)

force_net_history = np.array(force_net_history)
torque_drive_history = np.array(torque_drive_history)
torque_net_history = np.array(torque_net_history)

F_resist_history = np.array(F_resist_history)
tau_resist_history = np.array(tau_resist_history)


# Static trajectory plot
plt.figure(figsize=(8, 6))
plt.plot(x_history, y_history, label="Wheelchair trajectory", linewidth=2)
plt.plot(x_history[0], y_history[0], "go", label="Start")
plt.plot(x_history[-1], y_history[-1], "ro", label="End")
plt.title("Level 4 Differential-Drive Wheelchair Trajectory")
plt.xlabel("X Position [m]")
plt.ylabel("Y Position [m]")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# Torque input plot
plt.figure(figsize=(10, 6))
plt.plot(time, tau_left_history, label="Left wheel torque")
plt.plot(time, tau_right_history, label="Right wheel torque")
plt.title("Wheel Torques vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Torque [N m]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# Wheel force plot
plt.figure(figsize=(10, 6))
plt.plot(time, F_left_history, label="Left wheel force")
plt.plot(time, F_right_history, label="Right wheel force")
plt.plot(time, F_left_eff_history, "--", label="Effective left wheel force")
plt.plot(time, F_right_eff_history, "--", label="Effective right wheel force")
plt.title("Wheel Forces vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# Velocity plot
plt.figure(figsize=(10, 6))
plt.plot(time, v_history, label="Linear velocity")
plt.plot(time, omega_history, label="Angular velocity")
plt.title("Body Velocities vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Velocity")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# Acceleration plot
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


# Force and torque balance plot
plt.figure(figsize=(10, 6))
plt.plot(time, force_net_history, label="Net forward force")
plt.plot(time, torque_drive_history, label="Drive torque")
plt.plot(time, torque_net_history, label="Net turning torque")
plt.plot(time, F_resist_history, "--", label="Resistance force")
plt.plot(time, tau_resist_history, "--", label="Resistance torque")
plt.title("Force and Torque Balance")
plt.xlabel("Time [s]")
plt.ylabel("Force / Torque")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# Animation setup
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_title("Level 4 Differential-Drive Wheelchair Animation")
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


# Animation functions
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