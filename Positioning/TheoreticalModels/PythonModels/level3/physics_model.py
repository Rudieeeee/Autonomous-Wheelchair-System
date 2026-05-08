import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


"""
Level 3 differential-drive wheelchair simulation.

This file simulates a wheelchair using:
- differential-drive kinematics
- time-varying wheel angular velocities
- force/torque-based dynamics
- linear and angular damping
- speed saturation
- acceleration limits
- deadband

Model idea:
wheel angular velocities -> wheel traction forces -> body accelerations -> velocities -> pose
"""


# PARAMETERS
# Wheelchair physical parameters
r = 0.3                  # wheel radius [m]
b = 0.5                  # distance between left and right wheels [m]

# Dynamic physical parameters
m = 120.0                # total mass of wheelchair + user [kg]
J = 25.0                 # rotational inertia about vertical axis [kg m^2]

# Force / torque mapping
k_force = 35.0           # traction-force gain from wheel angular velocity [N / (rad/s)]

# Damping / resistance
c_v = 18.0               # linear damping coefficient [N / (m/s)]
c_omega = 12.0           # angular damping coefficient [N m / (rad/s)]

# Simulation parameters
simulation_time = 10.0   # total simulation time [s]
dt = 0.05                # time step [s]

# Saturation limits
v_max = 1.2              # maximum linear velocity [m/s]
omega_max = 1.5          # maximum angular velocity [rad/s]

# Acceleration limits
a_max = 0.8              # maximum linear acceleration [m/s^2]
alpha_max = 2.0          # maximum angular acceleration [rad/s^2]

# Deadband thresholds
omega_left_deadband = 0.05    # [rad/s]
omega_right_deadband = 0.05   # [rad/s]

# Initial state
x0 = 0.0                 # initial x position [m]
y0 = 0.0                 # initial y position [m]
theta0 = 0.0             # initial heading [rad]
v0 = 0.0                 # initial linear velocity [m/s]
omega0 = 0.0             # initial angular velocity [rad/s]


# WHEEL INPUT FUNCTIONS
def omega_left_function(t):
    return 1.6 + 0.5 * np.sin(0.5 * t)


def omega_right_function(t):
    return 1.6 + 0.5 * np.cos(0.5 * t)


# HELPER FUNCTIONS
def apply_deadband(value, threshold):
    if abs(value) < threshold:
        return 0.0
    return value


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def compute_wheel_forces(omega_left, omega_right, k_force):
    """
    Map wheel angular velocities to effective traction forces.
    """
    F_left = k_force * omega_left
    F_right = k_force * omega_right
    return F_left, F_right


def compute_body_accelerations(F_left, F_right, v, omega, b, m, J, c_v, c_omega):
    """
    Compute translational and rotational accelerations from force/torque balance.

    m * v_dot = F_left + F_right - c_v * v
    J * omega_dot = (b/2) * (F_right - F_left) - c_omega * omega
    """
    force_net = F_left + F_right - c_v * v
    torque_net = (b / 2.0) * (F_right - F_left) - c_omega * omega

    v_dot = force_net / m
    omega_dot = torque_net / J
    return v_dot, omega_dot, force_net, torque_net


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

omega_left_history = [apply_deadband(omega_left_function(time[0]), omega_left_deadband)]
omega_right_history = [apply_deadband(omega_right_function(time[0]), omega_right_deadband)]

F_left_history = [0.0]
F_right_history = [0.0]

v_dot_history = [0.0]
omega_dot_history = [0.0]

force_net_history = [0.0]
torque_net_history = [0.0]


# SIMULATION LOOP
for i in range(num_steps - 1):
    t_current = time[i]

    # Evaluate wheel speeds
    omega_left = omega_left_function(t_current)
    omega_right = omega_right_function(t_current)

    # Apply deadband at wheel-input level
    omega_left = apply_deadband(omega_left, omega_left_deadband)
    omega_right = apply_deadband(omega_right, omega_right_deadband)

    # Convert wheel speeds to wheel forces
    F_left, F_right = compute_wheel_forces(omega_left, omega_right, k_force)

    # Compute body accelerations from force/torque balance
    v_dot, omega_dot, force_net, torque_net = compute_body_accelerations(
        F_left, F_right, v, omega, b, m, J, c_v, c_omega
    )

    # Apply acceleration limits
    v_dot = clamp(v_dot, -a_max, a_max)
    omega_dot = clamp(omega_dot, -alpha_max, alpha_max)

    # Update velocities
    v = v + v_dot * dt
    omega = omega + omega_dot * dt

    # Apply velocity saturation
    v = clamp(v, -v_max, v_max)
    omega = clamp(omega, -omega_max, omega_max)

    # Update pose
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
    omega_left_next = apply_deadband(omega_left_function(time[i + 1]), omega_left_deadband)
    omega_right_next = apply_deadband(omega_right_function(time[i + 1]), omega_right_deadband)
    omega_left_history.append(omega_left_next)
    omega_right_history.append(omega_right_next)

    # Store force and acceleration history
    F_left_history.append(F_left)
    F_right_history.append(F_right)

    v_dot_history.append(v_dot)
    omega_dot_history.append(omega_dot)

    force_net_history.append(force_net)
    torque_net_history.append(torque_net)

# Convert to arrays
x_history = np.array(x_history)
y_history = np.array(y_history)
theta_history = np.array(theta_history)

v_history = np.array(v_history)
omega_history = np.array(omega_history)

omega_left_history = np.array(omega_left_history)
omega_right_history = np.array(omega_right_history)

F_left_history = np.array(F_left_history)
F_right_history = np.array(F_right_history)

v_dot_history = np.array(v_dot_history)
omega_dot_history = np.array(omega_dot_history)

force_net_history = np.array(force_net_history)
torque_net_history = np.array(torque_net_history)


# STATIC TRAJECTORY PLOT
plt.figure(figsize=(8, 6))
plt.plot(x_history, y_history, label="Wheelchair trajectory", linewidth=2)
plt.plot(x_history[0], y_history[0], "go", label="Start")
plt.plot(x_history[-1], y_history[-1], "ro", label="End")
plt.title("Level 3 Differential-Drive Wheelchair Trajectory")
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


# WHEEL FORCE PLOT
plt.figure(figsize=(10, 6))
plt.plot(time, F_left_history, label="Left wheel traction force")
plt.plot(time, F_right_history, label="Right wheel traction force")
plt.title("Wheel Forces vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# VELOCITY PLOTS
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


# NET FORCE / TORQUE PLOTS
plt.figure(figsize=(10, 6))
plt.plot(time, force_net_history, label="Net forward force")
plt.plot(time, torque_net_history, label="Net turning torque")
plt.title("Net Force and Torque vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Force / Torque")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


# ANIMATION SETUP
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_title("Level 3 Differential-Drive Wheelchair Animation")
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