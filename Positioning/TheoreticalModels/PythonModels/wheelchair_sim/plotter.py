import matplotlib.pyplot as plt


def _has(data, *names):
    return all(name in data and len(data[name]) > 0 for name in names)


def show_all_plots(simulator):
    model = simulator.model
    data = simulator.history.as_arrays()
    name = model.spec.short_name

    if not _has(data, "x", "y"):
        print("No trajectory data to plot.")
        return

    plt.figure(figsize=(8, 6))
    plt.plot(data["x"], data["y"], label="Wheelchair trajectory", linewidth=2)
    plt.plot(data["x"][0], data["y"][0], "go", label="Start")
    plt.plot(data["x"][-1], data["y"][-1], "ro", label="End")
    plt.title(f"{name} - trajectory")
    plt.xlabel("X position [m]")
    plt.ylabel("Y position [m]")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    if _has(data, "time", "left_input", "right_input"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["left_input"], label="v_cmd input [m/s]")
        plt.plot(data["time"], data["right_input"], label="omega_cmd input [rad/s]")
        plt.title(f"{name} - velocity command inputs")
        plt.xlabel("Time [s]")
        plt.ylabel("Command value")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    if _has(data, "time", "v", "omega"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["v"], label="Linear velocity")
        plt.plot(data["time"], data["omega"], label="Angular velocity")
        plt.title(f"{name} - body velocities")
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    if _has(data, "time", "v", "omega_body"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["v"], label="Linear velocity")
        plt.plot(data["time"], data["omega_body"], label="Body yaw rate")
        plt.title(f"{name} - body velocities")
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    if _has(data, "time", "v_cmd", "v"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["v_cmd"], label="Commanded linear velocity")
        plt.plot(data["time"], data["v"], "--", label="Actual linear velocity")
        plt.title(f"{name} - commanded vs actual linear velocity")
        plt.xlabel("Time [s]")
        plt.ylabel("Linear velocity [m/s]")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    if _has(data, "time", "omega_cmd", "omega"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["omega_cmd"], label="Commanded angular velocity")
        plt.plot(data["time"], data["omega"], "--", label="Actual angular velocity")
        plt.title(f"{name} - commanded vs actual angular velocity")
        plt.xlabel("Time [s]")
        plt.ylabel("Angular velocity [rad/s]")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    if _has(data, "time", "v_dot"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["v_dot"], label="Linear acceleration")
        if "omega_dot" in data:
            plt.plot(data["time"], data["omega_dot"], label="Angular acceleration")
        if "omega_body_dot" in data:
            plt.plot(data["time"], data["omega_body_dot"], label="Body angular acceleration")
        plt.title(f"{name} - acceleration histories")
        plt.xlabel("Time [s]")
        plt.ylabel("Acceleration")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    if _has(data, "time", "F_left", "F_right"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["F_left"], label="Left wheel force")
        plt.plot(data["time"], data["F_right"], label="Right wheel force")
        if "F_left_eff" in data:
            plt.plot(data["time"], data["F_left_eff"], "--", label="Effective left force")
        if "F_right_eff" in data:
            plt.plot(data["time"], data["F_right_eff"], "--", label="Effective right force")
        plt.title(f"{name} - wheel forces")
        plt.xlabel("Time [s]")
        plt.ylabel("Force [N]")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    if _has(data, "time", "force_net", "torque_net"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["force_net"], label="Net forward force")
        plt.plot(data["time"], data["torque_net"], label="Net turning/yaw torque")
        if "torque_drive" in data:
            plt.plot(data["time"], data["torque_drive"], label="Drive torque")
        if "F_resist" in data:
            plt.plot(data["time"], data["F_resist"], "--", label="Resistance force")
        if "tau_resist" in data:
            plt.plot(data["time"], data["tau_resist"], "--", label="Resistance torque")
        plt.title(f"{name} - force and torque balance")
        plt.xlabel("Time [s]")
        plt.ylabel("Force / Torque")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    if _has(data, "time", "tau_motor_left", "tau_motor_right"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["tau_motor_left"], label="Left motor torque")
        plt.plot(data["time"], data["tau_motor_right"], label="Right motor torque")
        plt.title(f"{name} - motor torques")
        plt.xlabel("Time [s]")
        plt.ylabel("Torque [N m]")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    if _has(data, "time", "slip_left", "slip_right"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["slip_left"], label="Left slip ratio")
        plt.plot(data["time"], data["slip_right"], label="Right slip ratio")
        plt.title(f"{name} - slip ratios")
        plt.xlabel("Time [s]")
        plt.ylabel("Slip ratio")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    if _has(data, "time", "F_slip_left", "F_slip_right"):
        plt.figure(figsize=(10, 6))
        plt.plot(data["time"], data["F_slip_left"], label="Left desired slip force")
        plt.plot(data["time"], data["F_slip_right"], label="Right desired slip force")
        if "F_traction_left" in data:
            plt.plot(data["time"], data["F_traction_left"], "--", label="Left actual traction")
        if "F_traction_right" in data:
            plt.plot(data["time"], data["F_traction_right"], "--", label="Right actual traction")
        plt.title(f"{name} - slip force vs actual traction")
        plt.xlabel("Time [s]")
        plt.ylabel("Force [N]")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    plt.show()
