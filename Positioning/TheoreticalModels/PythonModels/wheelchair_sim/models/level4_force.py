import numpy as np

from .base import BaseWheelchairModel, ModelSpec, clamp, apply_deadband


class Level4ForceModel(BaseWheelchairModel):
    spec = ModelSpec(
        key="level4_force",
        name="Level 4 - Force/torque body dynamics from velocity commands",
        short_name="Level 4 force",
        description="Linear/angular velocity commands are converted to equivalent wheel speeds, then mapped to traction force for body dynamics.",
        input_type="body_velocity",
        input_label="Linear/angular velocity command",
        input_unit="m/s, rad/s",
    )
    dt = 0.05
    simulation_time = 20.0

    def __init__(self):
        self.r = 0.1778
        self.b = 0.615
        self.m = 233.5
        self.J = 30.0
        self.k_force = 35.0
        self.c_v = 18.0
        self.c_omega = 12.0
        self.v_max = 1.2
        self.omega_max = 1.5
        self.a_max = 0.8
        self.alpha_max = 2.0
        self.omega_left_deadband = 0.05
        self.omega_right_deadband = 0.05

    def initial_state(self):
        return {"x": 0.0, "y": 0.0, "theta": 0.0, "v": 0.0, "omega": 0.0}

    def step(self, state, v_cmd, omega_cmd, dt):
        v_cmd = clamp(v_cmd, -self.v_max, self.v_max)
        omega_cmd = clamp(omega_cmd, -self.omega_max, self.omega_max)

        omega_left = (v_cmd - (self.b / 2.0) * omega_cmd) / self.r
        omega_right = (v_cmd + (self.b / 2.0) * omega_cmd) / self.r

        omega_left = apply_deadband(omega_left, self.omega_left_deadband)
        omega_right = apply_deadband(omega_right, self.omega_right_deadband)

        F_left = self.k_force * omega_left
        F_right = self.k_force * omega_right

        force_net = F_left + F_right - self.c_v * state["v"]
        torque_net = (self.b / 2.0) * (F_right - F_left) - self.c_omega * state["omega"]

        v_dot = clamp(force_net / self.m, -self.a_max, self.a_max)
        omega_dot = clamp(torque_net / self.J, -self.alpha_max, self.alpha_max)

        v = clamp(state["v"] + v_dot * dt, -self.v_max, self.v_max)
        omega = clamp(state["omega"] + omega_dot * dt, -self.omega_max, self.omega_max)

        x = state["x"] + v * np.cos(state["theta"]) * dt
        y = state["y"] + v * np.sin(state["theta"]) * dt
        theta = state["theta"] + omega * dt

        new_state = {"x": x, "y": y, "theta": theta, "v": v, "omega": omega}
        info = {
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            "omega_left": omega_left,
            "omega_right": omega_right,
            "F_left": F_left,
            "F_right": F_right,
            "v": v,
            "omega": omega,
            "v_dot": v_dot,
            "omega_dot": omega_dot,
            "force_net": force_net,
            "torque_net": torque_net,
        }
        return new_state, info

    def history_fields(self):
        return [
            "time", "x", "y", "theta",
            "left_input", "right_input", "v_cmd", "omega_cmd", "omega_left", "omega_right",
            "F_left", "F_right", "v", "omega", "v_dot", "omega_dot", "force_net", "torque_net",
        ]
