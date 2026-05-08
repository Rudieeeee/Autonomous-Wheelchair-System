import numpy as np

from .base import BaseWheelchairModel, ModelSpec, clamp, apply_deadband, smooth_sign


class Level4TorqueSlipModel(BaseWheelchairModel):
    spec = ModelSpec(
        key="level4_torque_slip",
        name="Level 4+ - Torque, simple slip, and resistance",
        short_name="Level 4+ torque/slip",
        description="Linear/angular velocity commands are converted to equivalent wheel torques, then reduced by slip/traction and resistance effects.",
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
        self.v_max = 1.2
        self.omega_max = 1.5
        self.a_max = 1.0
        self.alpha_max = 2.5
        self.tau_left_max = 45.0
        self.tau_right_max = 45.0
        self.torque_per_wheel_rad_s = 6.0
        self.c_v = 18.0
        self.c_omega = 12.0
        self.F_c = 8.0
        self.tau_c = 4.0
        self.mu_left = 0.90
        self.mu_right = 0.90
        self.eta_turn = 0.90
        self.tau_left_deadband = 0.5
        self.tau_right_deadband = 0.5

    def initial_state(self):
        return {"x": 0.0, "y": 0.0, "theta": 0.0, "v": 0.0, "omega": 0.0}

    def step(self, state, v_cmd, omega_cmd, dt):
        v_cmd = clamp(v_cmd, -self.v_max, self.v_max)
        omega_cmd = clamp(omega_cmd, -self.omega_max, self.omega_max)

        omega_left_cmd = (v_cmd - (self.b / 2.0) * omega_cmd) / self.r
        omega_right_cmd = (v_cmd + (self.b / 2.0) * omega_cmd) / self.r

        tau_left = self.torque_per_wheel_rad_s * omega_left_cmd
        tau_right = self.torque_per_wheel_rad_s * omega_right_cmd

        tau_left = clamp(tau_left, -self.tau_left_max, self.tau_left_max)
        tau_right = clamp(tau_right, -self.tau_right_max, self.tau_right_max)
        tau_left = apply_deadband(tau_left, self.tau_left_deadband)
        tau_right = apply_deadband(tau_right, self.tau_right_deadband)

        F_left = tau_left / self.r
        F_right = tau_right / self.r
        F_left_eff = self.mu_left * F_left
        F_right_eff = self.mu_right * F_right

        F_resist = self.c_v * state["v"] + self.F_c * smooth_sign(state["v"])
        tau_resist = self.c_omega * state["omega"] + self.tau_c * smooth_sign(state["omega"])

        force_net = F_left_eff + F_right_eff - F_resist
        torque_drive = self.eta_turn * (self.b / 2.0) * (F_right_eff - F_left_eff)
        torque_net = torque_drive - tau_resist

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
            "omega_left": omega_left_cmd,
            "omega_right": omega_right_cmd,
            "tau_left": tau_left,
            "tau_right": tau_right,
            "F_left": F_left,
            "F_right": F_right,
            "F_left_eff": F_left_eff,
            "F_right_eff": F_right_eff,
            "F_resist": F_resist,
            "tau_resist": tau_resist,
            "v": v,
            "omega": omega,
            "v_dot": v_dot,
            "omega_dot": omega_dot,
            "force_net": force_net,
            "torque_drive": torque_drive,
            "torque_net": torque_net,
        }
        return new_state, info

    def history_fields(self):
        return [
            "time", "x", "y", "theta",
            "left_input", "right_input", "v_cmd", "omega_cmd", "omega_left", "omega_right",
            "tau_left", "tau_right", "F_left", "F_right", "F_left_eff", "F_right_eff", "F_resist", "tau_resist",
            "v", "omega", "v_dot", "omega_dot", "force_net", "torque_drive", "torque_net",
        ]
