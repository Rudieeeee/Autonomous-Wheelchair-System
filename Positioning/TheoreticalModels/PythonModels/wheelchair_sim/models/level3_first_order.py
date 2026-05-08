import numpy as np

from .base import BaseWheelchairModel, ModelSpec, clamp, apply_deadband


class Level3FirstOrderModel(BaseWheelchairModel):
    spec = ModelSpec(
        key="level3",
        name="Level 3 - First-order velocity dynamics",
        short_name="Level 3 first-order",
        description="Linear and angular velocity commands are followed with lag, saturation, deadband, and acceleration limits.",
        input_type="body_velocity",
        input_label="Linear/angular velocity command",
        input_unit="m/s, rad/s",
    )
    dt = 0.05
    simulation_time = 20.0

    def __init__(self):
        self.r = 0.1778
        self.b = 0.615
        self.tau_v = 0.8
        self.tau_omega = 0.5
        self.v_max = 1.2
        self.omega_max = 1.5
        self.a_max = 0.8
        self.alpha_max = 2.0
        self.v_deadband = 0.05
        self.omega_deadband = 0.08

    def initial_state(self):
        return {"x": 0.0, "y": 0.0, "theta": 0.0, "v": 0.0, "omega": 0.0}

    def step(self, state, v_cmd, omega_cmd, dt):
        v_cmd = clamp(v_cmd, -self.v_max, self.v_max)
        omega_cmd = clamp(omega_cmd, -self.omega_max, self.omega_max)

        v_cmd = apply_deadband(v_cmd, self.v_deadband)
        omega_cmd = apply_deadband(omega_cmd, self.omega_deadband)

        v_dot = (v_cmd - state["v"]) / self.tau_v
        omega_dot = (omega_cmd - state["omega"]) / self.tau_omega

        v_dot = clamp(v_dot, -self.a_max, self.a_max)
        omega_dot = clamp(omega_dot, -self.alpha_max, self.alpha_max)

        v = clamp(state["v"] + v_dot * dt, -self.v_max, self.v_max)
        omega = clamp(state["omega"] + omega_dot * dt, -self.omega_max, self.omega_max)

        omega_left = (v_cmd - (self.b / 2.0) * omega_cmd) / self.r
        omega_right = (v_cmd + (self.b / 2.0) * omega_cmd) / self.r

        x = state["x"] + v * np.cos(state["theta"]) * dt
        y = state["y"] + v * np.sin(state["theta"]) * dt
        theta = state["theta"] + omega * dt

        new_state = {"x": x, "y": y, "theta": theta, "v": v, "omega": omega}
        info = {
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            "omega_left": omega_left,
            "omega_right": omega_right,
            "v": v,
            "omega": omega,
            "v_dot": v_dot,
            "omega_dot": omega_dot,
        }
        return new_state, info

    def history_fields(self):
        return [
            "time", "x", "y", "theta",
            "left_input", "right_input", "v_cmd", "omega_cmd",
            "omega_left", "omega_right", "v", "omega", "v_dot", "omega_dot",
        ]
