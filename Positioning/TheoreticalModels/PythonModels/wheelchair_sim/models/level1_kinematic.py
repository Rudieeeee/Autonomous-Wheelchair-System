import numpy as np

from .base import BaseWheelchairModel, ModelSpec, clamp


class Level1KinematicModel(BaseWheelchairModel):
    spec = ModelSpec(
        key="level1",
        name="Level 1 - Pure kinematic differential drive",
        short_name="Level 1 kinematic",
        description="No dynamics or slip. Linear and angular velocity commands directly determine the body pose.",
        input_type="body_velocity",
        input_label="Linear/angular velocity command",
        input_unit="m/s, rad/s",
    )
    dt = 0.05
    simulation_time = 20.0

    def __init__(self):
        self.r = 0.1778
        self.b = 0.615
        self.v_max = 1.2
        self.omega_max = 1.5

    def initial_state(self):
        return {"x": 0.0, "y": 0.0, "theta": 0.0}

    def step(self, state, v_cmd, omega_cmd, dt):
        v = clamp(v_cmd, -self.v_max, self.v_max)
        omega = clamp(omega_cmd, -self.omega_max, self.omega_max)

        omega_left = (v - (self.b / 2.0) * omega) / self.r
        omega_right = (v + (self.b / 2.0) * omega) / self.r

        x = state["x"] + v * np.cos(state["theta"]) * dt
        y = state["y"] + v * np.sin(state["theta"]) * dt
        theta = state["theta"] + omega * dt

        new_state = {"x": x, "y": y, "theta": theta}
        info = {
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            "omega_left": omega_left,
            "omega_right": omega_right,
            "v": v,
            "omega": omega,
        }
        return new_state, info

    def history_fields(self):
        return [
            "time", "x", "y", "theta",
            "left_input", "right_input", "v_cmd", "omega_cmd",
            "omega_left", "omega_right", "v", "omega",
        ]
