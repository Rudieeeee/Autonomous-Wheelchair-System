import numpy as np

from .base import BaseWheelchairModel, ModelSpec, clamp, apply_deadband, smooth_sign


class AdvancedMotorSlipModel(BaseWheelchairModel):
    spec = ModelSpec(
        key="advanced_motor_slip",
        name="Advanced - Motor torque-speed, wheel dynamics, slip traction",
        short_name="Advanced motor/slip",
        description="Linear/angular velocity commands are converted to motor commands, then wheel dynamics, slip traction, nonlinear resistance, and body dynamics are simulated.",
        input_type="body_velocity",
        input_label="Linear/angular velocity command",
        input_unit="m/s, rad/s",
    )
    dt = 0.02
    simulation_time = 20.0

    def __init__(self):
        self.r = 0.1778
        self.b = 0.615
        self.m = 233.5
        self.J_body = 30.0
        self.J_wheel = 2.0
        self.v_max = 1.2
        self.omega_body_max = 1.5
        self.omega_wheel_max = 6.75
        self.a_max = 1.0
        self.alpha_body_max = 2.0
        self.alpha_wheel_max = 12.0
        self.tau_stall = 25.0
        self.omega_no_load = 8.0
        self.motor_damping = 0.6
        self.u_left_deadband = 0.02
        self.u_right_deadband = 0.02
        self.mu = 0.90
        self.g = 9.81
        self.slip_gain = 4.0
        self.slip_epsilon = 1e-3
        self.c_v = 18.0
        self.F_c = 10.0
        self.c_v2 = 6.0
        self.c_omega = 12.0
        self.tau_c = 4.0
        self.c_omega2 = 4.0
        self.caster_turn_resistance = 3.0

    def initial_state(self):
        return {
            "x": 0.0, "y": 0.0, "theta": 0.0,
            "v": 0.0, "omega_body": 0.0,
            "omega_left": 0.0, "omega_right": 0.0,
        }

    def _motor_torque_from_command(self, u_cmd, omega_wheel):
        speed_factor = max(0.0, 1.0 - abs(omega_wheel) / self.omega_no_load)
        return u_cmd * self.tau_stall * speed_factor

    def _slip_ratio(self, omega_wheel, v_wheel_side):
        rim_speed = self.r * omega_wheel
        denom = max(abs(v_wheel_side), abs(rim_speed), self.slip_epsilon)
        return (rim_speed - v_wheel_side) / denom

    def _traction_force_from_slip(self, slip, normal_force):
        return self.mu * normal_force * np.tanh(self.slip_gain * slip)

    def _limited_traction_force(self, F_slip, tau_motor):
        F_torque_max = abs(tau_motor) / self.r
        return clamp(F_slip, -F_torque_max, F_torque_max)

    def _body_resistance(self, v, omega_body):
        F_resist = self.c_v * v + self.F_c * smooth_sign(v) + self.c_v2 * v * abs(v)
        tau_resist = (
            self.c_omega * omega_body
            + self.tau_c * smooth_sign(omega_body)
            + self.c_omega2 * omega_body * abs(omega_body)
            + self.caster_turn_resistance * smooth_sign(omega_body) * abs(v)
        )
        return F_resist, tau_resist

    def step(self, state, v_cmd, omega_cmd, dt):
        v_cmd = clamp(v_cmd, -self.v_max, self.v_max)
        omega_cmd = clamp(omega_cmd, -self.omega_body_max, self.omega_body_max)

        omega_left_cmd = (v_cmd - (self.b / 2.0) * omega_cmd) / self.r
        omega_right_cmd = (v_cmd + (self.b / 2.0) * omega_cmd) / self.r

        u_left = clamp(omega_left_cmd / self.omega_wheel_max, -1.0, 1.0)
        u_right = clamp(omega_right_cmd / self.omega_wheel_max, -1.0, 1.0)
        u_left = clamp(apply_deadband(u_left, self.u_left_deadband), -1.0, 1.0)
        u_right = clamp(apply_deadband(u_right, self.u_right_deadband), -1.0, 1.0)

        tau_motor_left = self._motor_torque_from_command(u_left, state["omega_left"])
        tau_motor_right = self._motor_torque_from_command(u_right, state["omega_right"])

        v_left_contact = state["v"] - (self.b / 2.0) * state["omega_body"]
        v_right_contact = state["v"] + (self.b / 2.0) * state["omega_body"]

        slip_left = self._slip_ratio(state["omega_left"], v_left_contact)
        slip_right = self._slip_ratio(state["omega_right"], v_right_contact)

        N_left = 0.5 * self.m * self.g
        N_right = 0.5 * self.m * self.g

        F_slip_left = self._traction_force_from_slip(slip_left, N_left)
        F_slip_right = self._traction_force_from_slip(slip_right, N_right)

        F_traction_left = self._limited_traction_force(F_slip_left, tau_motor_left)
        F_traction_right = self._limited_traction_force(F_slip_right, tau_motor_right)

        omega_left_dot = (tau_motor_left - self.r * F_traction_left - self.motor_damping * state["omega_left"]) / self.J_wheel
        omega_right_dot = (tau_motor_right - self.r * F_traction_right - self.motor_damping * state["omega_right"]) / self.J_wheel

        omega_left_dot = clamp(omega_left_dot, -self.alpha_wheel_max, self.alpha_wheel_max)
        omega_right_dot = clamp(omega_right_dot, -self.alpha_wheel_max, self.alpha_wheel_max)

        omega_left = clamp(state["omega_left"] + omega_left_dot * dt, -self.omega_wheel_max, self.omega_wheel_max)
        omega_right = clamp(state["omega_right"] + omega_right_dot * dt, -self.omega_wheel_max, self.omega_wheel_max)

        F_resist, tau_resist = self._body_resistance(state["v"], state["omega_body"])

        force_net = F_traction_left + F_traction_right - F_resist
        torque_net = (self.b / 2.0) * (F_traction_right - F_traction_left) - tau_resist

        v_dot = clamp(force_net / self.m, -self.a_max, self.a_max)
        omega_body_dot = clamp(torque_net / self.J_body, -self.alpha_body_max, self.alpha_body_max)

        v = clamp(state["v"] + v_dot * dt, -self.v_max, self.v_max)
        omega_body = clamp(state["omega_body"] + omega_body_dot * dt, -self.omega_body_max, self.omega_body_max)

        x = state["x"] + v * np.cos(state["theta"]) * dt
        y = state["y"] + v * np.sin(state["theta"]) * dt
        theta = state["theta"] + omega_body * dt

        new_state = {
            "x": x, "y": y, "theta": theta,
            "v": v, "omega_body": omega_body,
            "omega_left": omega_left, "omega_right": omega_right,
        }
        info = {
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            "u_left": u_left,
            "u_right": u_right,
            "tau_motor_left": tau_motor_left,
            "tau_motor_right": tau_motor_right,
            "omega_left": omega_left,
            "omega_right": omega_right,
            "omega_body": omega_body,
            "v": v,
            "slip_left": slip_left,
            "slip_right": slip_right,
            "F_slip_left": F_slip_left,
            "F_slip_right": F_slip_right,
            "F_traction_left": F_traction_left,
            "F_traction_right": F_traction_right,
            "v_dot": v_dot,
            "omega_body_dot": omega_body_dot,
            "omega_left_dot": omega_left_dot,
            "omega_right_dot": omega_right_dot,
            "F_resist": F_resist,
            "tau_resist": tau_resist,
            "force_net": force_net,
            "torque_net": torque_net,
        }
        return new_state, info

    def history_fields(self):
        return [
            "time", "x", "y", "theta",
            "left_input", "right_input", "v_cmd", "omega_cmd", "u_left", "u_right",
            "tau_motor_left", "tau_motor_right",
            "omega_left", "omega_right", "omega_body", "v",
            "slip_left", "slip_right", "F_slip_left", "F_slip_right", "F_traction_left", "F_traction_right",
            "v_dot", "omega_body_dot", "omega_left_dot", "omega_right_dot",
            "F_resist", "tau_resist", "force_net", "torque_net",
        ]
