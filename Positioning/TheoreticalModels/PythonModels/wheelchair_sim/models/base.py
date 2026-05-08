from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))


def apply_deadband(value: float, threshold: float) -> float:
    if abs(value) < threshold:
        return 0.0
    return value


def smooth_sign(value: float, epsilon: float = 1e-3) -> float:
    return value / (abs(value) + epsilon)


@dataclass
class ModelSpec:
    key: str
    name: str
    short_name: str
    description: str
    input_type: str
    input_label: str
    input_unit: str


class BaseWheelchairModel:
    spec: ModelSpec
    dt: float = 0.05
    simulation_time: float = 10.0

    def initial_state(self) -> Dict[str, float]:
        raise NotImplementedError

    def step(self, state: Dict[str, float], left_input: float, right_input: float, dt: float):
        raise NotImplementedError

    def history_fields(self) -> List[str]:
        return ["time", "x", "y", "theta"]
