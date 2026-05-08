from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np


@dataclass
class SimulationHistory:
    fields: List[str]
    data: Dict[str, List[float]] = field(default_factory=dict)

    def __post_init__(self):
        for field_name in self.fields:
            self.data[field_name] = []

    def append(self, values: Dict[str, float]) -> None:
        for field_name in self.fields:
            self.data[field_name].append(float(values.get(field_name, 0.0)))

    def as_arrays(self) -> Dict[str, np.ndarray]:
        return {name: np.array(values) for name, values in self.data.items()}

    def __getattr__(self, name: str):
        if "data" in self.__dict__ and name in self.data:
            return self.data[name]
        raise AttributeError(name)


class Simulator:
    def __init__(self, model):
        self.model = model
        self.dt = model.dt
        self.simulation_time = model.simulation_time
        self.reset()

    def reset(self) -> None:
        self.t = 0.0
        self.state = self.model.initial_state()
        self.finished = False
        self.history = SimulationHistory(self.model.history_fields())
        self.last_left_input = 0.0
        self.last_right_input = 0.0
        self.last_info = {}
        self._store({}, 0.0, 0.0)

    def _store(self, info: Dict[str, float], left_input: float, right_input: float) -> None:
        row = {
            "time": self.t,
            "x": self.state.get("x", 0.0),
            "y": self.state.get("y", 0.0),
            "theta": self.state.get("theta", 0.0),
            "left_input": left_input,
            "right_input": right_input,
        }
        row.update({k: float(v) for k, v in self.state.items() if isinstance(v, (int, float))})
        row.update(info)
        self.history.append(row)
        self.last_left_input = left_input
        self.last_right_input = right_input
        self.last_info = info

    def step(self, left_input: float, right_input: float, stop_at_end: bool = True) -> None:
        if self.finished:
            return

        self.state, info = self.model.step(self.state, left_input, right_input, self.dt)
        self.t += self.dt
        self._store(info, left_input, right_input)

        if stop_at_end and self.t >= self.simulation_time:
            self.finished = True
