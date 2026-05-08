from .level1_kinematic import Level1KinematicModel
from .level3_first_order import Level3FirstOrderModel
from .level4_force import Level4ForceModel
from .level4_torque_slip import Level4TorqueSlipModel
from .advanced_motor_slip import AdvancedMotorSlipModel

MODEL_CLASSES = [
    Level1KinematicModel,
    Level3FirstOrderModel,
    Level4ForceModel,
    Level4TorqueSlipModel,
    AdvancedMotorSlipModel,
]


def create_model(key: str):
    for cls in MODEL_CLASSES:
        if cls.spec.key == key:
            return cls()
    raise ValueError(f"Unknown model key: {key}")
