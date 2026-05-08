from dataclasses import dataclass


@dataclass
class GuiParams:
    width: int = 2000
    height: int = 1000
    fps: int = 60
    playback_speed: float = 2
    pixels_per_meter: float = 90.0

    # Visual body size based on SANGO advanced overall dimensions.
    # This is only used for drawing the rectangle in Pygame.
    wheelchair_length: float = 1.10
    wheelchair_width: float = 0.615


@dataclass
class SimulationParams:
    simulation_time: float = 20.0


@dataclass
class WheelchairGeometry:
    # SANGO advanced SEGO source-based starting values.
    # 14 inch drive wheel -> radius = (14 * 0.0254) / 2 = 0.1778 m.
    # b is the approximate left-right drive-wheel track; measure your actual chair later.
    r: float = 0.1778
    b: float = 0.615
