import math
import numpy as np


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))


def predefined_inputs(t: float, input_type: str = "body_velocity") -> tuple[float, float]:
    """
    Predefined automatic input mode.

    The simulator now uses the same high-level command for every model:
        left_input  = v_cmd     [m/s]
        right_input = omega_cmd [rad/s]

    The more advanced models internally convert this command into wheel speeds,
    wheel torques, or motor commands so their extra physical effects stay active.
    """
    if input_type != "body_velocity":
        raise ValueError(f"Unknown input type: {input_type}")

    v_cmd = 0.65 + 0.15 * np.sin(0.35 * t)
    omega_cmd = 0.55 * np.sin(0.5 * t)
    return clamp(v_cmd, -1.2, 1.2), clamp(omega_cmd, -1.5, 1.5)


class JoystickInputController:
    """
    Mouse-controlled joystick input controller.

    Drag the joystick knob in the Pygame window:
    - up/down controls linear velocity v_cmd
    - right/left controls angular velocity omega_cmd
    - releasing the mouse returns to neutral, like a real joystick
    """

    def __init__(self, max_v: float = 1.2, max_omega: float = 1.5):
        self.max_v = max_v
        self.max_omega = max_omega
        self.radius = 90
        self.knob_radius = 18
        self.center = (0, 0)
        self.dragging = False
        self.jx = 0.0
        self.jy = 0.0

    def reset(self) -> None:
        self.dragging = False
        self.jx = 0.0
        self.jy = 0.0

    def set_center(self, center: tuple[int, int]) -> None:
        self.center = center

    def _set_from_mouse(self, pos: tuple[int, int]) -> None:
        cx, cy = self.center
        dx = pos[0] - cx
        dy = cy - pos[1]  # screen y is inverted; up should be positive
        distance = math.hypot(dx, dy)
        if distance > self.radius:
            dx = dx / distance * self.radius
            dy = dy / distance * self.radius
        self.jx = clamp(dx / self.radius, -1.0, 1.0)
        self.jy = clamp(dy / self.radius, -1.0, 1.0)

    def handle_event(self, event, pygame) -> None:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos
            cx, cy = self.center
            if math.hypot(mx - cx, my - cy) <= self.radius + self.knob_radius:
                self.dragging = True
                self._set_from_mouse(event.pos)

        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self._set_from_mouse(event.pos)

        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self.dragging:
                self.reset()

    def update(self, dt: float) -> tuple[float, float]:
        v_cmd = self.max_v * self.jy
        # Positive omega means counter-clockwise / left turn.
        # Joystick right should turn right, so right input gives negative omega.
        omega_cmd = -self.max_omega * self.jx
        return v_cmd, omega_cmd

    def draw(self, screen, pygame, font) -> None:
        cx, cy = self.center
        pygame.draw.circle(screen, (230, 234, 242), (cx, cy), self.radius)
        pygame.draw.circle(screen, (95, 110, 135), (cx, cy), self.radius, width=3)
        pygame.draw.line(screen, (180, 190, 205), (cx - self.radius, cy), (cx + self.radius, cy), 2)
        pygame.draw.line(screen, (180, 190, 205), (cx, cy - self.radius), (cx, cy + self.radius), 2)

        kx = int(cx + self.jx * self.radius)
        ky = int(cy - self.jy * self.radius)
        pygame.draw.circle(screen, (45, 55, 70), (kx, ky), self.knob_radius)

        labels = [
            ("fwd", (cx, cy - self.radius - 28)),
            ("rev", (cx, cy + self.radius + 24)),
            ("L", (cx - self.radius - 24, cy)),
            ("R", (cx + self.radius + 24, cy)),
        ]
        for text, pos in labels:
            label = font.render(text, True, (70, 80, 95))
            screen.blit(label, label.get_rect(center=pos))

        values = font.render(f"v_cmd={self.max_v * self.jy:+.2f} m/s   omega_cmd={-self.max_omega * self.jx:+.2f} rad/s", True, (45, 55, 70))
        screen.blit(values, values.get_rect(center=(cx, cy + self.radius + 55)))
