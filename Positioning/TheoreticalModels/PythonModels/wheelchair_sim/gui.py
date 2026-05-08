import math
import pygame

from config import GuiParams
from inputs import predefined_inputs, JoystickInputController
from models import MODEL_CLASSES, create_model
from plotter import show_all_plots
from simulation import Simulator


class Button:
    def __init__(self, rect, text, callback, font_size=20):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.callback = callback
        self.font_size = font_size

    def draw(self, screen):
        font = pygame.font.SysFont("Arial", self.font_size)
        mouse_pos = pygame.mouse.get_pos()
        hovered = self.rect.collidepoint(mouse_pos)
        color = (215, 225, 245) if hovered else (235, 240, 250)
        pygame.draw.rect(screen, color, self.rect, border_radius=12)
        pygame.draw.rect(screen, (55, 70, 100), self.rect, width=2, border_radius=12)
        label = font.render(self.text, True, (20, 30, 45))
        screen.blit(label, label.get_rect(center=self.rect.center))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.callback()


class WheelchairApp:
    def __init__(self):
        pygame.init()
        self.gui_params = GuiParams()
        self.screen = pygame.display.set_mode((self.gui_params.width, self.gui_params.height))
        pygame.display.set_caption("Smart Wheelchair Multi-Model Simulator")
        self.clock = pygame.time.Clock()
        self.title_font = pygame.font.SysFont("Arial", 34, bold=True)
        self.font = pygame.font.SysFont("Arial", 20)
        self.small_font = pygame.font.SysFont("Arial", 16)
        self.running = True
        self.screen_mode = "menu"
        self.simulator = None
        self.joystick_controller = None
        self.input_mode = "predefined"
        self.paused = False
        self.sim_time_accumulator = 0.0
        self.pending_plot = False
        self.menu_buttons = []
        self.sim_buttons = []
        self._build_menu_buttons()

    def _build_menu_buttons(self):
        self.menu_buttons = []

        button_width = int(self.gui_params.width * 0.55)
        button_height = 72
        button_gap = 20

        x = self.gui_params.width // 2 - button_width // 2
        y = 170

        for index, model_cls in enumerate(MODEL_CLASSES, start=1):
            spec = model_cls.spec
            self.menu_buttons.append(
                Button(
                    (x, y, button_width, button_height),
                    f"{index}. {spec.name}",
                    lambda key=spec.key: self.start_simulation(key),
                    font_size=20,
                )
            )
            y += button_height + button_gap

    def _build_sim_buttons(self):
        self.sim_buttons = [
            Button((20, 20, 115, 38), "Pause", self.toggle_pause, 18),
            Button((150, 20, 115, 38), "Reset", self.reset_simulation, 18),
            Button((280, 20, 160, 38), "Switch mode", self.switch_input_mode, 18),
            Button((455, 20, 120, 38), "Plots", self.request_plots, 18),
            Button((590, 20, 135, 38), "Main menu", self.return_to_menu, 18),
        ]

    def start_simulation(self, model_key: str):
        model = create_model(model_key)
        self.simulator = Simulator(model)
        self.joystick_controller = JoystickInputController()
        self.input_mode = "predefined"
        self.paused = False
        self.sim_time_accumulator = 0.0
        self.pending_plot = False
        self.screen_mode = "simulation"
        self._build_sim_buttons()

    def return_to_menu(self):
        self.simulator = None
        self.joystick_controller = None
        self.screen_mode = "menu"
        self.pending_plot = False

    def toggle_pause(self):
        self.paused = not self.paused
        self.sim_buttons[0].text = "Resume" if self.paused else "Pause"

    def reset_simulation(self):
        if self.simulator is not None:
            self.simulator.reset()
        if self.joystick_controller is not None:
            self.joystick_controller.reset()
        self.paused = False
        self.sim_time_accumulator = 0.0
        self.sim_buttons[0].text = "Pause"

    def switch_input_mode(self):
        if self.input_mode == "predefined":
            self.input_mode = "joystick"
            if self.simulator is not None:
                self.simulator.finished = False
        else:
            self.input_mode = "predefined"
        if self.joystick_controller is not None:
            self.joystick_controller.reset()

    def request_plots(self):
        self.pending_plot = True
        self.running = False

    def world_to_screen(self, x_m, y_m):
        origin_x = self.gui_params.width // 2
        origin_y = self.gui_params.height // 2 + 130
        sx = origin_x + x_m * self.gui_params.pixels_per_meter
        sy = origin_y - y_m * self.gui_params.pixels_per_meter
        return int(sx), int(sy)

    def draw_menu(self):
        self.screen.fill((248, 250, 253))
        title = self.title_font.render("Smart Wheelchair Multi-Model Simulator", True, (20, 30, 45))
        self.screen.blit(title, title.get_rect(center=(self.gui_params.width // 2, 70)))

        subtitle = self.font.render("Choose the physical model to visualize", True, (60, 75, 95))
        self.screen.blit(subtitle, subtitle.get_rect(center=(self.gui_params.width // 2, 112)))

        for button in self.menu_buttons:
            button.draw(self.screen)

        y = 640
        help_lines = [
            "After selecting a model: M switches predefined/joystick input, drag the joystick in joystick mode, P opens plots.",
            "Predefined mode stops at the model simulation time. Joystick mode keeps running until you quit, reset, or plot.",
        ]
        for line in help_lines:
            label = self.small_font.render(line, True, (70, 80, 95))
            self.screen.blit(label, label.get_rect(center=(self.gui_params.width // 2, y)))
            y += 26

    def draw_grid(self):
        spacing = max(20, int(self.gui_params.pixels_per_meter))
        width, height = self.gui_params.width, self.gui_params.height
        origin_x, origin_y = self.world_to_screen(0, 0)

        for x in range(origin_x % spacing, width, spacing):
            pygame.draw.line(self.screen, (228, 232, 238), (x, 0), (x, height))
        for y in range(origin_y % spacing, height, spacing):
            pygame.draw.line(self.screen, (228, 232, 238), (0, y), (width, y))

        pygame.draw.line(self.screen, (155, 165, 175), (0, origin_y), (width, origin_y), 2)
        pygame.draw.line(self.screen, (155, 165, 175), (origin_x, 0), (origin_x, height), 2)

    def draw_path(self):
        history = self.simulator.history
        points = [self.world_to_screen(x, y) for x, y in zip(history.x, history.y)]
        if len(points) > 1:
            pygame.draw.lines(self.screen, (35, 105, 220), False, points, 3)

    def draw_wheelchair(self):
        state = self.simulator.state
        cx, cy = self.world_to_screen(state.get("x", 0.0), state.get("y", 0.0))
        theta = state.get("theta", 0.0)

        length_px = self.gui_params.wheelchair_length * self.gui_params.pixels_per_meter
        width_px = self.gui_params.wheelchair_width * self.gui_params.pixels_per_meter
        corners = [
            (+length_px / 2, +width_px / 2),
            (+length_px / 2, -width_px / 2),
            (-length_px / 2, -width_px / 2),
            (-length_px / 2, +width_px / 2),
        ]
        rotated = []
        for lx, ly in corners:
            sx = cx + lx * math.cos(theta) - ly * math.sin(theta)
            sy = cy - (lx * math.sin(theta) + ly * math.cos(theta))
            rotated.append((sx, sy))

        pygame.draw.polygon(self.screen, (230, 75, 75), rotated)
        pygame.draw.polygon(self.screen, (90, 30, 30), rotated, width=3)

        arrow_length = 0.65 * self.gui_params.pixels_per_meter
        hx = cx + arrow_length * math.cos(theta)
        hy = cy - arrow_length * math.sin(theta)
        pygame.draw.line(self.screen, (20, 20, 25), (cx, cy), (hx, hy), 4)
        pygame.draw.circle(self.screen, (20, 20, 25), (int(hx), int(hy)), 6)

    def draw_sim_text(self):
        model = self.simulator.model
        state = self.simulator.state
        info = self.simulator.last_info
        lines = [
            f"Model: {model.spec.short_name}",
            f"Input mode: {self.input_mode.upper()}    Input: v_cmd [m/s], omega_cmd [rad/s]",
            f"t = {self.simulator.t:.2f} s     dt = {self.simulator.dt:.3f} s",
            f"x = {state.get('x', 0.0):.2f} m, y = {state.get('y', 0.0):.2f} m, theta = {state.get('theta', 0.0):.2f} rad",
            f"v_cmd input = {self.simulator.last_left_input:.2f} m/s, omega_cmd input = {self.simulator.last_right_input:.2f} rad/s",
        ]
        if "v" in state:
            lines.append(f"v = {state.get('v', 0.0):.2f} m/s")
        if "omega" in state:
            lines.append(f"omega = {state.get('omega', 0.0):.2f} rad/s")
        if "omega_body" in state:
            lines.append(f"omega_body = {state.get('omega_body', 0.0):.2f} rad/s")
        if "omega_left" in state and "omega_right" in state:
            lines.append(f"wheel speeds: left = {state.get('omega_left', 0.0):.2f}, right = {state.get('omega_right', 0.0):.2f} rad/s")

        lines += [
            "Controls: M switch input mode | drag the joystick in joystick mode | SPACE pause | R reset | P plots | ESC menu",
        ]

        y = 72
        for line in lines:
            label = self.small_font.render(line, True, (25, 30, 40))
            self.screen.blit(label, (20, y))
            y += 22

        if self.simulator.finished:
            label = self.font.render("Predefined simulation finished. Press M for joystick mode, R to restart, or P for plots.", True, (25, 30, 40))
            self.screen.blit(label, (20, self.gui_params.height - 42))

    def draw_joystick(self):
        if self.joystick_controller is None:
            return
        center = (self.gui_params.width - 180, self.gui_params.height - 180)
        self.joystick_controller.set_center(center)
        self.joystick_controller.draw(self.screen, pygame, self.small_font)

    def draw_simulation(self):
        self.screen.fill((250, 250, 250))
        self.draw_grid()
        self.draw_path()
        self.draw_wheelchair()
        self.draw_sim_text()
        if self.input_mode == "joystick":
            self.draw_joystick()
        for button in self.sim_buttons:
            button.draw(self.screen)

    def get_current_inputs(self):
        model = self.simulator.model
        if self.input_mode == "predefined":
            return predefined_inputs(self.simulator.t, model.spec.input_type)

        return self.joystick_controller.update(self.simulator.dt)

    def update_simulation(self):
        if self.paused or self.simulator is None or self.simulator.finished:
            return

        self.sim_time_accumulator += self.gui_params.playback_speed / self.gui_params.fps

        max_steps_per_frame = 20
        steps = 0
        while self.sim_time_accumulator >= self.simulator.dt and steps < max_steps_per_frame:
            left_input, right_input = self.get_current_inputs()
            stop_at_end = self.input_mode == "predefined"
            self.simulator.step(left_input, right_input, stop_at_end=stop_at_end)
            self.sim_time_accumulator -= self.simulator.dt
            steps += 1

    def handle_menu_key(self, key):
        if pygame.K_1 <= key <= pygame.K_9:
            index = key - pygame.K_1
            if 0 <= index < len(MODEL_CLASSES):
                self.start_simulation(MODEL_CLASSES[index].spec.key)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            if self.screen_mode == "menu":
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                    else:
                        self.handle_menu_key(event.key)
                for button in self.menu_buttons:
                    button.handle_event(event)

            elif self.screen_mode == "simulation":
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.return_to_menu()
                    elif event.key == pygame.K_SPACE:
                        self.toggle_pause()
                    elif event.key == pygame.K_r:
                        self.reset_simulation()
                    elif event.key == pygame.K_m:
                        self.switch_input_mode()
                    elif event.key == pygame.K_p:
                        self.request_plots()

                if self.joystick_controller is not None and self.input_mode == "joystick":
                    self.joystick_controller.handle_event(event, pygame)

                for button in self.sim_buttons:
                    button.handle_event(event)

    def run(self):
        while self.running:
            self.handle_events()
            if self.screen_mode == "simulation":
                self.update_simulation()
                self.draw_simulation()
            else:
                self.draw_menu()
            pygame.display.flip()
            self.clock.tick(self.gui_params.fps)

        simulator_to_plot = self.simulator if self.pending_plot else None
        pygame.quit()

        if simulator_to_plot is not None:
            show_all_plots(simulator_to_plot)
