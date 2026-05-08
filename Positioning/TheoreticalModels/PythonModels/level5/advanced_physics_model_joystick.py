
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def apply_deadband(value, threshold):
    if abs(value) < threshold:
        return 0.0
    return value


def smooth_sign(value, epsilon=1e-3):
    return value / (abs(value) + epsilon)


def joystick_to_body_command(jx, jy, v_limit, omega_limit):
    """
    Shared high-level input interface for all joystick files.
    Same meaning as NAV2:
        output v_cmd [m/s], omega_cmd [rad/s]
    Positive jx = turn right, therefore omega_cmd is negative.
    """
    v_cmd = v_limit * jy
    omega_cmd = -omega_limit * jx
    if abs(jx) > 0 and abs(jy) > 0:
        v_cmd *= 0.7071
        omega_cmd *= 0.7071
    return v_cmd, omega_cmd


def body_to_wheel_speeds(v, omega, r, b):
    omega_left = (v - (b / 2.0) * omega) / r
    omega_right = (v + (b / 2.0) * omega) / r
    return omega_left, omega_right

"""
Advanced joystick differential-drive wheelchair simulation.

Same high-level input as the NAV2 version:
    v_cmd, omega_cmd
Extra physics: wheel rotational dynamics, motor torque-speed curve, slip traction, nonlinear resistance.
"""

r = 0.3
b = 0.5
m = 120.0
J_body = 25.0
J_wheel = 2.0
dt = 0.01
V_LIMIT_INIT = 0.8
V_LIMIT_MAX = 1.5
OMEGA_LIMIT_INIT = 1.0
OMEGA_LIMIT_MAX = 2.0
omega_wheel_max = 15.0
a_max = 2.0
alpha_body_max = 4.0
alpha_wheel_max = 20.0
tau_stall = 25.0
omega_no_load = 12.0
motor_damping = 0.6
mu = 0.85
g = 9.81
slip_gain = 4.0
slip_epsilon = 1e-3
c_v = 18.0
F_c = 10.0
c_v2 = 6.0
c_omega = 12.0
tau_c = 4.0
c_omega2 = 4.0
caster_turn_resistance = 3.0
x0 = y0 = theta0 = 0.0
v0 = 0.0
omega0 = 0.0
WINDOW_TITLE = 'Advanced Physics — Joystick Simulator'
EXTRA_METRIC_LABELS = ['u_L', 'u_R', 'slip_L', 'slip_R']


def make_extra_state():
    return {'omega_left': 0.0, 'omega_right': 0.0}


def motor_torque_from_command(u_cmd, omega_wheel):
    speed_factor = max(0.0, 1.0 - abs(omega_wheel) / omega_no_load)
    return u_cmd * tau_stall * speed_factor


def slip_ratio(omega_wheel, v_wheel_side):
    rim_speed = r * omega_wheel
    denom = max(abs(v_wheel_side), abs(rim_speed), slip_epsilon)
    return (rim_speed - v_wheel_side) / denom


def traction_force_from_slip(slip):
    return mu * 0.5 * m * g * np.tanh(slip_gain * slip)


def limited_traction_force(F_slip, tau_motor):
    return clamp(F_slip, -abs(tau_motor) / r, abs(tau_motor) / r)


def model_step(state, extra_state, v_cmd, omega_cmd, dt):
    v_cmd = clamp(v_cmd, -V_LIMIT_MAX, V_LIMIT_MAX)
    omega_cmd = clamp(omega_cmd, -OMEGA_LIMIT_MAX, OMEGA_LIMIT_MAX)
    omega_left_target, omega_right_target = body_to_wheel_speeds(v_cmd, omega_cmd, r, b)
    u_left = clamp(omega_left_target / omega_no_load, -1.0, 1.0)
    u_right = clamp(omega_right_target / omega_no_load, -1.0, 1.0)

    omega_left = extra_state['omega_left']
    omega_right = extra_state['omega_right']
    tau_motor_left = motor_torque_from_command(u_left, omega_left)
    tau_motor_right = motor_torque_from_command(u_right, omega_right)

    v_left_contact = state['v'] - (b / 2.0) * state['omega']
    v_right_contact = state['v'] + (b / 2.0) * state['omega']
    s_left = slip_ratio(omega_left, v_left_contact)
    s_right = slip_ratio(omega_right, v_right_contact)
    F_traction_left = limited_traction_force(traction_force_from_slip(s_left), tau_motor_left)
    F_traction_right = limited_traction_force(traction_force_from_slip(s_right), tau_motor_right)

    omega_left_dot = clamp((tau_motor_left - r * F_traction_left - motor_damping * omega_left) / J_wheel, -alpha_wheel_max, alpha_wheel_max)
    omega_right_dot = clamp((tau_motor_right - r * F_traction_right - motor_damping * omega_right) / J_wheel, -alpha_wheel_max, alpha_wheel_max)
    extra_state['omega_left'] = clamp(omega_left + omega_left_dot * dt, -omega_wheel_max, omega_wheel_max)
    extra_state['omega_right'] = clamp(omega_right + omega_right_dot * dt, -omega_wheel_max, omega_wheel_max)

    F_resist = c_v * state['v'] + F_c * smooth_sign(state['v']) + c_v2 * state['v'] * abs(state['v'])
    tau_resist = (c_omega * state['omega'] + tau_c * smooth_sign(state['omega']) +
                  c_omega2 * state['omega'] * abs(state['omega']) +
                  caster_turn_resistance * smooth_sign(state['omega']) * abs(state['v']))
    force_net = F_traction_left + F_traction_right - F_resist
    torque_net = (b / 2.0) * (F_traction_right - F_traction_left) - tau_resist
    v_dot = clamp(force_net / m, -a_max, a_max)
    omega_dot = clamp(torque_net / J_body, -alpha_body_max, alpha_body_max)
    state['v'] = clamp(state['v'] + v_dot * dt, -V_LIMIT_MAX, V_LIMIT_MAX)
    state['omega'] = clamp(state['omega'] + omega_dot * dt, -OMEGA_LIMIT_MAX, OMEGA_LIMIT_MAX)
    state['x'] += state['v'] * np.cos(state['theta']) * dt
    state['y'] += state['v'] * np.sin(state['theta']) * dt
    state['theta'] += state['omega'] * dt
    return [u_left, u_right, s_left, s_right]

JOYSTICK_RADIUS = 1.0
THUMB_RADIUS = 0.18
MAX_PATH_POINTS = 3000

state = {'x': x0, 'y': y0, 'theta': theta0, 'v': v0, 'omega': omega0}
extra_state = make_extra_state()
path_x = [x0]
path_y = [y0]
joy = {'x': 0.0, 'y': 0.0, 'dragging': False}
key_state = {'w': False, 'a': False, 's': False, 'd': False}

v_limit = V_LIMIT_INIT
omega_limit = OMEGA_LIMIT_INIT

fig = plt.figure(figsize=(13, 7), facecolor='#f8f8f6')
fig.canvas.manager.set_window_title(WINDOW_TITLE)
ax_joy = fig.add_axes([0.03, 0.18, 0.22, 0.70])
ax_traj = fig.add_axes([0.30, 0.18, 0.45, 0.70])
ax_info = fig.add_axes([0.78, 0.38, 0.20, 0.50])
ax_slide_v = fig.add_axes([0.10, 0.08, 0.60, 0.035])
ax_slide_w = fig.add_axes([0.10, 0.03, 0.60, 0.035])

ax_joy.set_xlim(-1.3, 1.3); ax_joy.set_ylim(-1.3, 1.3)
ax_joy.set_aspect('equal'); ax_joy.set_facecolor('#eeede8')
ax_joy.set_title('joystick / WASD', fontsize=11, color='#555')
ax_joy.axhline(0, color='#ccc', linewidth=0.8); ax_joy.axvline(0, color='#ccc', linewidth=0.8)
ax_joy.set_xticks([]); ax_joy.set_yticks([])
for spine in ax_joy.spines.values(): spine.set_visible(False)
ax_joy.add_patch(plt.Circle((0, 0), JOYSTICK_RADIUS, color='#dddcd6', zorder=2))
joy_thumb = plt.Circle((0, 0), THUMB_RADIUS, color='#444', alpha=0.75, zorder=4)
ax_joy.add_patch(joy_thumb)
ax_joy.text(0, 1.15, 'fwd', ha='center', va='bottom', fontsize=9, color='#888')
ax_joy.text(0, -1.15, 'rev', ha='center', va='top', fontsize=9, color='#888')
ax_joy.text(1.15, 0, 'R', ha='left', va='center', fontsize=9, color='#888')
ax_joy.text(-1.15, 0, 'L', ha='right', va='center', fontsize=9, color='#888')

ax_traj.set_facecolor('#f4f3ef')
ax_traj.set_title('trajectory', fontsize=11, color='#555')
ax_traj.set_xlabel('x [m]', fontsize=9, color='#888')
ax_traj.set_ylabel('y [m]', fontsize=9, color='#888')
ax_traj.tick_params(colors='#aaa', labelsize=8)
ax_traj.set_aspect('equal', adjustable='datalim')
traj_line, = ax_traj.plot([], [], color='#333', linewidth=1.5, alpha=0.5)
chair_dot, = ax_traj.plot([x0], [y0], 'o', color='#111', markersize=9)
heading_line, = ax_traj.plot([], [], color='#111', linewidth=2)

ax_info.set_facecolor('#eeede8'); ax_info.set_xlim(0, 1); ax_info.set_ylim(0, 1)
ax_info.set_xticks([]); ax_info.set_yticks([])
for spine in ax_info.spines.values(): spine.set_visible(False)
metric_labels = ['v_cmd', 'ω_cmd', 'v actual', 'ω actual'] + EXTRA_METRIC_LABELS
metric_texts = []
for i, lbl in enumerate(metric_labels):
    y_pos = 0.92 - i * (0.86 / max(1, len(metric_labels) - 1))
    ax_info.text(0.06, y_pos + 0.035, lbl, fontsize=8, color='#888', transform=ax_info.transAxes)
    t = ax_info.text(0.52, y_pos + 0.02, '0.00', fontsize=12, fontweight='bold', color='#222', transform=ax_info.transAxes)
    metric_texts.append(t)

slider_v = Slider(ax_slide_v, 'v max (m/s)', 0.1, V_LIMIT_MAX, valinit=V_LIMIT_INIT, valstep=0.05, color='#aaa')
slider_w = Slider(ax_slide_w, 'ω max (rad/s)', 0.1, OMEGA_LIMIT_MAX, valinit=OMEGA_LIMIT_INIT, valstep=0.05, color='#aaa')

def on_slider(_):
    global v_limit, omega_limit
    v_limit = slider_v.val
    omega_limit = slider_w.val
slider_v.on_changed(on_slider); slider_w.on_changed(on_slider)

ax_reset = fig.add_axes([0.78, 0.08, 0.10, 0.05])
reset_btn = Button(ax_reset, 'reset path', color='#e8e7e2', hovercolor='#d8d7d2')

def on_reset(event):
    global path_x, path_y, extra_state
    state['x'] = x0; state['y'] = y0; state['theta'] = theta0; state['v'] = v0; state['omega'] = omega0
    extra_state = make_extra_state()
    path_x = [x0]; path_y = [y0]
reset_btn.on_clicked(on_reset)


def clamp_to_circle(dx, dy):
    dist = np.hypot(dx, dy)
    if dist > JOYSTICK_RADIUS:
        dx = dx / dist * JOYSTICK_RADIUS
        dy = dy / dist * JOYSTICK_RADIUS
    return dx, dy


def on_press(event):
    if event.inaxes != ax_joy:
        return
    joy['dragging'] = True
    dx, dy = clamp_to_circle(event.xdata or 0, event.ydata or 0)
    joy['x'] = dx; joy['y'] = dy
    joy_thumb.set_center((dx, dy))


def on_move(event):
    if not joy['dragging'] or event.inaxes != ax_joy:
        return
    dx, dy = clamp_to_circle(event.xdata or 0, event.ydata or 0)
    joy['x'] = dx; joy['y'] = dy
    joy_thumb.set_center((dx, dy))


def on_release(event):
    joy['dragging'] = False
    joy['x'] = 0.0; joy['y'] = 0.0
    joy_thumb.set_center((0, 0))


def on_key_press(event):
    k = event.key.lower() if event.key else ''
    if k in ('w', 'up'): key_state['w'] = True
    if k in ('s', 'down'): key_state['s'] = True
    if k in ('a', 'left'): key_state['a'] = True
    if k in ('d', 'right'): key_state['d'] = True


def on_key_release(event):
    k = event.key.lower() if event.key else ''
    if k in ('w', 'up'): key_state['w'] = False
    if k in ('s', 'down'): key_state['s'] = False
    if k in ('a', 'left'): key_state['a'] = False
    if k in ('d', 'right'): key_state['d'] = False


def keys_to_joy():
    jx = (1.0 if key_state['d'] else 0.0) - (1.0 if key_state['a'] else 0.0)
    jy = (1.0 if key_state['w'] else 0.0) - (1.0 if key_state['s'] else 0.0)
    if jx != 0 and jy != 0:
        jx *= 0.7071; jy *= 0.7071
    return jx, jy

fig.canvas.mpl_connect('button_press_event', on_press)
fig.canvas.mpl_connect('motion_notify_event', on_move)
fig.canvas.mpl_connect('button_release_event', on_release)
fig.canvas.mpl_connect('key_press_event', on_key_press)
fig.canvas.mpl_connect('key_release_event', on_key_release)


def update(frame):
    global extra_state
    if joy['dragging']:
        jx, jy = joy['x'], joy['y']
    else:
        jx, jy = keys_to_joy()
        joy_thumb.set_center((jx, jy))

    v_cmd, omega_cmd = joystick_to_body_command(jx, jy, v_limit, omega_limit)
    extra_metrics = model_step(state, extra_state, v_cmd, omega_cmd, dt)

    if abs(v_cmd) > 0.01 or abs(omega_cmd) > 0.01 or abs(state['v']) > 0.01 or abs(state['omega']) > 0.01:
        path_x.append(state['x']); path_y.append(state['y'])
        if len(path_x) > MAX_PATH_POINTS:
            path_x.pop(0); path_y.pop(0)

    traj_line.set_data(path_x, path_y)
    chair_dot.set_data([state['x']], [state['y']])
    hx = state['x'] + 0.4 * np.cos(state['theta'])
    hy = state['y'] + 0.4 * np.sin(state['theta'])
    heading_line.set_data([state['x'], hx], [state['y'], hy])

    all_x = path_x + [state['x']]
    all_y = path_y + [state['y']]
    mx, Mx = min(all_x), max(all_x)
    my, My = min(all_y), max(all_y)
    pad = max(0.8, 0.15 * max(Mx - mx, My - my, 0.1))
    ax_traj.set_xlim(mx - pad, Mx + pad)
    ax_traj.set_ylim(my - pad, My + pad)

    values = [v_cmd, omega_cmd, state['v'], state['omega']] + extra_metrics
    for text, value in zip(metric_texts, values):
        text.set_text(f'{value:+.2f}')

    return (traj_line, chair_dot, heading_line, joy_thumb, *metric_texts)

ani = FuncAnimation(fig, update, interval=int(dt * 1000), blit=False, cache_frame_data=False)
fig.suptitle(WINDOW_TITLE, fontsize=12, color='#444', y=0.97)
plt.show()
