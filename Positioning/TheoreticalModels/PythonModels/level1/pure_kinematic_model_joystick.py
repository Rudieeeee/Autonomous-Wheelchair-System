import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Slider
from matplotlib.animation import FuncAnimation


"""
Differential-drive wheelchair simulation with joystick control.

Joystick mapping (joyX, joyY in [-1, 1]):
    omega_L = omega_max * (joyY + joyX)
    omega_R = omega_max * (joyY - joyX)

    Scaled down if either wheel exceeds omega_max.

    joyY =  1, joyX = 0  -> full forward
    joyY = -1, joyX = 0  -> full reverse
    joyY =  0, joyX =  1 -> pivot right
    joyY =  0, joyX = -1 -> pivot left
    Diagonals combine forward + turn.
"""


# PARAMETERS
r = 0.1778   # wheel radius [m]
b = 0.615    # distance between left and right wheels [m]
dt = 0.05    # simulation time step [s]

OMEGA_MAX_INIT = 2.0   # initial wheel speed budget [rad/s]
OMEGA_MAX_MIN  = 0.5
OMEGA_MAX_MAX  = 6.0

JOYSTICK_RADIUS = 1.0  # normalised joystick range
THUMB_RADIUS    = 0.18

MAX_PATH_POINTS = 3000


# STATE
state = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
path_x = [0.0]
path_y = [0.0]
joy = {'x': 0.0, 'y': 0.0, 'dragging': False}
omega_max = OMEGA_MAX_INIT


# MAPPING
def joystick_to_wheel_speeds(jx, jy, wmax):
    # Positive jx should turn right:
    # right turn = left wheel faster, right wheel slower
    omL = wmax * (jy + jx)
    omR = wmax * (jy - jx)

    scale = max(1.0, abs(omL) / wmax, abs(omR) / wmax)
    return omL / scale, omR / scale


# FIGURE LAYOUT
fig = plt.figure(figsize=(13, 7), facecolor='#f8f8f6')
fig.canvas.manager.set_window_title('Wheelchair Joystick Simulator')

# Axes layout using subplot2grid
ax_joy   = fig.add_axes([0.03, 0.18, 0.22, 0.70])   # joystick
ax_traj  = fig.add_axes([0.30, 0.18, 0.45, 0.70])   # trajectory
ax_info  = fig.add_axes([0.78, 0.42, 0.20, 0.46])   # metrics text
ax_wl    = fig.add_axes([0.78, 0.30, 0.20, 0.06])   # left wheel bar
ax_wr    = fig.add_axes([0.78, 0.20, 0.20, 0.06])   # right wheel bar
ax_slide = fig.add_axes([0.10, 0.06, 0.60, 0.04])   # slider


# JOYSTICK AXIS
ax_joy.set_xlim(-1.3, 1.3)
ax_joy.set_ylim(-1.3, 1.3)
ax_joy.set_aspect('equal')
ax_joy.set_facecolor('#eeede8')
ax_joy.set_title('joystick', fontsize=11, color='#555')
ax_joy.axhline(0, color='#ccc', linewidth=0.8, zorder=1)
ax_joy.axvline(0, color='#ccc', linewidth=0.8, zorder=1)
ax_joy.set_xticks([]); ax_joy.set_yticks([])
for spine in ax_joy.spines.values():
    spine.set_visible(False)

joy_base = plt.Circle((0, 0), JOYSTICK_RADIUS, color='#dddcd6', zorder=2)
ax_joy.add_patch(joy_base)

joy_thumb = plt.Circle((0, 0), THUMB_RADIUS, color='#444', alpha=0.75, zorder=4)
ax_joy.add_patch(joy_thumb)

ax_joy.text(0,  1.15, 'fwd',  ha='center', va='bottom', fontsize=9, color='#888')
ax_joy.text(0, -1.15, 'rev',  ha='center', va='top',    fontsize=9, color='#888')
ax_joy.text( 1.15, 0, 'R',    ha='left',   va='center', fontsize=9, color='#888')
ax_joy.text(-1.15, 0, 'L',    ha='right',  va='center', fontsize=9, color='#888')


# TRAJECTORY AXIS
ax_traj.set_facecolor('#f4f3ef')
ax_traj.set_title('trajectory', fontsize=11, color='#555')
ax_traj.set_xlabel('x [m]', fontsize=9, color='#888')
ax_traj.set_ylabel('y [m]', fontsize=9, color='#888')
ax_traj.tick_params(colors='#aaa', labelsize=8)
ax_traj.set_aspect('equal', adjustable='datalim')
for spine in ax_traj.spines.values():
    spine.set_edgecolor('#ddd')

traj_line,      = ax_traj.plot([], [], color='#333', linewidth=1.5, alpha=0.5, zorder=2)
start_dot,      = ax_traj.plot([0], [0], 'o', color='#888', markersize=6, zorder=3)
chair_dot,      = ax_traj.plot([0], [0], 'o', color='#111', markersize=9, zorder=5)
heading_line,   = ax_traj.plot([], [], color='#111', linewidth=2, zorder=4)

HEADING_LEN = 0.4


# METRICS TEXT AXIS
ax_info.set_facecolor('#eeede8')
ax_info.set_xlim(0, 1); ax_info.set_ylim(0, 1)
ax_info.set_xticks([]); ax_info.set_yticks([])
for spine in ax_info.spines.values():
    spine.set_visible(False)

labels = ['v (m/s)', 'ω (rad/s)', 'ω_L (rad/s)', 'ω_R (rad/s)']
metric_texts = []
for i, lbl in enumerate(labels):
    ypos = 0.82 - i * 0.22
    ax_info.text(0.08, ypos + 0.09, lbl, fontsize=8, color='#888', transform=ax_info.transAxes)
    t = ax_info.text(0.08, ypos, '0.00', fontsize=16, fontweight='bold',
                     color='#222', transform=ax_info.transAxes)
    metric_texts.append(t)


# WHEEL BAR AXES
def setup_bar_ax(ax, label):
    ax.set_xlim(-1, 1)
    ax.set_ylim(0, 1)
    ax.set_facecolor('#eeede8')
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.axvline(0, color='#bbb', linewidth=1, zorder=1)
    ax.text(-1.0, 0.5, label, va='center', fontsize=9, color='#888')
    bar = ax.barh(0.5, 0, height=0.5, color='#333', alpha=0.45, zorder=2)[0]
    val_text = ax.text(0.98, 0.5, '0.00', va='center', ha='right',
                       fontsize=9, fontweight='bold', color='#333',
                       transform=ax.transAxes)
    return bar, val_text

bar_L, bval_L = setup_bar_ax(ax_wl, 'L')
bar_R, bval_R = setup_bar_ax(ax_wr, 'R')


# SLIDER
slider = Slider(ax_slide, 'ω_max (rad/s)', OMEGA_MAX_MIN, OMEGA_MAX_MAX,
                valinit=OMEGA_MAX_INIT, valstep=0.1, color='#aaa')
slider.label.set_fontsize(9)
slider.label.set_color('#666')
slider.valtext.set_fontsize(9)
slider.valtext.set_color('#444')

def on_slider(val):
    global omega_max
    omega_max = slider.val

slider.on_changed(on_slider)


# RESET BUTTON
ax_reset = fig.add_axes([0.78, 0.08, 0.10, 0.05])
from matplotlib.widgets import Button
reset_btn = Button(ax_reset, 'reset path', color='#e8e7e2', hovercolor='#d8d7d2')
reset_btn.label.set_fontsize(9)

def on_reset(event):
    global path_x, path_y
    state['x'] = 0.0; state['y'] = 0.0; state['theta'] = 0.0
    path_x = [0.0]; path_y = [0.0]

reset_btn.on_clicked(on_reset)


# JOYSTICK MOUSE INTERACTION
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
    dx, dy = clamp_to_circle(event.xdata, event.ydata)
    joy['x'] = dx / JOYSTICK_RADIUS
    joy['y'] = dy / JOYSTICK_RADIUS
    joy_thumb.set_center((dx, dy))

def on_move(event):
    if not joy['dragging'] or event.inaxes != ax_joy:
        return
    dx, dy = clamp_to_circle(event.xdata or 0, event.ydata or 0)
    joy['x'] = dx / JOYSTICK_RADIUS
    joy['y'] = dy / JOYSTICK_RADIUS
    joy_thumb.set_center((dx, dy))

def on_release(event):
    joy['dragging'] = False
    joy['x'] = 0.0; joy['y'] = 0.0
    joy_thumb.set_center((0, 0))

fig.canvas.mpl_connect('button_press_event',   on_press)
fig.canvas.mpl_connect('motion_notify_event',  on_move)
fig.canvas.mpl_connect('button_release_event', on_release)


# KEYBOARD CONTROL (WASD / arrow keys)
key_state = {'w': False, 'a': False, 's': False, 'd': False}

def on_key_press(event):
    k = event.key.lower() if event.key else ''
    if k in ('w', 'up'):    key_state['w'] = True
    if k in ('s', 'down'):  key_state['s'] = True
    if k in ('a', 'left'):  key_state['a'] = True
    if k in ('d', 'right'): key_state['d'] = True

def on_key_release(event):
    k = event.key.lower() if event.key else ''
    if k in ('w', 'up'):    key_state['w'] = False
    if k in ('s', 'down'):  key_state['s'] = False
    if k in ('a', 'left'):  key_state['a'] = False
    if k in ('d', 'right'): key_state['d'] = False

fig.canvas.mpl_connect('key_press_event',   on_key_press)
fig.canvas.mpl_connect('key_release_event', on_key_release)


def keys_to_joy():
    jx = (1.0 if key_state['d'] else 0.0) - (1.0 if key_state['a'] else 0.0)
    jy = (1.0 if key_state['w'] else 0.0) - (1.0 if key_state['s'] else 0.0)
    if jx != 0 and jy != 0:
        jx *= 0.7071; jy *= 0.7071
    return jx, jy


# ANIMATION
def update(frame):
    kx, ky = keys_to_joy()
    if not joy['dragging']:
        jx, jy = kx, ky
        joy_thumb.set_center((jx * JOYSTICK_RADIUS, jy * JOYSTICK_RADIUS))
    else:
        jx, jy = joy['x'], joy['y']

    omL, omR = joystick_to_wheel_speeds(jx, jy, omega_max)
    v     = r * (omL + omR) / 2.0
    omega = r * (omR - omL) / b

    state['x']     += v * np.cos(state['theta']) * dt
    state['y']     += v * np.sin(state['theta']) * dt
    state['theta'] += omega * dt

    if abs(jx) > 0.01 or abs(jy) > 0.01:
        path_x.append(state['x'])
        path_y.append(state['y'])
        if len(path_x) > MAX_PATH_POINTS:
            path_x.pop(0); path_y.pop(0)

    # trajectory
    traj_line.set_data(path_x, path_y)
    chair_dot.set_data([state['x']], [state['y']])
    hx = state['x'] + HEADING_LEN * np.cos(state['theta'])
    hy = state['y'] + HEADING_LEN * np.sin(state['theta'])
    heading_line.set_data([state['x'], hx], [state['y'], hy])

    all_x = path_x + [state['x']]
    all_y = path_y + [state['y']]
    mx, Mx = min(all_x), max(all_x)
    my, My = min(all_y), max(all_y)
    pad = max(0.8, 0.15 * max(Mx - mx, My - my, 0.1))
    ax_traj.set_xlim(mx - pad, Mx + pad)
    ax_traj.set_ylim(my - pad, My + pad)

    # metrics
    metric_texts[0].set_text(f'{v:+.2f}')
    metric_texts[1].set_text(f'{omega:+.2f}')
    metric_texts[2].set_text(f'{omL:+.2f}')
    metric_texts[3].set_text(f'{omR:+.2f}')

    # wheel bars
    for bar, val, btext in [(bar_L, omL, bval_L), (bar_R, omR, bval_R)]:
        norm = val / omega_max
        bar.set_width(norm)
        bar.set_x(min(0, norm))
        btext.set_text(f'{val:+.2f}')

    return (traj_line, chair_dot, heading_line, joy_thumb,
            *metric_texts, bar_L, bar_R)


ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

fig.suptitle('Differential-Drive Wheelchair — Joystick Simulator',
             fontsize=12, color='#444', y=0.97)

plt.show()