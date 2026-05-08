# Smart Wheelchair Multi-Model Simulator

Run from the `wheelchair_sim` folder:

```bash
python3 main.py
```

Install dependencies with either:

```bash
sudo apt install python3-numpy python3-matplotlib python3-pygame
```

or with a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r wheelchair_sim/requirements.txt
```

## Controls

Main menu:
- Click a model button or press 1-5.

Simulation:
- `M` switches between predefined and joystick input mode.
- Drag the on-screen joystick in joystick mode. Up/down controls linear velocity, left/right controls angular velocity.
- `P` closes the Pygame GUI and opens Matplotlib plots.
- `SPACE` pauses/resumes.
- `R` resets.
- `ESC` returns to the main menu.

## Models included

1. Level 1 pure kinematic model.
2. Level 3 first-order velocity dynamics model.
3. Level 4 force/torque body dynamics from wheel speeds.
4. Level 4+ torque/slip/resistance model.
5. Advanced motor command, wheel dynamics, slip traction model.

Predefined mode stops after the model simulation time. Joystick mode keeps running until you quit, reset, return to the menu, or open plots.

## SANGO tuned version

This version uses SANGO advanced SEGO starting parameters. See `SANGO_PARAMETERS.md` for the values that were changed and which values still need real measurement/tuning.


## Input change in this version

All models now receive the same high-level input: `v_cmd` in m/s and `omega_cmd` in rad/s. The predefined mode generates those two commands directly. The joystick mode also generates those two commands directly. More advanced models internally convert the command into wheel speeds, torques, or motor commands so their extra dynamics and slip effects remain active.
