# Autonomous Wheelchair System

## Overview

This project develops an autonomous smart wheelchair for users with limited motor control. The wheelchair operates in a known indoor environment: the user selects a destination, and the wheelchair navigates there on its own.

Navigation combines localization, path planning, and obstacle avoidance. The architecture is modular, so different input methods can be swapped in, including a graphical interface, eye-tracking, or EMG-based control.

## Setup

### Requirements

- Python 3.10 or 3.11
- NumPy 1.x (< 2.0)

### Vosk speech model

Download `vosk-model-en-us-0.22` from [alphacephei.com/vosk/models](https://alphacephei.com/vosk/models) and extract it into the `Models` folder inside the repository. The structure should look like this:

```
project/
└── UserInput/
    └── Voice control & GUI/
        ├── main.py
        └── Models/
            └── vosk-model-en-us-0.22/
```

### Virtual environment

It is recommended to run everything inside a virtual environment:

```bash
python -m venv venv
venv\Scripts\activate           # Windows (Command Prompt)
./venv/Scripts/Activate.ps1     # Windows (PowerShell)
source venv/bin/activate        # Linux / macOS
```

### Install dependencies

Install PyTorch (CPU build) first:

```bash
pip install torch torchaudio --index-url https://download.pytorch.org/whl/cpu
```

Then install the rest:

```bash
pip install -r requirements.txt
pip install deepfilternet
```

Everything else the system needs is listed in `requirements.txt`.

## Key Features

- Autonomous navigation to a user-selected destination
- Real-time localization within a known indoor map
- Obstacle detection and collision avoidance
- Multiple user input methods: GUI, eye-tracking, EMG, and manual control
- Modular system design

## System Architecture

The system has four main subsystems.

The **user input subsystem** handles interaction and destination selection. The **positioning subsystem** takes care of map generation and localization. The **navigation subsystem** computes and executes collision-free paths. The **integration subsystem** manages communication, power, and coordination between all components.

## Reports and Documentation

- Literature Study: https://www.overleaf.com/read/pxfggznvhxvc#7b4c1a
- Program of Requirements (PoR): https://www.overleaf.com/read/cvwwdwxjmvfz#97bd66
- Meeting Notes: https://docs.google.com/document/d/1JzAj3k3fk30Rmm3bZLfRvXuysjHTRjCx-jTIksM33J0/edit?tab=t.0
- Gantt Chart: https://docs.google.com/spreadsheets/d/1LeIonz3t87s3dJxeFGJFFnhabDehzJCSFnnsxjkrOPM/edit?gid=1115838130#gid=1115838130
- Wheelchair Measurements: https://docs.google.com/spreadsheets/d/1bqGMkIiJ7xfXbivKcnknhoT9pYHNBYwShCx7t04MR1Q/edit?usp=sharing
- Plan B: https://www.overleaf.com/read/nyvbtqkbtnqg#3270e9
- Verification: https://www.overleaf.com/6864522186prfshsymjchx#0db520
- Design Report: https://www.overleaf.com/read/fdnncktpzdds#f19f40

## Team

BAP 2026 – Group nA6

- Ethan Croeze
- Rudrh Kapoor
- Ansh Kaushal
- Omar Shousha
- Guido Nuijt
- Dyorno Pavion

## License

This project is developed for academic purposes at TU Delft.
