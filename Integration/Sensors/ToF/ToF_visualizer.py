import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

# Change this to your Arduino/XIAO serial port
# Windows example: "COM5"
# Linux example: "/dev/ttyUSB0"
# macOS example: "/dev/cu.usbmodemXXXX"
SERIAL_PORT = "COM17"
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# 8x8 grid positions
x = np.arange(8)
y = np.arange(8)
X, Y = np.meshgrid(x, y)

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

def read_frame():
    rows = []

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        # Ignore startup/status lines
        if line.startswith("Starting") or line.startswith("STATUS") or line.startswith("FORMAT"):
            continue

        # End of one frame
        if line.startswith("---"):
            if len(rows) == 8:
                return np.array(rows, dtype=float)
            else:
                rows = []
                continue

        parts = line.split()

        if len(parts) == 8:
            try:
                row = [float(p) for p in parts]
                rows.append(row)
            except ValueError:
                rows = []

        if len(rows) > 8:
            rows = []

while True:
    Z = read_frame()

    ax.clear()

    # Optional: replace 0 values with NaN so they don't show as zero surface
    Z[Z <= 0] = np.nan

    ax.plot_surface(X, Y, Z, cmap=cm.viridis, edgecolor="k")

    ax.set_title("DFRobot 8x8 ToF Distance Map")
    ax.set_xlabel("Column")
    ax.set_ylabel("Row")
    ax.set_zlabel("Distance [mm]")

    # Adjust these to your sensor range
    ax.set_zlim(0, 3000)

    plt.pause(0.05)