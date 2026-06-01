import serial
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import welch, detrend, find_peaks

PORT = "COM10"          # Change this: e.g. "/dev/ttyACM0" on Linux
BAUD = 460800
LOG_SECONDS = 60       # record duration
FS_EXPECTED = 400.0     # your Arduino outputs every 20 ms = 50 Hz

columns = [
    "time_ms",
    "gyro_x", "gyro_y", "gyro_z",
    "accel_x", "accel_y", "accel_z",
    "yaw_deg", "pitch_deg", "roll_deg",
    "cal_sys", "cal_gyro", "cal_accel", "cal_mag"
]

filename = f"bno055_log_{int(time.time())}.csv"

rows = []

print(f"Logging from {PORT} for {LOG_SECONDS} seconds...")

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    start = time.time()

    while time.time() - start < LOG_SECONDS:
        line = ser.readline().decode(errors="ignore").strip()

        if not line.startswith("DATA,"):
            print(line)
            continue

        parts = line.split(",")

        if len(parts) != 15:
            continue

        try:
            values = [float(x) for x in parts[1:]]
            rows.append(values)
        except ValueError:
            continue

df = pd.DataFrame(rows, columns=columns)
df.to_csv(filename, index=False)

print(f"Saved {len(df)} samples to {filename}")

# -----------------------------
# Timing analysis
# -----------------------------

t = df["time_ms"].to_numpy() / 1000.0
dt = np.diff(t)

fs_measured = 1.0 / np.mean(dt)

print()
print("Timing:")
print(f"Measured sample rate: {fs_measured:.3f} Hz")
print(f"Mean dt: {np.mean(dt)*1000:.3f} ms")
print(f"Std dt jitter: {np.std(dt)*1000:.3f} ms")

# -----------------------------
# Derived vibration signals
# -----------------------------

df["accel_mag"] = np.sqrt(
    df["accel_x"]**2 + df["accel_y"]**2 + df["accel_z"]**2
)

df["gyro_mag"] = np.sqrt(
    df["gyro_x"]**2 + df["gyro_y"]**2 + df["gyro_z"]**2
)

signals = {
    "accel_x": df["accel_x"].to_numpy(),
    "accel_y": df["accel_y"].to_numpy(),
    "accel_z": df["accel_z"].to_numpy(),
    "accel_mag": df["accel_mag"].to_numpy(),
    "gyro_x": df["gyro_x"].to_numpy(),
    "gyro_y": df["gyro_y"].to_numpy(),
    "gyro_z": df["gyro_z"].to_numpy(),
    "gyro_mag": df["gyro_mag"].to_numpy(),
    "yaw_deg": df["yaw_deg"].to_numpy(),
    "pitch_deg": df["pitch_deg"].to_numpy(),
    "roll_deg": df["roll_deg"].to_numpy(),
}

# -----------------------------
# FFT / PSD analysis
# -----------------------------

def analyze_signal(name, x, fs):
    x = np.asarray(x)
    x = x - np.mean(x)
    x = detrend(x)

    freqs, psd = welch(
        x,
        fs=fs,
        nperseg=min(1024, len(x)),
        scaling="density"
    )

    # Ignore DC and very low drift
    mask = freqs > 0.5

    peak_indices, properties = find_peaks(
        psd[mask],
        prominence=np.max(psd[mask]) * 0.05
    )

    peak_freqs = freqs[mask][peak_indices]
    peak_values = psd[mask][peak_indices]

    order = np.argsort(peak_values)[::-1]

    print()
    print(f"Strong frequency peaks for {name}:")

    for i in order[:5]:
        print(f"  {peak_freqs[i]:.2f} Hz")

    plt.figure()
    plt.semilogy(freqs, psd)
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Power spectral density")
    plt.title(f"PSD: {name}")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"psd_{name}.png", dpi=150)
    plt.close()

for name, x in signals.items():
    analyze_signal(name, x, fs_measured)

# -----------------------------
# Time-domain plots
# -----------------------------

plt.figure()
plt.plot(t, df["accel_x"], label="accel_x")
plt.plot(t, df["accel_y"], label="accel_y")
plt.plot(t, df["accel_z"], label="accel_z")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s²]")
plt.title("Acceleration over time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("acceleration_time.png", dpi=150)
plt.close()

plt.figure()
plt.plot(t, df["yaw_deg"], label="yaw")
plt.plot(t, df["pitch_deg"], label="pitch")
plt.plot(t, df["roll_deg"], label="roll")
plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")
plt.title("Fused orientation over time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("orientation_time.png", dpi=150)
plt.close()

print()
print("Analysis complete.")
print("Look at psd_accel_mag.png, psd_gyro_mag.png, psd_yaw_deg.png, psd_pitch_deg.png, psd_roll_deg.png")