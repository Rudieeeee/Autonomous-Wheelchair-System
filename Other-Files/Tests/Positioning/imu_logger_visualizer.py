#!/usr/bin/env python3

import serial
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import welch, detrend, find_peaks

PORT = "/dev/ttyACM0"
BAUD = 460800
LOG_SECONDS = 60

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

try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2.0)
        ser.reset_input_buffer()

        start = time.time()

        while time.time() - start < LOG_SECONDS:
            line = ser.readline().decode(errors="ignore").strip()

            if not line:
                continue

            if not line.startswith("DATA,"):
                print(line)
                continue

            parts = line.split(",")

            if len(parts) != 15:
                print(f"Skipping malformed line: {line}")
                continue

            try:
                values = [float(x) for x in parts[1:]]
                rows.append(values)
            except ValueError:
                print(f"Skipping invalid numbers: {line}")
                continue

except serial.SerialException as e:
    print()
    print(f"Serial error: {e}")
    print("Check the port with:")
    print("  ls /dev/ttyACM* /dev/ttyUSB*")
    raise SystemExit(1)

if len(rows) < 8:
    print()
    print(f"Only received {len(rows)} valid DATA rows.")
    print("No analysis was done.")
    raise SystemExit(1)

df = pd.DataFrame(rows, columns=columns)
df.to_csv(filename, index=False)

print(f"Saved {len(df)} samples to {filename}")

# -----------------------------
# Timing analysis
# -----------------------------

t_raw = df["time_ms"].to_numpy()
t_raw = t_raw - t_raw[0]

raw_duration = t_raw[-1]

# Auto-detect timestamp unit.
# If your Arduino uses micros(), this fixes the wrong 0.080 Hz result.
if raw_duration > LOG_SECONDS * 10000:
    print("Detected timestamp unit: microseconds")
    t = t_raw / 1_000_000.0
elif raw_duration > LOG_SECONDS * 10:
    print("Detected timestamp unit: milliseconds")
    t = t_raw / 1000.0
else:
    print("Detected timestamp unit: seconds")
    t = t_raw

dt = np.diff(t)
dt = dt[dt > 0]

if len(dt) == 0:
    print("Could not calculate timing because timestamps did not increase.")
    raise SystemExit(1)

fs_measured = 1.0 / np.mean(dt)

print()
print("Timing:")
print(f"Measured sample rate: {fs_measured:.3f} Hz")
print(f"Mean dt: {np.mean(dt) * 1000:.3f} ms")
print(f"Std dt jitter: {np.std(dt) * 1000:.3f} ms")

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

    if len(x) < 8:
        print()
        print(f"Strong frequency peaks for {name}:")
        print("  Not enough samples.")
        return

    x = x - np.mean(x)
    x = detrend(x)

    nperseg = min(1024, len(x))

    freqs, psd = welch(
        x,
        fs=fs,
        nperseg=nperseg,
        scaling="density"
    )

    mask = freqs > 0.5

    print()
    print(f"Strong frequency peaks for {name}:")

    if not np.any(mask):
        print("  No frequency range above 0.5 Hz available.")
    else:
        psd_masked = psd[mask]

        if len(psd_masked) == 0 or np.max(psd_masked) <= 0:
            print("  No clear peaks found.")
        else:
            peak_indices, properties = find_peaks(
                psd_masked,
                prominence=np.max(psd_masked) * 0.05
            )

            peak_freqs = freqs[mask][peak_indices]
            peak_values = psd_masked[peak_indices]

            if len(peak_freqs) == 0:
                print("  No clear peaks found.")
            else:
                order = np.argsort(peak_values)[::-1]

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
print("Generated files:")
print(f"  {filename}")
print("  acceleration_time.png")
print("  orientation_time.png")
print("  psd_accel_mag.png")
print("  psd_gyro_mag.png")
print("  psd_yaw_deg.png")
print("  psd_pitch_deg.png")
print("  psd_roll_deg.png")