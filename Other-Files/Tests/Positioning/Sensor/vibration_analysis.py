import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import welch

# --- CONFIGURATION: SET RUN FILENAMES HERE ---
FILE_VIBRATION_055 = "vibration_bno055.csv"
FILE_VIBRATION_085 = "vibration_bno085.csv"
# ---------------------------------------------

def analyze_structural_vibration(filepath, label):
    # Load dataset and drop empty rows
    df = pd.read_csv(filepath).dropna()
    
    # Calculate operational sampling frequency (fs)
    timestamps_sec = df['timestamp_ms'].values / 1000.0
    dt = np.mean(np.diff(timestamps_sec))
    fs = 1.0 / dt
    print(f"[{label}] Derived Sampling Rate: {fs:.2f} Hz (Nyquist Limit: {fs/2:.2f} Hz)")
    
    # Extract linear acceleration components
    ax, ay, az = df['ax'].values, df['ay'].values, df['az'].values
    
    # Calculate the magnitude of the acceleration vector
    acc_magnitude = np.sqrt(ax**2 + ay**2 + az**2)
    
    # Strip gravity/DC offset by subtracting the mean 
    # This leaves pure dynamic vibrational changes
    vibration_signal = acc_magnitude - np.mean(acc_magnitude)
    
    # Compute Power Spectral Density using Welch's method
    # nperseg limits resolution but stabilizes variance across short windows
    frequencies, psd_values = welch(vibration_signal, fs=fs, nperseg=512)
    
    return frequencies, psd_values, vibration_signal, timestamps_sec

# Execute parsing pipelines
freq_055, psd_055, vib_055, time_055 = analyze_structural_vibration(FILE_VIBRATION_055, "BNO055")
freq_085, psd_085, vib_085, time_085 = analyze_structural_vibration(FILE_VIBRATION_085, "BNO085")

# --- VISUALIZATION GRAPHING GENERATION ---
plt.figure(figsize=(14, 6))

# Plot 1: Time-Domain Raw Vibration Profile
plt.subplot(1, 2, 1)
plt.plot(time_055 - time_055[0], vib_055, label='BNO055 Chassis Shake', alpha=0.7)
plt.plot(time_085 - time_085[0], vib_085, label='BNO085 Chassis Shake', alpha=0.7)
plt.title('Chassis Dynamic Vibration (Time Domain)')
plt.xlabel('Elapsed Experiment Time [s]')
plt.ylabel('Dynamic Acceleration [m/s^2]')
plt.grid(True, linestyle='--')
plt.legend()

# Plot 2: Frequency-Domain Power Spectral Density Profile
plt.subplot(1, 2, 2)
plt.semilogy(freq_055, psd_055, label='BNO055 Frequency Profile')
plt.semilogy(freq_085, psd_085, label='BNO085 Frequency Profile', alpha=0.8)
plt.title('Vibration Power Spectral Density')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Power Spectral Density [(m/s^2)^2/Hz]')
plt.grid(True, which="both", linestyle='--')
plt.legend()

plt.tight_layout()
plt.savefig('chassis_vibration_analysis.png', dpi=300)
print("\n[SUCCESS] Structural vibration plots saved as 'chassis_vibration_analysis.png'")
plt.show()