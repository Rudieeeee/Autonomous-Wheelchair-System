

import allantools
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import welch

# --- CONFIGURATION: MATCH FILENAMES FOR PROCESSING ---
FILE_BNO055 = "bno055_center.csv"
FILE_BNO085 = "bno085_center.csv"
# -----------------------------------------------------

def analyze_imu_data(filepath, label):
    # Read the logged data
    df = pd.read_csv(filepath).dropna()
    
    # Identify the real sampling rate
    time_diffs = np.diff(df['timestamp_ms'].values) / 1000.0  # convert to seconds
    fs = 1.0 / np.mean(time_diffs)
    print(f"[{label}] Measured Sampling Rate: {fs:.2f} Hz")
    
    # Calculate Magnetometer Vector Magnitude (Total Field Strength)
    mx, my, mz = df['mx'].values, df['my'].values, df['mz'].values
    mag_magnitude = np.sqrt(mx**2 + my**2 + mz**2)
    
    # 1. Power Spectral Density (PSD)
    frequencies, psd_values = welch(mag_magnitude, fs=fs, nperseg=1024)
    
    # 2. Allan Deviation using industry-standard 'allantools'
    # data_type='freq' handles standard rate/measurement time series
    taus, adev_values, _, _ = allantools.adev(mag_magnitude, rate=fs, data_type='freq', taus='octave')
    
    return frequencies, psd_values, taus, adev_values

# Process both log outputs
freq_055, psd_055, tau_055, adev_055 = analyze_imu_data(FILE_BNO055, "BNO055")
freq_085, psd_085, tau_085, adev_085 = analyze_imu_data(FILE_BNO085, "BNO085")

# --- PLOTTING ---
plt.figure(figsize=(14, 6))

# Subplot 1: Power Spectral Density
plt.subplot(1, 2, 1)
plt.loglog(freq_055, psd_055, label='BNO055 (Raw Data Mode)')
plt.loglog(freq_085, psd_085, label='BNO085 (Calibrated Report Mode)', alpha=0.8)
plt.title('Magnetometer Vector Power Spectral Density')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Power Density [uT^2/Hz]')
plt.grid(True, which="both", ls="--")
plt.legend()

# Subplot 2: Allan Deviation via allantools library
plt.subplot(1, 2, 2)
plt.loglog(tau_055, adev_055, 'o-', label='BNO055 (allantools calculation)')
plt.loglog(tau_085, adev_085, 's-', label='BNO085 (allantools calculation)', alpha=0.8)
plt.title('Magnetometer Allan Deviation')
plt.xlabel('Averaging Time Tau [s]')
plt.ylabel('Allan Deviation Sigma(tau) [uT]')
plt.grid(True, which="both", ls="--")
plt.legend()

plt.tight_layout()
plt.savefig('imu_allantools_comparison.png', dpi=300)
print("\n[SUCCESS] Graph saved as 'imu_allantools_comparison.png'")
plt.show()
