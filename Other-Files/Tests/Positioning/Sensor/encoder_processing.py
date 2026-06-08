import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# --- CONFIGURATION: PROPERTIES AND LOG FILES ---
TICKS_PER_REV = 20         # Adjust to match the number of magnets on your wheel hub
D_TRUE_STRAIGHT = 5.00     # Known physical distance pushed in meters

FILE_STRAIGHT = "straight_push_test.csv"
FILE_SPIN = "spin_turn_test.csv"
# -----------------------------------------------

def parse_arduino_log(filepath):
    """Parses custom serial stream filtering out STATUS or FORMAT lines."""
    timestamps = []
    left_ticks = []
    right_ticks = []
    yaw_angles = []
    
    with open(filepath, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if parts[0] == "DATA" and len(parts) == 9:
                timestamps.append(float(parts[1]))
                left_ticks.append(float(parts[2]))
                right_ticks.append(float(parts[3]))
                yaw_angles.append(float(parts[6])) # yaw_deg
                
    df = pd.DataFrame({
        'time_ms': timestamps,
        'left_ticks': left_ticks,
        'right_ticks': right_ticks,
        'yaw': yaw_angles
    })
    return df

# =====================================================================
# PHASE 1: EFFECTIVE ROLLING RADIUS CALIBRATION (R)
# =====================================================================
print("--- Processing Phase 1: Effective Wheel Rolling Radius ---")
df_straight = parse_arduino_log(FILE_STRAIGHT)

# Extract net tick delta from the start to the stop of the test
delta_ticks_L = abs(df_straight['left_ticks'].iloc[-1] - df_straight['left_ticks'].iloc[0])
delta_ticks_R = abs(df_straight['right_ticks'].iloc[-1] - df_straight['right_ticks'].iloc[0])
avg_ticks = (delta_ticks_L + delta_ticks_R) / 2.0

# Calculate effective radius: D_true = (avg_ticks / TICKS_PER_REV) * 2 * pi * R
# Therefore: R = (D_true * TICKS_PER_REV) / (2 * pi * avg_ticks)
R_eff = (D_TRUE_STRAIGHT * TICKS_PER_REV) / (2.0 * np.pi * avg_ticks)
quantization_error = (2.0 * np.pi * R_eff) / TICKS_PER_REV

print(f"Total Left Ticks:  {delta_ticks_L}")
print(f"Total Right Ticks: {delta_ticks_R}")
print(f"Calculated Effective Wheel Radius (R): {R_eff:.4f} meters")
print(f"Sensor Quantization Step Size (Q):    {quantization_error:.4f} meters per tick\n")

# =====================================================================
# PHASE 2: EFFECTIVE TRACK WIDTH CALIBRATION (B)
# =====================================================================
print("--- Processing Phase 2: Effective Kinematic Track Width ---")
df_spin = parse_arduino_log(FILE_SPIN)

# Unwrap the IMU yaw to accumulate total structural rotation beyond 360 degrees
raw_yaw_rad = np.radians(df_spin['yaw'].values)
unwrapped_yaw_rad = np.unwrap(raw_yaw_rad)
total_imu_delta_theta = abs(unwrapped_yaw_rad[-1] - unwrapped_yaw_rad[0])

# Compute total distance traveled by each wheel using our newly calibrated R
spin_ticks_L = df_spin['left_ticks'].iloc[-1] - df_spin['left_ticks'].iloc[0]
spin_ticks_R = df_spin['right_ticks'].iloc[-1] - df_spin['right_ticks'].iloc[0]

distance_L = (spin_ticks_L / TICKS_PER_REV) * 2.0 * np.pi * R_eff
distance_R = (spin_ticks_R / TICKS_PER_REV) * 2.0 * np.pi * R_eff

# Kinematic equation for spin in place: delta_theta = (distance_R - distance_L) / B
# Therefore: B = (distance_R - distance_L) / total_imu_delta_theta
B_eff = abs(distance_R - distance_L) / total_imu_delta_theta

print(f"Total Turn IMU Rotation: {np.degrees(total_imu_delta_theta):.2f} degrees")
print(f"Calculated Effective Track Width (B): {B_eff:.4f} meters")

# =====================================================================
# PHASE 3: GRAPHING THE ODOMETRY VS IMU CORRELATION
# =====================================================================
# Calculate dead-reckoned heading profile across the spin test using B_eff
calculated_distances_L = ((df_spin['left_ticks'] - df_spin['left_ticks'].iloc[0]) / TICKS_PER_REV) * 2.0 * np.pi * R_eff
calculated_distances_R = ((df_spin['right_ticks'] - df_spin['right_ticks'].iloc[0]) / TICKS_PER_REV) * 2.0 * np.pi * R_eff

encoder_yaw_rad = (calculated_distances_R - calculated_distances_L) / B_eff
encoder_yaw_deg = np.degrees(encoder_yaw_rad)
imu_yaw_deg = np.degrees(unwrapped_yaw_rad - unwrapped_yaw_rad[0])

plt.figure(figsize=(10, 5))
time_axis_sec = (df_spin['time_ms'] - df_spin['time_ms'].iloc[0]) / 1000.0
plt.plot(time_axis_sec, imu_yaw_deg, label='BNO085 Ground Truth (Game Rotation Vector)', linewidth=2)
plt.plot(time_axis_sec, encoder_yaw_deg, '--', label='Calibrated Wheel Odometry Projection', alpha=0.8)

plt.title('Kinematic Track Width Calibration Profile')
plt.xlabel('Experiment Time [s]')
plt.ylabel('Accumulated Turn Rotation [Degrees]')
plt.grid(True, linestyle='--')
plt.legend()
plt.tight_layout()
plt.savefig('odometry_calibration_results.png', dpi=300)
print("\n[SUCCESS] Calibration analysis plot exported as 'odometry_calibration_results.png'")
plt.show()