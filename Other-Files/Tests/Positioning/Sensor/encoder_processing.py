import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
TICKS_PER_REV = 20
D_TRUE_STRAIGHT = 5.00

FILE_STRAIGHT = r"C:\Users\Paion\EE\BEP\Autonomous-Wheelchair-System\Other-Files\Tests\Positioning\Sensor\forward_drive_test5m.csv"
FILE_SPIN = r"C:\Users\Paion\EE\BEP\Autonomous-Wheelchair-System\Other-Files\Tests\Positioning\Sensor\spin_test.csv"


def load_encoder_csv(filepath):
    df = pd.read_csv(filepath)

    required_cols = [
        "time_ms",
        "left_ticks",
        "right_ticks",
        "yaw_deg",
    ]

    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        raise ValueError(f"Missing columns in {filepath}: {missing}")

    df = df.dropna(subset=required_cols)

    if df.empty:
        raise ValueError(f"No valid rows found in {filepath}")

    return df


# =====================================================================
# PHASE 1: EFFECTIVE ROLLING RADIUS CALIBRATION
# =====================================================================
print("--- Processing Phase 1: Effective Wheel Rolling Radius ---")

df_straight = load_encoder_csv(FILE_STRAIGHT)

delta_ticks_L = abs(df_straight["left_ticks"].iloc[-1] - df_straight["left_ticks"].iloc[0])
delta_ticks_R = abs(df_straight["right_ticks"].iloc[-1] - df_straight["right_ticks"].iloc[0])
avg_ticks = (delta_ticks_L + delta_ticks_R) / 2.0

if avg_ticks == 0:
    raise ValueError("Straight test has zero encoder ticks. Wheel radius cannot be calculated.")

R_eff = (D_TRUE_STRAIGHT * TICKS_PER_REV) / (2.0 * np.pi * avg_ticks)
quantization_error = (2.0 * np.pi * R_eff) / TICKS_PER_REV

print(f"Total Left Ticks:  {delta_ticks_L}")
print(f"Total Right Ticks: {delta_ticks_R}")
print(f"Calculated Effective Wheel Radius R: {R_eff:.4f} m")
print(f"Sensor Quantization Step Size Q:     {quantization_error:.4f} m/tick\n")


# =====================================================================
# PHASE 2: EFFECTIVE TRACK WIDTH CALIBRATION
# =====================================================================
print("--- Processing Phase 2: Effective Kinematic Track Width ---")

df_spin = load_encoder_csv(FILE_SPIN)

raw_yaw_rad = np.radians(df_spin["yaw_deg"].values)
unwrapped_yaw_rad = np.unwrap(raw_yaw_rad)

total_imu_delta_theta = abs(unwrapped_yaw_rad[-1] - unwrapped_yaw_rad[0])

if total_imu_delta_theta == 0:
    raise ValueError("Spin test has zero IMU yaw change. Track width cannot be calculated.")

spin_ticks_L = df_spin["left_ticks"].iloc[-1] - df_spin["left_ticks"].iloc[0]
spin_ticks_R = df_spin["right_ticks"].iloc[-1] - df_spin["right_ticks"].iloc[0]

distance_L = (spin_ticks_L / TICKS_PER_REV) * 2.0 * np.pi * R_eff
distance_R = (spin_ticks_R / TICKS_PER_REV) * 2.0 * np.pi * R_eff

B_eff = abs(distance_R - distance_L) / total_imu_delta_theta

print(f"Total Turn IMU Rotation: {np.degrees(total_imu_delta_theta):.2f} degrees")
print(f"Calculated Effective Track Width B: {B_eff:.4f} m")


# =====================================================================
# PHASE 3: GRAPHING ODOMETRY VS IMU
# =====================================================================
calculated_distances_L = (
    (df_spin["left_ticks"] - df_spin["left_ticks"].iloc[0])
    / TICKS_PER_REV
) * 2.0 * np.pi * R_eff

calculated_distances_R = (
    (df_spin["right_ticks"] - df_spin["right_ticks"].iloc[0])
    / TICKS_PER_REV
) * 2.0 * np.pi * R_eff

encoder_yaw_rad = (calculated_distances_R - calculated_distances_L) / B_eff
encoder_yaw_deg = np.degrees(encoder_yaw_rad)

imu_yaw_deg = np.degrees(unwrapped_yaw_rad - unwrapped_yaw_rad[0])

time_axis_sec = (df_spin["time_ms"] - df_spin["time_ms"].iloc[0]) / 1000.0

plt.figure(figsize=(10, 5))
plt.plot(time_axis_sec, imu_yaw_deg, label="BNO085 IMU yaw", linewidth=2)
plt.plot(time_axis_sec, encoder_yaw_deg, "--", label="Calibrated wheel odometry", alpha=0.8)

plt.title("Kinematic Track Width Calibration Profile")
plt.xlabel("Experiment Time [s]")
plt.ylabel("Accumulated Turn Rotation [deg]")
plt.grid(True, linestyle="--")
plt.legend()
plt.tight_layout()

plt.savefig("odometry_calibration_results.png", dpi=300)

print("\n[SUCCESS] Calibration plot exported as 'odometry_calibration_results.png'")
plt.show()