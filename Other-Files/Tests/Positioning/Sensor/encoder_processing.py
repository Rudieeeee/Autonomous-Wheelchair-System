import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


TICKS_PER_REV = 20

D_TRUE_STRAIGHT = 5.00

R_TRUE = 0.175
B_TRUE = 0.550

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

    return df.reset_index(drop=True)


def zero_data(df):
    df = df.copy().reset_index(drop=True)

    df["time_ms"] = np.subtract(
        df["time_ms"],
        df["time_ms"].iloc[0]
    )

    df["left_ticks_raw"] = df["left_ticks"].copy()
    df["right_ticks_raw"] = df["right_ticks"].copy()

    df["left_ticks"] = np.subtract(
        df["left_ticks"],
        df["left_ticks"].iloc[0]
    )

    df["right_ticks"] = np.subtract(
        df["right_ticks"],
        df["right_ticks"].iloc[0]
    )

    yaw_rad = np.radians(df["yaw_deg"].values)
    yaw_unwrapped_rad = np.unwrap(yaw_rad)

    df["yaw_zeroed_deg"] = np.degrees(
        np.subtract(
            yaw_unwrapped_rad,
            yaw_unwrapped_rad[0]
        )
    )

    return df


def ticks_to_distance(ticks, wheel_radius):
    return np.multiply(
        np.divide(ticks, TICKS_PER_REV),
        2.0 * np.pi * wheel_radius
    )


def radius_from_right_ticks(distance_m, right_ticks):
    if abs(right_ticks) == 0:
        raise ValueError("Right encoder has zero ticks. Radius cannot be calculated.")

    return np.divide(
        distance_m * TICKS_PER_REV,
        2.0 * np.pi * abs(right_ticks)
    )


def percent_error(estimated, measured):
    if measured == 0:
        return np.nan

    return np.multiply(
        np.divide(
            np.subtract(estimated, measured),
            measured
        ),
        100.0
    )


def expected_ticks_for_distance(distance_m, wheel_radius):
    wheel_circumference = 2.0 * np.pi * wheel_radius

    revolutions = np.divide(
        distance_m,
        wheel_circumference
    )

    return np.multiply(
        revolutions,
        TICKS_PER_REV
    )


print()
print("Processing straight drive using right encoder only")
print()

df_straight_raw = load_encoder_csv(FILE_STRAIGHT)
df_straight = zero_data(df_straight_raw)

right_ticks_straight = abs(df_straight["right_ticks"].iloc[-1])
left_ticks_straight = abs(df_straight["left_ticks"].iloc[-1])

R_eff = radius_from_right_ticks(
    D_TRUE_STRAIGHT,
    right_ticks_straight
)

R_error = np.subtract(R_eff, R_TRUE)
R_error_percent = percent_error(R_eff, R_TRUE)

expected_right_ticks = expected_ticks_for_distance(
    D_TRUE_STRAIGHT,
    R_TRUE
)

print(f"Left ticks ignored:                {left_ticks_straight}")
print(f"Right ticks used:                  {right_ticks_straight}")
print(f"Expected right ticks:              {expected_right_ticks:.2f}")
print()
print(f"Measured wheel radius:             {R_TRUE:.4f} m")
print(f"Estimated wheel radius:            {R_eff:.4f} m")
print(f"Wheel radius error:                {R_error:.4f} m")
print(f"Wheel radius error percentage:     {R_error_percent:.2f} percent")
print()


right_distance_est = ticks_to_distance(
    df_straight["right_ticks"],
    R_eff
)

time_straight_sec = np.divide(
    df_straight["time_ms"],
    1000.0
)

plt.figure(figsize=(10, 5))

plt.plot(
    time_straight_sec,
    right_distance_est,
    label="Estimated encoder distance using right encoder",
    linewidth=2
)

plt.axhline(
    y=D_TRUE_STRAIGHT,
    linestyle=":",
    label="Measured final distance"
)

plt.title("Estimated Distance From Right Encoder")
plt.xlabel("Experiment time [s]")
plt.ylabel("Distance [m]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("right_encoder_distance.png", dpi=300)
plt.show()


print("Processing spin test")
print()

df_spin_raw = load_encoder_csv(FILE_SPIN)
df_spin = zero_data(df_spin_raw)

spin_right_ticks = df_spin["right_ticks"].iloc[-1]
spin_left_ticks = df_spin["left_ticks"].iloc[-1]

imu_yaw_deg = df_spin["yaw_zeroed_deg"].values
imu_final_yaw_deg = imu_yaw_deg[-1]

print(f"Spin left ticks ignored:           {spin_left_ticks}")
print(f"Spin right ticks used:             {spin_right_ticks}")
print()
print("Encoder based wheelbase estimate is not valid because the left encoder is corrupted.")
print(f"Measured wheelbase used instead:   {B_TRUE:.4f} m")
print()


right_distance_spin = ticks_to_distance(
    df_spin["right_ticks"],
    R_eff
)

left_distance_spin_assumed = np.multiply(
    right_distance_spin,
    -1.0
)

encoder_yaw_rad = np.divide(
    np.subtract(
        right_distance_spin,
        left_distance_spin_assumed
    ),
    B_TRUE
)

encoder_yaw_deg = np.degrees(encoder_yaw_rad)

if imu_final_yaw_deg < 0:
    encoder_yaw_deg = np.multiply(
        encoder_yaw_deg,
        -1.0
    )

encoder_final_yaw_deg = encoder_yaw_deg.iloc[-1]

yaw_error_deg = np.subtract(
    encoder_final_yaw_deg,
    imu_final_yaw_deg
)

yaw_error_percent = percent_error(
    encoder_final_yaw_deg,
    imu_final_yaw_deg
)

print(f"Final IMU yaw:                     {imu_final_yaw_deg:.2f} deg")
print(f"Final encoder yaw estimate:        {encoder_final_yaw_deg:.2f} deg")
print(f"Yaw error:                         {yaw_error_deg:.2f} deg")
print(f"Yaw error percentage:              {yaw_error_percent:.2f} percent")
print()


time_spin_sec = np.divide(
    df_spin["time_ms"],
    1000.0
)

plt.figure(figsize=(10, 5))

plt.plot(
    time_spin_sec,
    imu_yaw_deg,
    label="IMU yaw",
    linewidth=2
)

plt.plot(
    time_spin_sec,
    encoder_yaw_deg,
    linestyle=":",
    label="Encoder yaw estimate using right encoder and measured wheelbase",
    linewidth=2
)

plt.title("IMU Yaw Compared With Right Encoder Yaw Estimate")
plt.xlabel("Experiment time [s]")
plt.ylabel("Accumulated yaw [deg]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("imu_vs_right_encoder_yaw.png", dpi=300)
plt.show()


print("Summary")
print()
print(f"Wheel radius measured:             {R_TRUE:.4f} m")
print(f"Wheel radius estimated:            {R_eff:.4f} m")
print(f"Wheel radius error percentage:     {R_error_percent:.2f} percent")
print()
print(f"Wheelbase measured:                {B_TRUE:.4f} m")
print("Wheelbase estimated:               not valid with only one working encoder")
print()
print("Saved plot: right_encoder_distance.png")
print("Saved plot: imu_vs_right_encoder_yaw.png")