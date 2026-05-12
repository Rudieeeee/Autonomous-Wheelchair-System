import serial
import numpy as np
import matplotlib.pyplot as plt

PORT = "/dev/ttyACM0"
BAUDRATE = 115200

GRID_SIZE = 8
MAX_DISTANCE_MM = 4000


def parse_row(line):
    line = line.strip()

    if not line.startswith("y") or ":" not in line:
        return None

    row_part, values_part = line.split(":", 1)

    try:
        row_index = int(row_part[1:])
    except ValueError:
        return None

    values = []
    for item in values_part.split(","):
        item = item.strip()
        if item == "":
            continue

        try:
            values.append(int(item))
        except ValueError:
            continue

    if len(values) < GRID_SIZE:
        return None

    return row_index, values[:GRID_SIZE]


def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)

    matrix = np.full((GRID_SIZE, GRID_SIZE), MAX_DISTANCE_MM)
    received_rows = set()

    plt.ion()
    fig, ax = plt.subplots()

    image = ax.imshow(matrix, vmin=0, vmax=MAX_DISTANCE_MM)
    cbar = plt.colorbar(image, ax=ax)
    cbar.set_label("Distance [mm]")

    ax.set_title("VL53L7CX 8x8 ToF Distance Matrix")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    frame_count = 0

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            parsed = parse_row(line)

            if parsed is None:
                continue

            row_index, values = parsed

            if 0 <= row_index < GRID_SIZE:
                matrix[row_index, :] = values
                received_rows.add(row_index)

            # Update only after all 8 rows are received
            if len(received_rows) == GRID_SIZE:
                frame_count += 1
                received_rows.clear()

                image.set_data(matrix)
                ax.set_title(
                    f"VL53L7CX 8x8 ToF Matrix | frame={frame_count} | min={np.min(matrix)} mm"
                )

                # Draw much faster than plt.pause every row
                fig.canvas.draw_idle()
                fig.canvas.flush_events()

    except KeyboardInterrupt:
        print("Stopped.")

    finally:
        ser.close()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()