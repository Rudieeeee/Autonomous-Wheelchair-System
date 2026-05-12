# -*- coding: utf-8 -*-

import time
import serial


class DFRobotMatrixLidar:
    CMD_SETMODE = 1
    CMD_ALLDATA = 2
    CMD_FIXED_POINT = 3

    STATUS_SUCCESS = 0x53
    STATUS_FAILED = 0x63

    ERR_CODE_NONE = 0x00
    ERR_CODE_RES_PKT = 0x02
    ERR_CODE_RES_TIMEOUT = 0x04

    DEBUG_TIMEOUT_S = 8.0

    INDEX_ARGS_NUM_H = 0
    INDEX_ARGS_NUM_L = 1
    INDEX_CMD = 2

    INDEX_RES_ERR = 0
    INDEX_RES_STATUS = 1
    INDEX_RES_CMD = 2
    INDEX_RES_LEN_L = 3
    INDEX_RES_LEN_H = 4
    INDEX_RES_DATA = 5

    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        print(f"Opening {port} at {baudrate} baud")

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1
        )

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _send_packet(self, pkt):
        packet = bytes([0x55] + pkt)
        print("TX:", packet.hex(" "))

        self.ser.reset_input_buffer()
        self.ser.write(packet)
        self.ser.flush()

    def _recv_data(self, length):
        raw = self.ser.read(length)
        return list(raw)

    def _recv_packet(self, expected_cmd):
        start = time.time()

        while time.time() - start < self.DEBUG_TIMEOUT_S:
            first = self._recv_data(1)

            if not first:
                continue

            status = first[0]

            # Ignore filler/header bytes
            if status == 0xFF:
                continue

            if status not in [self.STATUS_SUCCESS, self.STATUS_FAILED]:
                print("Unexpected byte:", hex(status))
                continue

            cmd_data = self._recv_data(1)
            if len(cmd_data) < 1:
                return [self.ERR_CODE_RES_PKT]

            cmd = cmd_data[0]

            len_data = self._recv_data(2)
            if len(len_data) < 2:
                return [self.ERR_CODE_RES_PKT]

            data_len = len_data[0] | (len_data[1] << 8)

            print(
                "RX header:",
                "status =", hex(status),
                "cmd =", hex(cmd),
                "len =", data_len
            )

            if cmd != expected_cmd:
                print("Wrong response command")
                return [self.ERR_CODE_RES_PKT]

            if data_len > 512:
                print("Response too long")
                return [self.ERR_CODE_RES_PKT]

            result = [
                self.ERR_CODE_NONE,
                status,
                cmd,
                len_data[0],
                len_data[1],
            ]

            if data_len > 0:
                data = self._recv_data(data_len)
                result += data

            return result

        print("RX timeout")
        return [self.ERR_CODE_RES_TIMEOUT]

    def set_ranging_mode(self, matrix_mode):
        """
        Try setting the matrix/ranging mode.
        Different DFRobot examples may use different mode values.
        """
        length = 4

        pkt = [0] * (3 + length)
        pkt[self.INDEX_ARGS_NUM_H] = ((length + 1) >> 8) & 0xFF
        pkt[self.INDEX_ARGS_NUM_L] = (length + 1) & 0xFF
        pkt[self.INDEX_CMD] = self.CMD_SETMODE

        pkt[3] = 0
        pkt[4] = 0
        pkt[5] = 0
        pkt[6] = matrix_mode

        self._send_packet(pkt)
        time.sleep(0.1)

        recv = self._recv_packet(self.CMD_SETMODE)

        if (
            len(recv) >= 5
            and recv[self.INDEX_RES_ERR] == self.ERR_CODE_NONE
            and recv[self.INDEX_RES_STATUS] == self.STATUS_SUCCESS
        ):
            return 0

        return 1

    def get_all_data(self):
        pkt = [0] * 3

        pkt[self.INDEX_ARGS_NUM_H] = 0
        pkt[self.INDEX_ARGS_NUM_L] = 1
        pkt[self.INDEX_CMD] = self.CMD_ALLDATA

        self._send_packet(pkt)
        time.sleep(0.1)

        recv = self._recv_packet(self.CMD_ALLDATA)

        if (
            len(recv) >= 5
            and recv[self.INDEX_RES_ERR] == self.ERR_CODE_NONE
            and recv[self.INDEX_RES_STATUS] == self.STATUS_SUCCESS
        ):
            data_len = recv[self.INDEX_RES_LEN_L] | (
                recv[self.INDEX_RES_LEN_H] << 8
            )

            if data_len > 0:
                return recv[self.INDEX_RES_DATA:]

        return []

    def get_fixed_point_data(self, x, y):
        length = 2

        pkt = [0] * (3 + length)
        pkt[self.INDEX_ARGS_NUM_H] = ((length + 1) >> 8) & 0xFF
        pkt[self.INDEX_ARGS_NUM_L] = (length + 1) & 0xFF
        pkt[self.INDEX_CMD] = self.CMD_FIXED_POINT

        pkt[3] = x
        pkt[4] = y

        self._send_packet(pkt)
        time.sleep(0.1)

        recv = self._recv_packet(self.CMD_FIXED_POINT)

        if (
            len(recv) >= 7
            and recv[self.INDEX_RES_ERR] == self.ERR_CODE_NONE
            and recv[self.INDEX_RES_STATUS] == self.STATUS_SUCCESS
        ):
            low = recv[self.INDEX_RES_DATA]
            high = recv[self.INDEX_RES_DATA + 1]
            return low | (high << 8)

        return -1


def bytes_to_uint16_list(data):
    """
    Convert byte list [low, high, low, high, ...] to mm distances.
    """
    values = []

    for i in range(0, len(data) - 1, 2):
        value = data[i] | (data[i + 1] << 8)
        values.append(value)

    return values


def print_matrix(values):
    """
    Print as 8x8 if 64 values, 4x4 if 16 values, otherwise flat.
    """
    if len(values) == 64:
        size = 8
    elif len(values) == 16:
        size = 4
    else:
        print("Distances:", values)
        return

    print(f"{size}x{size} distance matrix in mm:")

    for row in range(size):
        line = values[row * size:(row + 1) * size]
        print(" ".join(f"{v:5d}" for v in line))


def main():
    PORT = "/dev/ttyACM0"
    BAUDRATE = 115200

    sensor = None

    try:
        print("Starting DFRobot VL53L7CX / Matrix ToF reader")
        sensor = DFRobotMatrixLidar(PORT, BAUDRATE)

        # Try likely mode values until one works
        working_mode = None

        for mode in [8, 64, 4, 16]:
            print(f"\nTrying ranging mode: {mode}")
            result = sensor.set_ranging_mode(mode)
            print("Mode result:", result)

            if result == 0:
                working_mode = mode
                print("Working mode:", mode)
                break

            time.sleep(0.5)

        if working_mode is None:
            print("\nNo ranging mode worked.")
            print("Check:")
            print("1. Sensor switch is set to UART mode")
            print("2. Unplug/replug USB-C after changing I2C/UART switch")
            print("3. Correct port: ls /dev/ttyACM* /dev/ttyUSB*")
            print("4. Try another baudrate if needed")
            return

        print("\nReading ToF data. Press Ctrl+C to stop.\n")

        while True:
            raw_data = sensor.get_all_data()

            print("Raw byte length:", len(raw_data))

            if raw_data:
                distances_mm = bytes_to_uint16_list(raw_data)
                print_matrix(distances_mm)
            else:
                print("No data received")

            print("-" * 60)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    except serial.SerialException as e:
        print("Serial error:", e)
        print("Check port with:")
        print("  ls /dev/ttyACM* /dev/ttyUSB*")

    except Exception as e:
        print("Unexpected error:", e)

    finally:
        if sensor is not None:
            sensor.close()

        print("Disconnected.")


if __name__ == "__main__":
    main()