#!/usr/bin/env python3

import argparse
import signal
import socket
import struct
import sys
import threading
import time


# ============================================================
# SocketCAN helpers — no can2RNET module needed
# ============================================================

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000


def open_can_socket(interface="can0"):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    return sock


def build_can_frame(can_id, data=b"", extended=False):
    if extended:
        can_id |= CAN_EFF_FLAG

    data = data[:8]
    dlc = len(data)
    data = data.ljust(8, b"\x00")

    return struct.pack("<IB3x8s", can_id, dlc, data)


def cansend(sock, frame_text):
    frame_id_text, data_text = frame_text.split("#")

    can_id = int(frame_id_text, 16)
    data = bytes.fromhex(data_text) if data_text else b""

    extended = can_id > 0x7FF

    frame = build_can_frame(can_id, data, extended)
    sock.send(frame)


def dissect_frame(can_frame):
    can_id, dlc, data = struct.unpack("<IB3x8s", can_frame)

    extended = bool(can_id & CAN_EFF_FLAG)
    is_rtr = bool(can_id & CAN_RTR_FLAG)

    if extended:
        clean_id = can_id & 0x1FFFFFFF
        id_text = f"{clean_id:08x}"
    else:
        clean_id = can_id & 0x7FF
        id_text = f"{clean_id:03x}"

    if is_rtr:
        return id_text + "#R"

    return id_text + "#" + data[:dlc].hex()


# ============================================================
# PS4 /dev/input/event0 constants
# ============================================================

EV_ABS = 0x03

ABS_X = 0x00
ABS_Y = 0x01

# Linux input_event on 64-bit WSL:
# timeval sec, timeval usec, type, code, value
INPUT_EVENT_STRUCT = struct.Struct("qqHHi")


# ============================================================
# Global state
# ============================================================

running = True

can_socket = None
rnet_joystick_id = None

joystick_x = 0
joystick_y = 0


# ============================================================
# Utility functions
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def byte_hex(value):
    return f"{int(value) & 0xFF:02x}"


def axis_0_255_to_signed(value, deadzone=10, invert=False):
    """
    PS4 axis from /dev/input/event0 is usually:
      center = 128
      min    = 0
      max    = 255

    Convert to:
      -100 ... 0 ... +100
    """
    centered = int(value) - 128

    if abs(centered) <= deadzone:
        return 0

    scaled = int(centered * 100 / 127)
    scaled = clamp(scaled, -100, 100)

    if invert:
        scaled = -scaled

    return scaled


def signed_to_rnet_byte(value):
    """
    R-Net joystick byte is signed int8 stored as byte.
      0    -> 00
      100  -> 64
      -100 -> 9c
    """
    return int(value) & 0xFF


def send_neutral_frames(count=30):
    global can_socket, rnet_joystick_id

    if can_socket is None or rnet_joystick_id is None:
        return

    for _ in range(count):
        try:
            cansend(can_socket, rnet_joystick_id + "#0000")
            time.sleep(0.01)
        except Exception:
            break


def stop_program(sig=None, frame=None):
    global running

    running = False
    print("\nStopping...")
    print("Sending neutral joystick frames...")
    send_neutral_frames()
    print("Done.")
    sys.exit(0)


# ============================================================
# PS4 reader thread
# ============================================================

def ps4_event_reader(event_path, deadzone, invert_y):
    global joystick_x, joystick_y, running

    print(f"Opening PS4 controller event device: {event_path}")

    try:
        with open(event_path, "rb") as dev:
            print("PS4 reader started.")
            print("Use LEFT stick.")
            print("Centered stick should show X=00 Y=00.")

            while running:
                data = dev.read(INPUT_EVENT_STRUCT.size)

                if not data or len(data) != INPUT_EVENT_STRUCT.size:
                    continue

                sec, usec, ev_type, ev_code, ev_value = INPUT_EVENT_STRUCT.unpack(data)

                if ev_type != EV_ABS:
                    continue

                if ev_code == ABS_X:
                    signed_x = axis_0_255_to_signed(
                        ev_value,
                        deadzone=deadzone,
                        invert=False
                    )
                    joystick_x = signed_to_rnet_byte(signed_x)

                elif ev_code == ABS_Y:
                    signed_y = axis_0_255_to_signed(
                        ev_value,
                        deadzone=deadzone,
                        invert=invert_y
                    )
                    joystick_y = signed_to_rnet_byte(signed_y)

    except PermissionError:
        print(f"Permission denied for {event_path}. Run with sudo.")
        running = False

    except FileNotFoundError:
        print(f"Device not found: {event_path}")
        running = False

    except Exception as e:
        print(f"PS4 reader error: {e}")
        joystick_x = 0
        joystick_y = 0
        running = False


# ============================================================
# R-Net logic
# ============================================================

def wait_rnet_joystick_frame(timeout=5.0):
    """
    Wait for real wheelchair JSM joystick frame to detect the correct ID.
    Usually starts with 020, e.g. 02000400.
    """
    global can_socket, running

    print("Waiting for real R-Net joystick frame starting with 020...")

    start = time.time()

    while running:
        if time.time() - start > timeout:
            return None

        try:
            cf, addr = can_socket.recvfrom(16)
            text = dissect_frame(cf)
            frame_id = text.split("#")[0]

            if frame_id.startswith("020"):
                return frame_id

        except Exception as e:
            print(f"CAN receive error while detecting joystick ID: {e}")
            return None

    return None


def jsmerror_injector():
    """
    Triggers a network error frame ending with 000 payload to halt JSM frame transmission,
    then loops independently to inject the PS4 input using the detected R-Net ID.
    """
    global can_socket, rnet_joystick_id, joystick_x, joystick_y, running

    print("JSMerror injector initialized.")
    
    # 1. Trigger the JSM error frame ending with 000
    try:
        # 8-byte data field ending in 000 hex bytes (last 3 hex chars are 000, padded with 0)
        error_frame = "0C000100#0000000000000000"
        print(f"Sending error trigger frame: {error_frame}")
        cansend(can_socket, error_frame)
        
        # Give the bus a brief moment to settle/transition state
        time.sleep(0.05)
    except Exception as e:
        print(f"Failed to send error trigger: {e}")
        running = False
        return

    print(f"Starting injection loop using joystick ID: {rnet_joystick_id}")

    # 2. Continuous independent timed injection loop
    while running:
        try:
            data = byte_hex(joystick_x) + byte_hex(joystick_y)
            cansend(can_socket, rnet_joystick_id + "#" + data)
            time.sleep(0.01)  # 10ms frame period

        except Exception as e:
            print(f"Injector loop error: {e}")
            running = False
            break


# ============================================================
# Main
# ============================================================

def main():
    global can_socket, rnet_joystick_id, running

    parser = argparse.ArgumentParser(
        description="PS4 controller /dev/input/event0 to R-Net JSMerror CAN injector"
    )

    parser.add_argument(
        "--event",
        default="/dev/input/event0",
        help="PS4 input event device, default: /dev/input/event0"
    )

    parser.add_argument(
        "--can",
        default="can0",
        help="CAN interface, default: can0"
    )

    parser.add_argument(
        "--joy-id",
        default=None,
        help="Manual R-Net joystick ID, e.g. 02000400. If omitted, auto-detect."
    )

    parser.add_argument(
        "--deadzone",
        type=int,
        default=10,
        help="PS4 stick deadzone around 128, default: 10"
    )

    parser.add_argument(
        "--no-invert-y",
        action="store_true",
        help="Do not invert Y axis"
    )

    args = parser.parse_args()

    signal.signal(signal.SIGINT, stop_program)
    signal.signal(signal.SIGTERM, stop_program)

    print("Opening CAN interface...")
    try:
        can_socket = open_can_socket(args.can)
    except Exception as e:
        print(f"Could not open {args.can}: {e}")
        print("Try:")
        print(f"  sudo ip link set {args.can} up type can bitrate 125000")
        sys.exit(1)

    reader_thread = threading.Thread(
        target=ps4_event_reader,
        args=(args.event, args.deadzone, not args.no_invert_y),
        daemon=True
    )
    reader_thread.start()

    time.sleep(0.5)

    if not running:
        sys.exit(1)

    if args.joy_id:
        rnet_joystick_id = args.joy_id.lower()
        print(f"Using manual joystick ID: {rnet_joystick_id}")
    else:
        rnet_joystick_id = wait_rnet_joystick_frame(timeout=5.0)

        if rnet_joystick_id is None:
            print("No R-Net joystick frame found via auto-detection.")
            print("Check connection or specify manual --joy-id option.")
            sys.exit(1)

        print(f"Found R-Net joystick frame: {rnet_joystick_id}")

    print()
    print("SAFETY:")
    print("- Wheels off the ground for first test.")
    print("- Keep the real wheelchair joystick centered.")
    print("- Keep emergency stop / power button ready.")
    print("- Ctrl+C sends neutral frames.")
    print()

    # Changed target thread function from followjsm_injector to jsmerror_injector
    injector_thread = threading.Thread(
        target=jsmerror_injector,
        daemon=True
    )
    injector_thread.start()

    start = time.time()

    while running:
        time.sleep(0.5)

        elapsed = round(time.time() - start, 1)

        print(
            f"{elapsed}s\t"
            f"ID={rnet_joystick_id}\t"
            f"X={byte_hex(joystick_x)}\t"
            f"Y={byte_hex(joystick_y)}\t"
            f"threads={threading.active_count()}"
        )


if __name__ == "__main__":
    main()