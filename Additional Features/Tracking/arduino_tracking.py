import serial
import time

PORT = "COM6"
BAUD = 921600

ser = serial.Serial(PORT, BAUD, timeout=0.2)

def read_for(seconds):
    end_time = time.time() + seconds
    data = b""

    while time.time() < end_time:
        chunk = ser.read(4096)

        if chunk:
            data += chunk
            end_time = time.time() + 0.5

    if data:
        print(data.decode(errors="replace"))
    else:
        print("(no response)")

def send_raw(data, wait=1.5):
    print("Sending raw:", repr(data))
    ser.write(data)
    read_for(wait)

def send_cmd(cmd, ending, wait=1.5):
    packet = cmd.encode() + ending
    print("Sending:", repr(packet))
    ser.write(packet)
    read_for(wait)

print("Opened", PORT)

ser.reset_input_buffer()
ser.reset_output_buffer()

send_raw(b"+++", 2.0)

commands = [
    "AT+VERSION",
    "AT+ROLE=?",
    "AT+CH=?",
    "AT+POWER=?",
    "AT+SRCADDR=?",
    "AT+DSTADDR=?",
    "AT+INTV=?",
]

endings = [
    b"\r\n",
    b"\r",
    b"\n",
]

for ending in endings:
    print()
    print("Testing ending:", repr(ending))

    for cmd in commands:
        send_cmd(cmd, ending, 1.5)
        time.sleep(0.2)

ser.close()