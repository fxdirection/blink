#!/usr/bin/env python3
"""ESP32 car keyboard controller (Windows).

Usage:
  1. Connect PC to WiFi "ESP32-Car" (password 12345678)
  2. Run: python tools/car_ctrl.py

Keys:
  w / s  forward / backward
  a / d  strafe left / right
  q / e  rotate left / right
  space  emergency stop
  Ctrl+C quit
"""

import socket
import struct
import sys


HOST = "192.168.4.1"
PORT = 8888
SPEED = 1.5  # m/s
TURN = 1.5  # rad/s


def frame(vx, vy, vw):
    payload = struct.pack("<fff", vx, vy, vw)
    chk = (0xAA + 0x01 + 12 + sum(payload)) & 0xFF
    return bytes([0xAA, 0x01, 12]) + payload + bytes([chk])


def stop_frame():
    chk = (0xAA + 0x04) & 0xFF
    return bytes([0xAA, 0x04, 0x00, chk])


KEYS = {
    "a": (-SPEED, 0, 0),
    "d": (SPEED, 0, 0),
    "w": (0, SPEED, 0),
    "s": (0, -SPEED, 0),
    "q": (0, 0, TURN),
    "e": (0, 0, -TURN),
}

ARROW_MAP = {"H": "w", "P": "s", "K": "a", "M": "d"}


def getch():
    import msvcrt

    ch = msvcrt.getwch()
    if ch in ("\x00", "\xe0"):
        ch = msvcrt.getwch()
        return ARROW_MAP.get(ch, "")
    return ch


def main():
    sock = socket.socket()
    try:
        sock.connect((HOST, PORT))
    except OSError as exc:
        print(f"Connect failed: {exc}")
        print("Check: WiFi connected to ESP32-Car? ESP32 powered?")
        sys.exit(1)

    print("已连接：W/S 前后，A/D 平移，Q/E 旋转，空格急停，Ctrl+C 退出")
    try:
        while True:
            ch = getch()
            if ch == "\x03":
                break
            if ch == " ":
                sock.sendall(stop_frame())
            elif ch in KEYS:
                sock.sendall(frame(*KEYS[ch]))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.sendall(stop_frame())
        except OSError:
            pass
        sock.close()


if __name__ == "__main__":
    main()
