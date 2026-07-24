#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-S3 三轮全向轮小车 PC 调试面板
=====================================
- 串口收发协议与 ESP32 端 components/debug_comm 对应
- WASD 控制平移，QE 控制自转（按住式 + 单击式均可）
- 数字键 1/2/3 切换当前调参电机；+/- 调整 kp；[/] 调 ki；,/. 调 kd
- 空格：紧急停止；R：恢复默认 PID；X：退出
- 上方文本行显示每帧 STATE_REPORT

依赖：
    pip install pyserial
可选（键盘按住持续发，需 root/管理员以外的权限即可）：
    pip install pynput

用法：
    python tools/debug_panel.py --port COM7
    python tools/debug_panel.py --port /dev/ttyUSB0
"""
from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import threading
import time
from dataclasses import dataclass, field

import serial

# ====== 协议常量（与 ESP32 debug_comm.h 保持一致） ======
FRAME_HEAD = 0xAA
CMD_SET_VELOCITY   = 0x01
CMD_SET_PID        = 0x02
CMD_SET_MAX_VEL    = 0x03
CMD_EMERGENCY_STOP = 0x04
CMD_PING           = 0x05
CMD_STATE_REPORT   = 0x10


def _f32(v: float) -> bytes:
    return struct.pack("<f", float(v))


def make_frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([FRAME_HEAD, cmd & 0xFF, len(payload) & 0xFF]) + payload
    chk = sum(body) & 0xFF
    return body + bytes([chk])


@dataclass
class ChassisState:
    target_rpm: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    real_rpm: list   = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_kp: list     = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_ki: list     = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_kd: list     = field(default_factory=lambda: [0.0, 0.0, 0.0])
    vx: float = 0.0
    vy: float = 0.0
    vw: float = 0.0


class ESPClient:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.lock = threading.Lock()
        self.state = ChassisState()
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # ----- 发送 -----
    def set_velocity(self, vx, vy, vw):
        self._send(make_frame(CMD_SET_VELOCITY, _f32(vx) + _f32(vy) + _f32(vw)))

    def stop(self):
        self._send(make_frame(CMD_EMERGENCY_STOP))

    def ping(self):
        self._send(make_frame(CMD_PING))

    def set_pid(self, idx, kp, ki, kd):
        payload = bytes([idx & 0xFF]) + _f32(kp) + _f32(ki) + _f32(kd)
        self._send(make_frame(CMD_SET_PID, payload))

    def set_max_velocity(self, vx, vy, vw):
        self._send(make_frame(CMD_SET_MAX_VEL, _f32(vx) + _f32(vy) + _f32(vw)))

    def _send(self, frame: bytes):
        with self.lock:
            try:
                self.ser.write(frame)
            except serial.SerialException:
                pass

    # ----- 接收 -----
    def _rx_loop(self):
        buf = bytearray()
        state = "HEAD"
        cmd = 0
        length = 0
        payload = bytearray()
        chk_sum = 0

        while not self._stop.is_set():
            try:
                chunk = self.ser.read(64)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf.extend(chunk)
            i = 0
            while i < len(buf):
                b = buf[i]
                if state == "HEAD":
                    if b == FRAME_HEAD:
                        state, chk_sum = "CMD", 0
                    i += 1
                elif state == "CMD":
                    cmd = b
                    chk_sum = (chk_sum + b) & 0xFF
                    state = "LEN"
                    i += 1
                elif state == "LEN":
                    length = b
                    chk_sum = (chk_sum + b) & 0xFF
                    payload = bytearray()
                    state = "PAYLOAD" if length else "CHK"
                    i += 1
                elif state == "PAYLOAD":
                    payload.append(b)
                    chk_sum = (chk_sum + b) & 0xFF
                    if len(payload) >= length:
                        state = "CHK"
                    i += 1
                elif state == "CHK":
                    if chk_sum == b and cmd == CMD_STATE_REPORT and len(payload) == 72:
                        self._parse_state(payload)
                    state = "HEAD"
                    i += 1
            del buf[:i]

    def _parse_state(self, p: bytes):
        def f(off): return struct.unpack("<f", p[off:off+4])[0]
        s = self.state
        for i in range(3):
            s.target_rpm[i] = f(0  + i*4)
            s.real_rpm[i]   = f(12 + i*4)
            s.pid_kp[i]     = f(24 + i*12)
            s.pid_ki[i]     = f(28 + i*12)
            s.pid_kd[i]     = f(32 + i*12)
        s.vx = f(48); s.vy = f(52); s.vw = f(56)

    def close(self):
        self._stop.set()
        time.sleep(0.05)
        try:
            self.ser.close()
        except Exception:
            pass


# ====== 键盘跟踪（pynput，按住式） ======
class Keyboard:
    def __init__(self):
        self.pressed = set()
        self.lock = threading.Lock()
        self.installed = False
        try:
            from pynput import keyboard
            self._listener = keyboard.Listener(
                on_press=self._on_press, on_release=self._on_release)
            self._listener.daemon = True
            self._listener.start()
            self.installed = True
        except Exception:
            self.installed = False

    def _char(self, key):
        try:
            return key.char.lower() if (hasattr(key, "char") and key.char) else None
        except Exception:
            return None

    def _on_press(self, key):
        ch = self._char(key)
        with self.lock:
            if ch: self.pressed.add(ch)
            elif key == key.space: self.pressed.add(" ")

    def _on_release(self, key):
        ch = self._char(key)
        with self.lock:
            if ch and ch in self.pressed: self.pressed.discard(ch)
            elif key == key.space: self.pressed.discard(" ")

    def is_pressed(self, ch: str) -> bool:
        with self.lock:
            return ch in self.pressed


# ====== 主控制循环 ======
STEP_LINEAR = 0.15
STEP_ANG    = 0.6
PID_STEP    = 0.05
PID_DEFAULT = [(1.0, 0.0, 0.0)] * 3

# 单击式按键读取
def read_one_char():
    """非阻塞读一个字符，无键返回 None。Windows / POSIX 兼容。"""
    if os.name == "nt":
        try:
            import msvcrt
        except ImportError:
            return None
        if msvcrt.kbhit():
            return msvcrt.getwch().lower()
        return None
    fd = sys.stdin.fileno()
    try:
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
    except (ValueError, OSError):
        return None
    if not rlist:
        return None
    try:
        return os.read(fd, 1).decode("utf-8", "ignore").lower()
    except Exception:
        return None


class Controller:
    def __init__(self, cli: ESPClient, kb: Keyboard):
        self.cli = cli
        self.kb = kb
        self.vx = self.vy = self.vw = 0.0
        self.cur_idx = 0
        self.max_vx = self.max_vy = 1.0
        self.max_vw = 3.0
        self.pid = [list(PID_DEFAULT[i]) for i in range(3)]
        self._exit = False

    # ----- 按键分发 -----
    def on_key(self, c: str):
        if c == "x":
            self._exit = True
            return
        if c == " ":
            self.vx = self.vy = self.vw = 0.0
            self.cli.stop()
            return
        if c in ("1", "2", "3"):
            self.cur_idx = int(c) - 1
            print(f"\n[switch] current motor = {self.cur_idx}")
            return
        if c == "w": self.vy = min(self.max_vy, self.vy + STEP_LINEAR)
        elif c == "s": self.vy = max(-self.max_vy, self.vy - STEP_LINEAR)
        elif c == "a": self.vx = max(-self.max_vx, self.vx - STEP_LINEAR)
        elif c == "d": self.vx = min(self.max_vx, self.vx + STEP_LINEAR)
        elif c == "q": self.vw = min(self.max_vw, self.vw + STEP_ANG)
        elif c == "e": self.vw = max(-self.max_vw, self.vw - STEP_ANG)
        elif c in ("+", "="):
            self.pid[self.cur_idx][0] += PID_STEP
            self.cli.set_pid(self.cur_idx, *self.pid[self.cur_idx])
        elif c in ("-", "_"):
            self.pid[self.cur_idx][0] -= PID_STEP
            self.cli.set_pid(self.cur_idx, *self.pid[self.cur_idx])
        elif c == "[":
            self.pid[self.cur_idx][1] -= PID_STEP
            self.cli.set_pid(self.cur_idx, *self.pid[self.cur_idx])
        elif c == "]":
            self.pid[self.cur_idx][1] += PID_STEP
            self.cli.set_pid(self.cur_idx, *self.pid[self.cur_idx])
        elif c == ",":
            self.pid[self.cur_idx][2] -= PID_STEP
            self.cli.set_pid(self.cur_idx, *self.pid[self.cur_idx])
        elif c == ".":
            self.pid[self.cur_idx][2] += PID_STEP
            self.cli.set_pid(self.cur_idx, *self.pid[self.cur_idx])
        elif c == "r":
            for i in range(3):
                self.pid[i] = list(PID_DEFAULT[i])
                self.cli.set_pid(i, *self.pid[i])
            print("\n[reset] PID restored to default")
        elif c == "p":
            self.cli.ping()
            print("\n[ping] sent")

    def step(self):
        # 按住式控制（叠加增量；松开自动回零）
        if self.kb.is_pressed("w"): self.vy += STEP_LINEAR
        if self.kb.is_pressed("s"): self.vy -= STEP_LINEAR
        if self.kb.is_pressed("a"): self.vx -= STEP_LINEAR
        if self.kb.is_pressed("d"): self.vx += STEP_LINEAR
        if self.kb.is_pressed("q"): self.vw += STEP_ANG
        if self.kb.is_pressed("e"): self.vw -= STEP_ANG
        if self.kb.is_pressed(" "):
            self.vx = self.vy = self.vw = 0.0

        # 限幅
        self.vx = max(-self.max_vx, min(self.max_vx, self.vx))
        self.vy = max(-self.max_vy, min(self.max_vy, self.vy))
        self.vw = max(-self.max_vw, min(self.max_vw, self.vw))

        # 单击式
        c = read_one_char()
        if c:
            self.on_key(c)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    cli = ESPClient(args.port, args.baud)
    kb = Keyboard()
    ctrl = Controller(cli, kb)

    print("=== ESP32-S3 三轮全向轮调试面板 ===")
    print("WASD = 平移  Q/E = 自转   1/2/3 = 选电机  +/- = kp  [/] = ki  ,/. = kd")
    print("空格 = 紧急停止  R = 默认PID  P = ping  X = 退出")
    if not kb.installed:
        print("[提示] 未安装 pynput，按住式控制不可用，但单击按键仍可使用")

    last_send = 0.0
    last_print = 0.0
    try:
        while not ctrl._exit:
            now = time.time()
            ctrl.step()
            if now - last_send > 0.05:
                cli.set_velocity(ctrl.vx, ctrl.vy, ctrl.vw)
                last_send = now
            if now - last_print > 0.2:
                s = cli.state
                pid = ctrl.pid[ctrl.cur_idx]
                line = (
                    f"vx={ctrl.vx:+.2f} vy={ctrl.vy:+.2f} vw={ctrl.vw:+.2f}  "
                    f"rpm_t=({s.target_rpm[0]:6.1f},{s.target_rpm[1]:6.1f},{s.target_rpm[2]:6.1f})  "
                    f"rpm_r=({s.real_rpm[0]:6.1f},{s.real_rpm[1]:6.1f},{s.real_rpm[2]:6.1f})  "
                    f"PID{ctrl.cur_idx}=({pid[0]:+.2f},{pid[1]:+.2f},{pid[2]:+.2f})"
                )
                print("\r" + line[:140].ljust(140), end="", flush=True)
                last_print = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        cli.stop()
        cli.close()
        print("\nbye.")


if __name__ == "__main__":
    main()