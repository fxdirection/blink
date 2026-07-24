#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 串口 / TCP 协议客户端（与 components/debug_comm、components/wifi_debug 配套）

GUI 与命令行脚本都通过这个类收发帧。
- open(port, baud)     有线串口（UART0）
- open_tcp(host, port) 无线 TCP（espcar.local:8888 或 192.168.4.1:8888）
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Union

import serial  # pyserial


# ====== 协议常量（与 debug_comm.h 一致） ======
FRAME_HEAD = 0xAA
CMD_SET_VELOCITY   = 0x01
CMD_SET_PID        = 0x02
CMD_SET_MAX_VEL    = 0x03
CMD_EMERGENCY_STOP = 0x04
CMD_PING           = 0x05
CMD_STATE_REPORT   = 0x10

STATE_PAYLOAD_LEN = 72  # 12+12+36+12


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
    updated_at: float = 0.0


# 把 pyserial.Serial 和 socket.socket 包成统一的"读、写、关"接口
class _SerialLike:
    def read(self, n: int) -> bytes: ...
    def write(self, b: bytes) -> int: ...
    def close(self) -> None: ...
    def is_open(self) -> bool: ...


class _SerialAdapter(_SerialLike):
    def __init__(self, port: str, baud: int, timeout: float = 0.05):
        self._ser = serial.Serial(port, baud, timeout=timeout)
    def read(self, n: int) -> bytes:
        try: return self._ser.read(n)
        except serial.SerialException: return b""
    def write(self, b: bytes) -> int:
        try: return self._ser.write(b)
        except serial.SerialException: return 0
    def close(self) -> None:
        try: self._ser.close()
        except Exception: pass
    def is_open(self) -> bool:
        return self._ser.is_open


class _SocketAdapter(_SerialLike):
    def __init__(self, host: str, port: int, timeout: float = 0.5):
        self._sock = socket.create_connection((host, port), timeout=timeout)
        self._sock.settimeout(0.05)
    def read(self, n: int) -> bytes:
        try:
            return self._sock.recv(n)
        except (socket.timeout, OSError):
            return b""
    def write(self, b: bytes) -> int:
        try:
            return self._sock.sendall(b) or len(b)
        except OSError:
            return 0
    def close(self) -> None:
        try: self._sock.shutdown(socket.SHUT_RDWR)
        except Exception: pass
        try: self._sock.close()
        except Exception: pass
    def is_open(self) -> bool:
        return self._sock.fileno() != -1


class ESPLink:
    """线程安全的客户端，支持串口或 TCP。"""

    def __init__(self):
        self.io: Optional[_SerialLike] = None
        self.lock = threading.Lock()
        self.state = ChassisState()
        self._stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._state_evt = threading.Event()
        self.mode: str = ""         # "serial" / "tcp"
        self.endpoint: str = ""     # 描述信息

    # -------- 连接管理 --------
    def open(self, port: str, baud: int = 115200, timeout: float = 0.05) -> None:
        self.close()
        self.io = _SerialAdapter(port, baud, timeout)
        self.mode = "serial"; self.endpoint = f"{port}@{baud}"
        self._start_rx()

    def open_tcp(self, host: str, port: int = 8888, timeout: float = 5.0) -> None:
        """host 可以是 IP、hostname、或 'espcar.local' (需 PC 安装 Bonjour)"""
        self.close()
        self.io = _SocketAdapter(host, port, timeout)
        self.mode = "tcp"; self.endpoint = f"{host}:{port}"
        self._start_rx()

    def close(self) -> None:
        if self._rx_thread is not None:
            self._stop.set()
            self._rx_thread.join(timeout=0.5)
            self._rx_thread = None
        if self.io is not None:
            self.io.close()
            self.io = None
        self.mode = ""

    @property
    def is_open(self) -> bool:
        return self.io is not None

    def _start_rx(self):
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # -------- 发送 --------
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
        if not self.is_open:
            return
        with self.lock:
            self.io.write(frame)

    # -------- 接收 --------
    def _rx_loop(self):
        buf = bytearray()
        state = "HEAD"
        cmd = 0
        length = 0
        payload = bytearray()
        chk_sum = 0

        while not self._stop.is_set():
            if not self.is_open:
                break
            chunk = self.io.read(64)
            if not chunk:
                continue
            buf.extend(chunk)
            i = 0
            while i < len(buf):
                b = buf[i]
                if state == "HEAD":
                    if b == FRAME_HEAD:
                        state, chk_sum = "CMD", FRAME_HEAD  # 0xAA 也计入校验和
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
                    if chk_sum == b and cmd == CMD_STATE_REPORT and len(payload) == STATE_PAYLOAD_LEN:
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
        s.vx = f(60); s.vy = f(64); s.vw = f(68)
        s.updated_at = time.time()
        self._state_evt.set()

    def wait_state(self, timeout: float) -> bool:
        self._state_evt.clear()
        return self._state_evt.wait(timeout)


# 便捷：用 mDNS/自定义主机名解析
def resolve_tcp_host(host: str) -> str:
    """如果 host 含 '.local'，走 mDNS；否则直接返回。"""
    if host.endswith(".local") and hasattr(socket, "getaddrinfo"):
        # 标准 socket 通常不直接处理 mDNS，需要安装 zeroconf 或 Bonjour
        # 这里做一次普通解析，失败抛异常给上层处理
        try:
            infos = socket.getaddrinfo(host, 8888, type=socket.SOCK_STREAM)
            if infos: return infos[0][4][0]
        except socket.gaierror:
            pass
    return host