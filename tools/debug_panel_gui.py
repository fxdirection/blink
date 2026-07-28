#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-S3 三轮全向轮小车 - 可视化调试面板（PySide6）

依赖：
    pip install pyserial pyside6

运行：
    python tools/debug_panel_gui.py
"""
from __future__ import annotations

import sys
import time
from typing import List

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QPushButton, QLabel, QSlider, QDoubleSpinBox, QComboBox,
    QPlainTextEdit, QStatusBar, QSizePolicy, QFrame, QSpacerItem,
    QDialog, QDialogButtonBox, QTabWidget, QTextBrowser, QCheckBox
)

import pyqtgraph as pg
from collections import deque

from esp_link import ESPLink


# ---- 全局样式（深色主题） ----
STYLE = """
* {
    font-family: "Segoe UI", "Microsoft YaHei", "PingFang SC", sans-serif;
    font-size: 10pt;
}
QMainWindow, QWidget { background: #1e1f22; color: #e6e6e6; }
QGroupBox {
    border: 1px solid #3a3d41;
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 14px;
    background: #26272a;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0 6px;
    color: #9ab;
}
QPushButton {
    background: #3a3d41;
    border: 1px solid #4a4d51;
    border-radius: 4px;
    padding: 6px 10px;
    color: #e6e6e6;
}
QPushButton:hover    { background: #4a4d51; }
QPushButton:pressed  { background: #2a2d31; }
QPushButton#danger   { background: #b34a4a; border-color: #c56060; }
QPushButton#danger:hover  { background: #c55b5b; }
QPushButton#primary  { background: #3d6d9c; border-color: #5080b0; }
QPushButton#primary:hover { background: #5080b0; }
QPushButton:disabled { background: #2a2d31; color: #666; }
QLabel#big {
    font-size: 22pt;
    font-weight: 600;
    color: #9cdcfe;
}
QLabel#mid {
    font-size: 12pt;
    color: #c0c0c0;
}
QLabel#tiny {
    font-size: 8pt;
    color: #888;
}
QDoubleSpinBox, QComboBox {
    background: #1e1f22;
    border: 1px solid #3a3d41;
    border-radius: 3px;
    padding: 2px 4px;
    selection-background-color: #3d6d9c;
}
QSlider::groove:horizontal {
    background: #2a2d31;
    height: 6px;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #5080b0;
    width: 14px;
    margin: -6px 0;
    border-radius: 7px;
}
QSlider::sub-page:horizontal {
    background: #3d6d9c;
    border-radius: 3px;
}
QPlainTextEdit {
    background: #16171a;
    border: 1px solid #3a3d41;
    color: #c8c8c8;
    font-family: Consolas, "Cascadia Mono", monospace;
}
QStatusBar { background: #16171a; color: #888; }
QFrame#sep { background: #3a3d41; max-width: 1px; }
"""

# ---- 调参默认值（与 ESP 端 config.c 一致） ----
PID_DEFAULTS = [(2.0, 0.03, 0.1)] * 3
MAX_VX = 3.0
MAX_VY = 3.0
MAX_VW = 6.0


def list_serial_ports() -> List[str]:
    """列出可用串口。失败时退回 ['COM1'] / ['/dev/ttyUSB0']。"""
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()] or ["COM1"]
    except Exception:
        return ["COM1"]


# ===================== 主窗口 =====================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32-S3 三轮全向轮 调试面板")
        self.resize(1280, 760)
        self.setMinimumSize(960, 620)
        self.setStyleSheet(STYLE)

        self.link = ESPLink()
        self.vx = self.vy = self.vw = 0.0
        self.pressed = set()                # 当前按下的方向键
        self._velocity_keys = {"w": (0, +1), "s": (0, -1), "a": (-1, 0), "d": (+1, 0),
                               "q": ("w",), "e": ("-w",)}

        # 中央布局
        central = QWidget(); self.setCentralWidget(central)
        root = QVBoxLayout(central); root.setContentsMargins(10, 10, 10, 6); root.setSpacing(8)

        root.addLayout(self._build_topbar())

        # 控制区：底盘 + 电机。包成 widget 以便限制高度
        body = QWidget()
        body_layout = QHBoxLayout(body); body_layout.setContentsMargins(0, 0, 0, 0); body_layout.setSpacing(10)
        body_layout.addWidget(self._build_chassis_box(), 1)
        body_layout.addWidget(self._build_motors_box(), 2)
        body.setMinimumHeight(220)
        root.addWidget(body, 0)  # 不强行拉伸；让 chassis/motors 按 sizeHint 排

        # 实时曲线（可折叠，可拖拽调高度）
        self.plot_panel = PlotPanel()
        self.plot_panel.setMinimumHeight(280)

        plot_box = QGroupBox()
        plot_box.setTitle("实时曲线  （点标题展开 / 双击电机卡打开详情窗口）")
        plot_box.setCheckable(True); plot_box.setChecked(False)  # 默认折叠，留出空间给控制区
        plot_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        pv = QVBoxLayout(plot_box); pv.setContentsMargins(4, 4, 4, 4); pv.setSpacing(2)
        pv.addWidget(self.plot_panel)

        # 折叠时显示的信息条（连接状态 + 数据时间 + 提示）
        self.info_bar = QFrame()
        self.info_bar.setFrameShape(QFrame.StyledPanel)
        self.info_bar.setStyleSheet(
            "QFrame { background: #1a2530; border: 1px solid #2a4a6a; border-radius: 6px; }"
            "QLabel { color: #b0c4d8; }"
        )
        self.info_bar.setMaximumHeight(160)
        ib = QHBoxLayout(self.info_bar); ib.setContentsMargins(16, 12, 16, 12); ib.setSpacing(20)
        # 左：标题 + 连接状态
        left = QVBoxLayout(); left.setSpacing(4)
        title = QLabel("📊  实时曲线已收起")
        title.setStyleSheet("font-size: 13pt; color: #9cdcfe; font-weight: bold;")
        left.addWidget(title)
        self.lbl_info_conn = QLabel("●  未连接")
        self.lbl_info_conn.setStyleSheet("font-size: 11pt; color: #888;")
        left.addWidget(self.lbl_info_conn)
        ib.addLayout(left, 1)
        # 中：数据状态
        mid = QVBoxLayout(); mid.setSpacing(4)
        mid.addWidget(QLabel("数据状态"))  # spacer
        self.lbl_info_data = QLabel("最近状态数据：—")
        self.lbl_info_data.setStyleSheet("font-size: 10pt; color: #aaa;")
        mid.addWidget(self.lbl_info_data)
        ib.addLayout(mid, 1)
        # 右：提示
        right = QVBoxLayout(); right.setSpacing(4)
        right.addWidget(QLabel(""))  # spacer for alignment
        tip2 = QLabel("点上方「实时曲线」标题可展开曲线区 · "
                      "双击电机卡打开单电机详情窗口")
        tip2.setStyleSheet("font-size: 9pt; color: #777;")
        tip2.setWordWrap(True)
        right.addWidget(tip2)
        ib.addLayout(right, 2)

        # 自定义 toggle：折叠时只隐藏内部 plot_panel，标题保留可见
        def _on_plot_toggle(checked: bool):
            if checked:
                # 展开：显示曲线，info_bar 隐藏
                self.plot_panel.show()
                plot_box.setMaximumHeight(16777215)
                self.info_bar.hide()
                root.setStretchFactor(body, 0)
                root.setStretchFactor(plot_box, 3)
            else:
                # 折叠：只显示标题（~30px 高），info_bar 显示
                self.plot_panel.hide()
                plot_box.setMaximumHeight(40)   # 只留标题栏
                self.info_bar.show()
                root.setStretchFactor(body, 0)
                root.setStretchFactor(plot_box, 0)
        plot_box.toggled.connect(_on_plot_toggle)
        plot_box.setMaximumHeight(40)  # 初始折叠
        self.plot_panel.hide()
        self.plot_toggle_box = plot_box
        self.info_bar.show()
        root.addWidget(self.info_bar, 1)
        root.addWidget(plot_box, 1)

        root.addLayout(self._build_bottom_bar())

        # 状态栏
        self.status = QStatusBar(); self.setStatusBar(self.status)
        self.status.showMessage("未连接")

        # 定时器
        self.timer_send = QTimer(); self.timer_send.setInterval(50);   self.timer_send.timeout.connect(self._tick_send)
        self.timer_ui   = QTimer(); self.timer_ui.setInterval(100);    self.timer_ui.timeout.connect(self._tick_ui)
        self.timer_send.start(); self.timer_ui.start()

    # ---------- UI 构造 ----------
    def _build_topbar(self) -> QHBoxLayout:
        h = QHBoxLayout(); h.setSpacing(8)

        # 连接方式：串口 / WiFi(TCP)
        self.cmb_mode = QComboBox(); self.cmb_mode.addItems(["串口 (USB-UART)", "WiFi (TCP)"])
        self.cmb_mode.setMinimumWidth(140)
        self.cmb_mode.currentIndexChanged.connect(self._on_mode_changed)
        h.addWidget(self.cmb_mode)

        # 串口相关
        self.serial_widget = QWidget()
        sh = QHBoxLayout(self.serial_widget); sh.setContentsMargins(0,0,0,0); sh.setSpacing(6)
        sh.addWidget(QLabel("端口:"))
        self.cmb_port = QComboBox(); self.cmb_port.setMinimumWidth(110)
        self.cmb_port.addItems(list_serial_ports())
        sh.addWidget(self.cmb_port)
        self.btn_refresh = QPushButton("刷新"); self.btn_refresh.clicked.connect(self._refresh_ports)
        sh.addWidget(self.btn_refresh)
        sh.addWidget(QLabel("波特率:"))
        self.cmb_baud = QComboBox()
        self.cmb_baud.addItems(["115200", "230400", "460800", "921600"])
        self.cmb_baud.setCurrentText("115200")
        sh.addWidget(self.cmb_baud)
        h.addWidget(self.serial_widget)

        # WiFi 相关（默认隐藏）
        self.wifi_widget = QWidget()
        wh = QHBoxLayout(self.wifi_widget); wh.setContentsMargins(0,0,0,0); wh.setSpacing(6)
        wh.addWidget(QLabel("主机:"))
        self.ed_host = QComboBox(); self.ed_host.setEditable(True); self.ed_host.setMinimumWidth(160)
        self.ed_host.addItems(["espcar.local", "192.168.4.1"])
        self.ed_host.setCurrentText("espcar.local")
        wh.addWidget(self.ed_host)
        wh.addWidget(QLabel("端口:"))
        self.ed_tcp_port = QComboBox(); self.ed_tcp_port.setEditable(True)
        self.ed_tcp_port.addItems(["8888"])
        self.ed_tcp_port.setCurrentText("8888")
        self.ed_tcp_port.setFixedWidth(70)
        wh.addWidget(self.ed_tcp_port)
        self.wifi_widget.setVisible(False)
        h.addWidget(self.wifi_widget)

        self.btn_connect = QPushButton("连接"); self.btn_connect.setObjectName("primary")
        self.btn_connect.clicked.connect(self._on_connect)
        h.addWidget(self.btn_connect)
        self.btn_disconnect = QPushButton("断开"); self.btn_disconnect.clicked.connect(self._on_disconnect)
        self.btn_disconnect.setEnabled(False)
        h.addWidget(self.btn_disconnect)

        h.addStretch(1)
        self.lbl_conn = QLabel("● 未连接"); self.lbl_conn.setObjectName("tiny")
        h.addWidget(self.lbl_conn)
        return h

    def _on_mode_changed(self, idx: int):
        self.serial_widget.setVisible(idx == 0)
        self.wifi_widget.setVisible(idx == 1)

    def _build_chassis_box(self) -> QGroupBox:
        gb = QGroupBox("底盘控制")
        v = QVBoxLayout(gb); v.setSpacing(4); v.setContentsMargins(8, 10, 8, 8)

        # 键盘控制提示（紧凑单行）
        tip = QLabel("W/A/S/D 平移 · Q/E 自转 · Space 停止")
        tip.setAlignment(Qt.AlignCenter)
        tip.setStyleSheet("color: #aaa; font-size: 9pt;")
        v.addWidget(tip)

        # 速度显示（单行大字体，等宽数字）
        self.lbl_vel = QLabel("Vx= +0.00   Vy= +0.00   Vw= +0.00")
        self.lbl_vel.setAlignment(Qt.AlignCenter)
        self.lbl_vel.setFont(QFont("Consolas", 10, QFont.Bold))
        self.lbl_vel.setStyleSheet("color: #9cdcfe; padding: 2px;")
        v.addWidget(self.lbl_vel)

        v.addWidget(self._hline())

        # 手动速度（紧凑：标签和数值框同一行）
        self.spn_vx = QDoubleSpinBox(); self.spn_vx.setRange(-MAX_VX, MAX_VX); self.spn_vx.setSingleStep(0.05); self.spn_vx.setDecimals(2)
        self.spn_vy = QDoubleSpinBox(); self.spn_vy.setRange(-MAX_VY, MAX_VY); self.spn_vy.setSingleStep(0.05); self.spn_vy.setDecimals(2)
        self.spn_vw = QDoubleSpinBox(); self.spn_vw.setRange(-MAX_VW, MAX_VW); self.spn_vw.setSingleStep(0.1);  self.spn_vw.setDecimals(2)
        for s, lbl in zip((self.spn_vx, self.spn_vy, self.spn_vw), ("Vx", "Vy", "Vw")):
            row = QHBoxLayout(); row.setSpacing(4)
            row.addWidget(QLabel(lbl)); row.addWidget(s, 1)
            v.addLayout(row)
        btn_apply = QPushButton("应用速度 (单次)")
        btn_apply.clicked.connect(self._on_apply_velocity)
        v.addWidget(btn_apply)

        v.addStretch(1)

        v.addStretch(1)
        return gb

    def _build_motors_box(self) -> QGroupBox:
        gb = QGroupBox("电机调参")
        h = QHBoxLayout(gb); h.setSpacing(10); h.setContentsMargins(10, 14, 10, 10)
        self.motor_cards = []
        self._detail_windows = []
        for i in range(3):
            card = MotorCard(i)
            self.motor_cards.append(card)
            card.sig_changed.connect(self._on_pid_changed)
            card.sig_double_clicked.connect(self._open_motor_detail)
            h.addWidget(card, 1)
        return gb

    def _build_bottom_bar(self) -> QHBoxLayout:
        h = QHBoxLayout(); h.setSpacing(10)
        self.btn_stop = QPushButton("⛔ 紧急停止"); self.btn_stop.setObjectName("danger")
        self.btn_stop.setMinimumHeight(40)
        self.btn_stop.clicked.connect(self._on_emergency_stop)
        h.addWidget(self.btn_stop)

        self.btn_reset = QPushButton("↺ 恢复默认 PID")
        self.btn_reset.clicked.connect(self._on_reset_pid)
        h.addWidget(self.btn_reset)

        self.btn_ping = QPushButton("Ping")
        self.btn_ping.clicked.connect(self._on_ping)
        h.addWidget(self.btn_ping)

        h.addStretch(1)
        tip = QLabel("提示：WASD 平移  Q/E 自转  1/2/3 选电机  +/- kp  [/] ki  ,/. kd   |   双击电机卡 = 放大曲线")
        tip.setObjectName("tiny")
        h.addWidget(tip)

        self.btn_help = QPushButton("❓ 使用说明")
        self.btn_help.clicked.connect(self._on_help)
        h.addWidget(self.btn_help)
        return h

    # ---------- 小工具 ----------
    def _hline(self) -> QFrame:
        f = QFrame(); f.setFrameShape(QFrame.HLine); f.setStyleSheet("color:#3a3d41;"); return f

    # ---------- 帮助 ----------
    def _on_help(self):
        HelpDialog(self).exec()

    # ---------- 串口 ----------
    def _refresh_ports(self):
        cur = self.cmb_port.currentText()
        self.cmb_port.clear(); self.cmb_port.addItems(list_serial_ports())
        if cur: self.cmb_port.setCurrentText(cur)

    def _on_connect(self):
        mode = self.cmb_mode.currentIndex()
        try:
            if mode == 0:
                port = self.cmb_port.currentText()
                baud = int(self.cmb_baud.currentText())
                self.link.open(port, baud)
                ep = f"{port}@{baud}"
            else:
                host = self.ed_host.currentText().strip()
                port = int(self.ed_tcp_port.currentText())
                self.link.open_tcp(host, port)
                ep = f"{host}:{port}"
        except Exception as e:
            self.status.showMessage(f"连接失败: {e}", 5000)
            return
        self.btn_connect.setEnabled(False); self.btn_disconnect.setEnabled(True)
        self.lbl_conn.setText("● 已连接"); self.lbl_conn.setStyleSheet("color:#7ddf7d;")
        self.status.showMessage(f"已连接 {ep}")

    def _on_disconnect(self):
        self.link.close()
        self.btn_connect.setEnabled(True); self.btn_disconnect.setEnabled(False)
        self.lbl_conn.setText("● 未连接"); self.lbl_conn.setStyleSheet("color:#888;")
        self.status.showMessage("已断开")

    def closeEvent(self, ev):
        self.link.close()
        super().closeEvent(ev)

    # ---------- 控制 ----------
    def keyPressEvent(self, ev):
        t = ev.text().lower()
        if t in self._velocity_keys: self.pressed.add(t)
        elif ev.key() == Qt.Key_Space: self._on_emergency_stop()
        elif ev.key() in (Qt.Key_1, Qt.Key_2, Qt.Key_3):
            self.motor_cards[ev.key() - Qt.Key_1].set_selected(True)
            for i, c in enumerate(self.motor_cards):
                if i != ev.key() - Qt.Key_1: c.set_selected(False)
        elif t in ("+", "=", "-", "_", "[", "]", ",", "."):
            idx = next((i for i, c in enumerate(self.motor_cards) if c.selected), 0)
            self.motor_cards[idx].adjust_pid(t)
    def keyReleaseEvent(self, ev):
        t = ev.text().lower()
        if t in self._velocity_keys: self.pressed.discard(t)

    def _on_apply_velocity(self):
        if not self.link.is_open: return
        self.link.set_velocity(self.spn_vx.value(), self.spn_vy.value(), self.spn_vw.value())

    def _on_emergency_stop(self):
        self.vx = self.vy = self.vw = 0.0
        self.pressed.clear()
        if self.link.is_open: self.link.stop()

    def _on_reset_pid(self):
        for i, card in enumerate(self.motor_cards):
            card.set_pid(*PID_DEFAULTS[i])
            if self.link.is_open:
                self.link.set_pid(i, *PID_DEFAULTS[i])

    def _on_ping(self):
        if self.link.is_open: self.link.ping()

    def _on_pid_changed(self, idx: int, kp: float, ki: float, kd: float):
        if self.link.is_open: self.link.set_pid(idx, kp, ki, kd)

    def _open_motor_detail(self, idx: int):
        # 已开就 focus 已有窗口
        if idx < len(self._detail_windows) and self._detail_windows[idx] is not None:
            w = self._detail_windows[idx]
            w.show(); w.raise_(); w.activateWindow()
            return
        w = MotorDetailWindow(idx, self.plot_panel, self)
        w.show()
        # 扩展列表
        while len(self._detail_windows) <= idx:
            self._detail_windows.append(None)
        self._detail_windows[idx] = w
        w.destroyed.connect(lambda _=None, i=idx: self._on_detail_closed(i))

    def _on_detail_closed(self, idx: int):
        if idx < len(self._detail_windows):
            self._detail_windows[idx] = None

    # ---------- 周期任务 ----------
    def _tick_send(self):
        """根据按下的键增量计算速度，然后发送。"""
        # 持续按键 = 增量；松开 = 衰减到 0
        if "w" in self.pressed: self.vy = min( MAX_VY, self.vy + 0.2)
        if "s" in self.pressed: self.vy = max(-MAX_VY, self.vy - 0.2)
        if "a" in self.pressed: self.vx = max(-MAX_VX, self.vx - 0.2)
        if "d" in self.pressed: self.vx = min( MAX_VX, self.vx + 0.2)
        if "q" in self.pressed: self.vw = min( MAX_VW, self.vw + 0.4)
        if "e" in self.pressed: self.vw = max(-MAX_VW, self.vw - 0.4)

        # 衰减（按键释放后自然回到 0）
        decay = 0.85
        if "w" not in self.pressed and "s" not in self.pressed: self.vy *= decay
        if "a" not in self.pressed and "d" not in self.pressed: self.vx *= decay
        if "q" not in self.pressed and "e" not in self.pressed: self.vw *= decay
        if abs(self.vy) < 0.005: self.vy = 0.0
        if abs(self.vx) < 0.005: self.vx = 0.0
        if abs(self.vw) < 0.005: self.vw = 0.0

        if self.link.is_open:
            self.link.set_velocity(self.vx, self.vy, self.vw)

    def _tick_ui(self):
        """根据 ESP 上报的状态刷新 UI。"""
        s = self.link.state
        self.lbl_vel.setText(f"Vx= {s.vx:+5.2f}   Vy= {s.vy:+5.2f}   Vw= {s.vw:+5.2f}")

        for i, card in enumerate(self.motor_cards):
            card.update_state(
                target=s.target_rpm[i],
                real=s.real_rpm[i],
                kp=s.pid_kp[i], ki=s.pid_ki[i], kd=s.pid_kd[i],
            )

        # 更新 info_bar
        if self.link.is_open:
            self.lbl_info_conn.setText(f"●  已连接  {self.link.endpoint}")
            self.lbl_info_conn.setStyleSheet("font-size: 11pt; color: #7ddf7d;")
        else:
            self.lbl_info_conn.setText("●  未连接")
            self.lbl_info_conn.setStyleSheet("font-size: 11pt; color: #888;")
        if s.updated_at > 0:
            age = time.time() - s.updated_at
            if age < 2:
                self.lbl_info_data.setText(f"最近状态数据：{age:.1f} 秒前（实时）")
                self.lbl_info_data.setStyleSheet("font-size: 10pt; color: #7ddf7d;")
            elif age < 10:
                self.lbl_info_data.setText(f"最近状态数据：{age:.1f} 秒前（延迟）")
                self.lbl_info_data.setStyleSheet("font-size: 10pt; color: #f0c060;")
            else:
                self.lbl_info_data.setText(f"最近状态数据：{age:.0f} 秒前（断流？）")
                self.lbl_info_data.setStyleSheet("font-size: 10pt; color: #f08080;")
        else:
            self.lbl_info_data.setText("最近状态数据：—")

        # 推到曲线（即使没有收到新帧也推一帧空点，保证时间轴走）
        now = time.time()
        self.plot_panel.push(now,
            target=[s.target_rpm[0], s.target_rpm[1], s.target_rpm[2]],
            real=  [s.real_rpm[0],   s.real_rpm[1],   s.real_rpm[2]],
            vx=s.vx, vy=s.vy, vw=s.vw,
            kp=s.pid_kp, ki=s.pid_ki, kd=s.pid_kd)


# ===================== 实时曲线面板 =====================
class PlotPanel(QWidget):
    """3 个电机 RPM (target/real) + 底盘速度 (Vx/Vy/Vw) 实时曲线，10s 窗口自动滚动。"""

    WINDOW_SEC = 10.0
    MAX_POINTS = 600  # ~10s @ 60Hz 上限

    def __init__(self):
        super().__init__()
        pg.setConfigOptions(antialias=True, background="#16171a", foreground="#c0c0c0")

        v = QVBoxLayout(self); v.setContentsMargins(0, 0, 0, 0); v.setSpacing(2)

        # 顶部控件栏
        bar = QHBoxLayout()
        bar.setSpacing(4)
        self.chk_pause = QCheckBox("暂停")
        self.chk_pause.toggled.connect(self._on_pause)
        bar.addWidget(self.chk_pause)
        bar.addWidget(QLabel("  窗口:"))
        self.cmb_window = QComboBox(); self.cmb_window.addItems(["5 秒", "10 秒", "30 秒", "60 秒"])
        self.cmb_window.setCurrentText("10 秒")
        self.cmb_window.currentIndexChanged.connect(self._on_window)
        bar.addWidget(self.cmb_window)
        bar.addStretch(1)
        btn_zoom = QPushButton("🔍 放大到整段")
        btn_zoom.setToolTip("X 轴展开到全部历史范围")
        btn_zoom.clicked.connect(self._zoom_full)
        bar.addWidget(btn_zoom)
        btn_reset = QPushButton("⤺ 重置视图")
        btn_reset.clicked.connect(self._zoom_reset)
        bar.addWidget(btn_reset)
        self.lbl_rate = QLabel("— Hz")
        bar.addWidget(self.lbl_rate)
        v.addLayout(bar)

        # 绘图区：2 个子图（3 电机合并 + 速度）
        self.gw = pg.GraphicsLayoutWidget()
        self.gw.setBackground("#16171a")
        self.gw.ci.layout.setSpacing(2)
        v.addWidget(self.gw, 1)

        self.plots_motor = []     # 兼容旧代码
        self.curves_tgt  = [None, None, None]
        self.curves_real = [None, None, None]
        self.plot_vel = None
        self.curves_vel = {}

        # 跨图十字光标（竖线 + 顶部时间标签）
        self._vlines = []
        self.time_label = pg.TextItem(anchor=(0, 1), color="#e0e0e0", fill=pg.mkBrush(20,20,20,200))
        self.time_label.setParentItem(self.gw.getItem(row=0, col=0))
        self.time_label.setPos(0, 0)

        # 电机 RPM 图：3 电机 target(虚线) + real(实线) 合在一张
        self.plot_motor = self.gw.addPlot(row=0, col=0)
        self.plot_motor.showGrid(x=False, y=True, alpha=0.2)
        self.plot_motor.setLabel("left", "RPM")
        self.plot_motor.setLabel("bottom", "time", units="s")
        self.plot_motor.setMouseEnabled(x=True, y=True)
        self.plot_motor.setDownsampling(auto=True)
        self.plot_motor.setClipToView(True)
        self.plot_motor.setMinimumHeight(280)
        self.plot_motor.addLegend(offset=(-10, 10))
        tgt_colors  = ["#f0c060", "#90d090", "#80b0ff"]
        real_colors = ["#ff8030", "#30c060", "#3070ff"]
        for i in range(3):
            self.curves_tgt[i]  = self.plot_motor.plot(
                pen=pg.mkPen(tgt_colors[i],  width=1.5, style=Qt.DashLine),
                name=f"M{i+1}·tgt")
            self.curves_real[i] = self.plot_motor.plot(
                pen=pg.mkPen(real_colors[i], width=2.5),
                name=f"M{i+1}·real")
        self._vlines.append(pg.InfiniteLine(angle=90, movable=False,
                                            pen=pg.mkPen("#7090ff", width=1, style=Qt.DashLine)))
        self.plot_motor.addItem(self._vlines[0])
        self.plots_motor.append(self.plot_motor)

        # 速度图
        self.plot_vel = self.gw.addPlot(row=1, col=0)
        self.plot_vel.showGrid(x=False, y=True, alpha=0.2)
        self.plot_vel.setLabel("left", "Vel")
        self.plot_vel.setLabel("bottom", "time", units="s")
        self.plot_vel.setMouseEnabled(x=True, y=True)
        self.plot_vel.setDownsampling(auto=True)
        self.plot_vel.setClipToView(True)
        self.plot_vel.setMinimumHeight(180)
        self.plot_vel.setXLink(self.plot_motor)
        self.plot_vel.addLegend(offset=(-10, 10))
        self.curves_vel["vx"] = self.plot_vel.plot(pen=pg.mkPen("#ff7070", width=1.5), name="Vx")
        self.curves_vel["vy"] = self.plot_vel.plot(pen=pg.mkPen("#70ff70", width=1.5), name="Vy")
        self.curves_vel["vw"] = self.plot_vel.plot(pen=pg.mkPen("#7090ff", width=1.5), name="Vw")
        self._vlines.append(pg.InfiniteLine(angle=90, movable=False,
                                            pen=pg.mkPen("#7090ff", width=1, style=Qt.DashLine)))
        self.plot_vel.addItem(self._vlines[1])

        self.plot_motor.scene().sigMouseMoved.connect(self._on_mouse_moved)
        self._hide_crosshair()

        # 底部提示（紧凑）
        hint = QLabel("左键拖=平移  滚轮=缩放  双击=自适应  鼠标移动=精确值")
        hint.setObjectName("tiny")
        hint.setMaximumHeight(16)
        v.addWidget(hint)

        # 数据缓冲
        self.t = deque(maxlen=self.MAX_POINTS)
        self.tgt = [deque(maxlen=self.MAX_POINTS) for _ in range(3)]
        self.real = [deque(maxlen=self.MAX_POINTS) for _ in range(3)]
        self.vx = deque(maxlen=self.MAX_POINTS)
        self.vy = deque(maxlen=self.MAX_POINTS)
        self.vw = deque(maxlen=self.MAX_POINTS)
        # PID 历史（详情窗口用）
        self.kp = [deque(maxlen=self.MAX_POINTS) for _ in range(3)]
        self.ki = [deque(maxlen=self.MAX_POINTS) for _ in range(3)]
        self.kd = [deque(maxlen=self.MAX_POINTS) for _ in range(3)]

        self._t0 = None
        self._paused = False
        self._last_push = 0.0

    # ---------- 配置 ----------
    def _on_pause(self, on: bool):
        self._paused = on

    def _on_window(self, _idx: int):
        m = {"5 秒": 5, "10 秒": 10, "30 秒": 30, "60 秒": 60}[self.cmb_window.currentText()]
        self.WINDOW_SEC = float(m)

    def _zoom_full(self):
        """展开到全部历史范围"""
        if not self.t: return
        xmin, xmax = min(self.t), max(self.t)
        if xmax <= xmin: xmax = xmin + 1.0
        for p in self.plots_motor + [self.plot_vel]:
            p.setXRange(xmin, xmax, padding=0.02)

    def _zoom_reset(self):
        """恢复自动滚动"""
        self._user_xrange = None
        # 下一帧 push 会按 WINDOW_SEC 自动重设 X 范围

    # ---------- 数据 ----------
    def push(self, now: float, target, real, vx, vy, vw, kp=None, ki=None, kd=None):
        if self._paused:
            return
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        self.t.append(t)
        for i in range(3):
            self.tgt[i].append(target[i])
            self.real[i].append(real[i])
            if kp: self.kp[i].append(kp[i])
            if ki: self.ki[i].append(ki[i])
            if kd: self.kd[i].append(kd[i])
        self.vx.append(vx); self.vy.append(vy); self.vw.append(vw)

        # 刷新曲线
        xs = list(self.t)
        for i in range(3):
            self.curves_tgt[i].setData(xs, list(self.tgt[i]))
            self.curves_real[i].setData(xs, list(self.real[i]))
        self.curves_vel["vx"].setData(xs, list(self.vx))
        self.curves_vel["vy"].setData(xs, list(self.vy))
        self.curves_vel["vw"].setData(xs, list(self.vw))

        # 自动滚动
        xmax = max(t, self.WINDOW_SEC)
        xmin = max(0.0, xmax - self.WINDOW_SEC)
        for p in self.plots_motor + [self.plot_vel]:
            p.setXRange(xmin, xmax, padding=0)

        # 刷新率显示
        if self._last_push > 0:
            dt = now - self._last_push
            if dt > 0:
                self.lbl_rate.setText(f"~{1.0/dt:.1f} Hz")
        self._last_push = now

    # ---------- 鼠标交互 ----------
    def _hide_crosshair(self):
        for ln in self._vlines:
            ln.setVisible(False)
        self.time_label.setVisible(False)

    def _on_mouse_moved(self, pos):
        """鼠标在第一个 plot 区域里移动 → 同步更新所有子图的竖线和数值标签。"""
        p0 = self.plots_motor[0]
        if not p0.sceneBoundingRect().contains(pos):
            self._hide_crosshair()
            return
        mouse_point = p0.vb.mapSceneToView(pos)
        x = mouse_point.x()
        if len(self.t) == 0:
            self._hide_crosshair()
            return
        # 找到最接近的时间索引
        ts = list(self.t)
        idx = min(range(len(ts)), key=lambda i: abs(ts[i] - x))
        if idx < 0 or idx >= len(ts):
            self._hide_crosshair()
            return

        t_val = ts[idx]
        for ln in self._vlines:
            ln.setPos(t_val); ln.setVisible(True)

        # 顶部时间 + 数值标签
        text = f"t = {t_val:6.2f}s\n"
        text += f"M1: tgt={self.tgt[0][idx]:6.1f}  real={self.real[0][idx]:6.1f}\n"
        text += f"M2: tgt={self.tgt[1][idx]:6.1f}  real={self.real[1][idx]:6.1f}\n"
        text += f"M3: tgt={self.tgt[2][idx]:6.1f}  real={self.real[2][idx]:6.1f}\n"
        text += f"V=({self.vx[idx]:+.2f}, {self.vy[idx]:+.2f}, {self.vw[idx]:+.2f})"
        self.time_label.setText(text)
        self.time_label.setPos(t_val, p0.viewRange()[1][1])
        self.time_label.setVisible(True)

    def reset(self):
        self._t0 = None
        for d in [self.t, *self.tgt, *self.real, self.vx, self.vy, self.vw]:
            d.clear()


# ===================== 电机详情窗口（放大曲线 + PID 趋势） =====================
class MotorDetailWindow(QWidget):
    """单电机的全历史曲线：RPM(target/real) + kp/ki/kd 趋势。
    共享主窗口 PlotPanel 的数据缓冲，所以零拷贝。"""

    def __init__(self, idx: int, plot_panel: PlotPanel, parent=None):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle(f"电机 {idx + 1} 详情")
        self.resize(820, 560)
        self.setStyleSheet("background: #1e1f22; color: #e6e6e6;")

        self.idx = idx
        self.src = plot_panel  # 数据源
        self._t0 = None

        v = QVBoxLayout(self); v.setContentsMargins(6, 6, 6, 6); v.setSpacing(2)

        # 顶部控件栏
        bar = QHBoxLayout()
        bar.setSpacing(4)
        self.chk_pause = QCheckBox("暂停")
        bar.addWidget(self.chk_pause)
        bar.addWidget(QLabel("  窗口:"))
        self.cmb_window = QComboBox(); self.cmb_window.addItems(["10 秒", "30 秒", "60 秒", "全部"])
        self.cmb_window.setCurrentText("60 秒")
        self.cmb_window.currentIndexChanged.connect(self._on_window)
        bar.addWidget(self.cmb_window)
        bar.addStretch(1)
        self.lbl_info = QLabel("—")
        self.lbl_info.setObjectName("mid")
        bar.addWidget(self.lbl_info)
        v.addLayout(bar)

        # 绘图区
        self.gw = pg.GraphicsLayoutWidget()
        self.gw.setBackground("#16171a")
        self.gw.ci.layout.setSpacing(2)
        v.addWidget(self.gw, 1)

        # RPM 图
        self.plot_rpm = self.gw.addPlot(row=0, col=0)
        self.plot_rpm.showGrid(x=False, y=True, alpha=0.2)
        self.plot_rpm.setLabel("left", "RPM")
        self.plot_rpm.setMouseEnabled(x=True, y=True)
        self.plot_rpm.setDownsampling(auto=True)
        self.plot_rpm.setClipToView(True)
        self.plot_rpm.setMinimumHeight(160)
        self.plot_rpm.hideAxis("bottom")
        self.plot_rpm.addLegend(offset=(-10, 10))
        self.curve_tgt  = self.plot_rpm.plot(pen=pg.mkPen("#f0c060", width=1.5, style=Qt.DashLine), name="target")
        self.curve_real = self.plot_rpm.plot(pen=pg.mkPen("#ff8030", width=2.5), name="real")

        # PID 图
        self.plot_pid = self.gw.addPlot(row=1, col=0)
        self.plot_pid.showGrid(x=False, y=True, alpha=0.2)
        self.plot_pid.setLabel("left", "PID")
        self.plot_pid.setLabel("bottom", "time", units="s")
        self.plot_pid.setMouseEnabled(x=True, y=True)
        self.plot_pid.setDownsampling(auto=True)
        self.plot_pid.setClipToView(True)
        self.plot_pid.setMinimumHeight(160)
        self.plot_pid.setXLink(self.plot_rpm)
        self.plot_pid.addLegend(offset=(-10, 10))
        self.curve_kp = self.plot_pid.plot(pen=pg.mkPen("#ff7070", width=1.5), name="kp")
        self.curve_ki = self.plot_pid.plot(pen=pg.mkPen("#70ff70", width=1.5), name="ki")
        self.curve_kd = self.plot_pid.plot(pen=pg.mkPen("#7090ff", width=1.5), name="kd")

        # 跨图竖线 + 标签
        self.vline_rpm = pg.InfiniteLine(angle=90, movable=False,
                                          pen=pg.mkPen("#7090ff", width=1, style=Qt.DashLine))
        self.vline_pid = pg.InfiniteLine(angle=90, movable=False,
                                          pen=pg.mkPen("#7090ff", width=1, style=Qt.DashLine))
        self.plot_rpm.addItem(self.vline_rpm); self.plot_pid.addItem(self.vline_pid)
        self.label = pg.TextItem(anchor=(0, 1), color="#e0e0e0", fill=pg.mkBrush(20, 20, 20, 200))
        self.label.setParentItem(self.plot_rpm)
        self.plot_rpm.scene().sigMouseMoved.connect(self._on_mouse_moved)

        self.WINDOW_SEC = 60.0
        self._frozen_data = None  # 暂停时锁定快照

        # 定时刷新（20Hz 即可，无需太密）
        self.timer = QTimer(); self.timer.setInterval(50); self.timer.timeout.connect(self._refresh)
        self.timer.start()

        # 关闭时停定时器
        self.destroyed.connect(lambda _=None: self._stop())

    def _stop(self):
        if self.timer.isActive(): self.timer.stop()

    def _on_window(self, _idx):
        sel = self.cmb_window.currentText()
        self.WINDOW_SEC = 1e9 if sel == "全部" else {"10 秒": 10, "30 秒": 30, "60 秒": 60}[sel]

    def _refresh(self):
        src = self.src
        if not src.t:
            return
        ts = list(src.t); n = len(ts)

        # 暂停时固定快照，否则每次拷贝最新缓冲
        if self.chk_pause.isChecked():
            if self._frozen_data is None or len(self._frozen_data[0]) != n:
                self._frozen_data = (
                    list(src.tgt[self.idx]), list(src.real[self.idx]),
                    list(src.kp[self.idx]),  list(src.ki[self.idx]),  list(src.kd[self.idx]),
                )
            tgt, real, kp, ki, kd = self._frozen_data
        else:
            self._frozen_data = None
            tgt = list(src.tgt[self.idx]); real = list(src.real[self.idx])
            kp  = list(src.kp[self.idx]);  ki  = list(src.ki[self.idx]);  kd = list(src.kd[self.idx])

        xs = ts
        self.curve_tgt.setData(xs, tgt)
        self.curve_real.setData(xs, real)
        self.curve_kp.setData(xs, kp)
        self.curve_ki.setData(xs, ki)
        self.curve_kd.setData(xs, kd)

        # X 范围
        xmax = ts[-1]
        xmin = max(0.0, xmax - self.WINDOW_SEC) if self.WINDOW_SEC < 1e8 else ts[0]
        self.plot_rpm.setXRange(xmin, xmax, padding=0.02)
        self.plot_pid.setXRange(xmin, xmax, padding=0.02)

        # 顶部状态
        i = n - 1
        self.lbl_info.setText(
            f"t={ts[i]:.1f}s   target={tgt[i]:+.1f}   real={real[i]:+.1f}   "
            f"kp={kp[i]:.3f}  ki={ki[i]:.3f}  kd={kd[i]:.3f}"
        )

    def _on_mouse_moved(self, pos):
        if not self.src.t: return
        if not self.plot_rpm.sceneBoundingRect().contains(pos):
            self.vline_rpm.setVisible(False); self.vline_pid.setVisible(False); self.label.setVisible(False)
            return
        mp = self.plot_rpm.vb.mapSceneToView(pos)
        x = mp.x()
        ts = list(self.src.t)
        idx = min(range(len(ts)), key=lambda i: abs(ts[i] - x))
        if idx < 0 or idx >= len(ts): return
        t_val = ts[idx]
        self.vline_rpm.setPos(t_val); self.vline_pid.setPos(t_val)
        self.vline_rpm.setVisible(True); self.vline_pid.setVisible(True)
        tgt = self.src.tgt[self.idx][idx]; real = self.src.real[self.idx][idx]
        kp  = self.src.kp[self.idx][idx];   ki  = self.src.ki[self.idx][idx];   kd  = self.src.kd[self.idx][idx]
        text = f"t = {t_val:6.2f}s\ntarget = {tgt:6.1f}\nreal   = {real:6.1f}\nkp={kp:+.3f}  ki={ki:+.3f}  kd={kd:+.3f}"
        self.label.setText(text)
        self.label.setPos(t_val, self.plot_rpm.viewRange()[1][1])
        self.label.setVisible(True)


# ===================== 电机卡片 =====================
class MotorCard(QGroupBox):
    sig_changed = Signal(int, float, float, float)  # idx, kp, ki, kd
    sig_double_clicked = Signal(int)                  # idx

    def __init__(self, idx: int):
        super().__init__(f"电机 {idx + 1}")
        self.idx = idx
        self.selected = False
        self._user_dragging = False
        self.setToolTip("双击打开详情窗口（放大曲线 + PID 趋势）")

        v = QVBoxLayout(self); v.setSpacing(6); v.setContentsMargins(10, 14, 10, 10)

        # RPM 显示
        self.lbl_target = QLabel("目标:  0.0"); self.lbl_target.setObjectName("mid")
        self.lbl_real   = QLabel("实测:  0.0"); self.lbl_real.setObjectName("big")
        v.addWidget(self.lbl_target); v.addWidget(self.lbl_real)

        # RPM 进度条
        self.bar_rpm = QSlider(Qt.Horizontal); self.bar_rpm.setRange(-300, 300); self.bar_rpm.setEnabled(False)
        v.addWidget(self.bar_rpm)

        v.addWidget(self._hline())

        # PID 三组：滑条 + 数值框
        self.sld_kp = self._pid_slider(v, "kp", -10.0, 10.0)
        self.sld_ki = self._pid_slider(v, "ki", -10.0, 10.0)
        self.sld_kd = self._pid_slider(v, "kd", -10.0, 10.0)
        # 初始值
        kp, ki, kd = PID_DEFAULTS[idx]
        self.spn_kp.setValue(kp); self.sld_kp.setValue(int(kp * 1000))
        self.spn_ki.setValue(ki); self.sld_ki.setValue(int(ki * 1000))
        self.spn_kd.setValue(kd); self.sld_kd.setValue(int(kd * 1000))

        v.addStretch(1)
        self.set_selected(False)

    def _pid_slider(self, parent_layout, name: str, lo: float, hi: float):
        row = QHBoxLayout()
        lbl = QLabel(name); lbl.setFixedWidth(28)
        sld = QSlider(Qt.Horizontal); sld.setRange(int(lo*1000), int(hi*1000))
        spn = QDoubleSpinBox(); spn.setRange(lo, hi); spn.setSingleStep(0.05); spn.setDecimals(3)
        spn.setFixedWidth(80)
        sld.valueChanged.connect(lambda v, s=spn: s.setValue(v/1000))
        spn.valueChanged.connect(lambda v, s=sld: s.setValue(int(v*1000)))
        spn.valueChanged.connect(self._emit)
        row.addWidget(lbl); row.addWidget(sld, 1); row.addWidget(spn)
        parent_layout.addLayout(row)
        # 缓存到 self
        if name == "kp": self.spn_kp, self.sld_kp = spn, sld
        elif name == "ki": self.spn_ki, self.sld_ki = spn, sld
        else:              self.spn_kd, self.sld_kd = spn, sld
        return sld

    def _hline(self):
        f = QFrame(); f.setFrameShape(QFrame.HLine); f.setStyleSheet("color:#3a3d41;")
        return f

    def _emit(self, *_):
        self.sig_changed.emit(self.idx, self.spn_kp.value(), self.spn_ki.value(), self.spn_kd.value())

    def set_pid(self, kp: float, ki: float, kd: float):
        self.spn_kp.setValue(kp); self.spn_ki.setValue(ki); self.spn_kd.setValue(kd)

    def set_selected(self, on: bool):
        self.selected = on
        border = "#5080b0" if on else "#3a3d41"
        self.setStyleSheet(f"QGroupBox {{ border:2px solid {border}; }}")

    def adjust_pid(self, key: str):
        step = 0.05
        if key in ("+", "="): self.spn_kp.setValue(self.spn_kp.value() + step)
        elif key in ("-", "_"): self.spn_kp.setValue(self.spn_kp.value() - step)
        elif key == "[": self.spn_ki.setValue(self.spn_ki.value() - step)
        elif key == "]": self.spn_ki.setValue(self.spn_ki.value() + step)
        elif key == ",": self.spn_kd.setValue(self.spn_kd.value() - step)
        elif key == ".": self.spn_kd.setValue(self.spn_kd.value() + step)

    def update_state(self, target, real, kp, ki, kd):
        self.lbl_target.setText(f"目标: {target:+7.1f}")
        self.lbl_real.setText(f"实测: {real:+7.1f}")
        self.bar_rpm.setValue(int(max(-300, min(300, real))))

    def mouseDoubleClickEvent(self, ev):
        self.sig_double_clicked.emit(self.idx)
        super().mouseDoubleClickEvent(ev)


def main():
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec())


# ===================== 使用说明对话框 =====================
QUICKSTART_HTML = """
<style>
  body { font-family: "Segoe UI","Microsoft YaHei","PingFang SC",sans-serif; font-size: 10pt; }
  h2   { color: #9cdcfe; margin-top: 4px; }
  h3   { color: #c0c0c0; margin-top: 14px; margin-bottom: 4px; }
  code { background: #2a2d31; padding: 1px 6px; border-radius: 3px; color: #ce9178; }
  pre  { background: #16171a; padding: 8px; border-radius: 4px; color: #c8c8c8;
         font-family: Consolas, monospace; }
  table{ border-collapse: collapse; margin: 6px 0; }
  td,th{ border: 1px solid #3a3d41; padding: 4px 10px; }
  th   { background: #2a2d31; }
  .ok  { color: #7ddf7d; }
  .warn{ color: #f0c060; }
</style>

<h2>1 分钟上手</h2>
<ol>
  <li>烧固件：<code>idf.py -p COMx flash</code>（确保 <code>idf.py monitor</code> 已关）。</li>
  <li>开 GUI：<code>python tools/debug_panel_gui.py</code>。</li>
  <li>顶栏选连接方式：
    <ul>
      <li><b>串口</b>：选 COM 口 → 连接。</li>
      <li><b>WiFi</b>：ESP32 上电后建 WiFi <code>ESP32-Car</code>（密码 <code>12345678</code>），PC 连上后填 <code>192.168.4.1</code> 或 <code>espcar.local</code> → 连接。</li>
    </ul>
  </li>
  <li><b>点一下窗口空白处取得焦点</b>，然后按 WASD/QE 让车动起来。</li>
  <li>拖卡片里的 PID 滑条实时调参。</li>
  <li>出问题按 <b>Space</b> 或 <b>⛔ 紧急停止</b>。</li>
</ol>

<h3>按键速查</h3>
<table>
  <tr><th>键</th><th>作用</th></tr>
  <tr><td><code>W A S D</code></td><td>平移（前/左/后/右）</td></tr>
  <tr><td><code>Q E</code></td><td>自转（顺/逆）</td></tr>
  <tr><td><code>1 2 3</code></td><td>选中调参电机</td></tr>
  <tr><td><code>+ -</code></td><td>当前电机 kp ±0.05</td></tr>
  <tr><td><code>[ ]</code></td><td>当前电机 ki ±0.05</td></tr>
  <tr><td><code>, .</code></td><td>当前电机 kd ±0.05</td></tr>
  <tr><td><code>Space</code></td><td>紧急停止</td></tr>
</table>

<p class="warn">⚠ 窗口必须先点一下获得焦点，按键才有效；焦点在 SpinBox 上时 WASD 会被吃掉。</p>
"""

HELP_HTML = """
<style>
  body { font-family: "Segoe UI","Microsoft YaHei","PingFang SC",sans-serif; font-size: 10pt; }
  h2   { color: #9cdcfe; margin-top: 18px; margin-bottom: 6px; }
  h3   { color: #c0c0c0; margin-top: 14px; margin-bottom: 4px; }
  table{ border-collapse: collapse; margin: 6px 0; }
  td,th{ border: 1px solid #3a3d41; padding: 4px 10px; }
  th   { background: #2a2d31; }
  code { background: #2a2d31; padding: 1px 6px; border-radius: 3px; color: #ce9178; }
  pre  { background: #16171a; padding: 8px; border-radius: 4px; color: #c8c8c8;
         font-family: Consolas, "Cascadia Mono", monospace; }
  .ok  { color: #7ddf7d; }
  .warn{ color: #f0c060; }
  .err { color: #f08080; }
</style>

<h2>1. 连接方式</h2>
<p>顶栏左侧"串口 / WiFi"切换：</p>
<ul>
  <li><b>串口 (USB-UART)</b> — 默认。烧固件用的那根 USB 线，115200 8N1。</li>
  <li><b>WiFi (TCP)</b> — 无线。ESP32-S3 自己当 WiFi 热点（默认 SSID <code>ESP32-Car</code>，密码 <code>12345678</code>），PC 连上后用 TCP 连 <code>192.168.4.1:8888</code>，或 mDNS 名 <code>espcar.local:8888</code>。</li>
</ul>

<h3>三种网络场景</h3>
<table>
  <tr><th>你的情况</th><th>ESP32 模式</th><th>PC 怎么连</th><th>PC 还能上网吗</th></tr>
  <tr><td>没路由器</td><td>SoftAP（默认）</td><td>连 WiFi <code>ESP32-Car</code></td><td>✅ 网线/另一块 WiFi 继续</td></tr>
  <tr><td>有手机热点</td><td>STA 连手机</td><td>也连手机热点</td><td>✅ 共享上网</td></tr>
  <tr><td>有家用路由器</td><td>STA 连路由器</td><td>留在路由器上</td><td>✅ 完全不动</td></tr>
</table>
<p class="warn">⚠ 不要同时跑 <code>idf.py monitor</code> 和本工具，两者都要读 RX 会抢字节。</p>

<h2>2. 操作</h2>
<p>方向控制只接受<strong>键盘</strong>，没有 GUI 按钮。点击窗口取得焦点后再按。</p>
<table>
  <tr><th>动作</th><th>键盘</th><th>说明</th></tr>
  <tr><td>前进 / 后退</td><td><code>W</code> <code>S</code></td><td>按住持续加速</td></tr>
  <tr><td>左移 / 右移</td><td><code>A</code> <code>D</code></td><td>按住持续加速</td></tr>
  <tr><td>顺时针 / 逆时针自转</td><td><code>Q</code> <code>E</code></td><td>按住持续加速</td></tr>
  <tr><td>选中调参电机</td><td><code>1</code> <code>2</code> <code>3</code></td><td>边框变蓝为选中</td></tr>
  <tr><td>调 kp / ki / kd</td><td><code>+ / -</code> <code>[ / ]</code> <code>, / .</code></td><td>步长 0.05</td></tr>
  <tr><td>紧急停止</td><td><code>Space</code> 或 ⛔ 按钮</td><td>立即清零速度</td></tr>
  <tr><td>恢复默认 PID</td><td>↺ 按钮</td><td>三个电机都恢复</td></tr>
  <tr><td>请求状态</td><td>Ping 按钮</td><td>推一帧 STATE_REPORT</td></tr>
  <tr><td>查看单电机详情</td><td>双击电机卡</td><td>弹出放大曲线 + PID 趋势</td></tr>
</table>

<h2>3. 调参流程（建议）</h2>
<ol>
  <li>先把三个电机的 kp 设小一点（比如 0.5），i=d=0。</li>
  <li>低速前进（W），观察卡片里的 <i>目标</i> 与 <i>实测</i> RPM 差值。</li>
  <li>逐电机加大 kp，直到响应快但不抖动。</li>
  <li>如果稳态有差，加一点 ki（从 0.01 起）。</li>
  <li>最后极少量 kd 改善启动响应。</li>
  <li>觉得飞了按 <b>Space</b> 紧急停止。</li>
</ol>

<h2>4. 改 WiFi 设置</h2>
<pre>cd C:/Users/86181/Desktop/work/blink
idf.py menuconfig
# → WiFi Debug (SoftAP + TCP)
#   ├─ SoftAP SSID / password
#   ├─ SoftAP channel
#   ├─ TCP debug port
#   ├─ WiFi mode → "AP only" 或 "AP + STA"
#   └─ (AP+STA) STA SSID / password
idf.py -p COMx flash</pre>

<h2>5. 常见问题</h2>
<p><b>Q：串口下拉是空的？</b><br>
A：按一下 <code>刷新</code>。还没有就先装好 USB-UART 驱动。</p>

<p><b>Q：<code>espcar.local</code> 连不上？</b><br>
A：mDNS 解析依赖 PC 上有 Bonjour。Windows 装 iTunes / 苹果客户端会带上；或者直接填 <code>192.168.4.1</code>（SoftAP 默认 IP，永远有效）。</p>

<p><b>Q：连上了但 RPM 一直是 0？</b><br>
A：检查 <code>sdkconfig</code> 里 <code>CONFIG_ENCODER_COUNTS_PER_REVOLUTION</code> 是否填了电机每转一圈的四倍频计数；不填也能跑但只给 counts/s。</p>

<p><b>Q：连上后又断开？</b><br>
A：WiFi 调试只允许一个 client，被踢后重连就行。</p>

<p><b>Q：报 <code>ModuleNotFoundError: No module named 'PySide6'</code>？</b><br>
A：你装了别的 Python。找出你要用的那个：<br>
<pre>where python
C:/Users/86181/miniconda3/python.exe -m pip install pyserial pyside6</pre></p>
"""

class HelpDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("使用说明")
        self.resize(720, 580)
        self.setStyleSheet("""
            QDialog { background: #1e1f22; color: #e6e6e6; }
            QTabBar::tab { background: #2a2d31; padding: 6px 14px; }
            QTabBar::tab:selected { background: #3d6d9c; }
            QTextBrowser { background: #16171a; border: 1px solid #3a3d41;
                           padding: 12px; }
            QPushButton { background: #3d6d9c; border: 1px solid #5080b0;
                          padding: 6px 16px; border-radius: 4px; color: #fff; }
            QPushButton:hover { background: #5080b0; }
        """)

        v = QVBoxLayout(self); v.setContentsMargins(10, 10, 10, 10)
        tabs = QTabWidget()
        tabs.addTab(self._mk_page("快速上手", QUICKSTART_HTML), "快速上手")
        tabs.addTab(self._mk_page("完整说明", HELP_HTML),         "完整说明")
        tabs.addTab(self._mk_page("故障排查", TROUBLESHOOT_HTML),  "故障排查")
        v.addWidget(tabs)

        bb = QDialogButtonBox(QDialogButtonBox.Close)
        bb.rejected.connect(self.reject)
        v.addWidget(bb)

    def _mk_page(self, title: str, html: str) -> QWidget:
        w = QWidget(); lay = QVBoxLayout(w); lay.setContentsMargins(0, 0, 0, 0)
        tb = QTextBrowser(); tb.setOpenExternalLinks(True)
        tb.setHtml(html)
        lay.addWidget(tb)
        return w


TROUBLESHOOT_HTML = """
<style>
  body { font-family: "Segoe UI","Microsoft YaHei","PingFang SC",sans-serif; font-size: 10pt; }
  h2   { color: #9cdcfe; }
  code { background: #2a2d31; padding: 1px 6px; border-radius: 3px; color: #ce9178; }
  pre  { background: #16171a; padding: 8px; border-radius: 4px; color: #c8c8c8;
         font-family: Consolas, monospace; }
  table{ border-collapse: collapse; }
  td,th{ border: 1px solid #3a3d41; padding: 4px 10px; }
  th   { background: #2a2d31; }
</style>

<h2>故障排查速查表</h2>
<table>
  <tr><th>症状</th><th>原因 / 处理</th></tr>
  <tr>
    <td>窗口打不开 / <code>ModuleNotFoundError: PySide6</code></td>
    <td>多 Python 装到了别处。用 <code>where python</code> 看，给真正要用的那个装依赖：<br>
        <pre>C:/Users/86181/miniconda3/python.exe -m pip install pyserial pyside6</pre></td>
  </tr>
  <tr>
    <td>串口下拉是空的</td>
    <td>按 <b>刷新</b>。还没有说明 USB-UART 驱动没装好。</td>
  </tr>
  <tr>
    <td>连上了但 RPM 一直 0</td>
    <td><code>sdkconfig</code> 里 <code>CONFIG_ENCODER_COUNTS_PER_REVOLUTION</code> 没填。不填也能跑但只给 counts/s。</td>
  </tr>
  <tr>
    <td><code>espcar.local</code> 解析不到</td>
    <td>PC 上没装 Bonjour。改用 IP：<code>192.168.4.1:8888</code>。</td>
  </tr>
  <tr>
    <td>TCP 连接后立刻断</td>
    <td>ESP32 同时只允许一个 client；之前那个被踢了，重连。</td>
  </tr>
  <tr>
    <td>GUI 无反应 / 命令发不出去</td>
    <td>确认没有同时跑 <code>idf.py monitor</code>。</td>
  </tr>
  <tr>
    <td>电机不转</td>
    <td>检查 TB6612 接线、STBY 是否接 3V3、电机供电是否独立、<code>CONFIG_ENCODER_COUNTS_PER_REVOLUTION</code> 配置。</td>
  </tr>
  <tr>
    <td>按 WASD 没反应</td>
    <td>先把窗口 <b>点一下空白处</b> 取得焦点，再按键（焦点在 SpinBox/ComboBox 上时按键会被吃掉或切换选项）。</td>
  </tr>
</table>
"""


if __name__ == "__main__":
    main()
