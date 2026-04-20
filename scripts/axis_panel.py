#!/usr/bin/env python3
"""Qt control panel for rov_pid_controller.

Shows, per axis, the current mode / setpoint / pose / effort, and lets the user
toggle OFF / INNER / FULL and capture setpoints. Preset buttons at the top apply
common configurations via set_all_modes. Runs standalone (no rqt required)."""

import math
import signal
import sys
import threading
from functools import partial

import rclpy
from rclpy.node import Node
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QButtonGroup,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QRadioButton,
    QVBoxLayout,
    QWidget,
)

from rov_pid_controller.msg import ControllerStatus
from rov_pid_controller.srv import SetAllModes, SetAxisMode
from std_srvs.srv import Trigger

AXIS_NAMES = ["surge", "sway", "heave", "roll", "pitch", "yaw"]
# Angular axes are displayed in degrees in the panel (and in the status msg),
# but the service / internal PID still work in radians.
AXIS_ANGULAR = [False, False, False, True, True, True]
MODE_OFF, MODE_INNER, MODE_FULL = 0, 1, 2
MODE_NAMES = {MODE_OFF: "OFF", MODE_INNER: "INNER", MODE_FULL: "FULL"}


class PanelNode(Node):
    def __init__(self, namespace: str):
        super().__init__("rov_pid_controller_panel")
        self._ns = namespace.rstrip("/")

        self.latest_status = None
        self._status_lock = threading.Lock()

        status_topic   = self._topic("status")
        set_axis_name  = self._topic("set_axis_mode")
        set_all_name   = self._topic("set_all_modes")
        clear_name     = self._topic("clear_safety")

        self.get_logger().info(
            f"wiring to: status={status_topic} "
            f"set_axis_mode={set_axis_name} set_all_modes={set_all_name} "
            f"clear_safety={clear_name}"
        )

        self.create_subscription(ControllerStatus, status_topic, self._on_status, 10)
        self.set_axis_cli = self.create_client(SetAxisMode, set_axis_name)
        self.set_all_cli = self.create_client(SetAllModes, set_all_name)
        self.clear_safety_cli = self.create_client(Trigger, clear_name)
        self._warned = set()

    def _topic(self, name: str) -> str:
        return f"{self._ns}/{name}" if self._ns else name

    def _on_status(self, msg: ControllerStatus):
        with self._status_lock:
            self.latest_status = msg

    def get_status(self):
        with self._status_lock:
            return self.latest_status

    def _check_ready(self, client, label: str) -> bool:
        # Non-blocking: never stall the Qt thread on wait_for_service.
        if client.service_is_ready():
            return True
        if label not in self._warned:
            self.get_logger().warn(
                f"{label} service not yet discovered at "
                f"'{client.srv_name}'  (will keep trying silently)"
            )
            self._warned.add(label)
        return False

    def send_axis(self, axis: int, mode: int, setpoint: float, capture: bool):
        if not self._check_ready(self.set_axis_cli, "set_axis_mode"):
            return
        req = SetAxisMode.Request()
        req.axis = axis
        req.mode = mode
        req.setpoint = setpoint
        req.capture_current = capture
        self.set_axis_cli.call_async(req)

    def send_all(self, modes, setpoints, captures):
        if not self._check_ready(self.set_all_cli, "set_all_modes"):
            return
        req = SetAllModes.Request()
        req.modes = list(modes)
        req.setpoints = list(setpoints)
        req.capture_current = list(captures)
        self.set_all_cli.call_async(req)

    def clear_safety(self):
        if not self._check_ready(self.clear_safety_cli, "clear_safety"):
            return
        self.clear_safety_cli.call_async(Trigger.Request())


class AxisRow:
    """Widgets and behavior for a single axis."""

    def __init__(self, axis_idx: int, parent_grid: QGridLayout, row: int, node: PanelNode):
        self.axis_idx = axis_idx
        self.node = node
        self.suppress_signals = False

        self.is_angular = AXIS_ANGULAR[axis_idx]
        self.unit = "deg" if self.is_angular else "m"

        name_lbl = QLabel(f"<b>{AXIS_NAMES[axis_idx]}</b> <i>({self.unit})</i>")
        name_lbl.setMinimumWidth(90)

        self.mode_group = QButtonGroup(parent_grid.parentWidget())
        self.rb_off   = QRadioButton("OFF")
        self.rb_inner = QRadioButton("INNER")
        self.rb_full  = QRadioButton("FULL")
        self.rb_off.setChecked(True)
        # Angular axes run single-loop (pose -> torque); INNER has no meaning
        # for them and would silently emit zero torque. Disable the radio so
        # the UI matches what the controller accepts.
        if self.is_angular:
            self.rb_inner.setEnabled(False)
            self.rb_inner.setToolTip("no inner-velocity loop on angular axes")
        for rb, m in ((self.rb_off, MODE_OFF), (self.rb_inner, MODE_INNER),
                      (self.rb_full, MODE_FULL)):
            self.mode_group.addButton(rb, m)
        self.mode_group.idClicked.connect(self._on_mode_changed)

        mode_box = QHBoxLayout()
        mode_box.setContentsMargins(0, 0, 0, 0)
        mode_box.addWidget(self.rb_off)
        mode_box.addWidget(self.rb_inner)
        mode_box.addWidget(self.rb_full)
        mode_widget = QWidget()
        mode_widget.setLayout(mode_box)

        self.sp_box = QDoubleSpinBox()
        if self.is_angular:
            self.sp_box.setRange(-360.0, 360.0)
            self.sp_box.setDecimals(1)
            self.sp_box.setSingleStep(1.0)
        else:
            self.sp_box.setRange(-1000.0, 1000.0)
            self.sp_box.setDecimals(3)
            self.sp_box.setSingleStep(0.05)
        self.sp_box.setMinimumWidth(90)
        self.sp_box.editingFinished.connect(self._on_setpoint_edited)

        self.capture_btn = QPushButton("Capture")
        self.capture_btn.clicked.connect(self._on_capture)

        self.pose_lbl = QLabel("—")
        self.pose_lbl.setMinimumWidth(90)
        self.pose_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.effort_lbl = QLabel("—")
        self.effort_lbl.setMinimumWidth(90)
        self.effort_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        parent_grid.addWidget(name_lbl,         row, 0)
        parent_grid.addWidget(mode_widget,      row, 1)
        parent_grid.addWidget(QLabel("sp:"),    row, 2)
        parent_grid.addWidget(self.sp_box,      row, 3)
        parent_grid.addWidget(self.capture_btn, row, 4)
        parent_grid.addWidget(QLabel("pose:"),  row, 5)
        parent_grid.addWidget(self.pose_lbl,    row, 6)
        parent_grid.addWidget(QLabel("eff:"),   row, 7)
        parent_grid.addWidget(self.effort_lbl,  row, 8)

    def _current_mode(self) -> int:
        return self.mode_group.checkedId()

    def _sp_to_service(self) -> float:
        # Controller now works in degrees for angular axes — send spinbox value directly.
        return self.sp_box.value()

    def _on_mode_changed(self, mode: int):
        if self.suppress_signals:
            return
        # When switching into FULL, capture the current pose so it doesn't snap
        # to whatever happens to be in the spinbox.
        capture = (mode == MODE_FULL)
        self.node.send_axis(self.axis_idx, mode, self._sp_to_service(), capture)

    def _on_setpoint_edited(self):
        if self.suppress_signals:
            return
        if self._current_mode() != MODE_FULL:
            return  # only meaningful in FULL mode
        self.node.send_axis(self.axis_idx, MODE_FULL, self._sp_to_service(), False)

    def _on_capture(self):
        # Capture current pose as setpoint for this axis, keep current mode —
        # but if OFF, bump to FULL so the capture has an effect.
        mode = self._current_mode()
        if mode == MODE_OFF:
            mode = MODE_FULL
        self.node.send_axis(self.axis_idx, mode, 0.0, True)

    def refresh(self, status: ControllerStatus):
        self.suppress_signals = True
        try:
            mode = int(status.modes[self.axis_idx])
            btn = self.mode_group.button(mode)
            if btn is not None:
                btn.setChecked(True)
            if not self.sp_box.hasFocus():
                self.sp_box.setValue(float(status.setpoints[self.axis_idx]))
            self.pose_lbl.setText(f"{status.pose[self.axis_idx]:+.3f}")
            self.effort_lbl.setText(f"{status.effort[self.axis_idx]:+.2f}")
        finally:
            self.suppress_signals = False


class MainWindow(QWidget):
    def __init__(self, node: PanelNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("ROV PID Controller — Axis Panel")

        root = QVBoxLayout(self)
        root.addWidget(self._build_safety_bar())
        root.addWidget(self._build_presets())
        root.addWidget(self._hline())
        root.addWidget(self._build_axes())

        # Poll latest status at 10 Hz.
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start(100)

    def _build_safety_bar(self) -> QWidget:
        box = QHBoxLayout()
        self.safety_lbl = QLabel("Safety: OK")
        self.safety_lbl.setStyleSheet("padding: 3px 8px; font-weight: bold;")
        self.clear_btn = QPushButton("Clear safety")
        self.clear_btn.setStyleSheet("background: #d32f2f; color: white;")
        self.clear_btn.clicked.connect(self.node.clear_safety)
        self.clear_btn.setEnabled(False)
        box.addWidget(self.safety_lbl)
        box.addWidget(self.clear_btn)
        box.addStretch(1)
        w = QWidget()
        w.setLayout(box)
        return w

    def _hline(self) -> QFrame:
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    def _build_presets(self) -> QWidget:
        box = QHBoxLayout()
        box.addWidget(QLabel("<b>Presets:</b>"))
        presets = [
            ("All OFF",         [MODE_OFF] * 6,           [False] * 6),
            ("All HOLD",        [MODE_FULL] * 6,          [True]  * 6),
            ("Depth+Yaw hold",
                [MODE_OFF, MODE_OFF, MODE_FULL, MODE_OFF, MODE_OFF, MODE_FULL],
                [False,    False,    True,       False,    False,    True]),
            ("Attitude hold",
                [MODE_OFF, MODE_OFF, MODE_OFF, MODE_FULL, MODE_FULL, MODE_FULL],
                [False,    False,    False,    True,       True,       True]),
            ("Velocity cmd (linear INNER)",
                [MODE_INNER, MODE_INNER, MODE_INNER, MODE_OFF, MODE_OFF, MODE_OFF],
                [False] * 6),
        ]
        for label, modes, captures in presets:
            btn = QPushButton(label)
            btn.clicked.connect(partial(self._apply_preset, modes, captures))
            box.addWidget(btn)
        box.addStretch(1)
        w = QWidget()
        w.setLayout(box)
        return w

    def _build_axes(self) -> QWidget:
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)
        self.rows = [AxisRow(i, grid, i, self.node) for i in range(6)]
        w = QWidget()
        w.setLayout(grid)
        return w

    def _apply_preset(self, modes, captures):
        # Use current spinbox values as fallback setpoints, converting the
        # angular axes back from degrees (spinbox units) to radians (service).
        setpoints = [r._sp_to_service() for r in self.rows]
        self.node.send_all(modes, setpoints, captures)

    def _refresh(self):
        status = self.node.get_status()
        if status is None:
            return
        if bool(status.safety_tripped):
            self.safety_lbl.setText("Safety: TRIPPED — odom stale")
            self.safety_lbl.setStyleSheet(
                "padding: 3px 8px; font-weight: bold; "
                "background: #d32f2f; color: white;")
            self.clear_btn.setEnabled(True)
        else:
            self.safety_lbl.setText("Safety: OK")
            self.safety_lbl.setStyleSheet(
                "padding: 3px 8px; font-weight: bold;")
            self.clear_btn.setEnabled(False)
        for r in self.rows:
            r.refresh(status)


def _parse_namespace(argv):
    # Accept --ns <namespace>; defaults to /rov_pid_controller.
    ns = "/rov_pid_controller"
    if "--ns" in argv:
        i = argv.index("--ns")
        if i + 1 < len(argv):
            ns = argv[i + 1]
    return ns


def main(argv=None):
    argv = argv if argv is not None else sys.argv
    ns = _parse_namespace(argv)

    rclpy.init(args=argv)
    node = PanelNode(ns)

    # Spin rclpy in a background thread; Qt owns the main thread.
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    app = QApplication(sys.argv)
    # Allow Ctrl-C to kill the GUI.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    win = MainWindow(node)
    win.resize(900, 320)
    win.show()

    try:
        exit_code = app.exec_()
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
