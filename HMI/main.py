"""Aircraft Actuator Test Bench — USB HMI.

Install: pip install PySide6 pyserial
Run:     python aircraft_actuator_hmi_pro.py

Uses the existing STM32 protocol, unchanged:
  HMI -> STM32: STATUS\n, MOVE,<counts>\n, STOP\n
  STM32 -> HMI: DATA,command,position,error,time,current,temp,vibration,status

This is a TEST-MODE user interface. It is not a safety controller.
"""

import csv
import math
import os
import sys
import time
from collections import deque
from datetime import datetime

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QTimer, QRectF, QPointF
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPainterPath
from PySide6.QtWidgets import (
    QApplication, QWidget, QMainWindow, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QFrame, QSpinBox, QComboBox,
    QPlainTextEdit, QFileDialog, QTabWidget, QSplitter, QSizePolicy,
    QCheckBox, QMessageBox
)

BAUD = 115200
PREFERRED_PORT = '/dev/cu.usbmodem103'
DATA_TIMEOUT = 3.0
HISTORY_SECONDS = 120
MAX_SAMPLES = 2400

BG = '#0B1220'
PANEL = '#142035'
CARD = '#19283E'
BORDER = '#2B3B52'
TEXT = '#EDF4FF'
MUTED = '#98ABC5'
CYAN = '#48C8E8'
BLUE = '#6A9FFF'
GREEN = '#5DDBA1'
AMBER = '#F3C46A'
RED = '#FF7F87'

STYLE = f'''
QWidget {{ background: {BG}; color: {TEXT}; font-family: -apple-system, 'Segoe UI', Arial; font-size: 13px; }}
QFrame#Panel {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 13px; }}
QFrame#Card {{ background: {CARD}; border: 1px solid {BORDER}; border-radius: 10px; }}
QPushButton {{ background: #263A55; border: 1px solid #3D5675; border-radius: 8px;
               padding: 9px 14px; font-weight: 600; }}
QPushButton:hover:!disabled {{ background: #345174; }}
QPushButton:disabled {{ background: #1A2739; color: #65788F; border-color: #2A384A; }}
QPushButton#Move {{ background: #185B57; border-color: #2E8F7E; color: #DFFDF3; }}
QPushButton#Stop {{ background: #69303B; border-color: #A74A57; color: #FFE7E9; }}
QComboBox, QSpinBox {{ background: #101C2D; border: 1px solid #3A516B;
                       border-radius: 7px; padding: 7px; min-height: 21px; }}
QPlainTextEdit {{ background: #0D1828; border: 1px solid {BORDER}; border-radius: 8px;
                  color: #C4D5EB; font-family: Menlo, Consolas, monospace; font-size: 11px; }}
QTabWidget::pane {{ border: 0; }}
QTabBar::tab {{ background: #17253A; padding: 10px 18px; border-radius: 7px; margin-right: 5px; }}
QTabBar::tab:selected {{ background: #2D4969; color: white; }}
QCheckBox {{ spacing: 8px; }}
'''


def label(text, size=12, color=TEXT, bold=False):
    item = QLabel(text)
    font = QFont()
    font.setPointSize(size)
    font.setBold(bold)
    item.setFont(font)
    item.setStyleSheet(f'color: {color}; background: transparent;')
    return item


def panel():
    widget = QFrame()
    widget.setObjectName('Panel')
    return widget


class Metric(QFrame):
    """One big readout. set_value updates its number and color."""
    def __init__(self, title, initial='—', accent=TEXT):
        super().__init__()
        self.setObjectName('Card')
        self.setMinimumHeight(99)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(17, 13, 17, 12)
        layout.setSpacing(6)
        layout.addWidget(label(title.upper(), 10, MUTED, True))
        self.number = label(initial, 22, accent, True)
        self.number.setWordWrap(True)
        layout.addWidget(self.number)
        layout.addStretch()

    def set_value(self, value, color=TEXT):
        self.number.setText(str(value))
        self.number.setStyleSheet(f'color: {color}; background: transparent; font-weight: bold;')


class PositionPlot(QWidget):
    """Draws a position history with Qt. No matplotlib installation needed."""
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(245)
        self.samples = deque(maxlen=MAX_SAMPLES)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def add_sample(self, timestamp, position, command):
        self.samples.append((timestamp, position, command))
        self.update()

    def clear(self):
        self.samples.clear()
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(PANEL))
        area = QRectF(65, 19, max(1, self.width() - 87), max(1, self.height() - 61))
        painter.setPen(QPen(QColor(BORDER), 1))
        painter.drawRoundedRect(area, 5, 5)
        if not self.samples:
            painter.setPen(QColor(MUTED))
            painter.drawText(area, Qt.AlignCenter, 'Waiting for live position data…')
            return

        now = self.samples[-1][0]
        data = [s for s in self.samples if s[0] >= now - HISTORY_SECONDS]
        values = [v for _, actual, target in data for v in (actual, target)]
        minimum, maximum = min(values), max(values)
        pad = max(15, (maximum - minimum) * 0.12)
        minimum -= pad
        maximum += pad
        span = max(1.0, maximum - minimum)

        def x_pos(t):
            return area.right() - (now - t) / HISTORY_SECONDS * area.width()

        def y_pos(v):
            return area.bottom() - (v - minimum) / span * area.height()

        for index in range(5):
            y = area.top() + index * area.height() / 4
            painter.setPen(QPen(QColor('#293C53'), 1))
            painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))
            painter.setPen(QColor(MUTED))
            value = maximum - index * span / 4
            painter.drawText(QRectF(0, y - 9, 58, 18), Qt.AlignRight | Qt.AlignVCenter, f'{value:.0f}')

        for seconds in (120, 90, 60, 30, 0):
            x = area.right() - seconds / HISTORY_SECONDS * area.width()
            painter.setPen(QColor(MUTED))
            painter.drawText(QRectF(x - 24, area.bottom() + 8, 48, 18), Qt.AlignCenter, f'-{seconds}s')

        for idx, color in ((2, BLUE), (1, CYAN)):
            path = QPainterPath()
            for i, sample in enumerate(data):
                point = QPointF(x_pos(sample[0]), y_pos(sample[idx]))
                if i == 0:
                    path.moveTo(point)
                else:
                    path.lineTo(point)
            pen = QPen(QColor(color), 2.2)
            if idx == 2:
                pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.drawPath(path)


class ActuatorHMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Aircraft Actuator Test Bench  |  HMI Pro')
        self.resize(1150, 830)
        self.setMinimumSize(800, 620)
        self.setStyleSheet(STYLE)

        self.serial_port = None
        self.port_name = ''
        self.buffer = ''
        self.last_data = 0.0
        self.connected_at = 0.0
        self.status = None
        self.samples = deque(maxlen=MAX_SAMPLES)
        self.last_packet = None
        self.pending_move = False
        self.motion_message = 'IDLE / not verified'
        self.last_connection_state = None
        self.last_reported_status = None
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = ''
        self.session_start = time.monotonic()
        self.build_ui()

        self.read_timer = QTimer(self)
        self.read_timer.timeout.connect(self.poll_serial)
        self.read_timer.start(50)
        self.request_timer = QTimer(self)
        self.request_timer.timeout.connect(self.request_status)
        self.request_timer.start(1000)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_state)
        self.refresh_timer.start(250)
        self.scan_ports()
        self.connect_serial()

    def build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        page = QVBoxLayout(root)
        page.setContentsMargins(22, 16, 22, 14)
        page.setSpacing(12)

        header = QHBoxLayout()
        names = QVBoxLayout()
        names.setSpacing(3)
        names.addWidget(label('AIRCRAFT ACTUATOR TEST BENCH', 20, TEXT, True))
        names.addWidget(label('CONTROL & DIAGNOSTICS  /  STM32 USB  /  TEST MODE', 10, MUTED))
        header.addLayout(names, 1)
        self.health = label('●  OFFLINE', 12, RED, True)
        header.addWidget(self.health)
        page.addLayout(header)

        conn = panel()
        conn_layout = QHBoxLayout(conn)
        conn_layout.setContentsMargins(15, 11, 15, 11)
        self.connection_label = label('Searching for STM32…', 11, MUTED)
        conn_layout.addWidget(self.connection_label, 1)
        self.port_select = QComboBox()
        self.port_select.setMinimumWidth(195)
        conn_layout.addWidget(self.port_select)
        scan = QPushButton('Refresh ports')
        scan.clicked.connect(self.scan_ports)
        conn_layout.addWidget(scan)
        self.reconnect = QPushButton('Reconnect')
        self.reconnect.clicked.connect(self.connect_serial)
        conn_layout.addWidget(self.reconnect)
        page.addWidget(conn)

        self.tabs = QTabWidget()
        page.addWidget(self.tabs, 1)
        dashboard = QWidget()
        dash_layout = QVBoxLayout(dashboard)
        dash_layout.setContentsMargins(0, 12, 0, 0)
        dash_layout.setSpacing(12)

        metrics_grid = QGridLayout()
        metrics_grid.setSpacing(10)
        self.metrics = {
            'position': Metric('Actual position', '— counts', CYAN),
            'command': Metric('STM32 commanded position', '— counts'),
            'error': Metric('STM32 position error', '— counts'),
            'time': Metric('Movement time', '— ms'),
            'vibration': Metric('Vibration', '— g'),
            'temperature': Metric('Temperature', '— °C'),
            'current': Metric('Current sensor', 'NOT VERIFIED', AMBER),
            'status': Metric('Firmware status', 'UNKNOWN', MUTED),
        }
        for i, metric in enumerate(self.metrics.values()):
            metrics_grid.addWidget(metric, i // 4, i % 4)
        dash_layout.addLayout(metrics_grid)

        plot_panel = panel()
        plot_layout = QVBoxLayout(plot_panel)
        plot_layout.setContentsMargins(15, 11, 15, 10)
        plot_heading = QHBoxLayout()
        plot_heading.addWidget(label('POSITION HISTORY  ·  LAST 120 SECONDS', 11, TEXT, True))
        plot_heading.addStretch()
        plot_heading.addWidget(label('━━ Actual', 10, CYAN, True))
        plot_heading.addWidget(label('┄┄ Command', 10, BLUE, True))
        clear_plot = QPushButton('Clear chart')
        clear_plot.clicked.connect(self.clear_history)
        plot_heading.addWidget(clear_plot)
        plot_layout.addLayout(plot_heading)
        self.plot = PositionPlot()
        plot_layout.addWidget(self.plot, 1)
        dash_layout.addWidget(plot_panel, 1)

        control = panel()
        controls = QVBoxLayout(control)
        controls.setContentsMargins(15, 12, 15, 12)
        controls.addWidget(label('ACTUATOR COMMANDS  ·  COMMUNICATION TEST ONLY', 11, TEXT, True))
        buttons = QHBoxLayout()
        buttons.addWidget(label('Target (encoder counts)', 11, MUTED))
        self.target = QSpinBox()
        self.target.setRange(-30000, 30000)
        self.target.setMinimumWidth(155)
        buttons.addWidget(self.target)
        self.move_button = QPushButton('SEND MOVE  ·  TEST MODE')
        self.move_button.setObjectName('Move')
        self.move_button.setEnabled(False)
        self.move_button.clicked.connect(self.send_move)
        buttons.addWidget(self.move_button, 1)
        self.stop_button = QPushButton('SEND STOP  ·  SOFTWARE')
        self.stop_button.setObjectName('Stop')
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.send_stop)
        buttons.addWidget(self.stop_button, 1)
        controls.addLayout(buttons)
        self.motion_label = label('Motion: IDLE / not verified', 11, MUTED)
        controls.addWidget(self.motion_label)
        dash_layout.addWidget(control)
        self.tabs.addTab(dashboard, 'Overview')

        diagnostic = QWidget()
        diagnostic_layout = QVBoxLayout(diagnostic)
        diagnostic_layout.setContentsMargins(0, 12, 0, 0)
        diagnostic_layout.setSpacing(12)
        alarm_panel = panel()
        alarm_layout = QVBoxLayout(alarm_panel)
        alarm_layout.addWidget(label('SYSTEM / SENSOR NOTICES', 11, TEXT, True))
        self.notices = label('Waiting for telemetry…', 11, AMBER)
        self.notices.setWordWrap(True)
        alarm_layout.addWidget(self.notices)
        diagnostic_layout.addWidget(alarm_panel)
        log_panel = panel()
        log_layout = QVBoxLayout(log_panel)
        log_layout.addWidget(label('COMMUNICATION & COMMAND LOG', 11, TEXT, True))
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(500)
        log_layout.addWidget(self.log_box, 1)
        log_buttons = QHBoxLayout()
        clear_log = QPushButton('Clear log')
        clear_log.clicked.connect(self.log_box.clear)
        log_buttons.addWidget(clear_log)
        export_log = QPushButton('Export log (.txt)')
        export_log.clicked.connect(self.export_log)
        log_buttons.addWidget(export_log)
        log_buttons.addStretch()
        self.record_checkbox = QCheckBox('Record telemetry to CSV')
        self.record_checkbox.toggled.connect(self.toggle_recording)
        log_buttons.addWidget(self.record_checkbox)
        log_layout.addLayout(log_buttons)
        diagnostic_layout.addWidget(log_panel, 1)
        self.tabs.addTab(diagnostic, 'Diagnostics & logging')

        foot = label('TEST MODE: MOVE acknowledgement ≠ physical motion. Software STOP ≠ hardware E-stop. '
                     '12V motor power must remain disconnected during HMI command tests. '
                     'Current measurement is not verified.', 10, AMBER)
        foot.setWordWrap(True)
        page.addWidget(foot)
        self.statusBar().showMessage('USB serial only · Ethernet not required for this HMI')

    def log(self, message):
        self.log_box.appendPlainText(f'[{datetime.now():%H:%M:%S}] {message}')

    def scan_ports(self):
        """Find serial ports. Prefer the known STM32 port without hard-coding it forever."""
        selected = self.port_select.currentData()
        ports = [p.device for p in list_ports.comports()]
        self.port_select.clear()
        for port in sorted(ports):
            self.port_select.addItem(port, port)
        wanted = selected if selected in ports else PREFERRED_PORT
        if wanted not in ports:
            wanted = next((p for p in ports if 'usbmodem' in p), None)
        if wanted in ports:
            self.port_select.setCurrentIndex(ports.index(wanted) if ports == sorted(ports) else sorted(ports).index(wanted))

    def set_connection(self, state, message):
        if state != self.last_connection_state:
            self.log(f'Connection: {message}')
            self.last_connection_state = state
        color = {'live': GREEN, 'waiting': AMBER, 'stale': AMBER, 'offline': RED}[state]
        names = {'live': 'LIVE DATA', 'waiting': 'WAITING', 'stale': 'DATA STALE', 'offline': 'OFFLINE'}
        self.health.setText(f'●  {names[state]}')
        self.health.setStyleSheet(f'color: {color}; background: transparent; font-weight: bold;')
        self.connection_label.setText(message)
        self.connection_label.setStyleSheet(f'color: {color}; background: transparent;')

    def connect_serial(self):
        self.close_serial()
        self.scan_ports()
        port = self.port_select.currentData()
        if not port:
            self.set_connection('offline', 'STM32 USB serial port not found. Connect USB and press Reconnect.')
            return
        try:
            self.serial_port = serial.Serial(port, BAUD, timeout=0, write_timeout=0.2)
            self.port_name = port
            self.connected_at = time.monotonic()
            self.last_data = 0
            self.buffer = ''
            self.status = None
            self.last_reported_status = None
            self.pending_move = False
            self.set_connection('waiting', f'{port} connected · waiting for STM32 DATA')
            self.log(f'Opened {port} at {BAUD} baud')
        except (serial.SerialException, OSError) as exc:
            self.serial_port = None
            self.set_connection('offline', f'Unable to open {port}: {exc}')
        self.refresh_state()

    def close_serial(self):
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except (serial.SerialException, OSError):
                pass
        self.serial_port = None

    def disconnect(self, reason):
        self.close_serial()
        self.pending_move = False
        self.motion_message = 'COMMUNICATION LOST / motion unverified'
        self.motion_label.setText(f'Motion: {self.motion_message}')
        self.set_connection('offline', f'STM32 disconnected: {reason}')
        self.refresh_state()

    def request_status(self):
        if self.serial_port is None:
            # Try again periodically if the USB cable was reconnected.
            self.scan_ports()
            if self.port_select.currentData():
                self.connect_serial()
            return
        try:
            self.serial_port.write(b'STATUS\n')
        except (serial.SerialException, OSError) as exc:
            self.disconnect(exc)

    def poll_serial(self):
        if self.serial_port is None:
            return
        try:
            available = self.serial_port.in_waiting
            if available:
                self.buffer += self.serial_port.read(min(available, 8192)).decode('utf-8', 'replace')
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    self.parse_line(line.strip())
                self.buffer = self.buffer[-8192:]
        except (serial.SerialException, OSError) as exc:
            self.disconnect(exc)

    def parse_line(self, line):
        """Read existing STM32 messages; ignore unrelated W5500/sensor debug text."""
        replies = (
            ('COMMAND ERROR', 'COMMAND ERROR'),
            ('MOVE ACCEPTED | TEST MODE', 'MOVE ACCEPTED · TEST MODE ONLY'),
            ('MOVE ABORTED', 'MOVE ABORTED'),
            ('STOP CONFIRMED', 'STOP confirmed by STM32'),
            ('MOVE STOPPED', 'STOP reported by STM32'),
            ('TARGET REACHED', 'TARGET REACHED (reported by STM32)'),
            ('POSITION ERROR', 'POSITION ERROR / TIMEOUT'),
        )
        for marker, message in replies:
            if marker in line:
                self.motion_message = message
                self.motion_label.setText(f'Motion: {message}')
                self.log(message)
                if marker in ('MOVE ACCEPTED | TEST MODE', 'COMMAND ERROR', 'MOVE ABORTED',
                              'STOP CONFIRMED', 'MOVE STOPPED', 'TARGET REACHED', 'POSITION ERROR'):
                    self.pending_move = False
                break

        index = line.find('DATA,')
        if index < 0:
            return
        parts = line[index:].split(',')
        if len(parts) != 9:
            return
        try:
            command = int(parts[1])
            position = int(parts[2])
            error = int(parts[3])
            elapsed = int(parts[4])
            current = float(parts[5])
            temperature = float(parts[6])
            vibration = float(parts[7])
            status = int(parts[8])
        except ValueError:
            return
        if not all(math.isfinite(v) for v in (current, temperature, vibration)):
            return
        now = time.monotonic()
        self.last_data = now
        self.status = status
        self.last_packet = (command, position, error, elapsed, current, temperature, vibration, status)
        self.plot.add_sample(now, position, command)
        self.metrics['position'].set_value(f'{position:,} counts', CYAN)
        self.metrics['command'].set_value(f'{command:,} counts')
        self.metrics['error'].set_value(f'{error:,} counts')
        self.metrics['time'].set_value(f'{elapsed:,} ms')
        self.metrics['vibration'].set_value(f'{vibration:.3f} g')
        self.metrics['temperature'].set_value(f'{temperature:.2f} °C', AMBER if not (-20 <= temperature <= 85) else TEXT)
        self.metrics['current'].set_value('NOT VERIFIED', AMBER)
        status_name = {0: 'NORMAL', 1: 'WARNING', 2: 'ERROR'}.get(status, f'UNKNOWN ({status})')
        status_color = {0: GREEN, 1: AMBER, 2: RED}.get(status, AMBER)
        self.metrics['status'].set_value(status_name, status_color)
        if status != self.last_reported_status:
            self.log(f'Firmware status: {status_name}')
            self.last_reported_status = status
        notes = []
        if status != 0:
            notes.append(f'Firmware reports {status_name}.')
        if not (-20 <= temperature <= 85):
            notes.append('DS18B20 temperature appears invalid / connection may be loose.')
        notes.append('ACS724 current sensing is not verified; displayed current is intentionally suppressed.')
        self.notices.setText('\n'.join(notes))
        if self.csv_writer is not None:
            try:
                self.csv_writer.writerow([datetime.now().isoformat(timespec='milliseconds'),
                                          command, position, error, elapsed, current,
                                          temperature, vibration, status])
                self.csv_file.flush()
            except OSError as exc:
                self.log(f'CSV recording stopped: {exc}')
                self.record_checkbox.setChecked(False)
        self.refresh_state()

    def refresh_state(self):
        """Stale DATA is not a healthy connection, even if the USB port is open."""
        now = time.monotonic()
        if self.serial_port is None:
            self.move_button.setEnabled(False)
            self.stop_button.setEnabled(False)
            return
        fresh = self.last_data > 0 and now - self.last_data <= DATA_TIMEOUT
        if fresh:
            self.set_connection('live', f'STM32 · {self.port_name} · receiving DATA · {now - self.last_data:.1f}s ago')
        elif now - (self.last_data or self.connected_at) > DATA_TIMEOUT:
            self.set_connection('stale', f'{self.port_name} · no recent DATA · MOVE disabled')
        else:
            self.set_connection('waiting', f'{self.port_name} · waiting for DATA')
        # MOVE is a test command, and only allowed when the firmware reports NORMAL.
        self.move_button.setEnabled(fresh and self.status == 0 and not self.pending_move)
        # STOP is still offered whenever USB is open. It is never a hardware E-stop.
        self.stop_button.setEnabled(True)
        if self.pending_move and now - self.pending_move > 3.0:
            self.pending_move = False
            self.motion_message = 'MOVE acknowledgement timed out · motion unverified'
            self.motion_label.setText(f'Motion: {self.motion_message}')
            self.log('MOVE acknowledgement timeout')

    def send_move(self):
        if not self.move_button.isEnabled() or self.serial_port is None:
            return
        target = self.target.value()
        try:
            self.serial_port.write(f'MOVE,{target}\n'.encode('ascii'))
            self.pending_move = time.monotonic()
            self.motion_message = f'MOVE {target} requested · waiting for STM32 acknowledgement'
            self.motion_label.setText(f'Motion: {self.motion_message}')
            self.log(f'TX MOVE,{target} (TEST MODE)')
            self.refresh_state()
        except (serial.SerialException, OSError) as exc:
            self.disconnect(exc)

    def send_stop(self):
        if self.serial_port is None:
            return
        try:
            self.serial_port.write(b'STOP\n')
            self.pending_move = False
            self.motion_message = 'STOP requested · awaiting acknowledgement'
            self.motion_label.setText(f'Motion: {self.motion_message}')
            self.log('TX STOP (software only)')
            self.refresh_state()
        except (serial.SerialException, OSError) as exc:
            self.disconnect(exc)

    def clear_history(self):
        self.plot.clear()
        self.log('Position history cleared')

    def export_log(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Export log', 'actuator_hmi_log.txt', 'Text files (*.txt)')
        if path:
            try:
                with open(path, 'w', encoding='utf-8') as handle:
                    handle.write(self.log_box.toPlainText() + '\n')
                self.log(f'Log saved to {path}')
            except OSError as exc:
                QMessageBox.warning(self, 'Export failed', str(exc))

    def toggle_recording(self, checked):
        if checked:
            path, _ = QFileDialog.getSaveFileName(self, 'Record telemetry',
                f'actuator_telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv', 'CSV files (*.csv)')
            if not path:
                self.record_checkbox.blockSignals(True)
                self.record_checkbox.setChecked(False)
                self.record_checkbox.blockSignals(False)
                return
            try:
                self.csv_file = open(path, 'w', newline='', encoding='utf-8')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(['timestamp', 'command_counts', 'position_counts',
                    'error_counts', 'movement_ms', 'current_unverified_A', 'temperature_C',
                    'vibration_g', 'firmware_status'])
                self.csv_path = path
                self.log(f'CSV recording started: {path} (current unverified)')
            except OSError as exc:
                self.csv_file = None
                self.csv_writer = None
                self.record_checkbox.blockSignals(True)
                self.record_checkbox.setChecked(False)
                self.record_checkbox.blockSignals(False)
                QMessageBox.warning(self, 'Cannot record', str(exc))
        else:
            if self.csv_file is not None:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
                self.log('CSV recording stopped')

    def closeEvent(self, event):
        self.read_timer.stop()
        self.request_timer.stop()
        self.refresh_timer.stop()
        self.close_serial()
        if self.csv_file is not None:
            self.csv_file.close()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = ActuatorHMI()
    window.show()
    sys.exit(app.exec())
