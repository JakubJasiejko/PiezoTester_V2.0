from __future__ import annotations

import csv
import math
import queue
import statistics
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports
from PySide6.QtCore import QMarginsF, QRect, QRectF, QSize, QTimer, Qt
from PySide6.QtCharts import QChart, QChartView, QLineSeries, QScatterSeries, QValueAxis
from PySide6.QtGui import QColor, QPageLayout, QPageSize, QPainter, QPdfWriter, QPen, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QGraphicsScene,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


BAUDRATE = 115200
CALIBRATION_STAGE_TIMEOUT = 90.0
APP_DIR = Path(__file__).resolve().parent
LOG_DIR = APP_DIR / "logs"
LOG_DIR.mkdir(exist_ok=True)


@dataclass
class CalibrationStatus:
    calibrated: bool
    nominal_kg_per_v: float = 0.0
    kg_per_v: float = 0.0
    correction: float = 1.0
    cal_mass: float = 0.0
    nominal_mass: float = 0.0
    cal_signal: float = 0.0
    cal_zero: float = 0.0
    sensor_calibrated: bool = False
    sensor_factor: float = 1.0
    sensor_ref: float = 0.0
    sensor_raw: float = 0.0
    sensor_voltage: float = 0.0
    mag_points: int = 0
    mag_pre_valid: bool = False
    mag_move: int = 0
    mag_move_pct: float = 0.0
    mag_contact: int = 0
    mag_contact_pct: float = 0.0
    mag_full_scale_g: float = 0.0


@dataclass
class LoadPreview:
    zero: float
    measured: float
    signal: float
    mass_kg: float
    mass_g: float


@dataclass
class MagnetMeasurement:
    pwm_duty: int
    pwm_percent: float
    hold_ms: int
    load_v: float
    zero_v: float
    relative_v: float
    mass_g: float
    resistance: float
    sensor_v: float
    points: int


@dataclass
class TestResult:
    idx: int
    mode: str
    target_g: float
    pwm_duty: int
    pwm_percent: float
    load_v: float
    zero_v: float
    relative_v: float
    mass_g: float
    resistance: float
    sensor_v: float
    created_at: datetime


@dataclass
class TestSettings:
    mode: str
    sample_count: int
    start_mass_g: float = 0.0
    end_mass_g: float = 0.0


@dataclass
class MetricStats:
    mean: float
    min: float
    max: float
    std: float


@dataclass
class TrialSummary:
    started_at: datetime
    finished_at: datetime
    sample_count: int
    mass_stats: MetricStats | None
    resistance_stats: MetricStats | None
    csv_log_path: Path
    test_settings: TestSettings | None = None


class SerialProtocolError(RuntimeError):
    pass


def parse_key_value_line(line: str) -> dict[str, str]:
    parts = line.split()
    data: dict[str, str] = {}
    for token in parts[1:]:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        data[key] = value
    return data


def calculate_stats(values: list[float]) -> MetricStats | None:
    valid_values = [value for value in values if not math.isnan(value)]
    if not valid_values:
        return None

    return MetricStats(
        mean=statistics.mean(valid_values),
        min=min(valid_values),
        max=max(valid_values),
        std=statistics.pstdev(valid_values) if len(valid_values) > 1 else 0.0,
    )


def stats_to_cells(stats: MetricStats | None) -> list[str]:
    if stats is None:
        return ["-", "-", "-", "-"]

    return [
        f"{stats.mean:.3f}",
        f"{stats.min:.3f}",
        f"{stats.max:.3f}",
        f"{stats.std:.3f}",
    ]


def format_duration(started_at: datetime, finished_at: datetime) -> str:
    total_seconds = max(0, int((finished_at - started_at).total_seconds()))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)

    if hours > 0:
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"
    return f"{minutes:02d}:{seconds:02d}"


def default_export_dir() -> Path:
    documents_dir = Path.home() / "Documents"
    if documents_dir.exists():
        return documents_dir
    return Path.home()


class DeviceClient:
    def __init__(self) -> None:
        self.serial_port: serial.Serial | None = None
        self.lock = threading.Lock()

    def connect(self, port: str) -> str:
        self.close()
        self.serial_port = serial.Serial(
            port=port,
            baudrate=BAUDRATE,
            timeout=0.2,
            write_timeout=1,
            dsrdtr=False,
            rtscts=False,
        )
        self.serial_port.setDTR(False)
        self.serial_port.setRTS(False)
        time.sleep(0.6)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
        try:
            return self._read_until_prefix(("READY",), timeout=5.0)
        except SerialProtocolError:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(b"PING\n")
            self.serial_port.flush()
            pong = self._read_until_prefix(("OK PONG",), timeout=2.5)
            return f"{pong} (bez linii READY)"

    def close(self) -> None:
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None

    def is_connected(self) -> bool:
        return self.serial_port is not None and self.serial_port.is_open

    def request_line(self, command: str, expected_prefixes: tuple[str, ...], timeout: float = 10.0) -> str:
        if not self.is_connected():
            raise SerialProtocolError("Brak polaczenia z urzadzeniem.")

        with self.lock:
            assert self.serial_port is not None
            self.serial_port.reset_input_buffer()
            self.serial_port.write((command + "\n").encode("utf-8"))
            self.serial_port.flush()
            return self._read_until_prefix(expected_prefixes, timeout)

    def _read_until_prefix(self, expected_prefixes: tuple[str, ...], timeout: float) -> str:
        assert self.serial_port is not None
        deadline = datetime.now().timestamp() + timeout

        while datetime.now().timestamp() < deadline:
            raw = self.serial_port.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if line.startswith(expected_prefixes):
                return line

            if line.startswith("ERR "):
                raise SerialProtocolError(line)

        raise SerialProtocolError(f"Timeout oczekiwania na odpowiedz: {expected_prefixes}")


class TestSetupDialog(QDialog):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Parametry testu")
        self.setModal(True)

        layout = QVBoxLayout(self)

        hint = QLabel(
            "Wybierz typ testu.\n"
            "Standardowy wykonuje serie pomiarow z pelnym pobudzeniem elektromagnesu.\n"
            "Narastajacy dobiera PWM, aby przejsc od obciazenia poczatkowego do koncowego i uczy sie charakterystyki elektromagnesu."
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        form = QFormLayout()

        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Standardowy", "standard")
        self.mode_combo.addItem("Narastajacy", "ramp")
        self.mode_combo.currentIndexChanged.connect(self.update_mode_state)
        form.addRow("Typ testu", self.mode_combo)

        self.sample_count_spin = QSpinBox()
        self.sample_count_spin.setRange(1, 500)
        self.sample_count_spin.setValue(10)
        form.addRow("Ilosc probek", self.sample_count_spin)

        self.start_mass_spin = QDoubleSpinBox()
        self.start_mass_spin.setRange(0.0, 5000.0)
        self.start_mass_spin.setDecimals(1)
        self.start_mass_spin.setSingleStep(25.0)
        self.start_mass_spin.setSuffix(" g")
        self.start_mass_spin.setValue(100.0)
        form.addRow("Obciazenie wstepne", self.start_mass_spin)

        self.end_mass_spin = QDoubleSpinBox()
        self.end_mass_spin.setRange(0.0, 5000.0)
        self.end_mass_spin.setDecimals(1)
        self.end_mass_spin.setSingleStep(25.0)
        self.end_mass_spin.setSuffix(" g")
        self.end_mass_spin.setValue(1000.0)
        form.addRow("Obciazenie koncowe", self.end_mass_spin)

        layout.addLayout(form)

        self.mode_status_label = QLabel()
        self.mode_status_label.setWordWrap(True)
        layout.addWidget(self.mode_status_label)

        buttons = QHBoxLayout()
        start_btn = QPushButton("Start")
        start_btn.clicked.connect(self.accept)
        cancel_btn = QPushButton("Anuluj")
        cancel_btn.clicked.connect(self.reject)
        buttons.addStretch(1)
        buttons.addWidget(start_btn)
        buttons.addWidget(cancel_btn)
        layout.addLayout(buttons)

        self.update_mode_state()

    def update_mode_state(self) -> None:
        is_ramp = self.mode_combo.currentData() == "ramp"
        self.start_mass_spin.setEnabled(is_ramp)
        self.end_mass_spin.setEnabled(is_ramp)

        if is_ramp:
            self.mode_status_label.setText(
                "Tryb narastajacy: ESP32 wyliczy przyblizony PWM dla punktu poczatkowego i koncowego, "
                "a potem bedzie dopasowywac charakterystyke elektromagnesu na podstawie rzeczywistych pomiarow."
            )
        else:
            self.mode_status_label.setText(
                "Tryb standardowy: seria klasycznych pomiarow z pelnym pobudzeniem elektromagnesu."
            )

    def settings(self) -> TestSettings:
        mode = str(self.mode_combo.currentData())
        start_mass_g = self.start_mass_spin.value() if mode == "ramp" else 0.0
        end_mass_g = self.end_mass_spin.value() if mode == "ramp" else 0.0
        return TestSettings(
            mode=mode,
            sample_count=self.sample_count_spin.value(),
            start_mass_g=start_mass_g,
            end_mass_g=end_mass_g,
        )

    def accept(self) -> None:
        settings = self.settings()
        if settings.sample_count <= 0:
            QMessageBox.critical(self, "Test", "Ilosc probek musi byc wieksza od zera.")
            return
        if settings.mode == "ramp" and settings.end_mass_g < settings.start_mass_g:
            QMessageBox.critical(self, "Test", "Obciazenie koncowe nie moze byc mniejsze od wstepnego.")
            return
        super().accept()


class CalibrationDialog(QDialog):
    def __init__(self, app: "PiezoTesterWindow") -> None:
        super().__init__(app)
        self.app = app
        self.setWindowTitle("Kalibracja")
        self.setModal(True)
        self.step = 0
        self.busy = False
        self.finished = False
        self.preview_busy = False
        self.preview_auto_attempted = False
        self.preview_suspended = False
        self.preview_started_once = False
        self.preview_generation = 0
        self.preview_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.preview_message = "Inicjalizacja ciaglego podgladu masy..."

        layout = QVBoxLayout(self)

        load_group = QGroupBox("Kalibracja belki tensometrycznej")
        load_layout = QVBoxLayout(load_group)

        self.info = QLabel()
        self.info.setWordWrap(True)
        load_layout.addWidget(self.info)

        self.live_mass_label = QLabel("Masa aktualna: --")
        self.live_mass_label.setStyleSheet("font-size: 22px; font-weight: 700;")
        load_layout.addWidget(self.live_mass_label)

        self.live_signal_label = QLabel("Sygnal: -- | Zero: -- | Odczyt: --")
        self.live_signal_label.setWordWrap(True)
        load_layout.addWidget(self.live_signal_label)

        self.live_status_label = QLabel(self.preview_message)
        self.live_status_label.setWordWrap(True)
        load_layout.addWidget(self.live_status_label)

        form = QFormLayout()
        self.reference_mass_edit = QLineEdit()
        self.reference_mass_edit.setPlaceholderText("np. 1.000")
        if self.app.status.cal_mass > 0:
            self.reference_mass_edit.setText(f"{self.app.status.cal_mass:.3f}")
        form.addRow("Masa wzorcowa [kg]", self.reference_mass_edit)
        load_layout.addLayout(form)

        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        load_layout.addWidget(self.status_label)

        buttons = QHBoxLayout()
        self.primary_btn = QPushButton("Kalibracja 1-punktowa")
        self.primary_btn.clicked.connect(self.handle_primary_action)
        self.advanced_btn = QPushButton("Kalibracja zaawansowana")
        self.advanced_btn.clicked.connect(self.open_advanced_calibration)
        cancel_btn = QPushButton("Anuluj")
        cancel_btn.clicked.connect(self.reject)
        buttons.addWidget(self.primary_btn)
        buttons.addWidget(self.advanced_btn)
        buttons.addStretch(1)
        buttons.addWidget(cancel_btn)
        load_layout.addLayout(buttons)
        layout.addWidget(load_group)

        sensor_group = QGroupBox("Kalibracja toru czujnika")
        sensor_layout = QVBoxLayout(sensor_group)

        sensor_hint = QLabel(
            "Podepnij rezystor wzorcowy zamiast czujnika, wpisz jego wartosc i uruchom kalibracje. "
            "ESP32 obliczy wspolczynnik korekcji i bedzie nim korygowac kolejne pomiary rezystancji."
        )
        sensor_hint.setWordWrap(True)
        sensor_layout.addWidget(sensor_hint)

        sensor_form = QFormLayout()
        self.sensor_reference_edit = QLineEdit()
        self.sensor_reference_edit.setPlaceholderText("np. 100000.0")
        if self.app.status.sensor_ref > 0:
            self.sensor_reference_edit.setText(f"{self.app.status.sensor_ref:.3f}")
        sensor_form.addRow("Rezystor wzorcowy [Ohm]", self.sensor_reference_edit)
        sensor_layout.addLayout(sensor_form)

        self.sensor_status_label = QLabel()
        self.sensor_status_label.setWordWrap(True)
        sensor_layout.addWidget(self.sensor_status_label)

        sensor_buttons = QHBoxLayout()
        self.sensor_calibrate_btn = QPushButton("Kalibruj tor czujnika")
        self.sensor_calibrate_btn.clicked.connect(self.run_sensor_calibration)
        sensor_buttons.addWidget(self.sensor_calibrate_btn)
        sensor_buttons.addStretch(1)
        sensor_layout.addLayout(sensor_buttons)
        layout.addWidget(sensor_group)

        self.set_step(0)
        self.refresh_sensor_status()
        self.preview_timer = QTimer(self)
        self.preview_timer.timeout.connect(self.process_preview_cycle)

    def showEvent(self, event) -> None:
        super().showEvent(event)
        if self.preview_started_once:
            return
        self.preview_started_once = True
        self.preview_timer.start(350)
        QTimer.singleShot(0, self.ensure_live_preview_started)

    def closeEvent(self, event) -> None:
        self.suspend_preview()
        super().closeEvent(event)

    def set_step(self, step: int) -> None:
        self.step = step
        self.refresh_button_states()

        if step == 0:
            self.info.setText(
                "Model nominalny belki: 5 kg / 2 mV/V / 3.3 V / x62.\n"
                "ESP32 wylicza mase z tego modelu, a opcjonalna kalibracja 1-punktowa koryguje skale.\n"
                "Procedura: 1. zdejmij obciazenie i zapisz zero, 2. poloz znana mase i zapisz punkt."
            )
            self.status_label.setText("Po wejsciu do okna urzadzenie samo ustawia zero podgladu i uruchamia ciagly pomiar masy.")
            self.primary_btn.setText("Kalibracja 1-punktowa")
        elif step == 1:
            self.status_label.setText("Krok 1/2: ZDEJMIJ OBCIAZENIE i kliknij Nastepny krok, aby zapisac zero kalibracyjne.")
            self.primary_btn.setText("Nastepny krok")
        elif step == 2:
            self.status_label.setText("Krok 2/2: POLOZ MASE WZORCOWA, wpisz jej wartosc i kliknij Nastepny krok.")
            self.primary_btn.setText("Nastepny krok")
        elif step == 99:
            self.status_label.setText("Zapisywanie kalibracji...")
            self.primary_btn.setText("Przetwarzanie...")

    def set_busy(self, busy: bool) -> None:
        self.busy = busy
        self.refresh_button_states()

    def refresh_button_states(self) -> None:
        controls_locked = self.busy or self.preview_busy
        self.primary_btn.setEnabled(self.step in (0, 1, 2) and not controls_locked and not self.finished)
        self.sensor_calibrate_btn.setEnabled(not controls_locked)
        self.advanced_btn.setEnabled(not controls_locked)
        self.reference_mass_edit.setEnabled(self.step in (0, 2) and not controls_locked and not self.finished)

    def process_preview_cycle(self) -> None:
        if self.preview_suspended:
            return
        self.process_preview_queue()
        if not self.preview_busy:
            self.refresh_live_preview()

    def process_preview_queue(self) -> None:
        try:
            while True:
                event, generation, payload = self.preview_queue.get_nowait()
                if generation != self.preview_generation:
                    continue
                self.preview_busy = False
                self.refresh_button_states()

                if event == "tare_ok":
                    line = str(payload)
                    self.app.log(line)
                    data = parse_key_value_line(line)
                    zero = float(data.get("zero", "0"))
                    self.preview_message = f"Podglad masy aktywny. Zero odniesienia: {zero:.9f} V"
                    self.refresh_live_preview(force=True)
                    continue

                message = str(payload)
                self.preview_message = "Nie udalo sie uruchomic ciaglego podgladu. Mozesz sprobowac ponownie przyciskiem."
                self.update_live_preview_labels(None)
                self.live_status_label.setText(self.preview_message)
                QMessageBox.critical(self, "Podglad masy", message)
        except queue.Empty:
            return

    def ensure_live_preview_started(self) -> None:
        if (
            not self.isVisible()
            or not self.app.client.is_connected()
            or self.preview_busy
            or self.busy
            or self.preview_suspended
        ):
            return
        self.start_load_tare(background=True, auto=True)

    def suspend_preview(self) -> None:
        self.preview_suspended = True
        self.preview_generation += 1
        self.preview_busy = False
        self.preview_timer.stop()
        self.refresh_button_states()

    def resume_preview(self) -> None:
        self.preview_suspended = False
        self.preview_timer.start(350)
        self.refresh_button_states()
        QTimer.singleShot(0, self.ensure_live_preview_started)

    def start_load_tare(self, background: bool, auto: bool = False) -> None:
        if self.preview_busy or self.busy or self.preview_suspended:
            return

        self.preview_busy = True
        self.refresh_button_states()
        generation = self.preview_generation

        if auto:
            self.preview_auto_attempted = True
            self.live_status_label.setText("Uruchamianie ciaglego podgladu masy... ustawiam zero odniesienia.")
        else:
            self.live_status_label.setText("Ustawianie zera podgladu... to moze potrwac kilkadziesiat sekund.")

        if not background:
            try:
                line = self.app.run_load_tare()
                self.preview_queue.put(("tare_ok", generation, line))
            except Exception as exc:
                self.preview_queue.put(("tare_err", generation, str(exc)))
            self.process_preview_queue()
            return

        def worker() -> None:
            try:
                line = self.app.client.request_line(
                    "LOAD_TARE",
                    ("LOAD_TARE_OK",),
                    timeout=CALIBRATION_STAGE_TIMEOUT,
                )
                self.preview_queue.put(("tare_ok", generation, line))
            except Exception as exc:
                self.preview_queue.put(("tare_err", generation, str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def refresh_sensor_status(self, message: str | None = None) -> None:
        status = self.app.status

        if status.sensor_calibrated:
            details = (
                f"Aktywna kalibracja: x{status.sensor_factor:.6f} | "
                f"R wzorcowy={status.sensor_ref:.3f} Ohm | "
                f"R surowa={status.sensor_raw:.3f} Ohm | "
                f"U={status.sensor_voltage:.6f} V"
            )
        else:
            details = (
                f"Brak zapisanej kalibracji rezystorem. "
                f"Aktualnie aktywny wspolczynnik: x{status.sensor_factor:.6f}"
            )

        if message:
            self.sensor_status_label.setText(f"{message}\n{details}")
        else:
            self.sensor_status_label.setText(details)

    def update_live_preview_labels(self, preview: LoadPreview | None) -> None:
        if preview is None:
            self.live_mass_label.setText("Masa aktualna: --")
            self.live_signal_label.setText("Sygnal: -- | Zero: -- | Odczyt: --")
            self.live_status_label.setText(self.preview_message)
            return

        self.live_mass_label.setText(f"Masa aktualna: {preview.mass_g:.3f} g")
        self.live_signal_label.setText(
            f"Sygnal: {preview.signal:.9f} V | Zero: {preview.zero:.9f} V | Odczyt: {preview.measured:.9f} V"
        )
        self.live_status_label.setText(
            f"Podglad aktywny. Aktualny model: {self.app.status.kg_per_v:.6f} kg/V "
            f"(korekta x{self.app.status.correction:.6f})."
        )

    def refresh_live_preview(self, force: bool = False) -> None:
        if (self.busy and not force) or self.preview_busy or not self.app.client.is_connected():
            return

        try:
            preview = self.app.read_load_preview()
        except SerialProtocolError as exc:
            self.update_live_preview_labels(None)
            message = str(exc)
            if "load_preview_failed code=259" in message:
                if not self.preview_auto_attempted:
                    self.ensure_live_preview_started()
                return
            if "load_preview_failed" in message:
                return
            self.live_status_label.setText(f"Podglad chwilowo niedostepny: {message}")
            return
        except Exception as exc:
            self.update_live_preview_labels(None)
            self.live_status_label.setText(f"Podglad chwilowo niedostepny: {exc}")
            return

        self.update_live_preview_labels(preview)

    def start_calibration(self) -> None:
        try:
            self.set_busy(True)
            self.app.run_calibration_start()
            self.set_step(1)
        except Exception as exc:
            QMessageBox.critical(self, "Kalibracja", str(exc))
        finally:
            self.set_busy(False)

    def handle_primary_action(self) -> None:
        if self.step == 0:
            self.start_calibration()
            return
        self.save_step()

    def save_step(self) -> None:
        if self.busy or self.finished:
            return

        try:
            self.set_busy(True)
            current_step = self.step

            if current_step == 1:
                self.suspend_preview()
                self.live_status_label.setText("Zapisywanie zera kalibracyjnego... to moze potrwac kilkadziesiat sekund.")
                self.app.run_calibration_zero()
                self.set_step(2)
                self.preview_message = "Zero kalibracyjne zapisane. Poloz znana mase i obserwuj ciagly pomiar aktualnej masy."
                self.resume_preview()
                self.refresh_live_preview(force=True)
                return

            if current_step == 2:
                reference_mass = float(self.reference_mass_edit.text().replace(",", "."))
                self.finished = True
                self.set_step(99)
                self.app.run_calibration_point(reference_mass)
                saved_line = self.app.run_calibration_save()
                data = parse_key_value_line(saved_line)
                kg_per_v = float(data.get("kg_per_v", str(self.app.status.kg_per_v)))
                correction = float(data.get("correction", str(self.app.status.correction)))
                signal = float(data.get("cal_signal", str(self.app.status.cal_signal)))
                summary = (
                    "Kalibracja belki zakonczona.\n"
                    f"Masa wzorcowa: {reference_mass:.3f} kg | "
                    f"Sygnal: {signal:.9f} V | "
                    f"Nowe kg/V: {kg_per_v:.6f} | "
                    f"Korekta: x{correction:.6f}"
                )
                QMessageBox.information(self, "Kalibracja", summary)
                self.accept()
        except ValueError:
            self.finished = False
            if self.step == 99:
                self.set_step(2)
            QMessageBox.critical(self, "Kalibracja", "Podaj poprawna mase w kilogramach.")
        except Exception as exc:
            self.finished = False
            if self.step == 99:
                self.set_step(2)
            QMessageBox.critical(self, "Kalibracja", str(exc))
        finally:
            if not self.finished:
                if self.preview_suspended and self.step != 99:
                    self.resume_preview()
                self.set_busy(False)

    def run_sensor_calibration(self) -> None:
        if self.busy:
            return

        try:
            reference_resistance = float(self.sensor_reference_edit.text().replace(",", "."))
            if reference_resistance <= 0:
                raise ValueError

            self.set_busy(True)
            line = self.app.run_sensor_calibration(reference_resistance)
            data = parse_key_value_line(line)
            factor = float(data.get("factor", str(self.app.status.sensor_factor)))
            raw = float(data.get("raw", str(self.app.status.sensor_raw)))
            corrected = float(data.get("corrected", f"{reference_resistance:.6f}"))
            sensor_voltage = float(data.get("sensor_v", str(self.app.status.sensor_voltage)))

            summary = (
                "Kalibracja toru czujnika zakonczona.\n"
                f"R wzorcowy: {reference_resistance:.3f} Ohm | "
                f"R surowa: {raw:.3f} Ohm | "
                f"Wspolczynnik: x{factor:.6f} | "
                f"Po korekcji: {corrected:.3f} Ohm | "
                f"U={sensor_voltage:.6f} V"
            )
            self.refresh_sensor_status(summary)
            QMessageBox.information(self, "Kalibracja toru czujnika", summary)
        except ValueError:
            QMessageBox.critical(self, "Kalibracja toru czujnika", "Podaj poprawna wartosc rezystora wzorcowego w omach.")
        except Exception as exc:
            QMessageBox.critical(self, "Kalibracja toru czujnika", str(exc))
        finally:
            self.set_busy(False)

    def open_advanced_calibration(self) -> None:
        if self.busy or self.preview_busy:
            return
        self.suspend_preview()
        try:
            self.app.fetch_status()
            dialog = AdvancedCalibrationDialog(self.app)
            dialog.exec()
            self.app.fetch_status()
            self.refresh_sensor_status()
        finally:
            self.resume_preview()
        self.refresh_live_preview(force=True)


class AdvancedCalibrationDialog(QDialog):
    def __init__(self, app: "PiezoTesterWindow") -> None:
        super().__init__(app)
        self.app = app
        self.setWindowTitle("Kalibracja zaawansowana")
        self.setModal(True)
        self.resize(980, 760)

        self.worker_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.curve_running = False
        self.curve_stop_requested = False

        layout = QVBoxLayout(self)

        intro = QLabel(
            "Tutaj mozesz skalibrowac elektromagnes i krzywa obciazenia bez terminala.\n"
            "Kalibracja kontaktu pozwala zapisac progi elektromagnesu, a automat krzywej obciazenia "
            "mierzy belke dla kolejnych PWM i zapisuje punkty modelu w ESP32."
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        self.overview_label = QLabel()
        self.overview_label.setWordWrap(True)
        layout.addWidget(self.overview_label)

        contact_group = QGroupBox("Kalibracja elektromagnesu")
        contact_layout = QVBoxLayout(contact_group)

        contact_hint = QLabel(
            "Zacznij od 90% i wysylaj impulsy. Po obserwacji zaznacz, czy elektromagnes dotknal probki. "
            "Gdy trafisz punkt kontaktu, zapisz progi do ESP32."
        )
        contact_hint.setWordWrap(True)
        contact_layout.addWidget(contact_hint)

        contact_form = QFormLayout()
        self.contact_current_pct_spin = QDoubleSpinBox()
        self.contact_current_pct_spin.setRange(0.0, 100.0)
        self.contact_current_pct_spin.setDecimals(1)
        self.contact_current_pct_spin.setSingleStep(0.1)
        self.contact_current_pct_spin.setSuffix(" %")
        contact_form.addRow("Aktualny PWM", self.contact_current_pct_spin)

        self.contact_step_pct_spin = QDoubleSpinBox()
        self.contact_step_pct_spin.setRange(0.1, 5.0)
        self.contact_step_pct_spin.setDecimals(1)
        self.contact_step_pct_spin.setSingleStep(0.1)
        self.contact_step_pct_spin.setValue(0.5)
        self.contact_step_pct_spin.setSuffix(" %")
        contact_form.addRow("Krok PWM", self.contact_step_pct_spin)

        self.contact_hold_ms_spin = QSpinBox()
        self.contact_hold_ms_spin.setRange(200, 10000)
        self.contact_hold_ms_spin.setSingleStep(100)
        self.contact_hold_ms_spin.setValue(2000)
        self.contact_hold_ms_spin.setSuffix(" ms")
        contact_form.addRow("Czas impulsu", self.contact_hold_ms_spin)

        self.move_threshold_pct_spin = QDoubleSpinBox()
        self.move_threshold_pct_spin.setRange(0.0, 100.0)
        self.move_threshold_pct_spin.setDecimals(3)
        self.move_threshold_pct_spin.setSingleStep(0.1)
        self.move_threshold_pct_spin.setSuffix(" %")
        contact_form.addRow("Prog ruchu", self.move_threshold_pct_spin)

        self.contact_threshold_pct_spin = QDoubleSpinBox()
        self.contact_threshold_pct_spin.setRange(0.0, 100.0)
        self.contact_threshold_pct_spin.setDecimals(3)
        self.contact_threshold_pct_spin.setSingleStep(0.1)
        self.contact_threshold_pct_spin.setSuffix(" %")
        contact_form.addRow("Prog kontaktu", self.contact_threshold_pct_spin)

        self.full_scale_mass_spin = QDoubleSpinBox()
        self.full_scale_mass_spin.setRange(1.0, 50000.0)
        self.full_scale_mass_spin.setDecimals(1)
        self.full_scale_mass_spin.setSingleStep(50.0)
        self.full_scale_mass_spin.setSuffix(" g")
        contact_form.addRow("Pelna skala modelu", self.full_scale_mass_spin)
        contact_layout.addLayout(contact_form)

        self.contact_status_label = QLabel()
        self.contact_status_label.setWordWrap(True)
        contact_layout.addWidget(self.contact_status_label)

        contact_buttons = QHBoxLayout()
        self.contact_pulse_btn = QPushButton("Wyslij impuls")
        self.contact_pulse_btn.clicked.connect(self.send_contact_pulse)
        self.contact_no_btn = QPushButton("Nie dotknelo")
        self.contact_no_btn.clicked.connect(self.mark_no_contact)
        self.contact_yes_btn = QPushButton("Dotknelo probki")
        self.contact_yes_btn.clicked.connect(self.mark_contact)
        self.contact_save_btn = QPushButton("Zapisz progi do ESP32")
        self.contact_save_btn.clicked.connect(self.save_contact_thresholds)
        contact_buttons.addWidget(self.contact_pulse_btn)
        contact_buttons.addWidget(self.contact_no_btn)
        contact_buttons.addWidget(self.contact_yes_btn)
        contact_buttons.addWidget(self.contact_save_btn)
        contact_layout.addLayout(contact_buttons)

        layout.addWidget(contact_group)

        curve_group = QGroupBox("Kalibracja krzywej obciazenia")
        curve_layout = QVBoxLayout(curve_group)

        curve_hint = QLabel(
            "Automat bedzie kolejno ustawial PWM, mierzyl belke tensometryczna i zapisywal punkty modelu "
            "PWM -> masa bezposrednio w ESP32."
        )
        curve_hint.setWordWrap(True)
        curve_layout.addWidget(curve_hint)

        curve_form = QFormLayout()
        self.curve_start_pct_spin = QDoubleSpinBox()
        self.curve_start_pct_spin.setRange(0.0, 100.0)
        self.curve_start_pct_spin.setDecimals(3)
        self.curve_start_pct_spin.setSingleStep(0.1)
        self.curve_start_pct_spin.setSuffix(" %")
        curve_form.addRow("PWM start", self.curve_start_pct_spin)

        self.curve_end_pct_spin = QDoubleSpinBox()
        self.curve_end_pct_spin.setRange(0.0, 100.0)
        self.curve_end_pct_spin.setDecimals(3)
        self.curve_end_pct_spin.setSingleStep(0.1)
        self.curve_end_pct_spin.setValue(100.0)
        self.curve_end_pct_spin.setSuffix(" %")
        curve_form.addRow("PWM koniec", self.curve_end_pct_spin)

        self.curve_step_pct_spin = QDoubleSpinBox()
        self.curve_step_pct_spin.setRange(0.05, 10.0)
        self.curve_step_pct_spin.setDecimals(3)
        self.curve_step_pct_spin.setSingleStep(0.05)
        self.curve_step_pct_spin.setValue(0.5)
        self.curve_step_pct_spin.setSuffix(" %")
        curve_form.addRow("Krok", self.curve_step_pct_spin)

        self.curve_hold_ms_spin = QSpinBox()
        self.curve_hold_ms_spin.setRange(200, 10000)
        self.curve_hold_ms_spin.setSingleStep(100)
        self.curve_hold_ms_spin.setValue(2000)
        self.curve_hold_ms_spin.setSuffix(" ms")
        curve_form.addRow("Czas pomiaru", self.curve_hold_ms_spin)
        curve_layout.addLayout(curve_form)

        self.curve_status_label = QLabel()
        self.curve_status_label.setWordWrap(True)
        curve_layout.addWidget(self.curve_status_label)

        curve_buttons = QHBoxLayout()
        self.curve_clear_btn = QPushButton("Wyczysc model PWM->masa")
        self.curve_clear_btn.clicked.connect(self.clear_curve_model)
        self.curve_start_btn = QPushButton("Start automatu")
        self.curve_start_btn.clicked.connect(self.start_curve_calibration)
        self.curve_stop_btn = QPushButton("Zatrzymaj")
        self.curve_stop_btn.clicked.connect(self.stop_curve_calibration)
        curve_buttons.addWidget(self.curve_clear_btn)
        curve_buttons.addWidget(self.curve_start_btn)
        curve_buttons.addWidget(self.curve_stop_btn)
        curve_layout.addLayout(curve_buttons)

        self.curve_table = QTableWidget(0, 5)
        self.curve_table.setHorizontalHeaderLabels(
            ["PWM [%]", "Masa [g]", "Belka [V]", "Rezystancja", "Pkt modelu"]
        )
        self.curve_table.verticalHeader().setVisible(False)
        self.curve_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.curve_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.curve_table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.curve_table.setMinimumHeight(260)
        curve_layout.addWidget(self.curve_table)

        layout.addWidget(curve_group, 1)

        close_row = QHBoxLayout()
        close_row.addStretch(1)
        self.close_btn = QPushButton("Zamknij")
        self.close_btn.clicked.connect(self.accept)
        close_row.addWidget(self.close_btn)
        layout.addLayout(close_row)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.process_worker_queue)
        self.timer.start(120)

        self.sync_from_status()
        self.refresh_ui_state()

    def sync_from_status(self) -> None:
        status = self.app.status
        move_pct = status.mag_move_pct if status.mag_pre_valid else max(90.0, status.mag_contact_pct or 90.0)
        contact_pct = status.mag_contact_pct if status.mag_pre_valid else max(move_pct, 95.5)
        full_scale_g = status.mag_full_scale_g if status.mag_full_scale_g > 0 else 5000.0

        self.contact_current_pct_spin.setValue(max(90.0, min(100.0, contact_pct)))
        self.move_threshold_pct_spin.setValue(move_pct)
        self.contact_threshold_pct_spin.setValue(contact_pct)
        self.full_scale_mass_spin.setValue(full_scale_g)
        self.curve_start_pct_spin.setValue(max(90.0, min(100.0, contact_pct)))
        self.update_status_labels()

    def refresh_ui_state(self) -> None:
        idle = not self.curve_running
        for widget in (
            self.contact_current_pct_spin,
            self.contact_step_pct_spin,
            self.contact_hold_ms_spin,
            self.move_threshold_pct_spin,
            self.contact_threshold_pct_spin,
            self.full_scale_mass_spin,
            self.contact_pulse_btn,
            self.contact_no_btn,
            self.contact_yes_btn,
            self.contact_save_btn,
            self.curve_start_pct_spin,
            self.curve_end_pct_spin,
            self.curve_step_pct_spin,
            self.curve_hold_ms_spin,
            self.curve_clear_btn,
            self.curve_start_btn,
            self.close_btn,
        ):
            widget.setEnabled(idle)
        self.curve_stop_btn.setEnabled(self.curve_running)

    def update_status_labels(self) -> None:
        status = self.app.status
        self.overview_label.setText(
            "Aktualny model elektromagnesu: "
            f"punkty PWM->masa={status.mag_points}, "
            f"prog ruchu={status.mag_move_pct:.3f}%, "
            f"prog kontaktu={status.mag_contact_pct:.3f}%, "
            f"pelna skala={status.mag_full_scale_g:.1f} g"
        )
        self.contact_status_label.setText(
            "Kontakt: uzyj 'Wyslij impuls', obserwuj elektromagnes i oznacz wynik. "
            f"Biezacy kandydat kontaktu: {self.contact_threshold_pct_spin.value():.3f}%."
        )
        if self.curve_running:
            self.curve_status_label.setText("Automat kalibracji krzywej pracuje...")
        else:
            self.curve_status_label.setText(
                "Automat gotowy. Zaczynaj od progu kontaktu i stopniowo zblizaj sie do 100%."
            )

    def append_curve_row(self, measurement: MagnetMeasurement) -> None:
        row = self.curve_table.rowCount()
        self.curve_table.insertRow(row)
        values = [
            f"{measurement.pwm_percent:.3f}",
            f"{measurement.mass_g:.3f}",
            f"{measurement.load_v:.9f}",
            f"{measurement.resistance:.3f}",
            str(measurement.points),
        ]
        for column, value in enumerate(values):
            item = QTableWidgetItem(value)
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.curve_table.setItem(row, column, item)

    def send_contact_pulse(self) -> None:
        try:
            line = self.app.run_magnet_pulse(
                self.contact_current_pct_spin.value(),
                self.contact_hold_ms_spin.value(),
            )
            self.contact_status_label.setText(
                f"Impuls wyslany dla {self.contact_current_pct_spin.value():.3f}%.\n{line}"
            )
        except Exception as exc:
            QMessageBox.critical(self, "Kalibracja elektromagnesu", str(exc))

    def mark_no_contact(self) -> None:
        next_value = min(100.0, self.contact_current_pct_spin.value() + self.contact_step_pct_spin.value())
        self.contact_current_pct_spin.setValue(next_value)
        self.contact_status_label.setText(
            f"Zapisano: nie dotknelo. Kolejna propozycja PWM: {next_value:.3f}%."
        )

    def mark_contact(self) -> None:
        current_pct = self.contact_current_pct_spin.value()
        self.contact_threshold_pct_spin.setValue(current_pct)
        if self.move_threshold_pct_spin.value() <= 0.0:
            self.move_threshold_pct_spin.setValue(current_pct)
        self.contact_status_label.setText(
            f"Zapisano kandydat progu kontaktu: {current_pct:.3f}%. Mozesz od razu zapisac go do ESP32."
        )

    def save_contact_thresholds(self) -> None:
        move_pct = self.move_threshold_pct_spin.value()
        contact_pct = self.contact_threshold_pct_spin.value()
        if contact_pct < move_pct:
            QMessageBox.critical(self, "Kalibracja elektromagnesu", "Prog kontaktu nie moze byc mniejszy od progu ruchu.")
            return

        try:
            line = self.app.run_magnet_premodel_set(
                move_pct=move_pct,
                contact_pct=contact_pct,
                full_scale_g=self.full_scale_mass_spin.value(),
            )
            self.app.fetch_status()
            self.sync_from_status()
            self.contact_status_label.setText(f"Zapisano progi elektromagnesu.\n{line}")
        except Exception as exc:
            QMessageBox.critical(self, "Kalibracja elektromagnesu", str(exc))

    def clear_curve_model(self) -> None:
        try:
            line = self.app.run_magnet_model_clear()
            self.curve_table.setRowCount(0)
            self.app.fetch_status()
            self.sync_from_status()
            self.curve_status_label.setText(f"Model PWM->masa wyczyszczony.\n{line}")
        except Exception as exc:
            QMessageBox.critical(self, "Kalibracja krzywej obciazenia", str(exc))

    def start_curve_calibration(self) -> None:
        start_pct = self.curve_start_pct_spin.value()
        end_pct = self.curve_end_pct_spin.value()
        step_pct = self.curve_step_pct_spin.value()
        hold_ms = self.curve_hold_ms_spin.value()

        if end_pct < start_pct:
            QMessageBox.critical(self, "Kalibracja krzywej obciazenia", "PWM koniec nie moze byc mniejszy od PWM start.")
            return
        if step_pct <= 0:
            QMessageBox.critical(self, "Kalibracja krzywej obciazenia", "Krok PWM musi byc dodatni.")
            return

        self.curve_running = True
        self.curve_stop_requested = False
        self.refresh_ui_state()
        self.curve_status_label.setText("Start automatu kalibracji krzywej...")
        self.curve_table.setRowCount(0)

        def worker() -> None:
            current_pct = start_pct
            try:
                while current_pct <= end_pct + 1e-9:
                    if self.curve_stop_requested:
                        break
                    line = self.app.client.request_line(
                        f"MAG_MEASURE pct={current_pct:.3f} hold_ms={hold_ms}",
                        ("MAG_MEASURE_OK",),
                        timeout=max(CALIBRATION_STAGE_TIMEOUT, hold_ms / 1000.0 + 15.0),
                    )
                    self.worker_queue.put(("curve_measurement", line))
                    current_pct = round(current_pct + step_pct, 6)
                self.worker_queue.put(("curve_finished", None))
            except Exception as exc:
                self.worker_queue.put(("curve_error", exc))

        threading.Thread(target=worker, daemon=True).start()

    def stop_curve_calibration(self) -> None:
        if self.curve_running:
            self.curve_stop_requested = True
            self.curve_status_label.setText("Zatrzymywanie automatu po biezacym punkcie...")

    def process_worker_queue(self) -> None:
        try:
            while True:
                event, payload = self.worker_queue.get_nowait()
                if event == "curve_measurement":
                    line = str(payload)
                    self.app.log(line)
                    measurement = self.app.parse_magnet_measurement(line)
                    self.append_curve_row(measurement)
                    self.curve_status_label.setText(
                        f"Zmierzono {measurement.pwm_percent:.3f}% -> {measurement.mass_g:.3f} g. "
                        f"Punkty modelu: {measurement.points}."
                    )
                    self.app.fetch_status()
                    self.update_status_labels()
                elif event == "curve_finished":
                    self.curve_running = False
                    self.curve_stop_requested = False
                    self.app.fetch_status()
                    self.sync_from_status()
                    self.refresh_ui_state()
                    self.curve_status_label.setText("Automat kalibracji krzywej zakonczyl prace.")
                elif event == "curve_error":
                    self.curve_running = False
                    self.curve_stop_requested = False
                    self.refresh_ui_state()
                    QMessageBox.critical(self, "Kalibracja krzywej obciazenia", str(payload))
        except queue.Empty:
            return


class TrialSummaryDialog(QDialog):
    CHART_ITEMS = [
        (0, "Rezystancja / Nr probki"),
        (1, "Obciazenie / Nr probki"),
        (2, "Rezystancja / Obciazenie"),
    ]

    def __init__(self, app: "PiezoTesterWindow", summary: TrialSummary, results: list[TestResult]) -> None:
        super().__init__(app)
        self.app = app
        self.summary = summary
        self.results = list(results)
        self.chart_views: list[QChartView] = []

        self.setWindowTitle("Podsumowanie proby")
        self.setModal(True)
        self.resize(1180, 920)
        self.setStyleSheet(
            """
            QDialog {
                background: #ffffff;
                color: #111827;
            }
            QScrollArea {
                background: #ffffff;
                border: none;
            }
            QScrollArea > QWidget > QWidget {
                background: #ffffff;
            }
            QLabel {
                color: #111827;
            }
            QTableWidget {
                background: #ffffff;
                color: #111827;
                gridline-color: #d1d5db;
                border: 1px solid #d1d5db;
                border-radius: 8px;
            }
            QHeaderView::section {
                background: #e5e7eb;
                color: #111827;
                padding: 6px;
                border: none;
                border-bottom: 1px solid #d1d5db;
            }
            QLineEdit {
                background: #ffffff;
                color: #111827;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                padding: 8px 10px;
                selection-background-color: #c62828;
                selection-color: #ffffff;
            }
            QPlainTextEdit {
                background: #ffffff;
                color: #111827;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                padding: 10px;
                selection-background-color: #c62828;
                selection-color: #ffffff;
            }
            QPushButton {
                background: #f3f4f6;
                color: #111827;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                padding: 8px 14px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #e5e7eb;
            }
            QPushButton:pressed {
                background: #d1d5db;
            }
            """
        )

        base_name = f"piezotester_proba_{self.summary.started_at:%Y-%m-%d_%H-%M-%S}"
        export_dir = default_export_dir()

        root_layout = QVBoxLayout(self)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        root_layout.addWidget(scroll, 1)

        content = QWidget()
        scroll.setWidget(content)
        content_layout = QVBoxLayout(content)

        title_label = QLabel("Podsumowanie wykonanej proby")
        title_label.setStyleSheet("font-size: 22px; font-weight: 700; color: #111827;")
        content_layout.addWidget(title_label)

        subtitle_label = QLabel(
            "Mozesz zapisac dane sesji do pliku CSV oraz wygenerowac raport PDF z podsumowaniem, wykresami i wlasnym komentarzem."
        )
        subtitle_label.setWordWrap(True)
        subtitle_label.setStyleSheet("color: #4b5563; font-size: 13px;")
        content_layout.addWidget(subtitle_label)

        info_form = QFormLayout()
        info_form.addRow("Data rozpoczecia", QLabel(self.summary.started_at.strftime("%Y-%m-%d %H:%M:%S")))
        info_form.addRow("Data zakonczenia", QLabel(self.summary.finished_at.strftime("%Y-%m-%d %H:%M:%S")))
        info_form.addRow("Czas trwania", QLabel(format_duration(self.summary.started_at, self.summary.finished_at)))
        info_form.addRow("Liczba probek", QLabel(str(self.summary.sample_count)))
        if self.summary.test_settings is not None:
            settings = self.summary.test_settings
            info_form.addRow("Typ testu", QLabel("Narastajacy" if settings.mode == "ramp" else "Standardowy"))
            if settings.mode == "ramp":
                info_form.addRow("Obciazenie wstepne", QLabel(f"{settings.start_mass_g:.1f} g"))
                info_form.addRow("Obciazenie koncowe", QLabel(f"{settings.end_mass_g:.1f} g"))

        session_csv_label = QLabel(str(self.summary.csv_log_path))
        session_csv_label.setWordWrap(True)
        info_form.addRow("Autozapis sesji", session_csv_label)
        content_layout.addLayout(info_form)

        stats_label = QLabel("Wartosci statystyczne")
        stats_label.setStyleSheet("font-size: 16px; font-weight: 700; color: #111827;")
        content_layout.addWidget(stats_label)

        self.stats_table = self._create_stats_table()
        self._fill_stats_table(self.stats_table)
        content_layout.addWidget(self.stats_table)

        comment_label = QLabel("Komentarz do raportu PDF")
        comment_label.setStyleSheet("font-size: 16px; font-weight: 700; color: #111827;")
        content_layout.addWidget(comment_label)

        comment_hint = QLabel(
            "Tutaj mozesz dopisac uwagi do wykonanej proby, warunki pomiaru, opis probki albo wnioski. "
            "Ten komentarz zostanie dodany do raportu PDF."
        )
        comment_hint.setWordWrap(True)
        comment_hint.setStyleSheet("color: #4b5563; font-size: 13px;")
        content_layout.addWidget(comment_hint)

        self.report_comment_edit = QPlainTextEdit()
        self.report_comment_edit.setPlaceholderText(
            "Np. probka po wstepnym wygrzaniu, pomiar wykonany przy temperaturze pokojowej, "
            "widoczny dryf na poczatku serii..."
        )
        self.report_comment_edit.setMinimumHeight(140)
        self.report_comment_edit.setLineWrapMode(QPlainTextEdit.LineWrapMode.WidgetWidth)
        content_layout.addWidget(self.report_comment_edit)

        charts_label = QLabel("Wykresy")
        charts_label.setStyleSheet("font-size: 16px; font-weight: 700; color: #111827;")
        content_layout.addWidget(charts_label)

        for mode, chart_title in self.CHART_ITEMS:
            chart_title_label = QLabel(chart_title)
            chart_title_label.setStyleSheet("font-size: 14px; font-weight: 600; color: #111827;")
            content_layout.addWidget(chart_title_label)

            chart_view = QChartView(self.app.build_chart(mode))
            chart_view.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            chart_view.setMinimumHeight(360)
            chart_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            chart_view.setStyleSheet("background: white; border: 1px solid #d1d5db; border-radius: 8px;")
            self.chart_views.append(chart_view)
            content_layout.addWidget(chart_view)

        content_layout.addStretch(1)

        self.csv_path_edit = QLineEdit(str(export_dir / f"{base_name}.csv"))
        self.pdf_path_edit = QLineEdit(str(export_dir / f"{base_name}.pdf"))
        self.csv_path_edit.setMinimumHeight(42)
        self.pdf_path_edit.setMinimumHeight(42)

        export_panel = QWidget()
        export_panel.setStyleSheet("background: #f8fafc; border: 1px solid #d1d5db; border-radius: 10px;")
        export_layout = QVBoxLayout(export_panel)
        export_layout.setContentsMargins(16, 14, 16, 14)
        export_layout.setSpacing(12)

        export_label = QLabel("Eksport plikow")
        export_label.setStyleSheet("font-size: 16px; font-weight: 700; color: #111827;")
        export_layout.addWidget(export_label)

        csv_row = QHBoxLayout()
        csv_row.setSpacing(10)
        csv_label = QLabel("CSV")
        csv_label.setMinimumWidth(48)
        csv_label.setStyleSheet("font-weight: 700; color: #111827;")
        csv_row.addWidget(csv_label)
        csv_row.addWidget(self.csv_path_edit, 1)
        csv_browse_btn = QPushButton("Wybierz...")
        csv_browse_btn.clicked.connect(self.browse_csv_path)
        csv_save_btn = QPushButton("Zapisz CSV")
        csv_save_btn.clicked.connect(self.save_csv)
        csv_browse_btn.setMinimumHeight(42)
        csv_save_btn.setMinimumHeight(42)
        csv_row.addWidget(csv_browse_btn)
        csv_row.addWidget(csv_save_btn)
        export_layout.addLayout(csv_row)

        pdf_row = QHBoxLayout()
        pdf_row.setSpacing(10)
        pdf_label = QLabel("PDF")
        pdf_label.setMinimumWidth(48)
        pdf_label.setStyleSheet("font-weight: 700; color: #111827;")
        pdf_row.addWidget(pdf_label)
        pdf_row.addWidget(self.pdf_path_edit, 1)
        pdf_browse_btn = QPushButton("Wybierz...")
        pdf_browse_btn.clicked.connect(self.browse_pdf_path)
        pdf_save_btn = QPushButton("Zapisz raport PDF")
        pdf_save_btn.clicked.connect(self.save_pdf)
        pdf_browse_btn.setMinimumHeight(42)
        pdf_save_btn.setMinimumHeight(42)
        pdf_row.addWidget(pdf_browse_btn)
        pdf_row.addWidget(pdf_save_btn)
        export_layout.addLayout(pdf_row)

        root_layout.addWidget(export_panel)

        close_row = QHBoxLayout()
        close_row.addStretch(1)
        close_btn = QPushButton("Zamknij")
        close_btn.clicked.connect(self.accept)
        close_btn.setMinimumHeight(42)
        close_row.addWidget(close_btn)
        root_layout.addLayout(close_row)

    def _create_stats_table(self) -> QTableWidget:
        table = QTableWidget(2, 5)
        table.setHorizontalHeaderLabels(["Parametr", "Srednia", "Min", "Max", "Odch. std."])
        table.verticalHeader().setVisible(False)
        table.horizontalHeader().setStretchLastSection(True)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        table.setMinimumHeight(120)
        return table

    def _fill_stats_table(self, table: QTableWidget) -> None:
        rows = [
            ("Obciazenie [g]", stats_to_cells(self.summary.mass_stats)),
            ("Rezystancja", stats_to_cells(self.summary.resistance_stats)),
        ]

        for row_idx, (label, values) in enumerate(rows):
            items = [label, *values]
            for col_idx, value in enumerate(items):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                table.setItem(row_idx, col_idx, item)

    def browse_csv_path(self) -> bool:
        current_path = self.csv_path_edit.text().strip() or str(default_export_dir() / "wyniki.csv")
        chosen_path, _ = QFileDialog.getSaveFileName(
            self,
            "Zapisz CSV",
            current_path,
            "CSV (*.csv)",
        )
        if not chosen_path:
            return False
        self.csv_path_edit.setText(chosen_path)
        return True

    def browse_pdf_path(self) -> bool:
        current_path = self.pdf_path_edit.text().strip() or str(default_export_dir() / "raport.pdf")
        chosen_path, _ = QFileDialog.getSaveFileName(
            self,
            "Zapisz raport PDF",
            current_path,
            "PDF (*.pdf)",
        )
        if not chosen_path:
            return False
        self.pdf_path_edit.setText(chosen_path)
        return True

    def save_csv(self) -> None:
        raw_path = self.csv_path_edit.text().strip()
        if not raw_path:
            QMessageBox.critical(self, "CSV", "Podaj sciezke do pliku CSV.")
            return

        path = Path(raw_path)
        if path.suffix.lower() != ".csv":
            path = path.with_suffix(".csv")
            self.csv_path_edit.setText(str(path))

        try:
            self.app.write_results_csv(path, self.results, self.summary)
            QMessageBox.information(self, "CSV", f"Zapisano plik CSV:\n{path}")
        except Exception as exc:
            QMessageBox.critical(self, "CSV", str(exc))

    def save_pdf(self) -> None:
        raw_path = self.pdf_path_edit.text().strip()
        if not raw_path:
            QMessageBox.critical(self, "PDF", "Podaj sciezke do pliku PDF.")
            return

        path = Path(raw_path)
        if path.suffix.lower() != ".pdf":
            path = path.with_suffix(".pdf")
            self.pdf_path_edit.setText(str(path))

        fallback_path = default_export_dir() / path.name

        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            self.write_pdf_report(path)
            QMessageBox.information(self, "PDF", f"Zapisano raport PDF:\n{path}")
        except Exception as exc:
            if fallback_path != path:
                try:
                    self.write_pdf_report(fallback_path)
                    self.pdf_path_edit.setText(str(fallback_path))
                    QMessageBox.information(
                        self,
                        "PDF",
                        "Nie udalo sie zapisac raportu w wybranej lokalizacji.\n"
                        f"Zapisano go awaryjnie tutaj:\n{fallback_path}\n\n"
                        f"Przyczyna pierwotnego bledu: {exc}",
                    )
                    return
                except Exception:
                    pass

            QMessageBox.critical(self, "PDF", str(exc))
        finally:
            QApplication.restoreOverrideCursor()

    def write_pdf_report(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)

        writer = QPdfWriter(str(path))
        writer.setResolution(144)
        writer.setPageSize(QPageSize(QPageSize.PageSizeId.A4))
        writer.setPageMargins(QMarginsF(12, 12, 12, 12), QPageLayout.Unit.Millimeter)

        painter = QPainter(writer)
        if not painter.isActive():
            raise RuntimeError("Nie udalo sie przygotowac raportu PDF.")

        page_width = writer.width()
        page_height = writer.height()
        margin = 72
        content_width = page_width - 2 * margin
        y = 180
        report_comment = self.report_comment_edit.toPlainText().strip()

        def new_page() -> None:
            nonlocal y
            writer.newPage()
            y = margin

        def ensure_space(required_height: int) -> None:
            nonlocal y
            if y + required_height > page_height - margin:
                new_page()

        def set_font(size: int, bold: bool = False) -> None:
            font = painter.font()
            font.setPointSize(size)
            font.setBold(bold)
            painter.setFont(font)

        def wrap_text(text: str, max_width: int) -> list[str]:
            lines: list[str] = []
            metrics = painter.fontMetrics()

            def split_long_token(token: str) -> list[str]:
                if metrics.horizontalAdvance(token) <= max_width:
                    return [token]

                parts: list[str] = []
                current = ""
                for char in token:
                    candidate = current + char
                    if not current or metrics.horizontalAdvance(candidate) <= max_width:
                        current = candidate
                    else:
                        parts.append(current)
                        current = char
                if current:
                    parts.append(current)
                return parts

            for paragraph in text.splitlines():
                words = paragraph.split()
                if not words:
                    lines.append("")
                    continue

                initial_parts = split_long_token(words[0])
                current_line = initial_parts.pop()
                lines.extend(initial_parts)

                for word in words[1:]:
                    word_parts = split_long_token(word)
                    candidate = f"{current_line} {word_parts[0]}"
                    if metrics.horizontalAdvance(candidate) <= max_width:
                        current_line = candidate
                        if len(word_parts) > 1:
                            lines.append(current_line)
                            lines.extend(word_parts[1:-1])
                            current_line = word_parts[-1]
                        continue

                    lines.append(current_line)
                    if len(word_parts) > 1:
                        lines.extend(word_parts[:-1])
                    current_line = word_parts[-1]

                lines.append(current_line)

            return lines or [""]

        try:
            painter.fillRect(QRect(0, 0, page_width, 130), QColor("#111827"))
            painter.fillRect(QRect(0, 130, page_width, 10), QColor("#c62828"))
            painter.setPen(QColor("#ffffff"))
            set_font(20, bold=True)
            painter.drawText(margin, 60, "Raport proby PiezoTester V2.0")
            set_font(10)
            painter.drawText(
                margin,
                92,
                f"Wygenerowano: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            )

            info_items = [
                ("Data rozpoczecia", self.summary.started_at.strftime("%Y-%m-%d %H:%M:%S")),
                ("Data zakonczenia", self.summary.finished_at.strftime("%Y-%m-%d %H:%M:%S")),
                ("Czas trwania", format_duration(self.summary.started_at, self.summary.finished_at)),
                ("Liczba probek", str(self.summary.sample_count)),
            ]
            if self.summary.test_settings is not None:
                settings = self.summary.test_settings
                info_items.append(("Typ testu", "Narastajacy" if settings.mode == "ramp" else "Standardowy"))
                if settings.mode == "ramp":
                    info_items.append(("Obciazenie start", f"{settings.start_mass_g:.1f} g"))
                    info_items.append(("Obciazenie koniec", f"{settings.end_mass_g:.1f} g"))

            info_rows = max(1, (len(info_items) + 1) // 2)
            info_height = 44 + info_rows * 48
            ensure_space(info_height + 30)
            info_rect = QRect(margin, y, content_width, info_height)
            painter.setPen(QColor("#d1d5db"))
            painter.setBrush(QColor("#f8fafc"))
            painter.drawRoundedRect(info_rect, 10, 10)

            for index, (label, value) in enumerate(info_items):
                col = index % 2
                row = index // 2
                x_pos = margin + 24 + col * (content_width // 2)
                y_pos = y + 36 + row * 48
                painter.setPen(QColor("#6b7280"))
                set_font(9)
                painter.drawText(x_pos, y_pos, label)
                painter.setPen(QColor("#111827"))
                set_font(12, bold=True)
                painter.drawText(x_pos, y_pos + 22, value)

            y += info_height + 24

            if report_comment:
                ensure_space(96)
                painter.setPen(QColor("#111827"))
                set_font(13, bold=True)
                painter.drawText(margin, y, "Komentarz")
                y += 16

                set_font(10)
                comment_lines = wrap_text(report_comment, content_width - 32)
                line_height = max(18, painter.fontMetrics().lineSpacing())
                line_index = 0
                is_continued = False

                while line_index < len(comment_lines):
                    if y + 70 > page_height - margin:
                        new_page()
                        painter.setPen(QColor("#111827"))
                        set_font(13, bold=True)
                        painter.drawText(margin, y, "Komentarz (cd.)")
                        y += 16
                        set_font(10)
                        is_continued = True

                    available_height = page_height - margin - y - 18
                    max_lines = max(1, (available_height - 28) // line_height)
                    chunk = comment_lines[line_index:line_index + max_lines]
                    box_height = max(76, 28 + len(chunk) * line_height)
                    ensure_space(box_height + 18)

                    if y + box_height + 18 > page_height - margin:
                        new_page()
                        painter.setPen(QColor("#111827"))
                        set_font(13, bold=True)
                        painter.drawText(margin, y, "Komentarz (cd.)" if is_continued else "Komentarz")
                        y += 16
                        set_font(10)
                        continue

                    comment_rect = QRect(margin, y + 8, content_width, box_height)
                    painter.setPen(QColor("#d1d5db"))
                    painter.setBrush(QColor("#f8fafc"))
                    painter.drawRoundedRect(comment_rect, 10, 10)

                    text_x = comment_rect.x() + 16
                    text_y = comment_rect.y() + 24
                    painter.setPen(QColor("#111827"))
                    for line in chunk:
                        painter.drawText(text_x, text_y, line)
                        text_y += line_height

                    y += box_height + 24
                    line_index += len(chunk)
                    is_continued = True

            ensure_space(170)
            painter.setPen(QColor("#111827"))
            set_font(13, bold=True)
            painter.drawText(margin, y, "Wartosci statystyczne")
            y += 20

            table_headers = ["Parametr", "Srednia", "Min", "Max", "Odch. std."]
            table_rows = [
                ["Obciazenie [g]", *stats_to_cells(self.summary.mass_stats)],
                ["Rezystancja", *stats_to_cells(self.summary.resistance_stats)],
            ]

            table_x = margin
            table_y = y + 12
            row_height = 34
            header_widths = [content_width // 3, content_width // 6, content_width // 6, content_width // 6, content_width // 6]

            x_cursor = table_x
            for header, width in zip(table_headers, header_widths):
                rect = QRect(x_cursor, table_y, width, row_height)
                painter.setPen(QColor("#d1d5db"))
                painter.setBrush(QColor("#e5e7eb"))
                painter.drawRect(rect)
                painter.setPen(QColor("#111827"))
                set_font(10, bold=True)
                painter.drawText(rect, int(Qt.AlignmentFlag.AlignCenter), header)
                x_cursor += width

            for row_index, row_values in enumerate(table_rows):
                x_cursor = table_x
                for cell_value, width in zip(row_values, header_widths):
                    rect = QRect(x_cursor, table_y + row_height * (row_index + 1), width, row_height)
                    painter.setPen(QColor("#d1d5db"))
                    painter.setBrush(QColor("#ffffff") if row_index % 2 == 0 else QColor("#f8fafc"))
                    painter.drawRect(rect)
                    painter.setPen(QColor("#111827"))
                    set_font(10)
                    painter.drawText(rect, int(Qt.AlignmentFlag.AlignCenter), cell_value)
                    x_cursor += width

            y = table_y + row_height * (len(table_rows) + 1) + 28

            chart_pixmaps = [
                (title, self.app.render_chart_pixmap(mode, QSize(1400, 820)))
                for mode, title in self.CHART_ITEMS
            ]

            for chart_title, pixmap in chart_pixmaps:
                scaled_height = int(content_width * pixmap.height() / pixmap.width())
                ensure_space(scaled_height + 54)

                painter.setPen(QColor("#111827"))
                set_font(12, bold=True)
                painter.drawText(margin, y, chart_title)
                y += 14

                chart_rect = QRect(margin, y + 12, content_width, scaled_height)
                painter.setPen(QColor("#d1d5db"))
                painter.setBrush(QColor("#ffffff"))
                painter.drawRoundedRect(chart_rect, 8, 8)
                painter.drawPixmap(chart_rect, pixmap)
                y += scaled_height + 34
        finally:
            painter.end()
            del writer

        if not path.exists():
            raise RuntimeError(f"Raport PDF nie zostal zapisany: {path}")

        file_size = path.stat().st_size
        if file_size <= 0:
            raise RuntimeError(f"Raport PDF ma niepoprawny rozmiar: {path}")


class ChartWindow(QMainWindow):
    def __init__(self, app: "PiezoTesterWindow") -> None:
        super().__init__(app)
        self.app = app
        self.setWindowTitle("Wykres")
        self.resize(920, 620)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        top = QHBoxLayout()
        top.addWidget(QLabel("Typ wykresu"))
        self.chart_type_combo = QComboBox()
        self.chart_type_combo.addItems(
            [
                "Rezystancja / Nr probki",
                "Obciazenie / Nr probki",
                "Rezystancja / Obciazenie",
            ]
        )
        self.chart_type_combo.currentIndexChanged.connect(self.refresh_chart)
        top.addWidget(self.chart_type_combo)
        top.addStretch(1)
        layout.addLayout(top)

        self.chart_view = QChartView()
        self.chart_view.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        self.chart_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout.addWidget(self.chart_view)

    def refresh_chart(self) -> None:
        self.chart_view.setChart(self.app.build_chart(self.chart_type_combo.currentIndex()))


class PiezoTesterWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("PiezoTester V2.0")
        self.resize(1180, 760)

        self.client = DeviceClient()
        self.status = CalibrationStatus(calibrated=False)
        self.results: list[TestResult] = []
        self.ui_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.current_log_path = LOG_DIR / f"results_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"
        self.measurement_running = False
        self.stop_measurement_event = threading.Event()
        self.session_started_at: datetime | None = None
        self.session_finished_at: datetime | None = None
        self.pending_summary_on_stop = False
        self.closing = False
        self.current_test_settings: TestSettings | None = None

        self.port_combo = QComboBox()
        self.connection_label = QLabel("Niepolaczono")
        self.calibration_label = QLabel("Brak kalibracji w urzadzeniu")
        self.sensor_calibration_label = QLabel("Tor czujnika: korekcja domyslna")
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.table = QTableWidget(0, 18)
        self.test_button = QPushButton("Testuj")
        self.chart_button = QPushButton("Wykres")
        self.chart_window = ChartWindow(self)

        self._build_ui()
        self.refresh_ports()
        self.update_calibration_labels()
        self.chart_window.refresh_chart()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.process_ui_queue)
        self.timer.start(100)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        connection_row = QHBoxLayout()
        connection_row.addWidget(QLabel("Port"))
        self.port_combo.setMinimumWidth(220)
        connection_row.addWidget(self.port_combo)

        refresh_btn = QPushButton("Odswiez")
        refresh_btn.clicked.connect(self.refresh_ports)
        connect_btn = QPushButton("Polacz")
        connect_btn.clicked.connect(self.connect_device)
        connection_row.addWidget(refresh_btn)
        connection_row.addWidget(connect_btn)
        connection_row.addWidget(self.connection_label)
        connection_row.addStretch(1)
        layout.addLayout(connection_row)

        actions_row = QHBoxLayout()
        self.test_button.clicked.connect(self.handle_test_button)
        cal_btn = QPushButton("Kalibruj")
        cal_btn.clicked.connect(self.open_calibration_dialog)
        status_btn = QPushButton("Sprawdz status")
        status_btn.clicked.connect(self.fetch_status)
        self.chart_button.clicked.connect(self.toggle_chart_window)
        actions_row.addWidget(self.test_button)
        actions_row.addWidget(cal_btn)
        actions_row.addWidget(status_btn)
        actions_row.addWidget(self.chart_button)

        status_layout = QVBoxLayout()
        status_layout.setSpacing(2)
        self.calibration_label.setWordWrap(True)
        self.sensor_calibration_label.setWordWrap(True)
        status_layout.addWidget(self.calibration_label)
        status_layout.addWidget(self.sensor_calibration_label)
        actions_row.addLayout(status_layout)
        actions_row.addStretch(1)
        layout.addLayout(actions_row)

        self.table.setHorizontalHeaderLabels(
            [
                "Data",
                "Czas",
                "Nr",
                "Tryb",
                "Cel [g]",
                "PWM [%]",
                "Masa [g]",
                "Rezystancja",
                "Belka [V]",
                "Czujnik [V]",
                "M sr",
                "M min",
                "M max",
                "M std",
                "R sr",
                "R min",
                "R max",
                "R std",
            ]
        )
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        layout.addWidget(self.table, 1)

        layout.addWidget(self.console)

    def log(self, message: str) -> None:
        self.console.appendPlainText(f"{datetime.now():%H:%M:%S} | {message}")

    def refresh_ports(self) -> None:
        ports = [port.device for port in serial.tools.list_ports.comports()]
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def connect_device(self) -> None:
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.critical(self, "Polaczenie", "Wybierz port szeregowy.")
            return

        try:
            ready = self.client.connect(port)
            self.connection_label.setText(f"Polaczono: {port}")
            self.log(f"Urzadzenie odpowiedzialo: {ready}")
            self.fetch_status()
        except Exception as exc:
            QMessageBox.critical(self, "Polaczenie", str(exc))

    def fetch_status(self) -> None:
        try:
            line = self.client.request_line("STATUS", ("STATUS",))
            data = parse_key_value_line(line)
            self.status = CalibrationStatus(
                calibrated=data.get("calibrated", "0") == "1",
                nominal_kg_per_v=float(data.get("nominal_kg_per_v", data.get("a", "0"))),
                kg_per_v=float(data.get("kg_per_v", data.get("a", "0"))),
                correction=float(data.get("correction", data.get("b", "1"))),
                cal_mass=float(data.get("cal_mass", data.get("m1", "0"))),
                nominal_mass=float(data.get("nominal_mass", data.get("m2", "0"))),
                cal_signal=float(data.get("cal_signal", data.get("d1", "0"))),
                cal_zero=float(data.get("cal_zero", data.get("d2", "0"))),
                sensor_calibrated=data.get("sensor_calibrated", "0") == "1",
                sensor_factor=float(data.get("sensor_factor", "1")),
                sensor_ref=float(data.get("sensor_ref", "0")),
                sensor_raw=float(data.get("sensor_raw", "0")),
                sensor_voltage=float(data.get("sensor_voltage", "0")),
                mag_points=int(data.get("mag_points", "0")),
                mag_pre_valid=data.get("mag_pre_valid", "0") == "1",
                mag_move=int(data.get("mag_move", "0")),
                mag_move_pct=float(data.get("mag_move_pct", "0")),
                mag_contact=int(data.get("mag_contact", "0")),
                mag_contact_pct=float(data.get("mag_contact_pct", "0")),
                mag_full_scale_g=float(data.get("mag_full_scale_g", "0")),
            )
            self.update_calibration_labels()
            self.log(line)
        except Exception as exc:
            QMessageBox.critical(self, "Status", str(exc))

    def handle_test_button(self) -> None:
        if self.measurement_running:
            self.stop_measurement_loop()
        else:
            self.open_test_setup_dialog()

    def open_test_setup_dialog(self) -> None:
        if not self.client.is_connected():
            QMessageBox.critical(self, "Test", "Najpierw polacz sie z urzadzeniem.")
            return

        dialog = TestSetupDialog(self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return

        settings = dialog.settings()
        if settings.mode == "ramp" and not self.calibration_is_usable():
            QMessageBox.critical(
                self,
                "Test narastajacy",
                "Tryb narastajacy wymaga poprawnego modelu masy. Najpierw sprawdz status i ewentualnie skalibruj belke.",
            )
            return

        self.trigger_test(settings)

    def open_calibration_dialog(self) -> None:
        if not self.client.is_connected():
            QMessageBox.critical(self, "Kalibracja", "Najpierw polacz sie z urzadzeniem.")
            return
        if self.measurement_running:
            QMessageBox.critical(self, "Kalibracja", "Najpierw zatrzymaj biezaca probe pomiarowa.")
            return
        self.fetch_status()
        dialog = CalibrationDialog(self)
        dialog.exec()

    def toggle_chart_window(self) -> None:
        if self.chart_window.isVisible():
            self.chart_window.hide()
        else:
            self.chart_window.refresh_chart()
            self.chart_window.show()
            self.chart_window.raise_()
            self.chart_window.activateWindow()

    def run_calibration_start(self) -> None:
        line = self.client.request_line("CAL_START", ("CAL_START_OK",), timeout=20.0)
        self.log(line)

    def run_calibration_zero(self) -> None:
        line = self.client.request_line("CAL_ZERO", ("CAL_ZERO_OK",), timeout=CALIBRATION_STAGE_TIMEOUT)
        self.log(line)

    def run_calibration_point(self, mass: float) -> None:
        if mass <= 0:
            raise ValueError("Masa musi byc wieksza od zera.")
        line = self.client.request_line(
            f"CAL_POINT {mass:.6f}",
            ("CAL_POINT_OK",),
            timeout=CALIBRATION_STAGE_TIMEOUT,
        )
        self.log(line)

    def run_calibration_save(self) -> str:
        line = self.client.request_line("CAL_SAVE", ("CAL_SAVED",), timeout=20.0)
        self.log(line)
        self.fetch_status()
        return line

    def run_load_tare(self) -> str:
        line = self.client.request_line("LOAD_TARE", ("LOAD_TARE_OK",), timeout=CALIBRATION_STAGE_TIMEOUT)
        self.log(line)
        return line

    def read_load_preview(self) -> LoadPreview:
        line = self.client.request_line("LOAD_PREVIEW", ("LOAD_PREVIEW",), timeout=5.0)
        data = parse_key_value_line(line)
        return LoadPreview(
            zero=float(data.get("zero", "0")),
            measured=float(data.get("measured", "0")),
            signal=float(data.get("signal", "0")),
            mass_kg=float(data.get("mass_kg", "0")),
            mass_g=float(data.get("mass_g", "0")),
        )

    def run_sensor_calibration(self, reference_resistance: float) -> str:
        if reference_resistance <= 0:
            raise ValueError("Rezystor wzorcowy musi byc wiekszy od zera.")

        line = self.client.request_line(
            f"SENSOR_CAL {reference_resistance:.6f}",
            ("SENSOR_CAL_OK",),
            timeout=CALIBRATION_STAGE_TIMEOUT,
        )
        self.log(line)
        self.fetch_status()
        return line

    def run_magnet_pulse(self, pct: float, hold_ms: int) -> str:
        line = self.client.request_line(
            f"MAG_PULSE pct={pct:.3f} hold_ms={hold_ms}",
            ("MAG_PULSE_OK",),
            timeout=max(10.0, hold_ms / 1000.0 + 8.0),
        )
        self.log(line)
        return line

    def parse_magnet_measurement(self, line: str) -> MagnetMeasurement:
        data = parse_key_value_line(line)
        return MagnetMeasurement(
            pwm_duty=int(float(data.get("pwm", "0"))),
            pwm_percent=float(data.get("pwm_pct", "0")),
            hold_ms=int(float(data.get("hold_ms", "0"))),
            load_v=float(data.get("load_v", "0")),
            zero_v=float(data.get("zero_v", "0")),
            relative_v=float(data.get("relative_v", "0")),
            mass_g=float(data.get("mass_g", "0")),
            resistance=float(data.get("resistance", "0")),
            sensor_v=float(data.get("sensor_v", "0")),
            points=int(data.get("points", "0")),
        )

    def run_magnet_measure(self, pct: float, hold_ms: int) -> str:
        line = self.client.request_line(
            f"MAG_MEASURE pct={pct:.3f} hold_ms={hold_ms}",
            ("MAG_MEASURE_OK",),
            timeout=max(CALIBRATION_STAGE_TIMEOUT, hold_ms / 1000.0 + 15.0),
        )
        self.log(line)
        return line

    def run_magnet_premodel_set(self, move_pct: float, contact_pct: float, full_scale_g: float) -> str:
        line = self.client.request_line(
            (
                f"MAG_PREMODEL_SET move_pct={move_pct:.3f} "
                f"contact_pct={contact_pct:.3f} full_scale_g={full_scale_g:.3f}"
            ),
            ("MAG_PREMODEL_SET_OK",),
            timeout=20.0,
        )
        self.log(line)
        return line

    def run_magnet_model_clear(self) -> str:
        line = self.client.request_line("MAG_MODEL_CLEAR", ("MAG_MODEL_CLEAR_OK",), timeout=10.0)
        self.log(line)
        return line

    def run_test_setup(self, settings: TestSettings) -> str:
        line = self.client.request_line(
            (
                f"TEST_SETUP mode={settings.mode} "
                f"start_g={settings.start_mass_g:.3f} "
                f"end_g={settings.end_mass_g:.3f} "
                f"samples={settings.sample_count}"
            ),
            ("TEST_SETUP_OK",),
            timeout=20.0,
        )
        return line

    def abort_test_session(self) -> str:
        return self.client.request_line("TEST_ABORT", ("TEST_ABORT_OK",), timeout=10.0)

    def start_new_session(self, settings: TestSettings) -> None:
        self.results.clear()
        self.table.setRowCount(0)
        self.current_log_path = LOG_DIR / f"results_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"
        self.session_started_at = datetime.now()
        self.session_finished_at = None
        self.pending_summary_on_stop = False
        self.current_test_settings = settings
        self.chart_window.refresh_chart()

    def trigger_test(self, settings: TestSettings) -> None:
        if self.measurement_running:
            return

        self.start_new_session(settings)
        self.measurement_running = True
        self.stop_measurement_event.clear()
        self.test_button.setText("Przerwij")

        def worker() -> None:
            setup_done = False
            finished_normally = False
            try:
                setup_line = self.run_test_setup(settings)
                setup_done = True
                self.ui_queue.put(("test_log", setup_line))

                for _ in range(settings.sample_count):
                    if self.stop_measurement_event.is_set():
                        break

                    line = self.client.request_line("TEST_NEXT", ("TEST_RESULT", "TEST_DONE"), timeout=90.0)
                    if line.startswith("TEST_DONE"):
                        self.ui_queue.put(("test_log", line))
                        finished_normally = True
                        break
                    self.ui_queue.put(("test_ok", line))
                else:
                    if not self.stop_measurement_event.is_set():
                        finished_normally = True
            except Exception as exc:
                self.ui_queue.put(("test_err", exc))
            finally:
                if setup_done:
                    try:
                        abort_line = self.abort_test_session()
                        self.ui_queue.put(("test_log", abort_line))
                    except Exception:
                        pass
            if finished_normally:
                self.ui_queue.put(("test_finished", None))
            self.ui_queue.put(("test_stopped", None))

        threading.Thread(target=worker, daemon=True).start()
        if settings.mode == "ramp":
            self.log(
                f"Uruchomiono test narastajacy: start={settings.start_mass_g:.1f} g, "
                f"koniec={settings.end_mass_g:.1f} g, probki={settings.sample_count}."
            )
        else:
            self.log(f"Uruchomiono test standardowy: probki={settings.sample_count}.")

    def stop_measurement_loop(self) -> None:
        if not self.measurement_running:
            return
        self.pending_summary_on_stop = True
        self.stop_measurement_event.set()
        self.log("Zatrzymywanie pomiaru...")

    def finalize_measurement_loop(self) -> None:
        self.measurement_running = False
        self.stop_measurement_event.clear()
        self.test_button.setText("Testuj")
        if self.session_started_at is not None and self.session_finished_at is None:
            self.session_finished_at = datetime.now()
        self.log("Pomiar zatrzymany.")

    def process_ui_queue(self) -> None:
        try:
            while True:
                event, payload = self.ui_queue.get_nowait()
                if event == "test_ok":
                    self.handle_test_result(str(payload))
                elif event == "test_log":
                    self.log(str(payload))
                elif event == "test_err":
                    self.pending_summary_on_stop = False
                    self.finalize_measurement_loop()
                    QMessageBox.critical(self, "Test", str(payload))
                elif event == "test_finished":
                    self.pending_summary_on_stop = True
                elif event == "test_stopped":
                    if self.measurement_running:
                        self.finalize_measurement_loop()
                        if self.pending_summary_on_stop and not self.closing:
                            self.pending_summary_on_stop = False
                            self.open_trial_summary_dialog()
        except queue.Empty:
            return

    def calibration_is_usable(self) -> bool:
        return (
            self.status.nominal_kg_per_v > 1e-9
            and self.status.kg_per_v > 1e-9
            and self.status.correction > 0.0
        )

    def update_calibration_labels(self) -> None:
        if self.status.calibrated and self.calibration_is_usable():
            self.calibration_label.setText(
                f"Belka: kalibracja 1-punktowa | kg/V={self.status.kg_per_v:.6f} | "
                f"korekta x{self.status.correction:.6f} | "
                f"M wzorcowa={self.status.cal_mass:.3f} kg | sygnal={self.status.cal_signal:.9f} V"
            )
        elif self.calibration_is_usable():
            self.calibration_label.setText(
                f"Belka: model nominalny 5 kg / 2 mV/V / 3.3 V / x62 | "
                f"kg/V={self.status.kg_per_v:.6f} | bez zapisanej korekty"
            )
        else:
            self.calibration_label.setText("Belka: brak poprawnego modelu masy w ESP32")

        if self.status.sensor_calibrated:
            self.sensor_calibration_label.setText(
                f"Czujnik: OK | x{self.status.sensor_factor:.6f} | "
                f"R wzorcowy={self.status.sensor_ref:.3f} Ohm | "
                f"R surowa={self.status.sensor_raw:.3f} Ohm | "
                f"Elektromagnes: punkty={self.status.mag_points}, kontakt={self.status.mag_contact_pct:.3f}%"
            )
        else:
            self.sensor_calibration_label.setText(
                f"Czujnik: tryb domyslny | aktywna korekcja x{self.status.sensor_factor:.6f} | "
                f"Elektromagnes: punkty={self.status.mag_points}, kontakt={self.status.mag_contact_pct:.3f}%"
            )

    def calculate_display_mass_g(self, load_v: float) -> float:
        if self.calibration_is_usable():
            mass_g = self.status.kg_per_v * abs(load_v) * 1000.0
            return max(0.0, mass_g)
        return math.nan

    def mass_stats(self) -> MetricStats | None:
        return calculate_stats([result.mass_g for result in self.results])

    def resistance_stats(self) -> MetricStats | None:
        return calculate_stats([result.resistance for result in self.results])

    def update_statistics_columns(self) -> None:
        mass_cells = stats_to_cells(self.mass_stats())
        resistance_cells = stats_to_cells(self.resistance_stats())
        all_cells = mass_cells + resistance_cells

        for row in range(self.table.rowCount()):
            for offset, value in enumerate(all_cells, start=10):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setItem(row, offset, item)

    def handle_test_result(self, line: str) -> None:
        data = parse_key_value_line(line)
        mode = data.get("mode", "standard")
        target_g = float(data.get("target_g", "0"))
        pwm_duty = int(float(data.get("pwm", "0")))
        pwm_percent = float(data.get("pwm_pct", "0"))
        load_v = float(data["load_v"])
        mass_g = float(data.get("mass_g", "nan"))
        if not math.isfinite(mass_g) or mass_g < 0.0:
            mass_g = self.calculate_display_mass_g(load_v)
        result = TestResult(
            idx=len(self.results) + 1,
            mode=mode,
            target_g=target_g,
            pwm_duty=pwm_duty,
            pwm_percent=pwm_percent,
            load_v=load_v,
            zero_v=float(data["zero_v"]),
            relative_v=float(data["relative_v"]),
            mass_g=mass_g,
            resistance=float(data["resistance"]),
            sensor_v=float(data["sensor_v"]),
            created_at=datetime.now(),
        )
        self.results.append(result)

        row = self.table.rowCount()
        self.table.insertRow(row)
        values = [
            result.created_at.strftime("%Y-%m-%d"),
            result.created_at.strftime("%H:%M:%S"),
            str(result.idx),
            "Narast." if result.mode == "ramp" else "Std",
            "-" if result.mode != "ramp" else f"{result.target_g:.1f}",
            f"{result.pwm_percent:.2f}",
            "BLEDNA KAL" if math.isnan(result.mass_g) else f"{result.mass_g:.3f}",
            "BLEDNY ODCZYT" if math.isnan(result.resistance) else f"{result.resistance:.3f}",
            f"{result.load_v:.9f}",
            "BLEDNY ODCZYT" if math.isnan(result.sensor_v) else f"{result.sensor_v:.9f}",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
        ]
        for col, value in enumerate(values):
            item = QTableWidgetItem(value)
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table.setItem(row, col, item)

        self.update_statistics_columns()
        self.append_csv(result)
        self.chart_window.refresh_chart()
        self.log(line)

    def append_csv(self, result: TestResult) -> None:
        file_exists = self.current_log_path.exists()
        with self.current_log_path.open("a", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists:
                writer.writerow(
                    [
                        "date",
                        "time",
                        "idx",
                        "mode",
                        "target_g",
                        "pwm_duty",
                        "pwm_pct",
                        "mass_g",
                        "resistance",
                        "load_v",
                        "zero_v",
                        "relative_v",
                        "sensor_v",
                    ]
                )
            writer.writerow(
                [
                    result.created_at.strftime("%Y-%m-%d"),
                    result.created_at.strftime("%H:%M:%S"),
                    result.idx,
                    result.mode,
                    "" if result.mode != "ramp" else f"{result.target_g:.3f}",
                    result.pwm_duty,
                    f"{result.pwm_percent:.6f}",
                    "" if math.isnan(result.mass_g) else f"{result.mass_g:.6f}",
                    f"{result.resistance:.6f}",
                    f"{result.load_v:.9f}",
                    f"{result.zero_v:.9f}",
                    f"{result.relative_v:.9f}",
                    f"{result.sensor_v:.9f}",
                ]
            )

    def build_chart(self, mode: int) -> QChart:
        chart = QChart()
        chart.legend().setVisible(True)
        chart.setBackgroundVisible(False)

        results = self.results
        if not results:
            chart.setTitle("Brak danych")
            return chart

        series = QScatterSeries()
        series.setMarkerSize(10.0)

        connection_series = QLineSeries()
        connection_pen = QPen(QColor("#c62828"))
        connection_pen.setWidth(1)
        connection_series.setPen(connection_pen)
        connection_series.setName("Przebieg")

        if mode == 0:
            chart.setTitle("Rezystancja czujnika od numeru probki")
            points = [(result.idx, result.resistance) for result in results if not math.isnan(result.resistance)]
            x_title = "Nr probki"
            y_title = "Rezystancja"
            reference_label = "Srednia rezystancja"
        elif mode == 1:
            chart.setTitle("Obciazenie od numeru probki")
            points = [(result.idx, result.mass_g) for result in results if not math.isnan(result.mass_g)]
            x_title = "Nr probki"
            y_title = "Obciazenie [g]"
            reference_label = "Srednie obciazenie"
        else:
            chart.setTitle("Rezystancja czujnika od obciazenia")
            points = sorted(
                [
                    (result.mass_g, result.resistance)
                    for result in results
                    if not math.isnan(result.mass_g) and not math.isnan(result.resistance)
                ],
                key=lambda point: point[0],
            )
            x_title = "Obciazenie [g]"
            y_title = "Rezystancja"
            reference_label = "Srednia rezystancja"

        x_values = [x_val for x_val, _ in points]
        y_values = [y_val for _, y_val in points]
        if not x_values or not y_values:
            chart.setTitle("Brak poprawnych danych do wykresu")
            return chart

        for x_val, y_val in points:
            series.append(float(x_val), float(y_val))
            connection_series.append(float(x_val), float(y_val))

        series.setName("Pomiary")
        chart.addSeries(series)
        chart.addSeries(connection_series)

        x_axis = QValueAxis()
        y_axis = QValueAxis()
        x_axis.setTitleText(x_title)
        y_axis.setTitleText(y_title)

        x_min = min(x_values)
        x_max = max(x_values)
        y_min = min(y_values)
        y_max = max(y_values)

        if x_min == x_max:
            x_min -= 1.0
            x_max += 1.0
        if y_min == y_max:
            y_min -= 1.0
            y_max += 1.0

        x_margin = (x_max - x_min) * 0.05
        y_margin = (y_max - y_min) * 0.10
        x_axis.setRange(x_min - x_margin, x_max + x_margin)
        y_axis.setRange(y_min - y_margin, y_max + y_margin)

        chart.addAxis(x_axis, Qt.AlignmentFlag.AlignBottom)
        chart.addAxis(y_axis, Qt.AlignmentFlag.AlignLeft)
        series.attachAxis(x_axis)
        series.attachAxis(y_axis)
        connection_series.attachAxis(x_axis)
        connection_series.attachAxis(y_axis)

        avg_series = QLineSeries()
        avg_pen = QPen(QColor("#4b5563"))
        avg_pen.setStyle(Qt.PenStyle.DashLine)
        avg_pen.setWidth(1)
        avg_series.setPen(avg_pen)
        avg_value = statistics.mean(y_values)
        avg_series.append(x_axis.min(), avg_value)
        avg_series.append(x_axis.max(), avg_value)
        avg_series.setName(f"{reference_label} = {avg_value:.3f}")
        chart.addSeries(avg_series)
        avg_series.attachAxis(x_axis)
        avg_series.attachAxis(y_axis)

        for marker in chart.legend().markers(connection_series):
            marker.setVisible(False)

        return chart

    def render_chart_pixmap(self, mode: int, size: QSize) -> QPixmap:
        chart = self.build_chart(mode)
        chart.resize(size.width(), size.height())

        scene = QGraphicsScene()
        scene.setSceneRect(QRectF(0, 0, size.width(), size.height()))
        scene.addItem(chart)
        pixmap = QPixmap(size)
        pixmap.fill(Qt.GlobalColor.white)
        painter = QPainter(pixmap)
        scene.render(
            painter,
            QRectF(0, 0, size.width(), size.height()),
            QRectF(0, 0, size.width(), size.height()),
        )
        painter.end()
        scene.removeItem(chart)
        return pixmap

    def build_trial_summary(self) -> TrialSummary | None:
        if not self.results or self.session_started_at is None:
            return None

        finished_at = self.session_finished_at or datetime.now()
        return TrialSummary(
            started_at=self.session_started_at,
            finished_at=finished_at,
            sample_count=len(self.results),
            mass_stats=self.mass_stats(),
            resistance_stats=self.resistance_stats(),
            csv_log_path=self.current_log_path,
            test_settings=self.current_test_settings,
        )

    def open_trial_summary_dialog(self) -> None:
        summary = self.build_trial_summary()
        if summary is None:
            QMessageBox.information(self, "Podsumowanie", "Brak danych do podsumowania proby.")
            return

        dialog = TrialSummaryDialog(self, summary, self.results)
        dialog.exec()

    def write_results_csv(self, path: Path, results: list[TestResult], summary: TrialSummary) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)

        with path.open("w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Raport proby PiezoTester V2.0"])
            writer.writerow(["Data rozpoczecia", summary.started_at.strftime("%Y-%m-%d %H:%M:%S")])
            writer.writerow(["Data zakonczenia", summary.finished_at.strftime("%Y-%m-%d %H:%M:%S")])
            writer.writerow(["Czas trwania", format_duration(summary.started_at, summary.finished_at)])
            writer.writerow(["Liczba probek", summary.sample_count])
            if summary.test_settings is not None:
                writer.writerow(["Typ testu", "ramp" if summary.test_settings.mode == "ramp" else "standard"])
                if summary.test_settings.mode == "ramp":
                    writer.writerow(["Obciazenie wstepne [g]", f"{summary.test_settings.start_mass_g:.3f}"])
                    writer.writerow(["Obciazenie koncowe [g]", f"{summary.test_settings.end_mass_g:.3f}"])
            writer.writerow([])
            writer.writerow(["Statystyki"])
            writer.writerow(["Parametr", "Srednia", "Min", "Max", "Odch. std."])
            writer.writerow(["Obciazenie [g]", *stats_to_cells(summary.mass_stats)])
            writer.writerow(["Rezystancja", *stats_to_cells(summary.resistance_stats)])
            writer.writerow([])
            writer.writerow(
                [
                    "date",
                    "time",
                    "idx",
                    "mode",
                    "target_g",
                    "pwm_duty",
                    "pwm_pct",
                    "mass_g",
                    "resistance",
                    "load_v",
                    "zero_v",
                    "relative_v",
                    "sensor_v",
                ]
            )

            for result in results:
                writer.writerow(
                    [
                        result.created_at.strftime("%Y-%m-%d"),
                        result.created_at.strftime("%H:%M:%S"),
                        result.idx,
                        result.mode,
                        "" if result.mode != "ramp" else f"{result.target_g:.6f}",
                        result.pwm_duty,
                        f"{result.pwm_percent:.6f}",
                        "" if math.isnan(result.mass_g) else f"{result.mass_g:.6f}",
                        f"{result.resistance:.6f}",
                        f"{result.load_v:.9f}",
                        f"{result.zero_v:.9f}",
                        f"{result.relative_v:.9f}",
                        f"{result.sensor_v:.9f}",
                    ]
                )

    def closeEvent(self, event) -> None:
        self.closing = True
        self.pending_summary_on_stop = False
        self.stop_measurement_event.set()
        self.client.close()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    window = PiezoTesterWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
