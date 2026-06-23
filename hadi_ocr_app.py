
"""
HADI + OCR display/capture app

Install:
    pip install pyserial

Run:
    python hadi_ocr_app.py

Keep ocr_stream.py in the same folder.
"""

from __future__ import annotations

import csv
import json
import math
import queue
import re
import threading
import time
from collections import deque
import statistics
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from ocr_stream import OCRReceiver


APP_DIR = Path(__file__).resolve().parent
LOAD_CELLS_FILE = APP_DIR / "load_cells.json"
SYNC_STATE_FILE = APP_DIR / "sync_state.json"
AUTOSAVE_DIR = APP_DIR / "autosaves"

# Starting calibration from the current Morehouse certificate.
# Force (lbf) = B0 + B1*R + B2*R^2 + B3*R^3, where R is response in mV/V.
DEFAULT_LOAD_CELLS = [
    {
        "name": "Morehouse 2500 lbf P-9606",
        "capacity_lbf": 2500,
        "compression": {
            "B0": 9.095702e-03,
            "B1": -1.203595e03,
            "B2": -4.193786e-01,
            "B3": -3.360108e-02,
            "B4": 0.0,
            "B5": 0.0,
        },
        "tension": {
            "B0": 8.309679e-04,
            "B1": 1.203618e03,
            "B2": -3.388931e-01,
            "B3": 5.250494e-02,
            "B4": 0.0,
            "B5": 0.0,
        },
    }
]

RAW_NUMBER_RE = re.compile(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")


def parse_hadi_response(text: str) -> Optional[float]:
    matches = RAW_NUMBER_RE.findall(text.replace(",", ""))
    if not matches:
        return None
    try:
        return float(matches[-1])
    except ValueError:
        return None


def load_saved_load_cells() -> list[dict]:
    if not LOAD_CELLS_FILE.exists():
        save_load_cells(DEFAULT_LOAD_CELLS)
        return json.loads(json.dumps(DEFAULT_LOAD_CELLS))
    try:
        data = json.loads(LOAD_CELLS_FILE.read_text(encoding="utf-8"))
        if isinstance(data, list) and data:
            return data
    except Exception:
        pass
    save_load_cells(DEFAULT_LOAD_CELLS)
    return json.loads(json.dumps(DEFAULT_LOAD_CELLS))


def save_load_cells(load_cells: list[dict]) -> None:
    LOAD_CELLS_FILE.write_text(json.dumps(load_cells, indent=2), encoding="utf-8")


def force_from_response(response_mv_v: float, mode: str, load_cell: dict) -> float:
    key = "compression" if mode == "Compression" else "tension"
    c = load_cell[key]
    r = response_mv_v
    return (
        c.get("B0", 0.0)
        + c.get("B1", 0.0) * r
        + c.get("B2", 0.0) * (r ** 2)
        + c.get("B3", 0.0) * (r ** 3)
        + c.get("B4", 0.0) * (r ** 4)
        + c.get("B5", 0.0) * (r ** 5)
    )


def format_coeff(value) -> str:
    """Display coefficients like -2.610015E+04.

    Entry accepts normal decimals, uppercase E, lowercase e, +04, -04, etc.
    Saving converts to float; displaying uses uppercase E with two exponent digits.
    """
    try:
        return f"{float(value):.6E}"
    except Exception:
        return str(value)


STANDARD_GRAVITY = 9.80665

# When True the app locks onto the first valid GPS fix and computes the
# ASTM E74 MF from latitude/elevation for all W rows.  Before a fix arrives
# W rows fall back to standard gravity (MF = 1.0).
USE_GPS_GRAVITY_CORRECTION = True

_ASTM_AIR_DENSITY = 1.2       # kg/m³  (standard conditions)
_ASTM_WEIGHT_DENSITY = 8000.0  # kg/m³  (stainless-steel dead weights)
_ASTM_BUOYANCY = 1.0 - _ASTM_AIR_DENSITY / _ASTM_WEIGHT_DENSITY


def astm_multiplying_factor(latitude_deg: float, altitude_m: Optional[float] = None) -> float:
    """ASTM E74 Multiplying Factor for force measurement using dead weights.

    g_L = 9.80616(1 − 0.0026373·cos2φ + 0.0000059·cos²2φ) − 3.086×10⁻⁶·H
    MF  = (g_L / g_n) × (1 − ρ_air / ρ_weights)
    """
    lat_rad = math.radians(abs(float(latitude_deg)))
    H = 0.0 if altitude_m is None else max(0.0, float(altitude_m))

    cos2phi = math.cos(2.0 * lat_rad)
    g_local = (
        9.80616 * (1.0 - 0.0026373 * cos2phi + 0.0000059 * cos2phi ** 2)
        - 3.086e-6 * H
    )
    return (g_local / STANDARD_GRAVITY) * _ASTM_BUOYANCY


def normal_gravity_m_s2(latitude_deg: float, altitude_m: Optional[float] = None) -> float:
    """Local gravity derived from the ASTM E74 MF equation."""
    return astm_multiplying_factor(latitude_deg, altitude_m) * STANDARD_GRAVITY


def nearest_standard_weight_lbf(value_lbf: float) -> float:
    """Nearest common 1-2-5 / whole-pound standard weight."""
    if value_lbf == 0:
        return 0.0
    sign = -1 if value_lbf < 0 else 1
    v = abs(value_lbf)
    candidates = []
    for exp in range(-4, 7):
        scale = 10 ** exp
        for base in (1, 2, 5):
            candidates.append(base * scale)
    candidates.extend(float(x) for x in range(1, 1001))
    best = min(candidates, key=lambda c: abs(c - v))
    return sign * best


def fmt_lbf(value: float) -> str:
    if value is None:
        return ""
    if abs(value) >= 100:
        return f"{value:+.2f}"
    if abs(value) >= 10:
        return f"{value:+.3f}"
    return f"{value:+.4f}"


HADI_UNIT_FACTORS = {
    "LBF": 1.0,
    "KGF": 0.45359237,
    "N": 4.4482216152605,
    "kN": 0.0044482216152605,
    "gF": 453.59237,
    "t": 0.00045359237,
}

HADI_DECIMAL_OPTIONS = ["1", "0.1", "0.01", "0.001", "0.0001", "0.00001"]


def _decimals_from_step(step_text: str) -> int:
    text = str(step_text).strip()
    if "." not in text:
        return 0
    return max(0, len(text.split(".", 1)[1]))


@dataclass
class HADIReading:
    raw_response: float
    force_lbf: float
    received_at: float
    raw_text: str
    pc_time: float
    wall_time: float


@dataclass
class OCRTimedReading:
    value: float
    pc_time: float
    wall_time: float
    phone_time: float
    raw_text: str = ""


@dataclass
class GPSFix:
    latitude: float
    longitude: float
    altitude_m: Optional[float]
    phone_time: float
    received_at: float
    gravity_m_s2: float
    gravity_factor: float


class HADIWorker:
    def __init__(self):
        self.ser = None
        self.thread = None
        self.stop_event = threading.Event()
        self.out: "queue.Queue[HADIReading | Exception]" = queue.Queue(maxsize=100)
        self.mode = "Compression"
        self.load_cell = DEFAULT_LOAD_CELLS[0]
        self.read_command = "GN"
        self.tare_command = "SZ"
        self.line_ending = "\r"
        self.poll_hz = 10.0

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, baudrate: int = 19200):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")
        self.disconnect()
        self.stop_event.clear()
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            write_timeout=0.2,
        )
        self.thread = threading.Thread(target=self._loop, daemon=True, name="HADIWorker")
        self.thread.start()

    def disconnect(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1.0)
        self.thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    @staticmethod
    def probe_port(port: str, baudrate: int = 19200, command: str = "GN",
                   line_ending: str = "\r", timeout: float = 0.3) -> bool:
        if serial is None:
            return False
        try:
            with serial.Serial(port=port, baudrate=baudrate,
                               bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                               stopbits=serial.STOPBITS_ONE,
                               timeout=timeout, write_timeout=timeout) as s:
                s.reset_input_buffer()
                s.write((command + line_ending).encode("ascii", errors="ignore"))
                raw = s.read_until(b"\r", size=256)
                if not raw:
                    raw = s.read_until(b"\n", size=256)
                text = raw.decode("ascii", errors="ignore").strip()
                return parse_hadi_response(text) is not None
        except Exception:
            return False

    def send_tare_to_indicator(self):
        if not self.is_connected():
            return
        self.ser.write((self.tare_command + self.line_ending).encode("ascii", errors="ignore"))

    def _loop(self):
        while not self.stop_event.is_set():
            try:
                if not self.is_connected():
                    time.sleep(0.2)
                    continue
                self.ser.reset_input_buffer()
                self.ser.write((self.read_command + self.line_ending).encode("ascii", errors="ignore"))
                raw = self.ser.read_until(b"\r", size=256)
                if not raw:
                    raw = self.ser.read_until(b"\n", size=256)
                text = raw.decode("ascii", errors="ignore").strip()
                value = parse_hadi_response(text)
                if value is not None:
                    f_lbf = force_from_response(value, self.mode, self.load_cell)
                    now_wall = time.time()
                    reading = HADIReading(value, f_lbf, now_wall, text, time.perf_counter(), now_wall)
                    try:
                        self.out.put_nowait(reading)
                    except queue.Full:
                        try:
                            self.out.get_nowait()
                            self.out.put_nowait(reading)
                        except Exception:
                            pass
            except Exception as exc:
                try:
                    self.out.put_nowait(exc)
                except queue.Full:
                    pass
                time.sleep(0.5)
            time.sleep(max(0.02, 1.0 / max(1.0, self.poll_hz)))


def load_sync_state() -> dict:
    if not SYNC_STATE_FILE.exists():
        return {"lag_ms": 0.0, "confidence": 0.0, "manual": False, "calibration_lag_ms": 0.0}
    try:
        data = json.loads(SYNC_STATE_FILE.read_text(encoding="utf-8"))
        return {
            "lag_ms": float(data.get("lag_ms", 0.0)),
            "confidence": float(data.get("confidence", 0.0)),
            "manual": bool(data.get("manual", False)),
            "calibration_lag_ms": float(data.get("calibration_lag_ms", 0.0)),
        }
    except Exception:
        return {"lag_ms": 0.0, "confidence": 0.0, "manual": False, "calibration_lag_ms": 0.0}


def save_sync_state(lag_ms: float, confidence: float, manual: bool = False,
                    calibration_lag_ms: float = 0.0) -> None:
    SYNC_STATE_FILE.write_text(
        json.dumps({
            "lag_ms": lag_ms, "confidence": confidence,
            "manual": manual, "calibration_lag_ms": calibration_lag_ms,
        }, indent=2),
        encoding="utf-8",
    )



class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("HADI + OCR Capture")
        self.geometry("1060x710")
        self.minsize(980, 650)

        self.load_cells = load_saved_load_cells()
        self.selected_load_cell_name = tk.StringVar(value=self.load_cells[0]["name"])

        self.hadi = HADIWorker()
        self.hadi.load_cell = self.load_cells[0]
        self.ocr: Optional[OCRReceiver] = None
        self.latest_hadi: Optional[HADIReading] = None
        self.latest_ocr: Optional[OCRTimedReading] = None

        self._auto_scan_active = True
        self._last_auto_scan = 0.0
        self._auto_scan_interval = 3.0
        self._scan_thread: Optional[threading.Thread] = None
        self._last_good_port: Optional[str] = None
        self.capture_rows: list[dict] = []
        self.capture_target_index = 0
        self.capture_target_run = 1
        self.point_count_var = tk.StringVar(value="11")
        self.custom_point_count_var = tk.StringVar(value="11")
        self.ocr_edit_entry = None
        self.ocr_edit_item = None
        self.ocr_edit_row_index = None
        self.ocr_edit_tree = None
        self.suppress_next_tree_select = False
        self.suppress_next_tree_click = False
        self.suppress_next_tree_click = False

        # Auto-sync method:
        # continuously estimate a fixed OCR-vs-HADI lag from recent waveform shape,
        # save it, then capture by corrected time. This never matches by closest value.
        sync_state = load_sync_state()
        self.calibration_lag_seconds = sync_state["calibration_lag_ms"] / 1000.0
        if self.calibration_lag_seconds > 0 and sync_state["confidence"] < 0.5:
            self.sync_lag_seconds = self.calibration_lag_seconds
        else:
            self.sync_lag_seconds = sync_state["lag_ms"] / 1000.0
        self.sync_confidence = sync_state["confidence"]
        self.sync_manual = False
        self.sync_window_seconds = 8.0
        self.capture_median_half_window = 0.100   # 200 ms total around aligned target
        self.buffer_keep_seconds = 15.0
        self.hadi_buffer = deque()
        self.ocr_buffer = deque()
        self.ocr_lock = threading.Lock()
        self._last_sync_update = 0.0

        self.customer_capacity_var = tk.StringVar(value="")
        self.target_force_var = tk.StringVar(value="")
        self.target_forces: list[float] = []

        self.latest_gps: Optional[GPSFix] = None
        self.gps_status_var = tk.StringVar(value="Gravity: waiting for GPS")
        self.weight_status_var = tk.StringVar(value="Select rows and click SELECT / UNSELECT W")
        self.manual_weight_var = tk.StringVar(value="")
        self.weight_choice_var = tk.StringVar(value="")

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="19200")
        self.mode_var = tk.StringVar(value="Compression")
        self.hadi_units_var = tk.StringVar(value="LBF")
        self.hadi_decimals_var = tk.StringVar(value="0.00001")
        self.hadi_title_var = tk.StringVar(value="HADI Force (LBF)")
        self.mode_badge_var = tk.StringVar(value="COMPRESSION")
        self.capacity_warning_var = tk.StringVar(value="")
        self.poll_var = tk.StringVar(value="10")
        self.ocr_port_var = tk.StringVar(value="9999")
        self.status_var = tk.StringVar(value="Disconnected")

        self.hadi_raw_var = tk.StringVar(value="WAITING")
        self.hadi_lbf_var = tk.StringVar(value="WAITING FOR HADI")
        self.ocr_value_var = tk.StringVar(value="WAITING FOR OCR")
        self.live_error_var = tk.StringVar(value="--")
        self.hadi_last_pc_time = None
        self.ocr_last_pc_time = None
        self.hadi_wait_seconds = 1.5
        self.ocr_wait_seconds = 1.5
        self.ocr_age_var = tk.StringVar(value="")
        self.raw_text_var = tk.StringVar(value="")
        self.hadi_button_var = tk.StringVar(value="Connect HADI")
        self.ocr_button_var = tk.StringVar(value="Start OCR")
        self.count_var = tk.StringVar(value="0 captures")
        self.target_var = tk.StringVar(value="Next: P1 R1")
        self.sync_status_var = tk.StringVar(value="Sync: starting...")
        self.sync_lag_ms_var = tk.StringVar(value=f"{self.sync_lag_seconds * 1000.0:.0f}")
        self.sync_mode_var = tk.StringVar(value="Auto")
        self.cal_lag_ms_var = tk.StringVar(
            value=f"{self.calibration_lag_seconds * 1000.0:.0f}" if self.calibration_lag_seconds > 0 else ""
        )

        self.dirty_data = False
        self.last_manual_save_path = None
        self.last_autosave_path = None
        self.autosave_name = None
        AUTOSAVE_DIR.mkdir(exist_ok=True)

        self.editor_select_var = tk.StringVar()
        self.cell_name_var = tk.StringVar()
        self.capacity_var = tk.StringVar()
        self.coeff_vars: dict[str, tk.StringVar] = {}

        self._setup_styles()
        self._build_ui()
        self._set_hadi_display_options()
        self._refresh_hadi_mode_badge()
        self._refresh_ports()
        self._sync_load_cell_controls()
        self.after(50, self._ui_tick)
        self.after(700, self._auto_connect_on_launch)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.bind_all("<KeyPress-space>", self._capture_from_key)
        self.bind_all("<KeyPress-Return>", self._capture_from_key)

        # Tk keeps keyboard focus on the last button/table you clicked.
        # If a button has focus, Space/Enter can activate that button instead of acting
        # like a clean capture shortcut. These class bindings make Space/Enter always
        # mean capture and remove sticky button focus after mouse clicks.
        self.bind_class("TButton", "<KeyPress-space>", self._capture_from_key)
        self.bind_class("TButton", "<KeyPress-Return>", self._capture_from_key)
        self.bind_class("TButton", "<ButtonRelease-1>", self._release_button_focus, add="+")

    def _setup_styles(self):
        style = ttk.Style(self)
        style.configure("LiveTitle.TLabel", font=("Segoe UI", 13, "bold"))
        style.configure("BigValue.TLabel", font=("Consolas", 32, "bold"))
        style.configure("SmallValue.TLabel", font=("Consolas", 16))
        style.configure("TinyValue.TLabel", font=("Consolas", 9))
        style.configure("ModeBadgeCompression.TLabel", font=("Segoe UI", 11, "bold"), foreground="#0f5f5c", background="#d7f4f1", padding=(12, 6))
        style.configure("ModeBadgeTension.TLabel", font=("Segoe UI", 11, "bold"), foreground="#5b2a86", background="#efe3fb", padding=(12, 6))
        style.configure("CapacityWarning.TLabel", font=("Segoe UI", 11, "bold"), foreground="#c1121f", padding=(8, 6))
        style.configure("Zero.TButton", font=("Segoe UI", 9, "bold"))
        style.configure("Override.TButton", font=("Segoe UI", 10, "bold"), padding=(10, 6))
        style.configure("BigCapture.TButton", font=("Segoe UI", 16, "bold"), padding=(18, 14))

    def _build_ui(self):
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True)

        self.capture_tab = ttk.Frame(self.notebook)
        self.connection_tab = ttk.Frame(self.notebook)
        self.load_cells_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.capture_tab, text="Capture")
        self.notebook.add(self.connection_tab, text="Settings")
        self.notebook.add(self.load_cells_tab, text="Load Cells")

        self._build_capture_tab(self.capture_tab)
        self._build_connection_tab(self.connection_tab)
        self._build_load_cells_tab(self.load_cells_tab)

    def _build_capture_tab(self, root):
        pad = {"padx": 10, "pady": 8}

        controls = ttk.LabelFrame(root, text="Run")
        controls.pack(fill="x", **pad)

        ttk.Label(controls, text="Load Cell").grid(row=0, column=0, sticky="w", padx=(6, 2), pady=4)
        self.load_cell_combo = ttk.Combobox(
            controls,
            textvariable=self.selected_load_cell_name,
            values=[c["name"] for c in self.load_cells],
            state="readonly",
            width=30,
        )
        self.load_cell_combo.grid(row=0, column=1, sticky="w", padx=(0, 12), pady=4)
        self.load_cell_combo.bind("<<ComboboxSelected>>", lambda _e: self._select_load_cell())

        ttk.Label(controls, text="HADI Units").grid(row=0, column=2, sticky="w", padx=(12, 2), pady=4)
        units = ttk.Combobox(controls, textvariable=self.hadi_units_var, values=["LBF", "KGF", "N", "kN", "gF", "t", "mV/V"], state="readonly", width=8)
        units.grid(row=0, column=3, sticky="w", padx=(0, 12), pady=4)
        units.bind("<<ComboboxSelected>>", lambda _e: self._set_hadi_display_options())

        ttk.Label(controls, text="Decimals").grid(row=0, column=4, sticky="w", pady=4)
        decimals = ttk.Combobox(controls, textvariable=self.hadi_decimals_var, values=HADI_DECIMAL_OPTIONS, state="readonly", width=8)
        decimals.grid(row=0, column=5, sticky="w", padx=(0, 12), pady=4)
        decimals.bind("<<ComboboxSelected>>", lambda _e: self._set_hadi_display_options())

        mode_bar = ttk.Frame(root)
        mode_bar.pack(fill="x", padx=10, pady=(0, 2))
        mode_bar.columnconfigure(0, weight=1)
        mode_bar.columnconfigure(1, weight=0)
        mode_bar.columnconfigure(2, weight=1)
        self.mode_badge_label = ttk.Label(mode_bar, textvariable=self.mode_badge_var, style="ModeBadgeCompression.TLabel", cursor="hand2")
        self.mode_badge_label.grid(row=0, column=1, sticky="e")
        self.mode_badge_label.bind("<Button-1>", lambda _e: self._toggle_mode())
        self.capacity_warning_label = ttk.Label(mode_bar, textvariable=self.capacity_warning_var, style="CapacityWarning.TLabel")
        self.capacity_warning_label.grid(row=0, column=2, sticky="w", padx=(8, 0))

        display = ttk.Frame(root)
        display.pack(fill="both", expand=True, **pad)

        left = ttk.LabelFrame(display, text="Live Display")
        left.pack(side="left", fill="both", expand=True, padx=(0, 6))

        self._hadi_live_label(left, self.hadi_title_var, self.hadi_lbf_var, 0)

        ocr_card = ttk.Frame(left)
        ocr_card.grid(row=1, column=0, sticky="ew", padx=12, pady=(14, 0))
        ocr_card.columnconfigure(0, weight=1)
        ocr_header = ttk.Frame(ocr_card)
        ocr_header.grid(row=0, column=0, sticky="ew")
        ocr_header.columnconfigure(0, weight=1)
        ttk.Label(ocr_header, text="OCR Stream", style="LiveTitle.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(ocr_header, textvariable=self.target_force_var,
                  font=("Consolas", 14, "bold"), foreground="#999999").grid(row=0, column=1, sticky="e")
        ttk.Label(ocr_card, textvariable=self.ocr_value_var, style="BigValue.TLabel").grid(row=1, column=0, sticky="w")

        self._error_live_label(left, "Live % Error", self.live_error_var, 2)
        self._small_label(left, "HADI Raw R (mV/V)", self.hadi_raw_var, 3)

        self.big_capture_button = ttk.Button(
            left,
            text="CAPTURE  (Space / Enter)",
            command=self._capture,
            style="BigCapture.TButton",
        )
        self.big_capture_button.grid(row=4, column=0, sticky="ew", padx=12, pady=(18, 4), ipady=16)

        self.w_button_var = tk.StringVar(value="SELECT / UNSELECT W")
        ttk.Button(
            left,
            textvariable=self.w_button_var,
            command=self._set_selected_manual_weight,
        ).grid(row=5, column=0, sticky="ew", padx=12, pady=(10, 2), ipady=8)

        right = ttk.LabelFrame(display, text="Capture")
        right.pack(side="right", fill="both", expand=True, padx=(6, 0))

        btns = ttk.Frame(right)
        btns.pack(fill="x", padx=10, pady=10)

        ttk.Button(
            btns,
            text="SAVE TO CSV",
            command=self._save_csv,
            width=24,
        ).pack(side="left", padx=8)
        ttk.Button(btns, text="Clear", command=self._clear_captures).pack(side="left")

        ttk.Label(btns, text="Capacity:").pack(side="left", padx=(10, 2))
        cap_entry = ttk.Entry(btns, textvariable=self.customer_capacity_var, width=7)
        cap_entry.pack(side="left")
        cap_entry.bind("<Return>", lambda _e: self._apply_customer_capacity())
        ttk.Button(btns, text="Set", command=self._apply_customer_capacity, width=3).pack(side="left", padx=(2, 0))

        ttk.Label(btns, text="Points").pack(side="left", padx=(14, 4))
        self.point_count_combo = ttk.Combobox(
            btns,
            textvariable=self.point_count_var,
            values=["11", "20", "Custom"],
            state="readonly",
            width=8,
        )
        self.point_count_combo.pack(side="left")
        self.point_count_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_point_count_choice())

        self.custom_point_entry = ttk.Entry(btns, textvariable=self.custom_point_count_var, width=5)
        self.custom_point_set_btn = ttk.Button(btns, text="Set", command=self._apply_point_count)

        tables = ttk.Frame(right)
        tables.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        tables.columnconfigure(0, weight=1)
        tables.columnconfigure(1, weight=1)
        tables.rowconfigure(1, weight=1)

        ttk.Label(tables, text="Run 1", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, sticky="w", padx=(0, 6))
        ttk.Label(tables, text="Run 2", font=("Segoe UI", 10, "bold")).grid(row=0, column=1, sticky="w", padx=(6, 0))

        run_cols = ("point", "hadi", "ocr", "error")
        self.run1_tree = ttk.Treeview(tables, columns=run_cols, show="headings", height=18, selectmode="extended")
        self.run2_tree = ttk.Treeview(tables, columns=run_cols, show="headings", height=18, selectmode="extended")

        for tree in (self.run1_tree, self.run2_tree):
            tree.heading("point", text="#")
            tree.heading("hadi", text="HADI")
            tree.heading("ocr", text="OCR")
            tree.heading("error", text="%")
            tree.column("point", width=38, anchor="center", stretch=False)
            tree.column("hadi", width=104, anchor="center", stretch=False)
            tree.column("ocr", width=104, anchor="center", stretch=False)
            tree.column("error", width=58, anchor="center", stretch=False)
            tree.tag_configure("warn", background="#fff3cd")
            tree.tag_configure("bad", background="#f8d7da")

        self.run1_tree.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        self.run2_tree.grid(row=1, column=1, sticky="nsew", padx=(6, 0))

        self.run1_tree.bind("<<TreeviewSelect>>", lambda e: self._on_run_tree_select(e, 1))
        self.run2_tree.bind("<<TreeviewSelect>>", lambda e: self._on_run_tree_select(e, 2))
        self.run1_tree.bind("<ButtonRelease-1>", lambda e: self._on_run_tree_click(e, 1))
        self.run2_tree.bind("<ButtonRelease-1>", lambda e: self._on_run_tree_click(e, 2))
        self.run1_tree.bind("<Double-1>", lambda e: self._on_run_tree_double_click(e, 1))
        self.run2_tree.bind("<Double-1>", lambda e: self._on_run_tree_double_click(e, 2))

        # Compatibility alias for older helper code paths.
        self.tree = self.run1_tree

        self._initialize_point_rows(11)

    def _build_connection_tab(self, root):
        pad = {"padx": 10, "pady": 8}

        box = ttk.LabelFrame(root, text="Detailed Connections")
        box.pack(fill="x", **pad)

        ttk.Label(box, text="HADI COM Port").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        self.port_combo = ttk.Combobox(box, textvariable=self.port_var, width=18)
        self.port_combo.grid(row=0, column=1, sticky="w", pady=6)
        ttk.Button(box, text="Refresh Ports", command=self._refresh_ports).grid(row=0, column=2, sticky="w", padx=6)

        ttk.Label(box, text="Baud").grid(row=0, column=3, sticky="w", padx=(20, 4))
        ttk.Entry(box, textvariable=self.baud_var, width=8).grid(row=0, column=4, sticky="w")

        ttk.Button(box, textvariable=self.hadi_button_var, command=self._toggle_hadi).grid(row=0, column=5, padx=12)

        ttk.Label(box, text="OCR UDP Port").grid(row=1, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(box, textvariable=self.ocr_port_var, width=10).grid(row=1, column=1, sticky="w", pady=6)
        ttk.Button(box, textvariable=self.ocr_button_var, command=self._toggle_ocr).grid(row=1, column=2, sticky="w", padx=6)

        ttk.Label(box, text="HADI Update Hz").grid(row=2, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(box, textvariable=self.poll_var, width=8).grid(row=2, column=1, sticky="w", pady=6)
        ttk.Button(box, text="Apply", command=self._apply_poll_rate).grid(row=2, column=2, sticky="w", padx=6)

        ttk.Label(box, textvariable=self.status_var).grid(row=3, column=0, columnspan=6, sticky="w", padx=6, pady=(12, 6))

        gps_box = ttk.LabelFrame(root, text="Gravity / GPS")
        gps_box.pack(fill="x", **pad)
        ttk.Label(gps_box, textvariable=self.gps_status_var).grid(row=0, column=0, sticky="w", padx=6, pady=6)

        sync_box = ttk.LabelFrame(root, text="Sync Calibration")
        sync_box.pack(fill="x", **pad)
        ttk.Label(sync_box, textvariable=self.sync_status_var).grid(row=0, column=0, columnspan=4, sticky="w", padx=6, pady=(6, 2))
        ttk.Label(sync_box, text="Cal lag (ms)").grid(row=1, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(sync_box, textvariable=self.cal_lag_ms_var, width=8).grid(row=1, column=1, sticky="w", pady=6)
        ttk.Button(sync_box, text="Set", command=self._apply_calibration_lag).grid(row=1, column=2, sticky="w", padx=6)

    def _build_load_cells_tab(self, root):
        pad = {"padx": 10, "pady": 8}

        top = ttk.LabelFrame(root, text="Saved Load Cells")
        top.pack(fill="x", **pad)

        ttk.Label(top, text="Select").grid(row=0, column=0, sticky="w")
        self.editor_combo = ttk.Combobox(
            top,
            textvariable=self.editor_select_var,
            values=[c["name"] for c in self.load_cells],
            state="readonly",
            width=36,
        )
        self.editor_combo.grid(row=0, column=1, sticky="w")
        self.editor_combo.bind("<<ComboboxSelected>>", lambda _e: self._load_cell_into_editor())

        ttk.Button(top, text="New", command=self._new_load_cell).grid(row=0, column=2, padx=6)
        ttk.Button(top, text="Save / Update", command=self._save_load_cell_from_editor).grid(row=0, column=3, padx=6)
        ttk.Button(top, text="Delete", command=self._delete_load_cell).grid(row=0, column=4, padx=6)
        ttk.Button(top, text="Use Selected", command=self._use_editor_load_cell).grid(row=0, column=5, padx=6)

        form = ttk.LabelFrame(root, text="Load Cell Calibration Coefficients")
        form.pack(fill="both", expand=True, **pad)

        ttk.Label(form, text="Name").grid(row=0, column=0, sticky="w", padx=10, pady=(14, 4))
        ttk.Entry(form, textvariable=self.cell_name_var, width=42).grid(row=0, column=1, columnspan=2, sticky="w", pady=(14, 4))

        ttk.Label(form, text="Capacity lbf").grid(row=0, column=3, sticky="e", padx=(28, 8), pady=(14, 4))
        ttk.Entry(form, textvariable=self.capacity_var, width=12).grid(row=0, column=4, sticky="w", pady=(14, 4))

        note = (
            "Enter B coefficients for: Force (lbf) = B0 + B1*R + B2*R^2 + B3*R^3 + B4*R^4 + B5*R^5\n"
            "All B coefficients are optional and act as zero when left blank. "
            "Scientific notation like -2.610015E+04 is accepted."
        )
        ttk.Label(form, text=note).grid(row=1, column=0, columnspan=7, sticky="w", padx=10, pady=(4, 16))

        headers = ["B0", "B1", "B2", "B3", "B4", "B5"]

        ttk.Label(form, text="Compression", font=("Segoe UI", 11, "bold")).grid(row=2, column=0, sticky="w", padx=10)
        for i, coeff in enumerate(headers):
            ttk.Label(form, text=coeff).grid(row=2, column=i + 1, sticky="w", padx=6)
            v = tk.StringVar()
            self.coeff_vars[f"compression_{coeff}"] = v
            ttk.Entry(form, textvariable=v, width=18).grid(row=3, column=i + 1, sticky="w", padx=6, pady=(2, 14))

        ttk.Label(form, text="Tension", font=("Segoe UI", 11, "bold")).grid(row=4, column=0, sticky="w", padx=10)
        for i, coeff in enumerate(headers):
            ttk.Label(form, text=coeff).grid(row=4, column=i + 1, sticky="w", padx=6)
            v = tk.StringVar()
            self.coeff_vars[f"tension_{coeff}"] = v
            ttk.Entry(form, textvariable=v, width=18).grid(row=5, column=i + 1, sticky="w", padx=6, pady=(2, 14))

        ttk.Label(
            form,
            text="Saved load cells are stored in load_cells.json next to the app, so they will be there next time.",
        ).grid(row=6, column=0, columnspan=7, sticky="w", padx=10, pady=16)

    def _hadi_live_label(self, parent, title_var: tk.StringVar, var: tk.StringVar, row: int):
        card = ttk.Frame(parent)
        card.grid(row=row, column=0, sticky="ew", padx=12, pady=(14, 0))
        card.columnconfigure(0, weight=1)

        header = ttk.Frame(card)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        title_frame = ttk.Frame(header)
        title_frame.grid(row=0, column=0, sticky="w")
        ttk.Label(title_frame, textvariable=title_var, style="LiveTitle.TLabel").pack(side="left")
        self.hadi_status_light = tk.Canvas(title_frame, width=12, height=12, highlightthickness=0)
        self.hadi_status_light.pack(side="left", padx=(6, 0))
        self._hadi_light_id = self.hadi_status_light.create_oval(1, 1, 11, 11, fill="#cc0000", outline="#888888")
        ttk.Button(header, text="ZERO", command=self._tare_indicator, style="Zero.TButton", width=8).grid(row=0, column=1, sticky="e", padx=(10, 0))

        ttk.Label(card, textvariable=var, style="BigValue.TLabel").grid(row=1, column=0, sticky="w")

    def _error_live_label(self, parent, title: str, var: tk.StringVar, row: int):
        card = ttk.Frame(parent)
        card.grid(row=row, column=0, sticky="ew", padx=12, pady=(14, 0))
        ttk.Label(card, text=title, style="LiveTitle.TLabel").pack(anchor="w")
        self.live_error_label = ttk.Label(card, textvariable=var, style="BigValue.TLabel")
        self.live_error_label.pack(anchor="w")
        parent.columnconfigure(0, weight=1)

        parent.columnconfigure(0, weight=1)

    def _big_label(self, parent, title: str, var: tk.StringVar, row: int):
        card = ttk.Frame(parent)
        card.grid(row=row, column=0, sticky="ew", padx=12, pady=(14, 0))
        ttk.Label(card, text=title, style="LiveTitle.TLabel").pack(anchor="w")
        ttk.Label(card, textvariable=var, style="BigValue.TLabel").pack(anchor="w")
        parent.columnconfigure(0, weight=1)

    def _small_label(self, parent, title: str, var: tk.StringVar, row: int):
        card = ttk.Frame(parent)
        card.grid(row=row, column=0, sticky="ew", padx=12, pady=(8, 0))
        ttk.Label(card, text=title, font=("Segoe UI", 10, "bold")).pack(anchor="w")
        ttk.Label(card, textvariable=var, style="SmallValue.TLabel").pack(anchor="w")
        parent.columnconfigure(0, weight=1)

    def _tiny_label(self, parent, title: str, var: tk.StringVar, row: int):
        card = ttk.Frame(parent)
        card.grid(row=row, column=0, sticky="ew", padx=12, pady=(2, 0))
        ttk.Label(card, text=title, font=("Segoe UI", 8)).pack(anchor="w")
        ttk.Label(card, textvariable=var, style="TinyValue.TLabel").pack(anchor="w")
        parent.columnconfigure(0, weight=1)

    def _refresh_ports(self):
        if list_ports is None:
            self.port_combo["values"] = []
            return
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _release_button_focus(self, _event=None):
        self.after(1, self.focus_set)

    def _set_hadi_display_options(self):
        units = self.hadi_units_var.get()
        self.hadi_title_var.set("HADI Response (mV/V)" if units == "mV/V" else f"HADI Force ({units})")
        self._refresh_hadi_mode_badge()
        if self.latest_hadi:
            self.hadi_lbf_var.set(self._format_hadi_display_value(self.latest_hadi))
            self._update_capacity_warning(self.latest_hadi)
        else:
            self._update_capacity_warning(None)
        self._update_target_force_display()

    def _refresh_hadi_mode_badge(self):
        mode = self.mode_var.get()
        self.mode_badge_var.set(mode.upper())
        if hasattr(self, "mode_badge_label"):
            style = "ModeBadgeCompression.TLabel" if mode == "Compression" else "ModeBadgeTension.TLabel"
            self.mode_badge_label.configure(style=style)

    @staticmethod
    def _format_ocr_number(value) -> str:
        """Display OCR without forcing four decimals.

        When raw OCR text is unavailable, fall back to a compact numeric string
        rather than rounding to a fixed number of places.
        """
        if value in (None, ""):
            return ""
        try:
            return f"{float(value):+g}"
        except Exception:
            text = str(value).strip()
            return text if text.startswith(("+", "-")) else f"+{text}"

    @staticmethod
    def _format_ocr_text(raw_text, value=None, show_plus: bool = True) -> str:
        text = "" if raw_text in (None, "") else str(raw_text).strip()
        if not text and value not in (None, ""):
            text = f"{float(value):g}"
        if not text:
            return ""
        if show_plus and not text.startswith(("+", "-")):
            return "+" + text
        if not show_plus and text.startswith("+"):
            return text[1:]
        return text

    @classmethod
    def _ocr_text_for_row(cls, run_row, show_plus: bool = True) -> str:
        if not run_row or run_row.get("ocr") in (None, ""):
            return ""
        return cls._format_ocr_text(run_row.get("ocr_text"), run_row.get("ocr"), show_plus=show_plus)

    def _format_hadi_display_value(self, reading: HADIReading) -> str:
        units = self.hadi_units_var.get()
        decimals = _decimals_from_step(self.hadi_decimals_var.get())
        if units == "mV/V":
            value = reading.raw_response
        else:
            factor = HADI_UNIT_FACTORS.get(units, 1.0)
            value = reading.force_lbf * factor
        return f"{value:+.{decimals}f}"

    def _capacity_limit_lbf(self) -> Optional[float]:
        """Return the selected load-cell capacity in lbf, or None if unset."""
        cell = self._find_load_cell(self.selected_load_cell_name.get())
        if not cell:
            return None
        try:
            capacity = float(cell.get("capacity_lbf"))
        except Exception:
            return None
        return capacity if capacity > 0 else None

    def _format_force_for_current_units(self, force_lbf: float) -> str:
        """Format a force value in the currently selected display unit."""
        units = self.hadi_units_var.get()
        if units == "mV/V":
            return f"{force_lbf:.1f} LBF"
        factor = HADI_UNIT_FACTORS.get(units, 1.0)
        decimals = _decimals_from_step(self.hadi_decimals_var.get())
        return f"{force_lbf * factor:.{decimals}f} {units}"

    def _selected_hadi_decimals(self) -> int:
        """Number of decimal places selected for HADI display/capture output."""
        return _decimals_from_step(self.hadi_decimals_var.get())

    def _format_hadi_lbf_text(self, value, show_plus: bool = False) -> str:
        """Format captured HADI LBF with exactly the selected decimal count.

        Capture rows keep this text so the table, autosave, and manual CSV export
        all preserve the same decimal precision the operator selected at capture
        time instead of later shortening/reformatting the float.
        """
        if value in (None, ""):
            return ""
        try:
            decimals = self._selected_hadi_decimals()
            sign = "+" if show_plus else ""
            return f"{float(value):{sign}.{decimals}f}"
        except Exception:
            return str(value)

    def _set_row_hadi_text(self, row: dict) -> None:
        """Store the fixed-decimal HADI text used by table and CSV export."""
        if row is not None:
            row["hadi_text"] = self._format_hadi_lbf_text(row.get("hadi_lbf"), show_plus=False)

    def _update_capacity_warning(self, reading: Optional[HADIReading] = None) -> None:
        """Warn when the load cell force exceeds 105% of selected capacity.

        The comparison is always done in lbf because calibration coefficients
        produce lbf. The displayed warning follows the user's selected units.
        """
        capacity_lbf = self._capacity_limit_lbf()
        if capacity_lbf is None or reading is None:
            self.capacity_warning_var.set("")
            return

        force_lbf = abs(float(reading.force_lbf))
        limit_lbf = capacity_lbf * 1.05
        if force_lbf >= limit_lbf:
            self.capacity_warning_var.set(
                f"OVER CAPACITY: {self._format_force_for_current_units(force_lbf)} / "
                f"{self._format_force_for_current_units(limit_lbf)} max"
            )
        else:
            self.capacity_warning_var.set("")

    def _auto_connect_on_launch(self):
        if not self.ocr:
            self._start_ocr(show_error=False)
        self._kick_auto_scan()

    def _set_hadi_light(self, color: str):
        self.hadi_status_light.itemconfig(self._hadi_light_id, fill=color)

    def _kick_auto_scan(self):
        if self.hadi.is_connected():
            return
        if self._scan_thread and self._scan_thread.is_alive():
            return
        now = time.time()
        if now - self._last_auto_scan < self._auto_scan_interval:
            return
        self._last_auto_scan = now
        self.status_var.set("HADI: scanning ports...")
        self._set_hadi_light("#ccaa00")
        baud = int(self.baud_var.get())
        self._scan_thread = threading.Thread(
            target=self._background_scan, args=(baud,), daemon=True)
        self._scan_thread.start()

    def _background_scan(self, baudrate: int):
        if list_ports is None:
            return
        ports = [p.device for p in list_ports.comports()]
        if self._last_good_port and self._last_good_port in ports:
            ports.remove(self._last_good_port)
            ports.insert(0, self._last_good_port)
        for port in ports:
            if self.hadi.is_connected():
                return
            if HADIWorker.probe_port(port, baudrate):
                self.after(0, lambda p=port: self._auto_connect_port(p))
                return
        self.after(0, self._on_scan_failed)

    def _on_scan_failed(self):
        self.status_var.set("HADI: no device found, retrying...")
        self._set_hadi_light("#cc0000")

    def _auto_connect_port(self, port: str):
        if self.hadi.is_connected():
            return
        self.port_var.set(port)
        self._refresh_ports()
        try:
            self.hadi.connect(port, int(self.baud_var.get()))
            self._select_load_cell()
            self._set_mode()
            self._apply_poll_rate()
            self.hadi_button_var.set("Disconnect HADI")
            self._last_good_port = port
            self._set_hadi_light("#00aa00")
            self.status_var.set(f"HADI connected on {port}")
        except Exception:
            self._set_hadi_light("#cc0000")
            self.status_var.set(f"HADI: probe OK but connect failed on {port}")

    def _toggle_hadi(self):
        if self.hadi.is_connected():
            self._disconnect_hadi()
            self._auto_scan_active = False
        else:
            self._auto_scan_active = True
            self._connect_hadi()

    def _toggle_ocr(self):
        if self.ocr:
            self._stop_ocr()
        else:
            self._start_ocr()

    def _connect_hadi(self, show_error=True):
        try:
            self.hadi.connect(self.port_var.get(), int(self.baud_var.get()))
            self._select_load_cell()
            self._set_mode()
            self._apply_poll_rate()
            self.hadi_button_var.set("Disconnect HADI")
            self._last_good_port = self.port_var.get()
            self._auto_scan_active = True
            self._set_hadi_light("#00aa00")
            self.status_var.set(f"HADI connected on {self.port_var.get()}")
        except Exception as exc:
            self._set_hadi_light("#cc0000")
            self.status_var.set(f"HADI waiting: {exc}")
            if show_error:
                messagebox.showerror("HADI connection failed", str(exc))

    def _disconnect_hadi(self):
        self.hadi.disconnect()
        self.hadi_button_var.set("Connect HADI")
        self._set_hadi_light("#cc0000")
        self.status_var.set("HADI disconnected")
        self.latest_hadi = None
        self.hadi_last_pc_time = None
        self.hadi_lbf_var.set("WAITING FOR HADI")
        self.hadi_raw_var.set("WAITING")
        self.raw_text_var.set("")
        self._update_live_percent_error()

    def _start_ocr(self, show_error=True):
        try:
            self._stop_ocr()
            self.ocr = OCRReceiver(port=int(self.ocr_port_var.get()))
            self.ocr.on_reading = self._on_ocr_reading
            self.ocr.on_gps = self._on_gps_fix
            self.ocr.start()
            self.ocr_button_var.set("Stop OCR")
            self.status_var.set("OCR running")
        except Exception as exc:
            self.status_var.set(f"OCR waiting: {exc}")
            if show_error:
                messagebox.showerror("OCR start failed", str(exc))

    def _stop_ocr(self):
        if self.ocr:
            self.ocr.stop()
            self.ocr = None
        self.ocr_button_var.set("Start OCR")
        self.latest_ocr = None
        self.ocr_last_pc_time = None
        self.ocr_value_var.set("WAITING FOR OCR")
        self._update_live_percent_error()

    def _toggle_mode(self):
        if self.mode_var.get() == "Compression":
            self.mode_var.set("Tension")
        else:
            self.mode_var.set("Compression")
        self._set_mode()

    def _set_mode(self):
        self.hadi.mode = self.mode_var.get()
        self._refresh_hadi_mode_badge()
        self._update_capacity_warning(self.latest_hadi)

    def _apply_poll_rate(self):
        try:
            self.hadi.poll_hz = float(self.poll_var.get())
        except ValueError:
            messagebox.showerror("Invalid speed", "Update speed must be a number.")

    def _tare_indicator(self):
        try:
            self.hadi.send_tare_to_indicator()
        except Exception as exc:
            messagebox.showerror("Tare failed", str(exc))

    def _find_load_cell(self, name: str) -> Optional[dict]:
        for cell in self.load_cells:
            if cell["name"] == name:
                return cell
        return None

    def _select_load_cell(self):
        cell = self._find_load_cell(self.selected_load_cell_name.get())
        if cell:
            self.hadi.load_cell = cell
            self.status_var.set(f"Using load cell: {cell['name']}")
            self._update_capacity_warning(self.latest_hadi)

    def _sync_load_cell_controls(self):
        names = [c["name"] for c in self.load_cells]
        if hasattr(self, "load_cell_combo"):
            self.load_cell_combo["values"] = names
        if hasattr(self, "editor_combo"):
            self.editor_combo["values"] = names

        if names:
            if self.selected_load_cell_name.get() not in names:
                self.selected_load_cell_name.set(names[0])
            if not self.editor_select_var.get() or self.editor_select_var.get() not in names:
                self.editor_select_var.set(self.selected_load_cell_name.get())
            self._select_load_cell()
            self._load_cell_into_editor()

    def _load_cell_into_editor(self):
        cell = self._find_load_cell(self.editor_select_var.get())
        if not cell:
            return
        self.cell_name_var.set(cell.get("name", ""))
        self.capacity_var.set("" if cell.get("capacity_lbf") is None else str(cell.get("capacity_lbf")))
        for side in ("compression", "tension"):
            for coeff in ("B0", "B1", "B2", "B3", "B4", "B5"):
                value = cell.get(side, {}).get(coeff, 0.0)
                self.coeff_vars[f"{side}_{coeff}"].set("" if float(value or 0) == 0 else format_coeff(value))

    def _new_load_cell(self):
        self.editor_select_var.set("")
        self.cell_name_var.set("")
        self.capacity_var.set("")
        for v in self.coeff_vars.values():
            v.set("")

    def _read_editor_load_cell(self) -> dict:
        name = self.cell_name_var.get().strip()
        if not name:
            raise ValueError("Load cell name is required.")

        capacity_text = self.capacity_var.get().strip()
        capacity = float(capacity_text) if capacity_text else None

        cell = {
            "name": name,
            "capacity_lbf": capacity,
            "compression": {},
            "tension": {},
        }

        for side in ("compression", "tension"):
            for coeff in ("B0", "B1", "B2", "B3", "B4", "B5"):
                value_text = self.coeff_vars[f"{side}_{coeff}"].get().strip()
                if not value_text:
                    cell[side][coeff] = 0.0
                    continue
                normalized = value_text.replace(" ", "").replace("−", "-")
                try:
                    cell[side][coeff] = float(normalized)
                except ValueError:
                    raise ValueError(
                        f"{side.title()} {coeff} must be a number, for example -2.610015E+04."
                    )

        return cell

    def _save_load_cell_from_editor(self):
        try:
            cell = self._read_editor_load_cell()
        except Exception as exc:
            messagebox.showerror("Invalid load cell", str(exc))
            return

        existing = self._find_load_cell(cell["name"])
        if existing:
            existing.clear()
            existing.update(cell)
        else:
            self.load_cells.append(cell)

        save_load_cells(self.load_cells)
        self.selected_load_cell_name.set(cell["name"])
        self.editor_select_var.set(cell["name"])
        self._sync_load_cell_controls()
        self._load_cell_into_editor()
        messagebox.showinfo("Saved", f"Saved load cell: {cell['name']}")

    def _delete_load_cell(self):
        name = self.editor_select_var.get()
        if len(self.load_cells) <= 1:
            messagebox.showwarning("Cannot delete", "At least one load cell must remain.")
            return
        if not name:
            return
        self.load_cells = [c for c in self.load_cells if c["name"] != name]
        save_load_cells(self.load_cells)
        self.selected_load_cell_name.set(self.load_cells[0]["name"])
        self.editor_select_var.set(self.load_cells[0]["name"])
        self._sync_load_cell_controls()

    def _use_editor_load_cell(self):
        name = self.editor_select_var.get() or self.cell_name_var.get().strip()
        cell = self._find_load_cell(name)
        if not cell:
            messagebox.showwarning("Not saved", "Save this load cell before using it.")
            return
        self.selected_load_cell_name.set(name)
        self._select_load_cell()
        self.notebook.select(self.capture_tab)

    def _prune_buffers(self, now_pc: Optional[float] = None):
        if now_pc is None:
            now_pc = time.perf_counter()
        cutoff = now_pc - self.buffer_keep_seconds
        while self.hadi_buffer and self.hadi_buffer[0].pc_time < cutoff:
            self.hadi_buffer.popleft()
        with self.ocr_lock:
            while self.ocr_buffer and self.ocr_buffer[0].pc_time < cutoff:
                self.ocr_buffer.popleft()

    def _on_ocr_reading(self, reading):
        now_pc = time.perf_counter()
        sample = OCRTimedReading(
            value=reading.value,
            pc_time=now_pc,
            wall_time=reading.received_at,
            phone_time=reading.timestamp,
            raw_text=getattr(reading, "raw_text", "") or f"{reading.value:g}",
        )
        with self.ocr_lock:
            self.latest_ocr = sample
            self.ocr_buffer.append(sample)
        self.ocr_last_pc_time = now_pc
        self._prune_buffers(now_pc)

    def _on_gps_fix(self, gps):
        altitude_m = getattr(gps, "altitude_m", None)
        g = normal_gravity_m_s2(gps.latitude, altitude_m)
        fix = GPSFix(
            latitude=gps.latitude,
            longitude=gps.longitude,
            altitude_m=altitude_m,
            phone_time=gps.timestamp,
            received_at=gps.received_at,
            gravity_m_s2=g,
            gravity_factor=g / STANDARD_GRAVITY,
        )

        # Always keep/display the latest GPS fix so the operator can verify that
        # the phone is sending coordinates. By default W-row math still ignores
        # GPS and uses standard gravity, so moving the phone cannot change the
        # captured weight values.
        self.latest_gps = fix

        if not USE_GPS_GRAVITY_CORRECTION:
            self.gps_status_var.set(
                f"GPS receiving | MF {fix.gravity_factor:.7f} | W uses standard g"
            )
            return

        if not hasattr(self, "locked_gps"):
            self.locked_gps = None
        if self.locked_gps is None:
            self.locked_gps = fix

        locked = self.locked_gps
        self.gps_status_var.set(
            f"Gravity locked | MF {locked.gravity_factor:.7f}"
        )
        self.w_button_var.set(f"SELECT / UNSELECT W    MF {locked.gravity_factor:.7f}")

    @staticmethod
    def _interp(samples, t, attr):
        if len(samples) < 2 or t < samples[0].pc_time or t > samples[-1].pc_time:
            return None
        prev = samples[0]
        for cur in samples[1:]:
            if cur.pc_time >= t:
                span = cur.pc_time - prev.pc_time
                if span <= 0:
                    return getattr(cur, attr)
                f = (t - prev.pc_time) / span
                return getattr(prev, attr) + (getattr(cur, attr) - getattr(prev, attr)) * f
            prev = cur
        return None

    def _nearest_sample(self, samples, target_time, corrected_ocr=False, max_age_seconds=0.75):
        """Return the nearest real sample, or None if it is too far away."""
        if not samples:
            return None
        nearest = min(
            samples,
            key=lambda s: abs((self._corrected_ocr_time(s) if corrected_ocr else s.pc_time) - target_time),
        )
        nearest_time = self._corrected_ocr_time(nearest) if corrected_ocr else nearest.pc_time
        if abs(nearest_time - target_time) > max_age_seconds:
            return None
        return nearest

    def _nearest_sample_value(self, samples, target_time, attr, corrected_ocr=False, max_age_seconds=0.75):
        """Return the exact value from the nearest real sample.

        This is intentionally used for OCR so the app never invents in-between
        display values like 10.0473 when the phone only saw 10.0 or 10.1.
        """
        nearest = self._nearest_sample(samples, target_time, corrected_ocr, max_age_seconds)
        return None if nearest is None else getattr(nearest, attr)

    @staticmethod
    def _corr(xs, ys):
        n = len(xs)
        if n < 8:
            return None
        mx = sum(xs) / n
        my = sum(ys) / n
        vx = sum((x - mx) ** 2 for x in xs)
        vy = sum((y - my) ** 2 for y in ys)
        if vx <= 1e-9 or vy <= 1e-9:
            return None
        return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / ((vx * vy) ** 0.5)

    def _estimate_sync_lag(self):
        now_pc = time.perf_counter()
        if now_pc - self._last_sync_update < 1.0:
            return
        self._last_sync_update = now_pc
        self._prune_buffers(now_pc)

        hadi = list(self.hadi_buffer)
        with self.ocr_lock:
            ocr = list(self.ocr_buffer)

        if len(hadi) < 12 or len(ocr) < 12:
            cal_note = f" | cal {self.calibration_lag_seconds*1000:+.0f}" if self.calibration_lag_seconds > 0 else ""
            self.sync_status_var.set(f"Sync: waiting for data | lag {self.sync_lag_seconds*1000:+.0f} ms{cal_note}")
            return

        # Use recent data only.
        recent_start = now_pc - self.sync_window_seconds
        hadi = [s for s in hadi if s.pc_time >= recent_start]
        ocr = [s for s in ocr if s.pc_time >= recent_start]
        if len(hadi) < 8 or len(ocr) < 8:
            return

        # Need enough actual movement; otherwise any lag estimate is meaningless.
        h_vals = [s.force_lbf for s in hadi]
        o_vals = [s.value for s in ocr]
        if (max(h_vals) - min(h_vals)) < 5 or (max(o_vals) - min(o_vals)) < 5:
            cal_note = f" | cal {self.calibration_lag_seconds*1000:+.0f}" if self.calibration_lag_seconds > 0 else ""
            self.sync_status_var.set(f"Sync: steady load | lag {self.sync_lag_seconds*1000:+.0f} ms{cal_note}")
            return

        best_lag = None
        best_corr = -2.0

        # When calibration is set, search a narrow window around it with
        # finer steps.  Otherwise scan the full ±500 ms range.
        cal = self.calibration_lag_seconds
        if cal > 0:
            center_ms = int(round(cal * 1000.0))
            search_range = range(center_ms - 200, center_ms + 201, 10)
        else:
            search_range = range(-500, 501, 25)

        for lag_ms in search_range:
            lag = lag_ms / 1000.0
            t0 = max(hadi[0].pc_time, ocr[0].pc_time - lag)
            t1 = min(hadi[-1].pc_time, ocr[-1].pc_time - lag)
            if t1 - t0 < 1.0:
                continue

            xs = []
            ys = []
            t = t0
            while t <= t1:
                hv = self._interp(hadi, t, "force_lbf")
                ov = self._interp(ocr, t + lag, "value")
                if hv is not None and ov is not None:
                    xs.append(hv)
                    ys.append(ov)
                t += 0.05

            c = self._corr(xs, ys)
            if c is not None and c > best_corr:
                best_corr = c
                best_lag = lag

        if best_lag is None:
            if cal > 0:
                self.sync_lag_seconds = 0.95 * self.sync_lag_seconds + 0.05 * cal
                self.sync_status_var.set(
                    f"Cal anchor {cal*1000:+.0f} ms | lag {self.sync_lag_seconds*1000:+.0f} ms (drifting to cal)")
            return

        if best_corr >= 0.65:
            self.sync_lag_seconds = 0.80 * self.sync_lag_seconds + 0.20 * best_lag
            if cal > 0:
                self.sync_lag_seconds = 0.95 * self.sync_lag_seconds + 0.05 * cal
            self.sync_confidence = best_corr
            save_sync_state(self.sync_lag_seconds * 1000.0, self.sync_confidence,
                            calibration_lag_ms=cal * 1000.0)

        cal_note = f" | cal {cal*1000:+.0f}" if cal > 0 else ""
        self.sync_status_var.set(
            f"Sync lag {self.sync_lag_seconds*1000:+.0f} ms | conf {self.sync_confidence:.2f}{cal_note}"
        )

    def _apply_calibration_lag(self):
        raw = self.cal_lag_ms_var.get().strip()
        if not raw:
            self.calibration_lag_seconds = 0.0
            save_sync_state(self.sync_lag_seconds * 1000.0, self.sync_confidence,
                            calibration_lag_ms=0.0)
            self.sync_status_var.set(f"Calibration cleared | lag {self.sync_lag_seconds*1000:+.0f} ms")
            return
        try:
            cal_ms = float(raw)
        except ValueError:
            return
        self.calibration_lag_seconds = cal_ms / 1000.0
        self.sync_lag_seconds = self.calibration_lag_seconds
        self.sync_confidence = 0.0
        save_sync_state(self.sync_lag_seconds * 1000.0, self.sync_confidence,
                        calibration_lag_ms=cal_ms)
        self.sync_status_var.set(f"Cal {cal_ms:+.0f} ms applied | auto sync will refine from here")

    def _corrected_ocr_time(self, sample):
        return sample.pc_time - self.sync_lag_seconds

    def _median_near_time(self, samples, target_time, attr, corrected_ocr=False):
        half = self.capture_median_half_window
        vals = []
        for s in samples:
            t = self._corrected_ocr_time(s) if corrected_ocr else s.pc_time
            if abs(t - target_time) <= half:
                vals.append(getattr(s, attr))
        if vals:
            return statistics.median(vals), len(vals)
        if not samples:
            return None, 0
        # fallback: nearest in time
        nearest = min(samples, key=lambda s: abs((self._corrected_ocr_time(s) if corrected_ocr else s.pc_time) - target_time))
        return getattr(nearest, attr), 1

    def _hadi_is_fresh(self, now_pc=None) -> bool:
        if now_pc is None:
            now_pc = time.perf_counter()
        return self.hadi_last_pc_time is not None and (now_pc - self.hadi_last_pc_time) <= self.hadi_wait_seconds

    def _ocr_is_fresh(self, now_pc=None) -> bool:
        if now_pc is None:
            now_pc = time.perf_counter()
        return self.ocr_last_pc_time is not None and (now_pc - self.ocr_last_pc_time) <= self.ocr_wait_seconds

    def _set_hadi_waiting_display(self):
        self.latest_hadi = None
        self.hadi_lbf_var.set("WAITING FOR HADI")
        self.hadi_raw_var.set("WAITING")
        self.raw_text_var.set("")
        self._update_capacity_warning(None)

    def _set_ocr_waiting_display(self):
        with self.ocr_lock:
            self.latest_ocr = None
        self.ocr_value_var.set("WAITING FOR OCR")

    def _ui_tick(self):
        now_pc = time.perf_counter()
        got_hadi = False

        while True:
            try:
                item = self.hadi.out.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, Exception):
                self.status_var.set(f"HADI error: {item}")
                self._set_hadi_light("#cc0000")
                self.hadi_last_pc_time = None
                self._set_hadi_waiting_display()
                if self._auto_scan_active:
                    self.hadi.disconnect()
                    self.hadi_button_var.set("Connect HADI")
                continue

            got_hadi = True
            self.latest_hadi = item
            self.hadi_last_pc_time = item.pc_time
            self.hadi_buffer.append(item)
            self._prune_buffers(item.pc_time)
            self.hadi_raw_var.set(f"{item.raw_response:+.5f}")
            self.hadi_lbf_var.set(self._format_hadi_display_value(item))
            self._update_capacity_warning(item)
            self.raw_text_var.set(item.raw_text)

        if not got_hadi and not self._hadi_is_fresh(now_pc):
            self._set_hadi_waiting_display()

        if self.ocr:
            with self.ocr_lock:
                r = self.latest_ocr
            if r is None or not self._ocr_is_fresh(now_pc):
                self._set_ocr_waiting_display()
            else:
                self.ocr_value_var.set(self._format_ocr_text(r.raw_text, r.value))
        else:
            self._set_ocr_waiting_display()

        self._estimate_sync_lag()
        self._update_live_percent_error()

        if self._auto_scan_active and not self.hadi.is_connected():
            self._kick_auto_scan()

        self.after(50, self._ui_tick)

    def _tree_for_run(self, run: int):
        return self.run1_tree if run == 1 else self.run2_tree

    def _both_trees(self):
        return (self.run1_tree, self.run2_tree)

    def _on_point_count_choice(self):
        if self.point_count_var.get() == "Custom":
            self.custom_point_entry.pack(side="left", padx=(4, 0))
            self.custom_point_set_btn.pack(side="left", padx=4)
            self.custom_point_entry.focus_set()
            self.custom_point_entry.select_range(0, "end")
        else:
            self.custom_point_entry.pack_forget()
            self.custom_point_set_btn.pack_forget()
            self._apply_point_count()

    def _apply_point_count(self):
        choice = self.point_count_var.get()
        if choice == "Custom":
            try:
                count = int(self.custom_point_count_var.get())
            except ValueError:
                messagebox.showerror("Invalid point count", "Enter a whole number of points.")
                return
        else:
            count = int(choice)

        if count <= 0:
            messagebox.showerror("Invalid point count", "Point count must be greater than zero.")
            return

        current_count = len(self.capture_rows) if self.capture_rows else 0
        if current_count == 0:
            self._initialize_point_rows(count)
            return
        if count == current_count:
            return
        if count > current_count:
            self._resize_point_rows(count, keep_existing=True)
            return

        removed_rows = self.capture_rows[count:]
        losing_saved_values = any(self._point_has_data(row) for row in removed_rows)
        if losing_saved_values:
            if not messagebox.askyesno(
                "Remove points?",
                f"Changing from {current_count} to {count} points will remove saved values after point {count}. Continue?",
            ):
                if current_count in (11, 20):
                    self.point_count_var.set(str(current_count))
                else:
                    self.point_count_var.set("Custom")
                    self.custom_point_count_var.set(str(current_count))
                return

        self._resize_point_rows(count, keep_existing=True)

    def _blank_point_row(self, point_number: int) -> dict:
        return {"point": point_number, "run1": None, "run2": None}

    def _point_has_data(self, point_row: dict) -> bool:
        return bool(point_row and (point_row.get("run1") is not None or point_row.get("run2") is not None))

    def _resize_point_rows(self, count: int, keep_existing: bool = True):
        old_rows = list(self.capture_rows) if keep_existing else []
        new_rows = old_rows[:count]
        while len(new_rows) < count:
            new_rows.append(self._blank_point_row(len(new_rows) + 1))

        for i, row in enumerate(new_rows):
            row["point"] = i + 1

        self.capture_rows = new_rows
        self._redraw_point_table()
        self._regenerate_targets()

        next_target = self._first_empty_from(0, self.capture_target_run)
        if next_target is None:
            next_target = (min(max(0, len(old_rows) - 1), count - 1), self.capture_target_run)
        self._update_point_count_label()
        self._select_target(*next_target)

    def _initialize_point_rows(self, count: int):
        if not hasattr(self, "run1_tree"):
            return

        self.capture_rows = [self._blank_point_row(i + 1) for i in range(count)]
        self._redraw_point_table()
        self._regenerate_targets()
        self.capture_target_index = 0
        self.capture_target_run = 1
        self._update_point_count_label()
        self._select_target(0, 1)

    def _generate_targets(self, capacity: float, point_count: int) -> list[float]:
        """Build target force list from capacity and point count.

        11-point → 10%, 20%, … 100% of capacity, then 0   (11 rows)
        20-point → 1%, 2%, … 10%, 20%, 30%, … 100%, then 0 (20 rows)
        Other    → evenly spaced up to capacity, then 0
        """
        if capacity <= 0 or point_count <= 0:
            return []
        if point_count == 11:
            step = capacity / 10.0
            targets = [step * i for i in range(1, 11)]
        elif point_count == 20:
            fine_step = capacity / 100.0
            coarse_step = capacity / 10.0
            targets = [fine_step * i for i in range(1, 11)]
            targets += [coarse_step * i for i in range(2, 11)]
        else:
            step = capacity / point_count
            targets = [step * i for i in range(1, point_count + 1)]
        targets.append(0.0)
        return targets

    def _regenerate_targets(self):
        raw = self.customer_capacity_var.get().strip()
        if not raw:
            self.target_forces = []
            self.target_force_var.set("")
            return
        try:
            capacity = float(raw)
        except ValueError:
            self.target_forces = []
            self.target_force_var.set("")
            return
        count = len(self.capture_rows)
        self.target_forces = self._generate_targets(capacity, count)
        self._update_target_force_display()

    def _apply_customer_capacity(self):
        raw = self.customer_capacity_var.get().strip()
        if not raw:
            self.target_forces = []
            self.target_force_var.set("")
            return
        try:
            capacity = float(raw)
        except ValueError:
            return
        count = len(self.capture_rows)
        self.target_forces = self._generate_targets(capacity, count)
        needed = len(self.target_forces)
        if needed > count:
            self._resize_point_rows(needed, keep_existing=True)
            if needed in (11, 20):
                self.point_count_var.set(str(needed))
            else:
                self.point_count_var.set("Custom")
                self.custom_point_count_var.set(str(needed))
        self._update_target_force_display()

    def _update_target_force_display(self):
        idx = self.capture_target_index
        if not self.target_forces or idx is None or idx >= len(self.target_forces):
            self.target_force_var.set("")
            return
        val = self.target_forces[idx]
        units = self.hadi_units_var.get()
        if units == "mV/V":
            self.target_force_var.set(f"Target: {val:g} LBF")
        else:
            factor = HADI_UNIT_FACTORS.get(units, 1.0)
            converted = val * factor
            decimals = _decimals_from_step(self.hadi_decimals_var.get())
            self.target_force_var.set(f"Target: {converted:.{decimals}f} {units}")

    def _redraw_point_table(self):
        for tree in self._both_trees():
            for item in tree.get_children():
                tree.delete(item)

        for i in range(len(self.capture_rows)):
            self.run1_tree.insert("", "end", values=self._tree_values_for_run(i, 1))
            self.run2_tree.insert("", "end", values=self._tree_values_for_run(i, 2))

        self._refresh_row_tags()

    def _tree_values_for_run(self, index: int, run: int):
        point_row = self.capture_rows[index]
        run_row = point_row.get(f"run{run}")
        is_weight = bool(run_row and run_row.get("nominal_weight_lbf") not in (None, ""))
        point_label = f"{index + 1}w" if is_weight else str(index + 1)
        return (
            point_label,
            (run_row.get("hadi_text") or self._format_hadi_lbf_text(run_row.get("hadi_lbf"), show_plus=False))
            if run_row and run_row.get("hadi_lbf") is not None else "",
            self._ocr_text_for_row(run_row),
            self._format_run_percent(run_row),
        )

    @staticmethod
    def _format_run_value(run_row, key, fmt):
        if not run_row or run_row.get(key) is None:
            return ""
        try:
            return format(float(run_row.get(key)), fmt)
        except Exception:
            return ""

    @staticmethod
    def _format_run_percent(run_row):
        if not run_row:
            return ""
        if run_row.get("percent_error_na"):
            return "NA"
        if run_row.get("percent_error") is None:
            return ""
        try:
            return f"{float(run_row.get('percent_error')):+.2f}%"
        except Exception:
            return ""

    @staticmethod
    def _calculate_percent_error(ocr_value, hadi_value):
        """Return (percent_error, is_na).

        A zero OCR/reference value is still a valid capture value, but percent
        error cannot be computed because the denominator would be zero. Store it
        as NA instead of rejecting the row.
        """
        try:
            ocr = float(ocr_value)
            hadi = float(hadi_value)
        except Exception:
            return None, False
        if ocr == 0:
            return None, True
        return ((ocr - hadi) / ocr) * 100.0, False

    def _filled_count(self):
        return sum(
            1
            for point_row in self.capture_rows
            for run_key in ("run1", "run2")
            if point_row.get(run_key) is not None
        )

    def _update_point_count_label(self):
        total = len(self.capture_rows) * 2
        self.count_var.set(f"{self._filled_count()} / {total} runs")

    def _first_empty_from(self, start_index=0, start_run=1):
        total = len(self.capture_rows)
        if total == 0:
            return None

        run = 1 if start_run not in (1, 2) else start_run
        order = list(range(start_index, total)) + list(range(0, start_index))
        for i in order:
            if self.capture_rows[i].get(f"run{run}") is None:
                return (i, run)
        return None

    def _select_target(self, index: Optional[int], run: int = 1):
        if index is None or not self.capture_rows:
            self.capture_target_index = None
            self.capture_target_run = 1 if run not in (1, 2) else run
            self.target_var.set("Next: full")
            self._update_target_force_display()
            for tree in self._both_trees():
                tree.selection_remove(tree.selection())
            self._refresh_row_tags()
            return

        index = max(0, min(index, len(self.capture_rows) - 1))
        run = 1 if run not in (1, 2) else run
        self.capture_target_index = index
        self.capture_target_run = run

        active_tree = self._tree_for_run(run)
        inactive_tree = self._tree_for_run(2 if run == 1 else 1)

        item_id = active_tree.get_children()[index]
        inactive_tree.selection_remove(inactive_tree.selection())
        active_tree.selection_set(item_id)
        active_tree.focus(item_id)
        active_tree.see(item_id)
        inactive_tree.see(inactive_tree.get_children()[index])

        self.target_var.set(f"Next: P{index + 1} R{run}")
        self._update_target_force_display()
        self._refresh_row_tags()

    def _select_target_row(self, index: Optional[int]):
        if index is None:
            self._select_target(None, self.capture_target_run)
            return
        self._select_target(index, self.capture_target_run)

    def _refresh_row_tags(self):
        if not hasattr(self, "run1_tree"):
            return
        for run, tree in ((1, self.run1_tree), (2, self.run2_tree)):
            children = list(tree.get_children())
            for i, item_id in enumerate(children):
                point_row = self.capture_rows[i] if i < len(self.capture_rows) else None
                run_row = point_row.get(f"run{run}") if point_row else None
                tag = self._error_row_tag(run_row)

                if self.ocr_edit_entry is None:
                    tree.item(item_id, values=self._tree_values_for_run(i, run), tags=(tag,) if tag else ())
                else:
                    tree.item(item_id, tags=(tag,) if tag else ())

    def _advance_target_after(self, index, run):
        next_index = index + 1
        if next_index >= len(self.capture_rows):
            next_index = 0

        current_row = self.capture_rows[index].get(f"run{run}") if 0 <= index < len(self.capture_rows) else None
        if current_row and current_row.get("nominal_weight_lbf") not in (None, ""):
            next_weight = self._first_weight_needing_ocr_from(next_index, run)
            if next_weight is not None:
                self._select_target(*next_weight)
                return

        next_empty = self._first_empty_from(next_index, run)
        self._select_target(*(next_empty if next_empty is not None else (index, run)))

    def _on_run_tree_select(self, _event, run: int):
        if self.suppress_next_tree_select:
            self.suppress_next_tree_select = False
            return
        if self.ocr_edit_entry is not None:
            return

    def _on_run_tree_click(self, event, run: int):
        if self.ocr_edit_entry is not None:
            return "break"
        if self.suppress_next_tree_click:
            self.suppress_next_tree_click = False
            return "break"

        tree = self._tree_for_run(run)
        item_id = tree.identify_row(event.y)
        if not item_id:
            return

        children = list(tree.get_children())
        try:
            row_index = children.index(item_id)
        except ValueError:
            return

        # For normal clicks, update target. For Ctrl/Shift multi-select, let
        # Treeview manage selection and only update the active run/target label.
        self.capture_target_index = row_index
        self.capture_target_run = run
        self.target_var.set(f"Next: P{row_index + 1} R{run}")
        self._tree_for_run(2 if run == 1 else 1).selection_remove(self._tree_for_run(2 if run == 1 else 1).selection())

        if not (event.state & 0x0001 or event.state & 0x0004):
            self._select_target(row_index, run)
        else:
            self._refresh_row_tags()

        self.after(1, self.focus_set)

    def _on_run_tree_double_click(self, event, run: int):
        tree = self._tree_for_run(run)
        region = tree.identify("region", event.x, event.y)
        if region != "cell":
            return "break"

        item_id = tree.identify_row(event.y)
        col_id = tree.identify_column(event.x)

        # Columns in each run tree are #1 Point, #2 HADI/weight, #3 OCR, #4 %.
        if not item_id or col_id not in ("#2", "#3"):
            return "break"

        children = list(tree.get_children())
        try:
            row_index = children.index(item_id)
        except ValueError:
            return "break"

        # HADI/weight can be edited even when the run is empty.
        # OCR can only be edited after the run exists.
        if col_id == "#3" and (row_index >= len(self.capture_rows) or self.capture_rows[row_index].get(f"run{run}") is None):
            return "break"

        self.suppress_next_tree_click = True
        self.suppress_next_tree_select = True
        kind = "hadi" if col_id == "#2" else "ocr"
        self._start_cell_edit(tree, item_id, row_index, run, kind)
        return "break"

    def _start_cell_edit(self, tree, item_id, row_index: int, run: int, kind: str):
        if self.ocr_edit_entry is not None:
            self._cancel_ocr_edit()

        column_id = "#2" if kind == "hadi" else "#3"
        bbox = tree.bbox(item_id, column_id)
        if not bbox:
            return

        x, y, w, h = bbox
        run_row = self.capture_rows[row_index].get(f"run{run}") if row_index < len(self.capture_rows) else None

        current_value = ""
        if run_row:
            if kind == "hadi":
                # Prefer nominal weight if this row came from a manual/weight reference.
                if run_row.get("nominal_weight_lbf") not in (None, ""):
                    current_value = run_row.get("nominal_weight_lbf")
                else:
                    current_value = run_row.get("hadi_lbf", "")
            else:
                current_value = run_row.get("ocr_text", run_row.get("ocr", ""))

        entry = ttk.Entry(tree)
        if current_value != "":
            try:
                if kind == "hadi":
                    entry.insert(0, f"{float(current_value):g}")
                else:
                    entry.insert(0, self._format_ocr_text(current_value, run_row.get("ocr"), show_plus=False))
            except Exception:
                entry.insert(0, str(current_value).replace("+", ""))
        entry.select_range(0, "end")
        entry.place(x=x, y=y, width=w, height=h)
        entry.focus_force()

        self.ocr_edit_entry = entry
        self.ocr_edit_tree = tree
        self.ocr_edit_item = item_id
        self.ocr_edit_row_index = row_index
        self.ocr_edit_run = run
        self.edit_cell_kind = kind

        entry.bind("<Return>", self._commit_ocr_edit)
        entry.bind("<KP_Enter>", self._commit_ocr_edit)
        entry.bind("<Escape>", self._cancel_ocr_edit)
        entry.bind("<FocusOut>", self._commit_ocr_edit)

    def _start_ocr_cell_edit(self, tree, item_id, row_index: int, run: int):
        self._start_cell_edit(tree, item_id, row_index, run, "ocr")

    def _cancel_ocr_edit(self, _event=None):
        if self.ocr_edit_entry is not None:
            try:
                self.ocr_edit_entry.destroy()
            except Exception:
                pass
        self.ocr_edit_entry = None
        self.ocr_edit_tree = None
        self.ocr_edit_item = None
        self.ocr_edit_row_index = None
        self.ocr_edit_run = None
        self.edit_cell_kind = None
        self.suppress_next_tree_select = True
        self.suppress_next_tree_click = True
        return "break"

    def _commit_ocr_edit(self, _event=None):
        if self.ocr_edit_entry is None:
            return "break"

        # Excel-style entry only for HADI/weight cells:
        # pressing Enter commits and opens the next HADI cell down.
        advance_hadi_entry = (
            _event is not None
            and getattr(_event, "keysym", "") in ("Return", "KP_Enter")
            and self.edit_cell_kind == "hadi"
        )

        entry = self.ocr_edit_entry
        tree = self.ocr_edit_tree
        item_id = self.ocr_edit_item
        row_index = self.ocr_edit_row_index
        run = self.ocr_edit_run
        kind = self.edit_cell_kind or "ocr"
        raw = entry.get().strip().replace("+", "").replace("%", "")

        try:
            entry.destroy()
        except Exception:
            pass
        self.ocr_edit_entry = None
        self.ocr_edit_tree = None
        self.ocr_edit_item = None
        self.ocr_edit_row_index = None
        self.ocr_edit_run = None
        self.edit_cell_kind = None

        try:
            new_value = float(raw)
        except ValueError:
            label = "weight" if kind == "hadi" else "OCR value"
            messagebox.showerror("Invalid value", f"Enter a numeric {label}.")
            return "break"

        if row_index is None or run not in (1, 2) or not (0 <= row_index < len(self.capture_rows)):
            return "break"

        if kind == "hadi":
            existing = self.capture_rows[row_index].get(f"run{run}") or {}
            was_weight = existing.get("nominal_weight_lbf") not in (None, "")

            if was_weight:
                # Editing a W row means "change the nominal weight", not
                # "turn W off". Keep the W marker, apply GPS gravity again,
                # preserve OCR if it already exists, and recalc % error.
                existing_ocr = existing.get("ocr")
                row = self._make_manual_weight_row(
                    new_value,
                    ocr_value=existing_ocr if existing_ocr not in ("", None) else None,
                    ocr_text=existing.get("ocr_text", ""),
                )
                row["point"] = row_index + 1
                row["run"] = run
                row["pre_w_hadi_lbf"] = new_value
                row["ocr_edited"] = existing.get("ocr_edited", False)
                self.capture_rows[row_index][f"run{run}"] = row
                pass
            else:
                existing.update({
                    "point": row_index + 1,
                    "run": run,
                    "time": datetime.now().isoformat(timespec="milliseconds"),
                    "load_cell": self.selected_load_cell_name.get(),
                    "mode": self.mode_var.get(),
                    "method": "manual_hadi_value",
                    "sync_mode": "manual_entry",
                    "sync_lag_ms": "",
                    "sync_confidence": "",
                    "target_delay_ms": "",
                    "hadi_raw": "",
                    "hadi_lbf": new_value,
                    "captured_hadi_lbf": "",
                    "conventional_lbf_from_hadi": "",
                    "nominal_weight_lbf": "",
                    "gravity_factor": "",
                    "gravity_m_s2": "",
                    "gps_latitude": "",
                    "gps_longitude": "",
                })
                self._set_row_hadi_text(existing)
                existing.setdefault("ocr", "")
                existing.setdefault("ocr_text", "")
                existing.setdefault("ocr_edited", False)
                self._recalculate_row_error(existing)
                self.capture_rows[row_index][f"run{run}"] = existing
                pass
        else:
            run_row = self.capture_rows[row_index].get(f"run{run}")
            if run_row is None:
                return "break"

            run_row["ocr"] = new_value
            run_row["ocr_text"] = raw
            run_row["ocr_edited"] = True

            try:
                hadi_value = float(run_row.get("hadi_lbf"))
            except Exception:
                hadi_value = None

            if hadi_value is not None:
                err, is_na = self._calculate_percent_error(new_value, hadi_value)
                run_row["percent_error"] = err
                run_row["percent_error_na"] = is_na
            else:
                run_row["percent_error"] = None
                run_row["percent_error_na"] = False

        if tree is not None and item_id:
            tree.item(item_id, values=self._tree_values_for_run(row_index, run))

        active_run = self.capture_target_run if self.capture_target_run in (1, 2) else run

        if advance_hadi_entry:
            # Go to the next physical row in the same run, even if it already has
            # a value. This is for entering a column of weights quickly.
            next_index = row_index + 1
            if next_index < len(self.capture_rows):
                self.suppress_next_tree_select = True
                self.suppress_next_tree_click = True
                self._select_target(next_index, run)
                self._update_point_count_label()

                next_tree = self._tree_for_run(run)
                next_item = next_tree.get_children()[next_index]
                self.after(35, lambda: self._start_cell_edit(next_tree, next_item, next_index, run, "hadi"))
                return "break"

        next_empty = self._first_empty_from(row_index + 1, active_run)
        if next_empty is None:
            next_empty = self._first_empty_from(0, active_run)

        self.suppress_next_tree_select = True
        self.suppress_next_tree_click = True
        self._select_target(*(next_empty if next_empty is not None else (row_index, active_run)))
        self._update_point_count_label()
        self._mark_data_changed()
        self.focus_set()
        return "break"

    def _on_capture_row_select(self, _event=None):
        return

    def _clear_override_selection(self, update_tree=True):
        run = self.capture_target_run if self.capture_target_run in (1, 2) else 1
        next_empty = self._first_empty_from(0, run)
        self._select_target(*(next_empty if next_empty is not None else (None, run)))

    def _error_row_tag(self, row: dict) -> str:
        if row is None:
            return ""
        try:
            err = abs(float(row.get("percent_error", 0)))
        except Exception:
            return ""
        if err > 1.0:
            return "bad"
        if err > 0.5:
            return "warn"
        return ""

    def _has_capture_data(self):
        return any(self._point_has_data(row) for row in self.capture_rows)

    @staticmethod
    def _safe_filename_part(value) -> str:
        text = str(value or "").strip().lower()
        text = re.sub(r"[^a-z0-9]+", "_", text)
        text = re.sub(r"_+", "_", text).strip("_")
        return text or "run"

    def _runs_with_data(self):
        has_run1 = any((row.get("run1") is not None) for row in self.capture_rows)
        has_run2 = any((row.get("run2") is not None) for row in self.capture_rows)
        runs = []
        if has_run1:
            runs.append("run1")
        if has_run2:
            runs.append("run2")
        return runs or ["run"]

    def _mode_for_filename(self):
        # Prefer the mode stored with the captured data; fall back to the current mode selector.
        for point_row in self.capture_rows:
            for key in ("run1", "run2"):
                run_row = point_row.get(key)
                if run_row and run_row.get("mode"):
                    return self._safe_filename_part(run_row.get("mode"))
        return self._safe_filename_part(self.mode_var.get())

    def _default_csv_filename(self, autosave=False):
        mode = self._mode_for_filename()
        runs = "_".join(self._runs_with_data())
        stamp = datetime.now().strftime("%Y%m%d_%H%M")
        suffix = "SN"  # Leave this at the end so you can quickly type the serial number.
        prefix = "autosave_" if autosave else ""
        return f"{prefix}{mode}_{runs}_{stamp}_{suffix}.csv"

    def _capture_csv_fields(self):
        return ["OCR", "HADI", "% Error"]

    @staticmethod
    def _csv_fmt(value, decimals=4, suffix=""):
        if value in (None, ""):
            return ""
        try:
            txt = f"{float(value):.{decimals}f}"
        except Exception:
            txt = str(value)
        return f"{txt}{suffix}"

    @staticmethod
    def _csv_fmt_error(value, is_na=False):
        if is_na:
            return "NA"
        if value in (None, ""):
            return ""
        try:
            return f"{float(value):.2f}%"
        except Exception:
            return str(value)

    def _run_has_data(self, run_key: str) -> bool:
        return any((row.get(run_key) is not None) for row in self.capture_rows)

    def _capture_csv_rows_for_run(self, run_key: str):
        rows = []
        for point_row in self.capture_rows:
            run_row = point_row.get(run_key)
            if not run_row:
                continue
            rows.append([
                self._ocr_text_for_row(run_row, show_plus=False),
                run_row.get("hadi_text") or self._format_hadi_lbf_text(run_row.get("hadi_lbf"), show_plus=False),
                self._csv_fmt_error(run_row.get("percent_error"), run_row.get("percent_error_na", False)),
            ])
        return rows

    def _write_capture_csv(self, path) -> int:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)

        total_rows = 0
        with path.open("w", newline="") as f:
            writer = csv.writer(f)

            if self._run_has_data("run1"):
                writer.writerow(["Run 1 (HADI: LBF)"])
                writer.writerow(self._capture_csv_fields())
                run_rows = self._capture_csv_rows_for_run("run1")
                writer.writerows(run_rows)
                total_rows += len(run_rows)

            if self._run_has_data("run2"):
                if total_rows:
                    writer.writerow([])
                writer.writerow(["Run 2 (HADI: LBF)"])
                writer.writerow(self._capture_csv_fields())
                run_rows = self._capture_csv_rows_for_run("run2")
                writer.writerows(run_rows)
                total_rows += len(run_rows)

        return total_rows

    def _autosave_captures(self, quiet=True):
        if not self._has_capture_data():
            return None
        if not self.autosave_name:
            self.autosave_name = self._default_csv_filename(autosave=True)
        path = AUTOSAVE_DIR / self.autosave_name
        try:
            self._write_capture_csv(path)
            self.last_autosave_path = path
            if not quiet:
                messagebox.showinfo("Autosaved", f"Autosaved backup to:\n{path}")
            return path
        except Exception as exc:
            if not quiet:
                messagebox.showerror("Autosave failed", str(exc))
            return None

    def _mark_data_changed(self):
        self.dirty_data = True
        self._autosave_captures(quiet=True)

    def _target_weight_row(self):
        index = self.capture_target_index
        run = self.capture_target_run
        if index is None or run not in (1, 2) or not (0 <= index < len(self.capture_rows)):
            return None, None, None
        row = self.capture_rows[index].get(f"run{run}")
        if row and row.get("nominal_weight_lbf") not in (None, ""):
            return index, run, row
        return index, run, None

    def _row_needs_ocr(self, run_row: dict) -> bool:
        return bool(run_row and run_row.get("ocr") in (None, ""))

    def _first_weight_needing_ocr_from(self, start_index=0, run=1):
        total = len(self.capture_rows)
        if total == 0:
            return None
        order = list(range(start_index, total)) + list(range(0, start_index))
        for i in order:
            row = self.capture_rows[i].get(f"run{run}")
            if row and row.get("nominal_weight_lbf") not in (None, "") and self._row_needs_ocr(row):
                return (i, run)
        return None

    def _current_exact_ocr_value(self):
        now_pc = time.perf_counter()
        self._prune_buffers(now_pc)
        if not self._ocr_is_fresh(now_pc):
            raise RuntimeError("Waiting for fresh OCR packets.")

        with self.ocr_lock:
            ocr_samples = [s for s in self.ocr_buffer if (now_pc - s.pc_time) <= self.ocr_wait_seconds]
            latest_ocr = self.latest_ocr

        if latest_ocr is not None and (now_pc - latest_ocr.pc_time) <= self.ocr_wait_seconds:
            return now_pc, time.time(), latest_ocr.value, latest_ocr.raw_text

        if ocr_samples:
            sample = ocr_samples[-1]
            return now_pc, time.time(), sample.value, sample.raw_text

        raise RuntimeError("Need at least one fresh OCR reading.")

    def _capture_into_weight_row(self, index: int, run: int, run_row: dict):
        try:
            _now_pc, now_wall, ocr_value, ocr_text = self._current_exact_ocr_value()
        except RuntimeError as exc:
            messagebox.showwarning("No OCR value", str(exc))
            return False

        try:
            hadi_lbf = float(run_row.get("hadi_lbf"))
        except Exception:
            messagebox.showwarning("Missing weight", "This W row does not have a HADI/weight value.")
            return False

        percent_error, percent_error_na = self._calculate_percent_error(ocr_value, hadi_lbf)

        run_row["time"] = datetime.fromtimestamp(now_wall).isoformat(timespec="milliseconds")
        run_row["load_cell"] = self.selected_load_cell_name.get()
        run_row["mode"] = self.mode_var.get()
        run_row["method"] = "weight_reference_ocr_only"
        run_row["sync_mode"] = "ocr_only"
        run_row["sync_lag_ms"] = ""
        run_row["sync_confidence"] = ""
        run_row["target_delay_ms"] = ""
        run_row["ocr"] = ocr_value
        run_row["ocr_text"] = ocr_text
        run_row["ocr_edited"] = False
        run_row["percent_error"] = percent_error
        run_row["percent_error_na"] = percent_error_na
        self._set_row_hadi_text(run_row)

        tree = self._tree_for_run(run)
        item_id = tree.get_children()[index]
        tree.item(item_id, values=self._tree_values_for_run(index, run))
        self._refresh_row_tags()
        self._mark_data_changed()
        self._update_point_count_label()

        # When filling W rows, stay in the W sequence and move to the next W row
        # that still needs OCR. Do not jump to the bottom/first empty row.
        next_target = self._first_weight_needing_ocr_from(index + 1, run)
        if next_target is None:
            next_index = index + 1
            if next_index < len(self.capture_rows):
                self._select_target(next_index, run)
            else:
                self._select_target(index, run)
        else:
            self._select_target(*next_target)
        return True

    def _insert_or_replace_capture_row(self, row: dict, values: tuple):
        if not self.capture_rows:
            self._initialize_point_rows(11)

        index = self.capture_target_index
        run = self.capture_target_run
        if index is None or not (0 <= index < len(self.capture_rows)) or run not in (1, 2):
            target = self._first_empty_from(0, 1)
            if target is None:
                index, run = len(self.capture_rows) - 1, 1
            else:
                index, run = target

        row["point"] = index + 1
        row["run"] = run
        self.capture_rows[index][f"run{run}"] = row

        tree = self._tree_for_run(run)
        item_id = tree.get_children()[index]
        tree.item(item_id, values=self._tree_values_for_run(index, run))
        self._refresh_row_tags()
        self._mark_data_changed()
        self._advance_target_after(index, run)

    def _set_live_error_color(self, err):
        if not hasattr(self, "live_error_label"):
            return
        if err is None:
            self.live_error_label.configure(foreground="")
        elif abs(err) <= 0.5:
            self.live_error_label.configure(foreground="#198754")
        elif abs(err) <= 1.0:
            self.live_error_label.configure(foreground="#b58100")
        else:
            self.live_error_label.configure(foreground="#c1121f")

    def _update_live_percent_error(self):
        now_pc = time.perf_counter()
        if not self._hadi_is_fresh(now_pc) or not self._ocr_is_fresh(now_pc):
            self.live_error_var.set("--")
            self._set_live_error_color(None)
            return

        try:
            hadi = self.latest_hadi.force_lbf if self.latest_hadi else None
            with self.ocr_lock:
                ocr = self.latest_ocr.value if self.latest_ocr else None
            if hadi is None or ocr is None:
                raise ValueError
            err, is_na = self._calculate_percent_error(ocr, hadi)
            if is_na:
                self.live_error_var.set("NA")
                self._set_live_error_color(None)
                return
            self.live_error_var.set(f"{err:+.2f}%")
            self._set_live_error_color(err)
        except Exception:
            self.live_error_var.set("--")
            self._set_live_error_color(None)

    def _current_synced_values(self):
        now_pc = time.perf_counter()
        self._prune_buffers(now_pc)

        if not self._hadi_is_fresh(now_pc):
            raise RuntimeError("Waiting for fresh HADI readings.")
        if not self._ocr_is_fresh(now_pc):
            raise RuntimeError("Waiting for fresh OCR packets.")

        hadi_samples = [s for s in self.hadi_buffer if (now_pc - s.pc_time) <= self.hadi_wait_seconds]
        with self.ocr_lock:
            ocr_samples = [s for s in self.ocr_buffer if (now_pc - s.pc_time) <= self.ocr_wait_seconds]

        if len(hadi_samples) < 2:
            raise RuntimeError("Need at least two fresh HADI readings.")
        if len(ocr_samples) < 2:
            raise RuntimeError("Need at least two fresh OCR readings.")

        latest_hadi_t = hadi_samples[-1].pc_time
        latest_ocr_corrected_t = self._corrected_ocr_time(ocr_samples[-1])
        target_t = min(now_pc, latest_hadi_t, latest_ocr_corrected_t)

        hadi_lbf = self._interp(hadi_samples, target_t, "force_lbf")
        hadi_raw = self._interp(hadi_samples, target_t, "raw_response")

        # OCR is a screen reading, so use the nearest actual OCR packet instead
        # of interpolating and inventing decimal values that were never displayed.
        ocr_sample = self._nearest_sample(ocr_samples, target_t, corrected_ocr=True)
        ocr_value = None if ocr_sample is None else ocr_sample.value
        ocr_text = "" if ocr_sample is None else ocr_sample.raw_text

        if hadi_lbf is None or ocr_value is None:
            raise RuntimeError("Could not match fresh HADI/OCR readings at the same corrected instant.")
        return now_pc, time.time(), target_t, hadi_lbf, hadi_raw, ocr_value, ocr_text

    def _gravity_context(self):
        gps = getattr(self, "locked_gps", None) if USE_GPS_GRAVITY_CORRECTION else None
        if gps is None:
            return {
                "gravity_factor": 1.0,
                "gravity_m_s2": STANDARD_GRAVITY,
                "gps_latitude": "",
                "gps_longitude": "",
                "gps_altitude_m": "",
                "note": "standard gravity; GPS not required",
                "using_gps": False,
            }
        return {
            "gravity_factor": gps.gravity_factor,
            "gravity_m_s2": gps.gravity_m_s2,
            "gps_latitude": gps.latitude,
            "gps_longitude": gps.longitude,
            "gps_altitude_m": gps.altitude_m if gps.altitude_m is not None else "",
            "note": (
                f"ASTM MF {gps.gravity_factor:.7f}, alt {gps.altitude_m:.1f}m"
                if gps.altitude_m is not None
                else f"ASTM MF {gps.gravity_factor:.7f}, alt 0m"
            ),
            "using_gps": True,
        }

    def _current_target_ocr_value(self):
        index = self.capture_target_index
        run = self.capture_target_run
        if index is None or run not in (1, 2) or not (0 <= index < len(self.capture_rows)):
            return None
        run_row = self.capture_rows[index].get(f"run{run}")
        if not run_row:
            return None
        return run_row.get("ocr")

    def _make_manual_weight_row(self, nominal_weight_lbf: float, ocr_value=None, ocr_text="") -> dict:
        now_wall = time.time()
        ctx = self._gravity_context()
        gravity_factor = ctx["gravity_factor"]
        reference_lbf = nominal_weight_lbf * gravity_factor

        percent_error, percent_error_na = self._calculate_percent_error(ocr_value, reference_lbf)

        row = {
            "point": (self.capture_target_index + 1) if self.capture_target_index is not None else "",
            "time": datetime.fromtimestamp(now_wall).isoformat(timespec="milliseconds"),
            "load_cell": self.selected_load_cell_name.get(),
            "mode": self.mode_var.get(),
            "method": "manual_weight_gps_gravity" if ctx.get("using_gps") else "manual_weight_standard_gravity",
            "sync_mode": "manual_entry",
            "sync_lag_ms": "",
            "sync_confidence": "",
            "target_delay_ms": "",
            "hadi_raw": "",
            "hadi_lbf": reference_lbf,
            "ocr": ocr_value if ocr_value is not None else "",
            "ocr_text": ocr_text if ocr_value is not None else "",
            "ocr_edited": False,
            "percent_error": percent_error,
            "percent_error_na": percent_error_na,
            "captured_hadi_lbf": "",
            "conventional_lbf_from_hadi": "",
            "nominal_weight_lbf": nominal_weight_lbf,
            "gravity_factor": gravity_factor,
            "gravity_m_s2": ctx["gravity_m_s2"],
            "gps_latitude": ctx["gps_latitude"],
            "gps_longitude": ctx["gps_longitude"],
            "gps_altitude_m": ctx.get("gps_altitude_m", ""),
        }
        self._set_row_hadi_text(row)
        return row

    def _selected_indexes_for_active_run(self):
        run = self.capture_target_run if self.capture_target_run in (1, 2) else 1
        tree = self._tree_for_run(run)
        children = list(tree.get_children())
        selected = list(tree.selection())
        indexes = []
        for item_id in selected:
            try:
                indexes.append(children.index(item_id))
            except ValueError:
                pass
        indexes = sorted(set(indexes))
        if indexes:
            return run, indexes
        if self.capture_target_index is not None:
            return run, [self.capture_target_index]
        return run, []

    def _parse_weight_values(self):
        raw = self.manual_weight_var.get().strip().replace("+", "")
        if not raw:
            raise ValueError("Enter a weight in lb, like 10 or 0.1.")
        parts = [p for p in re.split(r"[,;\\s]+", raw) if p]
        values = []
        for p in parts:
            values.append(float(p))
        return values

    def _recalculate_row_error(self, row: dict):
        err, is_na = self._calculate_percent_error(row.get("ocr"), row.get("hadi_lbf"))
        row["percent_error"] = err
        row["percent_error_na"] = is_na

    def _nominal_weight_from_existing_row(self, index: int, run: int):
        existing = self.capture_rows[index].get(f"run{run}")
        if not existing:
            return None

        # If this is already a W row, preserve the stored nominal weight.
        if existing.get("nominal_weight_lbf") not in (None, ""):
            try:
                return float(existing.get("nominal_weight_lbf"))
            except Exception:
                pass

        # If the user double-clicked the HADI cell and typed 10, treat that as
        # the nominal 10 lb weight. Do NOT back-calculate it by gravity here.
        try:
            return float(existing.get("hadi_lbf"))
        except Exception:
            return None

    def _apply_manual_weight_to_index(self, index: int, run: int, nominal_weight_lbf: float):
        existing = self.capture_rows[index].get(f"run{run}") or {}
        ocr_value = existing.get("ocr")
        original_hadi = existing.get("pre_w_hadi_lbf")
        if original_hadi in (None, ""):
            original_hadi = existing.get("hadi_lbf", nominal_weight_lbf)
        row = self._make_manual_weight_row(nominal_weight_lbf, ocr_value=ocr_value if ocr_value not in ("", None) else None, ocr_text=existing.get("ocr_text", ""))
        row["point"] = index + 1
        row["run"] = run
        row["pre_w_hadi_lbf"] = original_hadi
        self.capture_rows[index][f"run{run}"] = row

        tree = self._tree_for_run(run)
        item_id = tree.get_children()[index]
        tree.item(item_id, values=self._tree_values_for_run(index, run))
        return row

    def _set_selected_manual_weight(self):
        run, indexes = self._selected_indexes_for_active_run()
        if not indexes:
            messagebox.showwarning("Select rows", "Select one or more rows in Run 1 or Run 2.")
            return

        toggled_on = 0
        toggled_off = 0
        missing = []

        for index in indexes:
            existing = self.capture_rows[index].get(f"run{run}")
            if not existing:
                missing.append(index + 1)
                continue

            # Toggle OFF if already marked as a weight row.
            if existing.get("nominal_weight_lbf") not in (None, ""):
                restore_value = existing.get("pre_w_hadi_lbf", existing.get("nominal_weight_lbf", existing.get("hadi_lbf")))
                existing["method"] = "manual_hadi_value"
                existing["hadi_lbf"] = restore_value
                existing["nominal_weight_lbf"] = ""
                existing["gravity_factor"] = ""
                existing["gravity_m_s2"] = ""
                existing["gps_latitude"] = ""
                existing["gps_longitude"] = ""
                existing["pre_w_hadi_lbf"] = restore_value
                self._set_row_hadi_text(existing)
                self._recalculate_row_error(existing)
                tree = self._tree_for_run(run)
                item_id = tree.get_children()[index]
                tree.item(item_id, values=self._tree_values_for_run(index, run))
                toggled_off += 1
            else:
                # Toggle ON: use the current HADI/local lbf value as the typed value.
                # GPS correction is disabled by default, so this stays usable with no GPS.
                nominal_weight_lbf = self._nominal_weight_from_existing_row(index, run)
                if nominal_weight_lbf is None:
                    missing.append(index + 1)
                    continue

                row = self._apply_manual_weight_to_index(index, run, nominal_weight_lbf)
                self._recalculate_row_error(row)
                toggled_on += 1

        self._refresh_row_tags()
        self._update_point_count_label()

        if missing:
            messagebox.showwarning(
                "Missing HADI values",
                "These selected rows do not have a HADI/weight value yet: "
                + ", ".join(str(x) for x in missing)
                + ". Double-click the HADI cell and enter the value first.",
            )

        if indexes:
            # After marking rows as W, keep the next capture at the first selected
            # W row that needs OCR, instead of jumping down to the first empty row.
            next_weight = self._first_weight_needing_ocr_from(indexes[0], run)
            if next_weight is not None:
                self._select_target(*next_weight)
            else:
                next_start = indexes[-1] + 1
                next_empty = self._first_empty_from(next_start if next_start < len(self.capture_rows) else 0, run)
                self._select_target(*(next_empty if next_empty is not None else (indexes[-1], run)))

        if toggled_on or toggled_off:
            self._mark_data_changed()
            pass

    def _set_selected_weight_reference(self):
        try:
            now_pc, now_wall, target_t, hadi_lbf, hadi_raw, ocr_value, ocr_text = self._current_synced_values()
        except RuntimeError as exc:
            messagebox.showwarning("No synced values", str(exc))
            return

        ctx = self._gravity_context()
        gravity_factor = ctx["gravity_factor"]
        gravity_m_s2 = ctx["gravity_m_s2"]
        lat = ctx["gps_latitude"]
        lon = ctx["gps_longitude"]
        gps_note = ctx["note"]

        conventional_lbf = hadi_lbf / gravity_factor if gravity_factor else hadi_lbf
        nominal_weight_lbf = nearest_standard_weight_lbf(conventional_lbf)
        reference_lbf = nominal_weight_lbf * gravity_factor

        percent_error, percent_error_na = self._calculate_percent_error(ocr_value, reference_lbf)
        target_delay_ms = (now_pc - target_t) * 1000.0

        row = {
            "point": (self.capture_target_index + 1) if self.capture_target_index is not None else "",
            "time": datetime.fromtimestamp(now_wall).isoformat(timespec="milliseconds"),
            "load_cell": self.selected_load_cell_name.get(),
            "mode": self.mode_var.get(),
            "method": "weight_reference_gps_gravity" if ctx.get("using_gps") else "weight_reference_standard_gravity",
            "sync_mode": "manual" if self.sync_manual else "auto",
            "sync_lag_ms": self.sync_lag_seconds * 1000.0,
            "sync_confidence": self.sync_confidence,
            "target_delay_ms": target_delay_ms,
            "hadi_raw": hadi_raw,
            "hadi_lbf": reference_lbf,
            "ocr": ocr_value,
            "ocr_text": ocr_text,
            "ocr_edited": False,
            "percent_error": percent_error,
            "percent_error_na": percent_error_na,
            "captured_hadi_lbf": hadi_lbf,
            "conventional_lbf_from_hadi": conventional_lbf,
            "nominal_weight_lbf": nominal_weight_lbf,
            "gravity_factor": gravity_factor,
            "gravity_m_s2": gravity_m_s2,
            "gps_latitude": lat,
            "gps_longitude": lon,
            "gps_altitude_m": ctx.get("gps_altitude_m", ""),
        }
        self._set_row_hadi_text(row)

        self._insert_or_replace_capture_row(
            row,
            (
                row.get("hadi_text", self._format_hadi_lbf_text(reference_lbf)),
                self._format_ocr_text(ocr_text, ocr_value),
                "NA" if percent_error_na else f"{percent_error:+.2f}%",
            ),
        )
        self._update_point_count_label()
        pass

    def _capture_from_key(self, _event=None):
        if self.ocr_edit_entry is not None:
            return "break"
        self.focus_set()
        self._capture()
        return "break"

    def _capture(self):
        target_index, target_run, weight_row = self._target_weight_row()
        if weight_row is not None:
            # W rows are pre-filled weight references. They only need OCR.
            # Do not require HADI, and do not overwrite the HADI/weight value.
            self._capture_into_weight_row(target_index, target_run, weight_row)
            return

        try:
            now_pc, now_wall, target_t, hadi_lbf, hadi_raw, ocr_value, ocr_text = self._current_synced_values()
        except RuntimeError as exc:
            messagebox.showwarning("No synced values", str(exc))
            return

        percent_error, percent_error_na = self._calculate_percent_error(ocr_value, hadi_lbf)
        target_delay_ms = (now_pc - target_t) * 1000.0

        row = {
            "point": (self.capture_target_index + 1) if self.capture_target_index is not None else "",
            "time": datetime.fromtimestamp(now_wall).isoformat(timespec="milliseconds"),
            "load_cell": self.selected_load_cell_name.get(),
            "mode": self.mode_var.get(),
            "method": "lag_corrected_instant_interpolated",
            "sync_mode": "manual" if self.sync_manual else "auto",
            "sync_lag_ms": self.sync_lag_seconds * 1000.0,
            "sync_confidence": self.sync_confidence,
            "target_delay_ms": target_delay_ms,
            "hadi_raw": hadi_raw,
            "hadi_lbf": hadi_lbf,
            "ocr": ocr_value,
            "ocr_text": ocr_text,
            "ocr_edited": False,
            "percent_error": percent_error,
            "percent_error_na": percent_error_na,
            "captured_hadi_lbf": hadi_lbf,
            "conventional_lbf_from_hadi": "",
            "nominal_weight_lbf": "",
            "gravity_factor": "",
            "gravity_m_s2": "",
            "gps_latitude": "",
            "gps_longitude": "",
            "gps_altitude_m": "",
        }
        self._set_row_hadi_text(row)

        self._insert_or_replace_capture_row(
            row,
            (
                row.get("hadi_text", self._format_hadi_lbf_text(hadi_lbf)),
                self._format_ocr_text(ocr_text, ocr_value),
                "NA" if percent_error_na else f"{percent_error:+.2f}%",
            ),
        )
        self._update_point_count_label()

    def _save_csv(self):
        if not self._has_capture_data():
            messagebox.showinfo("Nothing to save", "No captures yet.")
            return False
        default = self._default_csv_filename(autosave=False)
        path = filedialog.asksaveasfilename(
            title="Save captures",
            defaultextension=".csv",
            initialfile=default,
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return False

        try:
            row_count = self._write_capture_csv(path)
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))
            return False

        self.last_manual_save_path = Path(path)
        self.dirty_data = False
        messagebox.showinfo("Saved", f"Saved {row_count} rows to:\n{path}")
        return True

    def _clear_captures(self):
        if self._has_capture_data():
            self._autosave_captures(quiet=True)
            if not messagebox.askyesno(
                "Clear capture table?",
                "This will clear all captured points in the table. A backup autosave is kept in the autosaves folder. Continue?",
            ):
                return

        count = len(self.capture_rows) if self.capture_rows else 10
        self._initialize_point_rows(count)
        self.dirty_data = False
        self.autosave_name = None

    def _on_close(self):
        if self._has_capture_data() and self.dirty_data:
            autosave_path = self._autosave_captures(quiet=True)
            msg = "You have capture data that has not been manually saved to CSV.\n\n"
            if autosave_path:
                msg += f"A backup was autosaved here:\n{autosave_path}\n\n"
            msg += "Save to CSV before exiting?"
            choice = messagebox.askyesnocancel("Unsaved capture data", msg)
            if choice is None:
                return
            if choice is True:
                if not self._save_csv():
                    return

        self.hadi.disconnect()
        self._stop_ocr()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
