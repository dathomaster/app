
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
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from ocr_stream import OCRReceiver

try:
    from webcam_ocr import WebcamOCR
except ImportError:
    WebcamOCR = None

try:
    import cv2 as _cv2
    from PIL import Image, ImageTk
    _HAS_WEBCAM_DEPS = True
except ImportError:
    _HAS_WEBCAM_DEPS = False

# The one place the version lives: window title, build name and installer
# (packaging/build.ps1 reads it from here).
APP_VERSION = "1.6"

if getattr(sys, "frozen", False):
    # Running as a PyInstaller-frozen exe: __file__ resolves to the
    # temporary extraction directory (sys._MEIPASS), which is wiped after
    # each run, so persistent files (load cells, sync state, autosaves)
    # must live next to the actual .exe instead. Bundled read-only assets
    # (icons etc.) still come from the extraction directory.
    APP_DIR = Path(sys.executable).resolve().parent
    RESOURCE_DIR = Path(getattr(sys, "_MEIPASS", APP_DIR))
else:
    APP_DIR = Path(__file__).resolve().parent
    RESOURCE_DIR = APP_DIR
LOAD_CELLS_FILE = APP_DIR / "load_cells.json"
SYNC_STATE_FILE = APP_DIR / "sync_state.json"
CAMERA_PREFS_FILE = APP_DIR / "camera_prefs.json"
AUTOSAVE_DIR = APP_DIR / "autosaves"
FREEZE_LOG_FILE = APP_DIR / "freeze_log.txt"
DECIMAL_POINT_FILE = APP_DIR / "decimal_point.json"

# Fallback default webcam (used only until a camera has actually been
# selected/connected at least once) — matched by DirectShow device name
# rather than index, since indices shift depending on enumeration order or
# what's plugged in, but the name of a given physical camera stays stable.
DEFAULT_CAMERA_NAME = "Insta360 Link 2C"


def load_camera_prefs() -> dict:
    if not CAMERA_PREFS_FILE.exists():
        return {"camera_name": None}
    try:
        data = json.loads(CAMERA_PREFS_FILE.read_text(encoding="utf-8"))
        return {"camera_name": data.get("camera_name")}
    except Exception:
        return {"camera_name": None}


def save_camera_prefs(camera_name: str) -> None:
    try:
        CAMERA_PREFS_FILE.write_text(
            json.dumps({"camera_name": camera_name}, indent=2),
            encoding="utf-8",
        )
    except OSError:
        pass



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
    # Guarded like every settings write: if the app's folder isn't writable
    # (e.g. installed under Program Files) the app must still start.
    try:
        LOAD_CELLS_FILE.write_text(json.dumps(load_cells, indent=2), encoding="utf-8")
    except OSError:
        pass




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


_STANDARD_WEIGHT_CANDIDATES = tuple(
    base * 10 ** exp for exp in range(-4, 7) for base in (1, 2, 5)
) + tuple(float(x) for x in range(1, 1001))


def nearest_standard_weight_lbf(value_lbf: float) -> float:
    """Nearest common 1-2-5 / whole-pound standard weight."""
    if value_lbf == 0:
        return 0.0
    sign = -1 if value_lbf < 0 else 1
    v = abs(value_lbf)
    best = min(_STANDARD_WEIGHT_CANDIDATES, key=lambda c: abs(c - v))
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

# "Auto" = one more decimal than the OCR reading (see _selected_hadi_decimals).
HADI_DECIMAL_OPTIONS = ["1", "0.1", "0.01", "0.001", "0.0001", "0.00001", "Auto"]


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
        self._close_thread: Optional[threading.Thread] = None
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
        # Normally already finished; only waits if Connect follows a
        # Disconnect within the second or so the old port takes to close.
        self.wait_closed(2.0)
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            write_timeout=0.2,
        )
        # Each connection gets its own stop event and port handle, so a
        # worker still winding down from a previous connection can never be
        # revived by (or write to) the next one.
        stop_event = threading.Event()
        self.ser = ser
        self.stop_event = stop_event
        self.thread = threading.Thread(target=self._loop, args=(ser, stop_event),
                                       daemon=True, name="HADIWorker")
        self.thread.start()

    def disconnect(self, wait: bool = True):
        """Stop the worker and close the port.

        With wait=False the join and close happen on a background thread:
        both can block for a second or more (worker mid-read, or a USB-serial
        adapter that has dropped out), and doing that on the Tk thread is
        what froze the whole window whenever the HADI hiccupped.
        """
        ser, thread = self.ser, self.thread
        self.stop_event.set()
        self.ser = None
        self.thread = None

        def finish():
            if thread:
                thread.join(timeout=1.0)
            if ser:
                try:
                    ser.close()
                except Exception:
                    pass

        if wait:
            finish()
        else:
            self._close_thread = threading.Thread(target=finish, daemon=True, name="HADIClose")
            self._close_thread.start()

    def wait_closed(self, timeout: float) -> None:
        """Wait for a background close from disconnect(wait=False) to finish.

        Windows won't reopen a COM port while the old handle is still open,
        so anything about to open a port (connect, the auto-scan probe) must
        let the previous close complete first.
        """
        t = self._close_thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout)

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

    # A write timeout is usually the USB-serial adapter stalling briefly, not
    # the HADI going away. Retry this many times in a row before reporting it,
    # since reporting triggers a full disconnect + port rescan (seconds dead).
    MAX_WRITE_TIMEOUT_RETRIES = 3

    def _loop(self, ser, stop_event: threading.Event):
        write_timeouts = 0
        # Polls run on a fixed schedule: sleeping a full period *after* each
        # ~35 ms command/reply made "10 Hz" really ~7.3 Hz.
        next_poll = time.perf_counter()
        while not stop_event.is_set():
            next_poll += 1.0 / max(1.0, self.poll_hz)
            try:
                if not ser.is_open:
                    time.sleep(0.2)
                    continue
                ser.reset_input_buffer()
                ser.write((self.read_command + self.line_ending).encode("ascii", errors="ignore"))
                raw = ser.read_until(b"\r", size=256)
                if not raw:
                    raw = ser.read_until(b"\n", size=256)
                write_timeouts = 0
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
                if stop_event.is_set():
                    break  # port was closed under us by disconnect()
                if (isinstance(exc, serial.SerialTimeoutException)
                        and write_timeouts < self.MAX_WRITE_TIMEOUT_RETRIES):
                    write_timeouts += 1
                    log_freeze(f"HADI write timeout, retrying ({write_timeouts}/"
                               f"{self.MAX_WRITE_TIMEOUT_RETRIES})")
                    try:
                        ser.reset_output_buffer()
                    except Exception:
                        pass
                    stop_event.wait(0.05)
                    continue
                try:
                    self.out.put_nowait(exc)
                except queue.Full:
                    pass
                stop_event.wait(0.5)
            delay = next_poll - time.perf_counter()
            if delay < 0:
                # Fell behind (slow reply, error back-off): restart the
                # schedule from now rather than firing a burst to catch up.
                next_poll = time.perf_counter()
                delay = 0.0
            stop_event.wait(max(0.01, delay))


def load_sync_state() -> dict:
    if not SYNC_STATE_FILE.exists():
        return {"lag_ms": 239.0, "confidence": 0.0, "manual": False}
    try:
        data = json.loads(SYNC_STATE_FILE.read_text(encoding="utf-8"))
        return {
            "lag_ms": float(data.get("lag_ms", 239.0)),
            "confidence": float(data.get("confidence", 0.0)),
            "manual": bool(data.get("manual", False)),
        }
    except Exception:
        return {"lag_ms": 239.0, "confidence": 0.0, "manual": False}


def log_freeze(message: str) -> None:
    """Append a timestamped entry to freeze_log.txt (rotated at 1 MB)."""
    try:
        if FREEZE_LOG_FILE.exists() and FREEZE_LOG_FILE.stat().st_size > 1_000_000:
            FREEZE_LOG_FILE.replace(FREEZE_LOG_FILE.with_suffix(".old.txt"))
        with FREEZE_LOG_FILE.open("a", encoding="utf-8") as f:
            f.write(f"[{datetime.now():%Y-%m-%d %H:%M:%S.%f}] {message}\n")
    except Exception:
        pass


DECIMAL_POINT_OPTIONS = ["Off", "Auto", "0", "1", "2", "3", "4", "5"]


def load_decimal_point_mode() -> str:
    try:
        mode = json.loads(DECIMAL_POINT_FILE.read_text(encoding="utf-8")).get("mode")
        return mode if mode in DECIMAL_POINT_OPTIONS else "Auto"
    except Exception:
        return "Auto"


def save_decimal_point_mode(mode: str) -> None:
    try:
        DECIMAL_POINT_FILE.write_text(json.dumps({"mode": mode}, indent=2), encoding="utf-8")
    except Exception:
        pass


class DecimalPointFixer:
    """Restore a decimal point the OCR missed (e.g. 987.65 read as 98765).

    Only touches an all-digit number with no '.'; anything where the dot was
    read is returned unchanged. Modes:
      Off   - return the number exactly as read.
      0..5  - put the dot that many digits from the right ("98765" with 2 is
              "987.65", "000" with 2 is "0.00", "12776" with 5 is "0.12776"
              for displays that omit the leading zero; fewer digits than
              decimals is a partial read and is left alone).
      Auto  - pick the dot position whose value is closest to the HADI force
              (within 3x); otherwise use the decimals last actually read;
              otherwise show the raw digits.
    Called from the OCR thread; `mode` is a plain str set from the Tk thread.
    """

    # Dot positions are 10x apart, so halfway between them on a log scale
    # (sqrt(10) ~ 3.16) is the furthest a match can be and still be decisive.
    MATCH_MAX_RATIO = 3.0

    def __init__(self, mode: str = "Auto"):
        self.mode = mode
        self.remembered_decimals: Optional[int] = None

    @staticmethod
    def _insert(sign: str, digits: str, decimals: int) -> str:
        # The dot may go before every digit: displays that drop the leading
        # zero show ".12776", so a missed dot leaves "12776" = 0.12776.
        # Fewer digits than decimals means part of the number was missed;
        # leave that alone rather than invent zeros.
        if decimals <= 0 or len(digits) < decimals:
            return sign + digits
        whole = digits[:-decimals].lstrip("0") or "0"  # "001" -> "0.1", not "00.1"
        return f"{sign}{whole}.{digits[-decimals:]}"

    def fix(self, text: str, hadi_force: Optional[float] = None) -> str:
        """`hadi_force` is the HADI force in display units, or None when it
        shouldn't be used (no/stale HADI, no cell, or near zero load)."""
        text = text.strip()
        sign = text[0] if text.startswith(("+", "-")) else ""
        digits = text[len(sign):]
        if "." in digits:
            # "987." is a partial read (a display with no decimals shows no
            # dot), so it mustn't teach us the display has 0 decimals.
            decimals = len(digits.split(".", 1)[1])
            if decimals:
                self.remembered_decimals = decimals
            return text
        if self.mode == "Off" or not digits.isdigit():
            return text
        if self.mode != "Auto":
            return self._insert(sign, digits, int(self.mode))

        number = int(digits)
        target = abs(hadi_force) if hadi_force else 0.0
        if number and target:
            best = None
            for decimals in range(len(digits) + 1):
                value = number / 10 ** decimals
                ratio = max(value, target) / min(value, target)
                if ratio <= self.MATCH_MAX_RATIO and (best is None or ratio < best[0]):
                    best = (ratio, decimals)
            if best is not None:
                return self._insert(sign, digits, best[1])
        if self.remembered_decimals is not None:
            return self._insert(sign, digits, self.remembered_decimals)
        # Nothing known yet. A leading zero ("0345") still pins it down: no
        # display shows an integer with a redundant leading 0, so it was 0.345.
        if len(digits) > 1 and digits[0] == "0":
            return f"{sign}0.{digits[1:]}"
        return text


def save_sync_state(lag_ms: float, confidence: float, manual: bool = False) -> None:
    try:
        SYNC_STATE_FILE.write_text(
            json.dumps({"lag_ms": lag_ms, "confidence": confidence, "manual": manual}, indent=2),
            encoding="utf-8",
        )
    except OSError:
        pass



class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(f"HADI + OCR Capture v{APP_VERSION}")
        # Fit the screen it opens on: shrink if needed and place it high enough
        # that the taskbar doesn't cover the bottom (Settings scrolls if short).
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w = min(1115, sw - 40)
        h = min(720, sh - 110)          # room for the title bar and taskbar
        self.geometry(f"{w}x{h}+{max(0, (sw - w) // 2)}+{max(0, (sh - h) // 2 - 40)}")
        self.minsize(min(980, w), min(650, h))

        self.load_cells = load_saved_load_cells()
        self.selected_load_cell_name = tk.StringVar(value="")

        self.hadi = HADIWorker()
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
        self.edit_original_text = ""
        self.suppress_next_tree_select = False
        self.suppress_next_tree_click = False

        # Auto-sync method:
        # continuously estimate a fixed OCR-vs-HADI lag from recent waveform shape,
        # save it, then capture by corrected time. This never matches by closest value.
        sync_state = load_sync_state()
        self.sync_lag_seconds = sync_state["lag_ms"] / 1000.0
        self.sync_confidence = sync_state["confidence"]
        self.sync_manual = False
        self.sync_window_seconds = 8.0
        self.buffer_keep_seconds = 15.0
        self.hadi_buffer = deque()
        self.ocr_buffer = deque()
        self.ocr_lock = threading.Lock()
        self._last_sync_update = 0.0
        self._reset_lag_tracking()

        self.serial_number_var = tk.StringVar(value="")
        self.customer_capacity_var = tk.StringVar(value="")
        self.target_force_var = tk.StringVar(value="")
        self.target_forces: list[float] = []

        self.flip_sign_var = tk.BooleanVar(value=False)
        self.auto_capture_enabled = tk.BooleanVar(value=False)
        self.auto_capture_tolerance_var = tk.StringVar(value="1.0")
        self.auto_capture_dwell_var = tk.StringVar(value="3.0")
        self.auto_capture_voice_var = tk.BooleanVar(value=False)

        self.manual_median_window_var = tk.StringVar(value="0.5")
        self.require_both_var = tk.BooleanVar(value=False)
        self._auto_capture_in_range_since: Optional[float] = None
        self._auto_capture_last_index: Optional[int] = None
        self._auto_capture_last_run: Optional[int] = None

        self.latest_gps: Optional[GPSFix] = None
        self._mf_override: Optional[float] = None
        self._manual_location: Optional[dict] = None
        # What was last applied, so Cancel in the "Gravity changed" prompt
        # can put the entry boxes back.
        self._location_applied_text = ("", "", "ft")
        self._mf_override_applied_text = ""
        self.gps_status_var = tk.StringVar(value="Gravity: waiting for GPS")
        self.mf_override_var = tk.StringVar(value="")
        self.loc_lat_var = tk.StringVar(value="")
        self.loc_elev_var = tk.StringVar(value="")
        self.loc_elev_unit_var = tk.StringVar(value="ft")
        self.manual_weight_var = tk.StringVar(value="")

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="19200")
        self.mode_var = tk.StringVar(value="Tension")
        self.hadi_units_var = tk.StringVar(value="LBF")
        self._applied_hadi_units = "LBF"
        self.hadi_decimals_var = tk.StringVar(value="0.00001")
        self.zero_b0_var = tk.BooleanVar(value=False)   # "Better Zero" (temporary, not saved)
        self.decimal_point_var = tk.StringVar(value=load_decimal_point_mode())
        self.decimal_fixer = DecimalPointFixer(self.decimal_point_var.get())
        self.decimal_point_var.trace_add("write", lambda *_: self._on_decimal_point_change())
        # (HADI force in display units, pc_time), refreshed by the Tk thread
        # each UI tick for the OCR thread's Auto decimal placement — the OCR
        # thread must not touch Tk variables itself.
        self._dp_hadi: Optional[tuple] = None
        # Decimals in the latest OCR reading, for Dec = Auto (OCR thread writes).
        self._last_ocr_decimals: Optional[int] = None
        self.hadi_title_var = tk.StringVar(value="HADI Force (LBF)")
        self.mode_badge_var = tk.StringVar(value="COMPRESSION")
        self._hadi_overload = False
        self._hadi_overload_text = ""
        self._device_overload = False
        self._capacity_state: Optional[dict] = None   # see _capacity_in_units
        # Units before the first units change, and the Capacity as typed:
        # what "Restore original" goes back to (see _on_hadi_units_selected).
        self._units_original: Optional[str] = None
        self._capacity_captured: Optional[dict] = None
        self.poll_var = tk.StringVar(value="10")
        self.ocr_port_var = tk.StringVar(value="9999")
        self.status_var = tk.StringVar(value="Disconnected")

        self.hadi_raw_var = tk.StringVar(value="WAITING")
        self.hadi_lbf_var = tk.StringVar(value="---")
        self.ocr_value_var = tk.StringVar(value="---")
        self.live_error_var = tk.StringVar(value="--")
        self.hadi_last_pc_time = None
        self.ocr_last_pc_time = None
        self._ocr_last_packet_pc = None
        self.hadi_wait_seconds = 1.5
        self.ocr_wait_seconds = 1.5
        self.raw_text_var = tk.StringVar(value="")
        self.hadi_button_var = tk.StringVar(value="Connect HADI")
        self.ocr_button_var = tk.StringVar(value="Start OCR")
        self.count_var = tk.StringVar(value="0 captures")
        self.target_var = tk.StringVar(value="Next: P1 R1")
        self.sync_status_var = tk.StringVar(value="Sync: starting...")
        self.sync_lag_ms_var = tk.StringVar(value=f"{self.sync_lag_seconds * 1000.0:.0f}")

        self.dirty_data = False
        self.last_manual_save_path = None
        self.last_autosave_path = None
        self.autosave_name = None
        AUTOSAVE_DIR.mkdir(exist_ok=True)

        self.report_entries: list[dict] = []

        # Webcam OCR state
        self.webcam_ocr: Optional[WebcamOCR] = None
        self.webcam_cam_var = tk.StringVar(value="1")
        self.webcam_latency_var = tk.StringVar(value="190")
        self.webcam_status_var = tk.StringVar(value="Webcam OCR stopped")
        self.webcam_ms_var = tk.StringVar(value="--")
        self.webcam_reading_var = tk.StringVar(value="--")
        self.webcam_start_btn_var = tk.StringVar(value="Start Webcam OCR")
        self.ocr_source_var = tk.StringVar(value="webcam")
        self._webcam_roi = (540, 322, 740, 397)
        self._camera_scan_active = False
        self._pending_webcam_start = False
        self._webcam_connecting = False
        self._webcam_connect_token = 0
        self._camera_default_pending = True
        self._camera_prefs = load_camera_prefs()

        self.editor_select_var = tk.StringVar()
        self.cell_name_var = tk.StringVar()
        self.capacity_var = tk.StringVar()
        self.coeff_vars: dict[str, tk.StringVar] = {}
        self.in_use_cell_var = tk.StringVar(value="")

        self._setup_styles()
        self._build_ui()
        self._set_hadi_display_options()
        self._refresh_hadi_mode_badge()
        self._refresh_ports()
        self._sync_load_cell_controls()
        self._refresh_gravity_status()   # show the starting MF on the W button
        self._ui_heartbeat = time.perf_counter()
        self._last_tick_error: Optional[str] = None
        self._hadi_went_stale: Optional[float] = None
        self._start_freeze_watchdog()
        self.after(50, self._ui_tick)
        self.after(700, lambda: self._refresh_camera_list(then=self._auto_connect_on_launch))
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.bind_all("<KeyPress-space>", self._capture_from_key)

        self.bind_class("TButton", "<KeyPress-space>", self._capture_from_key)
        self.bind_class("TButton", "<ButtonRelease-1>", self._release_button_focus, add="+")

    def _setup_styles(self):
        style = ttk.Style(self)
        style.configure("LiveTitle.TLabel", font=("Segoe UI", 13, "bold"))
        style.configure("BigValue.TLabel", font=("Consolas", 38, "bold"))
        style.configure("ErrorValue.TLabel", font=("Consolas", 32, "bold"))
        style.configure("SmallValue.TLabel", font=("Consolas", 16))
        style.configure("TinyValue.TLabel", font=("Consolas", 9))
        style.configure("ModeBadgeCompression.TLabel", font=("Segoe UI", 12, "bold"), foreground="#0f5f5c", background="#d7f4f1", padding=(18, 10))
        style.configure("ModeBadgeTension.TLabel", font=("Segoe UI", 12, "bold"), foreground="#5b2a86", background="#efe3fb", padding=(18, 10))
        style.configure("OverloadBadge.TLabel", font=("Segoe UI", 13, "bold"), foreground="#ffffff", background="#c1121f", padding=(48, 11))
        style.configure("OverloadBadgeHidden.TLabel", font=("Segoe UI", 13, "bold"), foreground="#c1121f", background="#c1121f", padding=(48, 11))
        style.configure("Zero.TButton", font=("Segoe UI", 9, "bold"))
        style.configure("Override.TButton", font=("Segoe UI", 10, "bold"), padding=(10, 6))
        style.configure("BigCapture.TButton", font=("Segoe UI", 12, "bold"), padding=(14, 8))

    def _build_ui(self):
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True)

        self.capture_tab = ttk.Frame(self.notebook)
        self.connection_tab = ttk.Frame(self.notebook)
        self.load_cells_tab = ttk.Frame(self.notebook)
        self.converter_tab = ttk.Frame(self.notebook)
        self.report_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.capture_tab, text="Capture")
        self.notebook.add(self.load_cells_tab, text="Load Cells")
        self.notebook.add(self.converter_tab, text="Converter")
        self.notebook.add(self.connection_tab, text="Settings")

        self._build_capture_tab(self.capture_tab)
        # Settings is taller than the default window, so it scrolls rather
        # than cutting off the bottom sections on smaller screens.
        self._build_connection_tab(self._scrollable(self.connection_tab))
        self._build_load_cells_tab(self.load_cells_tab)
        self._build_converter_tab(self.converter_tab)
        self._build_report_tab(self.report_tab)

        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

    def _build_capture_tab(self, root):
        pad = {"padx": 10, "pady": 8}

        # Top section: load cell + mode on left, camera preview on right
        top_section = ttk.Frame(root)
        top_section.pack(fill="x", padx=10, pady=(4, 2))
        top_section.columnconfigure(0, weight=1)
        top_section.columnconfigure(1, weight=0)

        left_col = ttk.Frame(top_section)
        left_col.grid(row=0, column=0, sticky="nsew")

        self._load_cell_icon_img = tk.PhotoImage(file=str(RESOURCE_DIR / "load_cell_icon.png"))
        controls = ttk.LabelFrame(left_col, text="Load Cell")
        controls.pack(fill="x", pady=(0, 2))

        ttk.Label(controls, image=self._load_cell_icon_img).grid(row=0, column=0, sticky="w", padx=(6, 2), pady=1)
        self.load_cell_combo = ttk.Combobox(
            controls,
            textvariable=self.selected_load_cell_name,
            values=[c["name"] for c in self.load_cells],
            state="readonly",
            width=26,
            font=("Segoe UI", 10),
        )
        self.load_cell_combo.grid(row=0, column=1, sticky="w", padx=(6, 8), pady=4)
        self.load_cell_combo.bind("<<ComboboxSelected>>", lambda _e: self._select_load_cell())

        ttk.Label(controls, text="Units").grid(row=0, column=2, sticky="w", padx=(8, 2), pady=4)
        units = ttk.Combobox(controls, textvariable=self.hadi_units_var, values=["LBF", "KGF", "N", "kN", "gF", "t", "mV/V"], state="readonly", width=6, font=("Segoe UI", 10))
        units.grid(row=0, column=3, sticky="w", padx=(0, 8), pady=4)
        units.bind("<<ComboboxSelected>>", lambda _e: self._on_hadi_units_selected())

        ttk.Label(controls, text="Dec").grid(row=0, column=4, sticky="w", pady=4)
        decimals = ttk.Combobox(controls, textvariable=self.hadi_decimals_var, values=HADI_DECIMAL_OPTIONS, state="readonly", width=7, font=("Segoe UI", 10))
        decimals.grid(row=0, column=5, sticky="w", padx=(0, 6), pady=4)
        decimals.bind("<<ComboboxSelected>>", lambda _e: self._set_hadi_display_options())

        mode_bar = ttk.Frame(top_section)
        self._mode_bar = mode_bar

        def _position_mode_bar(_e=None):
            mode_bar.place(in_=top_section, relx=0.5, anchor="n",
                           y=controls.winfo_height() + 16)

        controls.bind("<Configure>", _position_mode_bar)
        top_section.after(0, _position_mode_bar)
        mode_bar.columnconfigure(0, weight=1)
        mode_bar.columnconfigure(1, weight=0)
        mode_bar.columnconfigure(2, weight=1)
        self.overload_badge_var = tk.StringVar(value="")
        # Parented to top_section (not mode_bar) so the full-width banner isn't
        # clipped to the narrow mode-badge frame, and lifted above the mode
        # badge when shown so the clickable Tension/Compression badge can't hide it.
        self.overload_badge = ttk.Label(top_section, textvariable=self.overload_badge_var, style="OverloadBadge.TLabel")
        self._overload_flash_visible = True
        self._overload_flash_id = None
        self.mode_badge_label = ttk.Label(mode_bar, textvariable=self.mode_badge_var, style="ModeBadgeCompression.TLabel", cursor="hand2")
        self.mode_badge_label.grid(row=0, column=1)
        self.mode_badge_label.bind("<Button-1>", lambda _e: self._toggle_mode())

        ac_frame = ttk.Frame(mode_bar)
        ac_frame.place(relx=0.5, rely=0.5, anchor="w", x=240)
        self._ac_indicator_frame = ac_frame
        ttk.Label(ac_frame, text="±", font=("Segoe UI", 10)).pack(side="left")
        ttk.Entry(ac_frame, textvariable=self.auto_capture_tolerance_var, width=5,
                  font=("Segoe UI", 9)).pack(side="left", padx=(1, 2))
        self._ac_units_var = tk.StringVar(value="LBF")
        ttk.Label(ac_frame, textvariable=self._ac_units_var, font=("Segoe UI", 9)).pack(side="left", padx=(0, 6))
        self._ac_status_label_var = tk.StringVar(value="")
        ttk.Label(ac_frame, textvariable=self._ac_status_label_var,
                  font=("Segoe UI", 9), foreground="#666666").pack(side="left")

        # Camera preview spanning load cell row + mode badge row
        cam_frame = ttk.Frame(top_section)
        cam_frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        cam_header = ttk.Frame(cam_frame)
        cam_header.pack(fill="x", pady=(0, 2))
        ttk.Button(cam_header, text="⟳ Refresh", width=10,
                   command=self._reconnect_to_default).pack(side="left")

        self._capture_cam_canvas = tk.Canvas(cam_frame, width=300, height=100,
                                             bg="#1a1a1a", highlightthickness=1,
                                             highlightbackground="#555555")
        self._capture_cam_canvas.pack(fill="both", expand=True)
        self._capture_cam_canvas.configure(cursor="hand2")
        self._capture_cam_canvas.bind("<Button-1>", lambda _e: self._open_cam_popup())
        self._capture_cam_photo = None
        self._capture_cam_img_id = None
        self._cam_popup = None
        self._cam_popup_canvas = None
        self._cam_popup_photo = None
        self._cam_popup_img_id = None
        self._cam_popup_geom = None
        self._cam_popup_drag = None
        self._start_capture_cam_preview()

        self._ac_gauge = tk.Canvas(root, height=24, highlightthickness=0, bg="#e5e7eb")
        self._ac_gauge.bind("<Configure>", lambda _e: self._redraw_ac_gauge())
        self._ac_gauge_error = 0.0
        self._ac_gauge_tolerance = 1.0
        self._ac_gauge_active = False

        display = ttk.Frame(root)
        display.pack(fill="both", expand=True, **pad)
        self._ac_display_frame = display

        self._update_ac_indicator_visibility()

        left = ttk.LabelFrame(display, text="Live Display")
        left.pack(side="left", fill="both", expand=True, padx=(0, 6))

        self._hadi_live_label(left, self.hadi_title_var, self.hadi_lbf_var, 0)

        ocr_card = ttk.Frame(left)
        ocr_card.grid(row=1, column=0, sticky="ew", padx=12, pady=(14, 0))
        ocr_card.columnconfigure(0, weight=1)
        ocr_header = ttk.Frame(ocr_card)
        ocr_header.grid(row=0, column=0, sticky="ew")
        ocr_header.columnconfigure(0, weight=1)
        ocr_title_frame = ttk.Frame(ocr_header)
        ocr_title_frame.grid(row=0, column=0, sticky="w")
        ttk.Label(ocr_title_frame, text="OCR Stream", style="LiveTitle.TLabel").pack(side="left")
        self.ocr_status_light = tk.Canvas(ocr_title_frame, width=12, height=12, highlightthickness=0)
        self.ocr_status_light.pack(side="left", padx=(6, 0))
        self._ocr_light_id = self.ocr_status_light.create_oval(1, 1, 11, 11, fill="#cc0000", outline="#888888")
        ttk.Label(ocr_header, textvariable=self.target_force_var,
                  font=("Consolas", 14, "bold"), foreground="#999999", width=22, anchor="e").grid(row=0, column=1, sticky="e", padx=(0, 40))
        ttk.Label(ocr_card, textvariable=self.ocr_value_var, style="BigValue.TLabel", width=14, anchor="w").grid(row=1, column=0, sticky="w")

        self._error_live_label(left, "Live % Error", self.live_error_var, 2)
        self._small_label(left, "HADI Raw R (mV/V)", self.hadi_raw_var, 3)

        self.big_capture_button = ttk.Button(
            left,
            text="CAPTURE  (Space)",
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

        ttk.Label(btns, text="S/N:").pack(side="left", padx=(0, 2))
        ttk.Entry(btns, textvariable=self.serial_number_var, width=14).pack(side="left")

        self.capacity_label_var = tk.StringVar(value="Capacity (LBF):")
        ttk.Label(btns, textvariable=self.capacity_label_var).pack(side="left", padx=(8, 2))
        cap_entry = ttk.Entry(btns, textvariable=self.customer_capacity_var, width=7)
        cap_entry.pack(side="left")
        self._capacity_entry = cap_entry   # units-change note pops up above it
        cap_entry.bind("<Return>", lambda _e: self._apply_customer_capacity())
        cap_entry.bind("<FocusOut>", lambda _e: self._apply_customer_capacity())

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

        ttk.Button(btns, text="Clear", command=self._clear_captures, width=5).pack(side="right", padx=(10, 0))
        ttk.Button(
            btns,
            text="⤓ Save",
            command=self._save_csv,
            width=18,
        ).pack(side="right")

        tables = ttk.Frame(right)
        tables.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        tables.columnconfigure(0, weight=1)
        tables.columnconfigure(1, weight=1)
        tables.rowconfigure(1, weight=1)

        ttk.Label(tables, text="Run 1", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, sticky="w", padx=(0, 6))
        run2_header = ttk.Frame(tables)
        run2_header.grid(row=0, column=1, sticky="ew", padx=(6, 0))
        ttk.Label(run2_header, text="Run 2", font=("Segoe UI", 10, "bold")).pack(side="left")
        self.copy_w_btn = ttk.Button(run2_header, text="← Copy W from Run 1", command=self._copy_weights_to_run2)
        self.copy_w_btn.pack(side="right")
        self.copy_w_btn.pack_forget()

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

        self._initialize_point_rows(11)

    _ROI_MIN = 10
    _ROI_HANDLE_PX = 8
    _ROI_CURSORS = {
        "": "crosshair", "move": "fleur",
        "n": "sb_v_double_arrow", "s": "sb_v_double_arrow",
        "e": "sb_h_double_arrow", "w": "sb_h_double_arrow",
        "nw": "size_nw_se", "se": "size_nw_se", "ne": "size_ne_sw", "sw": "size_ne_sw",
    }

    def _open_cam_popup(self):
        if self._cam_popup and self._cam_popup.winfo_exists():
            self._cam_popup.lift()
            return

        self._cam_popup = tk.Toplevel(self)
        self._cam_popup.title(
            "Camera — drag box to move · drag edges/corners to resize · drag outside to draw new · "
            "arrows nudge (Shift = 10) · right-click reset")
        self._cam_popup.geometry("800x560")
        self._cam_popup.attributes("-topmost", True)
        self._cam_popup.protocol("WM_DELETE_WINDOW", self._close_cam_popup)

        c = tk.Canvas(self._cam_popup, bg="#1a1a1a", cursor="crosshair", highlightthickness=0)
        c.pack(fill="both", expand=True)
        self._cam_popup_canvas = c
        c.bind("<ButtonPress-1>", self._cam_popup_mouse_down)
        c.bind("<B1-Motion>", self._cam_popup_mouse_drag)
        c.bind("<ButtonRelease-1>", self._cam_popup_mouse_up)
        c.bind("<Motion>", self._cam_popup_hover)
        c.bind("<ButtonPress-3>", self._webcam_reset_roi)
        for key, dx, dy in (("Left", -1, 0), ("Right", 1, 0), ("Up", 0, -1), ("Down", 0, 1)):
            self._cam_popup.bind(f"<{key}>", lambda _e, dx=dx, dy=dy: self._cam_popup_nudge(dx, dy))
            self._cam_popup.bind(f"<Shift-{key}>", lambda _e, dx=dx, dy=dy: self._cam_popup_nudge(dx * 10, dy * 10))

        self._cam_popup_photo = None
        self._cam_popup_img_id = None
        self._cam_popup_geom = None
        self._cam_popup_drag = None
        self._cam_popup_box_ids = None
        self._cam_popup_msg_id = None
        self._cam_popup.focus_force()
        self._cam_popup_tick()

    def _close_cam_popup(self):
        if self._cam_popup:
            self._cam_popup.destroy()
            self._cam_popup = None
            self._cam_popup_canvas = None
            self._cam_popup_drag = None
            self._cam_popup_geom = None

    def _cam_popup_tick(self):
        if not self._cam_popup or not self._cam_popup.winfo_exists():
            return
        try:
            self._cam_popup_tick_body()
        except Exception:
            pass
        finally:
            self._cam_popup.after(100, self._cam_popup_tick)

    def _cam_popup_tick_body(self):
        c = self._cam_popup_canvas
        frame = None
        if self.webcam_ocr and self.webcam_ocr.last_frame is not None:
            frame = self.webcam_ocr.last_frame

        cw, ch = c.winfo_width(), c.winfo_height()
        if frame is None or not _HAS_WEBCAM_DEPS or cw <= 1 or ch <= 1:
            if self._cam_popup_msg_id is None and cw > 1:
                self._cam_popup_msg_id = c.create_text(
                    cw // 2, ch // 2, fill="#bbbbbb", font=("Segoe UI", 11),
                    text="No camera image — start Webcam OCR")
            return
        if self._cam_popup_msg_id is not None:
            c.delete(self._cam_popup_msg_id)
            self._cam_popup_msg_id = None

        fh, fw = frame.shape[:2]
        scale = min(cw / fw, ch / fh)
        new_w, new_h = max(1, int(fw * scale)), max(1, int(fh * scale))
        off_x, off_y = (cw - new_w) // 2, (ch - new_h) // 2
        self._cam_popup_geom = (scale, off_x, off_y, fw, fh)

        if self._cam_popup_drag is None:
            # Keep the box inside this camera's frame so it can never end up
            # off-image (e.g. after switching to a lower-resolution camera).
            clamped = self._clamp_roi(self._webcam_roi, fw, fh)
            if clamped != tuple(self._webcam_roi):
                self._set_webcam_roi(clamped)

        rgb = _cv2.cvtColor(_cv2.resize(frame, (new_w, new_h)), _cv2.COLOR_BGR2RGB)
        self._cam_popup_photo = ImageTk.PhotoImage(Image.fromarray(rgb))
        if self._cam_popup_img_id:
            c.itemconfig(self._cam_popup_img_id, image=self._cam_popup_photo)
            c.coords(self._cam_popup_img_id, off_x, off_y)
        else:
            self._cam_popup_img_id = c.create_image(off_x, off_y, anchor="nw", image=self._cam_popup_photo)
        self._cam_popup_draw_box()

    def _clamp_roi(self, roi, fw, fh):
        m = self._ROI_MIN
        x1, y1, x2, y2 = (int(v) for v in roi)
        w = max(m, min(x2 - x1, fw))
        h = max(m, min(y2 - y1, fh))
        x1 = max(0, min(x1, fw - w))
        y1 = max(0, min(y1, fh - h))
        return (x1, y1, x1 + w, y1 + h)

    def _set_webcam_roi(self, roi, apply_to_ocr=True):
        self._webcam_roi = tuple(int(v) for v in roi)
        if apply_to_ocr and self.webcam_ocr:
            # Swap in a whole new ROI so the OCR thread never reads a box
            # with some edges updated and others not.
            self.webcam_ocr.roi = type(self.webcam_ocr.roi)(*self._webcam_roi)

    def _roi_to_canvas(self, roi):
        scale, off_x, off_y, _fw, _fh = self._cam_popup_geom
        x1, y1, x2, y2 = roi
        return (off_x + x1 * scale, off_y + y1 * scale, off_x + x2 * scale, off_y + y2 * scale)

    def _cam_popup_to_frame(self, cx, cy):
        scale, off_x, off_y, fw, fh = self._cam_popup_geom
        fx = int(round((cx - off_x) / scale))
        fy = int(round((cy - off_y) / scale))
        return max(0, min(fx, fw)), max(0, min(fy, fh))

    def _cam_popup_draw_box(self):
        c = self._cam_popup_canvas
        if c is None or self._cam_popup_geom is None:
            return
        x1, y1, x2, y2 = self._roi_to_canvas(self._webcam_roi)
        if self._cam_popup_box_ids is None:
            rect = c.create_rectangle(0, 0, 0, 0, outline="#00ff00", width=2)
            handles = [c.create_rectangle(0, 0, 0, 0, outline="#003300", fill="#00ff00") for _ in range(8)]
            label = c.create_text(0, 0, anchor="sw", fill="#00ff00", font=("Segoe UI", 9))
            self._cam_popup_box_ids = (rect, handles, label)
        rect, handles, label = self._cam_popup_box_ids
        c.coords(rect, x1, y1, x2, y2)
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        s = 4
        for hid, (hx, hy) in zip(handles, ((x1, y1), (mx, y1), (x2, y1), (x2, my),
                                           (x2, y2), (mx, y2), (x1, y2), (x1, my))):
            c.coords(hid, hx - s, hy - s, hx + s, hy + s)
        rx1, ry1, rx2, ry2 = self._webcam_roi
        c.coords(label, x1, y1 - 6)
        c.itemconfig(label, text=f"{rx2 - rx1} × {ry2 - ry1}")
        for item in (rect, *handles, label):
            c.tag_raise(item)

    def _cam_popup_hit(self, cx, cy):
        """'' outside the box, 'move' inside, else the edges to drag ('n', 'se', ...)."""
        x1, y1, x2, y2 = self._roi_to_canvas(self._webcam_roi)
        tol = self._ROI_HANDLE_PX
        if not (x1 - tol <= cx <= x2 + tol and y1 - tol <= cy <= y2 + tol):
            return ""
        edge = ""
        if abs(cy - y1) <= tol:
            edge += "n"
        elif abs(cy - y2) <= tol:
            edge += "s"
        if abs(cx - x1) <= tol:
            edge += "w"
        elif abs(cx - x2) <= tol:
            edge += "e"
        if edge:
            return edge
        return "move" if (x1 < cx < x2 and y1 < cy < y2) else ""

    def _cam_popup_hover(self, event):
        if self._cam_popup_geom is None or self._cam_popup_drag is not None:
            return
        self._cam_popup_canvas.configure(cursor=self._ROI_CURSORS[self._cam_popup_hit(event.x, event.y)])

    def _cam_popup_mouse_down(self, event):
        if self._cam_popup_geom is None:
            return
        self._cam_popup.focus_force()
        mode = self._cam_popup_hit(event.x, event.y) or "new"
        self._cam_popup_drag = {
            "mode": mode,
            "start": self._cam_popup_to_frame(event.x, event.y),
            "roi": tuple(self._webcam_roi),
        }

    def _cam_popup_mouse_drag(self, event):
        d = self._cam_popup_drag
        if d is None or self._cam_popup_geom is None:
            return
        _scale, _ox, _oy, fw, fh = self._cam_popup_geom
        m = self._ROI_MIN
        fx, fy = self._cam_popup_to_frame(event.x, event.y)
        sx, sy = d["start"]
        x1, y1, x2, y2 = d["roi"]
        mode = d["mode"]
        if mode == "new":
            roi = (min(sx, fx), min(sy, fy), max(sx, fx), max(sy, fy))
        elif mode == "move":
            dx = max(-x1, min(fx - sx, fw - x2))
            dy = max(-y1, min(fy - sy, fh - y2))
            roi = (x1 + dx, y1 + dy, x2 + dx, y2 + dy)
        else:
            if "w" in mode:
                x1 = max(0, min(fx, x2 - m))
            if "e" in mode:
                x2 = min(fw, max(fx, x1 + m))
            if "n" in mode:
                y1 = max(0, min(fy, y2 - m))
            if "s" in mode:
                y2 = min(fh, max(fy, y1 + m))
            roi = (x1, y1, x2, y2)
        # Live on screen only; the OCR gets the box on release.
        self._set_webcam_roi(roi, apply_to_ocr=False)
        self._cam_popup_draw_box()

    def _cam_popup_mouse_up(self, event):
        d = self._cam_popup_drag
        if d is None:
            return
        self._cam_popup_mouse_drag(event)
        self._cam_popup_drag = None
        x1, y1, x2, y2 = self._webcam_roi
        if (x2 - x1) < self._ROI_MIN or (y2 - y1) < self._ROI_MIN:
            # A click or tiny drag outside the box keeps the old box.
            self._set_webcam_roi(d["roi"])
        else:
            self._set_webcam_roi(self._webcam_roi)
        self._cam_popup_draw_box()
        self._cam_popup_hover(event)

    def _cam_popup_nudge(self, dx, dy):
        if self._cam_popup_geom is None or self._cam_popup_drag is not None:
            return "break"
        _scale, _ox, _oy, fw, fh = self._cam_popup_geom
        x1, y1, x2, y2 = self._webcam_roi
        dx = max(-x1, min(dx, fw - x2))
        dy = max(-y1, min(dy, fh - y2))
        self._set_webcam_roi((x1 + dx, y1 + dy, x2 + dx, y2 + dy))
        self._cam_popup_draw_box()
        return "break"

    def _start_capture_cam_preview(self):
        self._capture_cam_tick()

    def _capture_cam_tick(self):
        # Guarded like _ui_tick: an escaped exception here (e.g. a stale ROI
        # that falls outside a new camera's frame producing a zero-size crop)
        # would otherwise stop the preview loop permanently.
        try:
            self._capture_cam_tick_body()
        except Exception:
            pass
        finally:
            interval = 200 if self.notebook.index(self.notebook.select()) == 0 else 1000
            self.after(interval, self._capture_cam_tick)

    def _capture_cam_tick_body(self):
        frame = None
        if self.webcam_ocr and self.webcam_ocr.last_frame is not None:
            frame = self.webcam_ocr.last_frame

        if frame is not None and _HAS_WEBCAM_DEPS:
            cw = self._capture_cam_canvas.winfo_width()
            ch = self._capture_cam_canvas.winfo_height()
            if cw > 1 and ch > 1:
                fh, fw = frame.shape[:2]
                # Crop around ROI with 50% padding for a zoomed-in preview
                rx1, ry1, rx2, ry2 = self._webcam_roi
                roi_w = rx2 - rx1
                roi_h = ry2 - ry1
                pad_x = int(roi_w * 0.5)
                pad_y = int(roi_h * 0.5)
                cx1 = max(0, rx1 - pad_x)
                cy1 = max(0, ry1 - pad_y)
                cx2 = min(fw, rx2 + pad_x)
                cy2 = min(fh, ry2 + pad_y)
                cropped = frame[cy1:cy2, cx1:cx2]

                crop_h, crop_w = cropped.shape[:2]
                if crop_w < 2 or crop_h < 2:
                    return
                scale = min(cw / crop_w, ch / crop_h)
                new_w = int(crop_w * scale)
                new_h = int(crop_h * scale)
                resized = _cv2.resize(cropped, (new_w, new_h))
                rgb = _cv2.cvtColor(resized, _cv2.COLOR_BGR2RGB)

                # Draw ROI box relative to the crop
                x1, y1, x2, y2 = self._webcam_roi
                rx1 = int((x1 - cx1) * scale)
                ry1 = int((y1 - cy1) * scale)
                rx2 = int((x2 - cx1) * scale)
                ry2 = int((y2 - cy1) * scale)
                _cv2.rectangle(rgb, (rx1, ry1), (rx2, ry2), (0, 255, 0), 1)

                img = Image.fromarray(rgb)
                self._capture_cam_photo = ImageTk.PhotoImage(img)
                if self._capture_cam_img_id:
                    self._capture_cam_canvas.itemconfig(
                        self._capture_cam_img_id, image=self._capture_cam_photo)
                else:
                    self._capture_cam_img_id = self._capture_cam_canvas.create_image(
                        cw // 2, ch // 2, image=self._capture_cam_photo)

        if self.webcam_ocr:
            fps = getattr(self.webcam_ocr, "capture_fps", 0.0)
            fps_note = f" @ {fps:.0f} fps" if fps else ""
            self.webcam_ms_var.set(f"{self.webcam_ocr.last_elapsed_ms:.0f} ms{fps_note}")
            r = self.webcam_ocr.latest_reading()
            if r:
                self.webcam_reading_var.set(r.raw_text)

    def _apply_webcam_latency(self):
        try:
            ms = float(self.webcam_latency_var.get())
        except ValueError:
            return
        if self.webcam_ocr:
            self.webcam_ocr.camera_latency_ms = ms
            self.sync_lag_seconds = ms / 1000.0
            self._reset_lag_tracking()  # typed-in value is the new starting point
            self.sync_status_var.set(f"Sync: webcam (lag {ms:.0f} ms)")

    def _on_ocr_source_change(self):
        source = self.ocr_source_var.get()
        if source == "phone":
            self._stop_webcam_ocr()
            if not self.ocr:
                self._start_ocr(show_error=False)
            self.webcam_status_var.set("Using iPhone UDP stream")
        else:
            self._stop_ocr()
            self.webcam_status_var.set("Webcam selected — press Start Webcam OCR")

    def _toggle_webcam_ocr(self):
        if self.webcam_ocr or self._webcam_connecting:
            self._stop_webcam_ocr()
        else:
            self._start_webcam_ocr()

    def _on_camera_selected(self):
        self.ocr_source_var.set("webcam")
        # Only an explicit user selection updates the saved default camera.
        self._start_webcam_ocr(remember=True)

    def _refresh_camera_list(self, then=None):
        if WebcamOCR is None or not _HAS_WEBCAM_DEPS:
            if then:
                then()
            return
        if self._camera_scan_active:
            if then:
                then()
            return

        self._camera_scan_active = True

        def worker():
            try:
                cams = WebcamOCR.list_cameras()
            except Exception:
                cams = []
            self.after(0, lambda: self._on_camera_list_ready(cams, then))

        threading.Thread(target=worker, daemon=True).start()

    def _on_camera_list_ready(self, cams, then=None):
        self._camera_scan_active = False
        if cams:
            labels = [f"{c['index']}: {c['name']}" for c in cams]
            self.settings_cam_combo["values"] = labels

            selected_label = None
            if self._camera_default_pending:
                self._camera_default_pending = False
                preferred_name = self._camera_prefs.get("camera_name") or DEFAULT_CAMERA_NAME
                selected_label = next(
                    (label for label, cam in zip(labels, cams)
                     if cam["name"] == preferred_name), None)
            if selected_label is None:
                current_index = self._current_cam_index()
                selected_label = next(
                    (label for label, cam in zip(labels, cams)
                     if cam["index"] == current_index), None)
            if selected_label is not None:
                self.webcam_cam_var.set(selected_label)
        if self._pending_webcam_start:
            self._pending_webcam_start = False
            self._start_webcam_ocr()
        if then:
            then()

    def _current_cam_index(self) -> Optional[int]:
        raw = str(self.webcam_cam_var.get())
        try:
            return int(raw.split(":", 1)[0].strip())
        except ValueError:
            return None

    def _current_cam_name(self) -> Optional[str]:
        raw = str(self.webcam_cam_var.get())
        if ":" in raw:
            return raw.split(":", 1)[1].strip()
        return None

    def _start_webcam_ocr(self, show_error=True, remember=False):
        if WebcamOCR is None or not _HAS_WEBCAM_DEPS:
            if show_error:
                messagebox.showerror("Missing dependencies",
                                     "Install opencv-python, Pillow, and rapidocr")
            return
        if self._camera_scan_active:
            # A background camera probe is using the DirectShow/MSMF backend
            # right now; opening the real capture at the same time is what
            # causes the driver to wedge, so wait for the scan to finish.
            self.webcam_status_var.set("Waiting for camera scan to finish…")
            self._pending_webcam_start = True
            return
        if self._webcam_connecting:
            return
        cam = self._current_cam_index()
        if cam is None:
            if show_error:
                messagebox.showerror("Webcam OCR failed",
                                     f"Invalid camera selection: {self.webcam_cam_var.get()!r}")
            return
        cam_name = self._current_cam_name()

        self._stop_ocr()
        if self.webcam_ocr:
            self.webcam_ocr.stop()
            self.webcam_ocr = None

        try:
            latency_ms = float(self.webcam_latency_var.get())
        except ValueError:
            latency_ms = None
        x1, y1, x2, y2 = self._webcam_roi

        # Opening the camera and loading the OCR model can take several
        # seconds (some virtual-cam drivers are especially slow to spin up),
        # and both happen inside WebcamOCR.start(). Running that on the main
        # thread would freeze the whole GUI for the duration, so it's done
        # in a background thread and the result is applied back via `after`.
        self._webcam_connecting = True
        self._webcam_connect_token += 1
        token = self._webcam_connect_token
        self.webcam_start_btn_var.set("Connecting…")
        self.webcam_status_var.set(f"Connecting to camera {cam}…")

        def worker():
            try:
                wc = WebcamOCR(cam_index=cam)
                if latency_ms is not None:
                    wc.camera_latency_ms = latency_ms
                wc.roi.x1 = x1
                wc.roi.y1 = y1
                wc.roi.x2 = x2
                wc.roi.y2 = y2
                wc.on_reading = self._on_ocr_reading
                wc.start()
            except Exception as exc:
                # Bind exc now: Python deletes it when the except block ends,
                # so a plain closure raised NameError later and the failure
                # was never reported (status stuck on "Connecting…").
                self.after(0, lambda e=exc: self._on_webcam_connect_failed(token, e, show_error))
                return
            self.after(0, lambda: self._on_webcam_connected(token, wc, cam, cam_name, remember))

        threading.Thread(target=worker, daemon=True).start()

    def _on_webcam_connected(self, token, wc, cam, cam_name, remember=False):
        self._webcam_connecting = False
        if token != self._webcam_connect_token:
            # A stop/switch happened while this connect was in flight;
            # this result is stale, so discard the camera it opened.
            wc.stop()
            return
        self.webcam_ocr = wc
        if remember and cam_name and self._camera_prefs.get("camera_name") != cam_name:
            # Only an explicit user selection updates the saved default.
            # Auto-connect fallbacks must never overwrite it — that was how
            # the integrated camera silently replaced the preferred camera.
            self._camera_prefs["camera_name"] = cam_name
            save_camera_prefs(cam_name)
        # Starting lag: the one measured last session if there is one (same
        # camera and display, so it's the best guess), else Cam Latency.
        # The estimator then refines it continuously from live data.
        saved = load_sync_state()
        if saved.get("confidence", 0.0) >= 0.65 and not saved.get("manual"):
            start_ms = saved["lag_ms"]
            source = "measured last session"
        else:
            start_ms = wc.camera_latency_ms
            source = "Cam Latency"
        self.sync_lag_seconds = start_ms / 1000.0
        self.sync_confidence = 0.0
        self._reset_lag_tracking()  # new camera: re-measure from scratch
        self.sync_status_var.set(f"Sync: starting at {start_ms:.0f} ms ({source})")
        self.webcam_start_btn_var.set("Stop Webcam OCR")
        self.webcam_status_var.set(f"Webcam OCR running (cam {cam})")
        self.ocr_source_var.set("webcam")
        self.ocr_button_var.set("Start OCR")

    def _on_webcam_connect_failed(self, token, exc, show_error=True):
        self._webcam_connecting = False
        if token != self._webcam_connect_token:
            return
        self.webcam_start_btn_var.set("Start Webcam OCR")
        self.webcam_status_var.set(f"Webcam error: {exc}")
        if show_error:
            messagebox.showerror("Webcam OCR failed", str(exc))

    def _reconnect_to_default(self):
        """Rescan cameras, then (re)connect to the saved default by name."""
        def after_scan():
            preferred = self._camera_prefs.get("camera_name") or DEFAULT_CAMERA_NAME
            label = self._label_for_camera_name(preferred)
            if label is not None:
                self.webcam_cam_var.set(label)
                self._start_webcam_ocr()
            else:
                self.webcam_status_var.set(
                    f"Default camera '{preferred}' not found — check its USB "
                    "connection, or pick a camera in Settings to make it the new default.")

        self._refresh_camera_list(then=after_scan)

    def _stop_webcam_ocr(self):
        self._webcam_connect_token += 1
        self._webcam_connecting = False
        if self.webcam_ocr:
            self.webcam_ocr.stop()
            self.webcam_ocr = None
        self.webcam_start_btn_var.set("Start Webcam OCR")
        self.webcam_status_var.set("Webcam OCR stopped")
        self.webcam_ms_var.set("--")
        self.webcam_reading_var.set("--")

    def _webcam_reset_roi(self, event=None):
        if self.webcam_ocr and self.webcam_ocr.last_frame is not None:
            h, w = self.webcam_ocr.last_frame.shape[:2]
            self._webcam_roi = (w // 4, h // 4, 3 * w // 4, 3 * h // 4)
        else:
            self._webcam_roi = (540, 322, 740, 397)
        self._set_webcam_roi(self._webcam_roi)
        if self._cam_popup_canvas is not None:
            self._cam_popup_draw_box()

    def _scrollable(self, parent) -> ttk.Frame:
        """A frame inside `parent` that scrolls vertically (scrollbar and mouse
        wheel) when its contents are taller than the window."""
        bg = ttk.Style().lookup("TFrame", "background") or self.cget("background")
        canvas = tk.Canvas(parent, highlightthickness=0, borderwidth=0, background=bg)
        bar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        window = canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=bar.set)

        def update_bar(_e=None):
            canvas.configure(scrollregion=canvas.bbox("all"))
            # Only show the scrollbar when there's something to scroll.
            if inner.winfo_reqheight() > canvas.winfo_height():
                bar.pack(side="right", fill="y")
            else:
                bar.pack_forget()
                canvas.yview_moveto(0)

        inner.bind("<Configure>", update_bar)
        canvas.bind("<Configure>", lambda e: (canvas.itemconfigure(window, width=e.width), update_bar()))

        def on_wheel(e):
            if inner.winfo_reqheight() > canvas.winfo_height():
                canvas.yview_scroll(int(-e.delta / 120), "units")

        # Wheel scrolls this tab only while the pointer is over it.
        canvas.bind("<Enter>", lambda _e: canvas.bind_all("<MouseWheel>", on_wheel))
        canvas.bind("<Leave>", lambda _e: canvas.unbind_all("<MouseWheel>"))
        canvas.pack(side="left", fill="both", expand=True)
        return inner

    def _build_connection_tab(self, root):
        pad = {"padx": 10, "pady": 8}

        gps_box = ttk.LabelFrame(root, text="Gravity / GPS")
        gps_box.pack(fill="x", **pad)
        ttk.Label(gps_box, textvariable=self.gps_status_var).grid(row=0, column=0, columnspan=6, sticky="w", padx=6, pady=6)

        ttk.Label(gps_box, text="Latitude (° N)").grid(row=1, column=0, sticky="w", padx=6, pady=6)
        lat_entry = ttk.Entry(gps_box, textvariable=self.loc_lat_var, width=12)
        lat_entry.grid(row=1, column=1, sticky="w", pady=6)
        elev_row = ttk.Frame(gps_box)
        elev_row.grid(row=1, column=2, columnspan=2, sticky="w", padx=(10, 0), pady=6)
        ttk.Label(elev_row, text="Elevation").pack(side="left")
        elev_entry = ttk.Entry(elev_row, textvariable=self.loc_elev_var, width=10)
        elev_entry.pack(side="left", padx=(6, 2))
        ttk.Combobox(elev_row, textvariable=self.loc_elev_unit_var, values=["ft", "m"],
                     state="readonly", width=4).pack(side="left")
        ttk.Button(gps_box, text="Apply", command=self._apply_location).grid(row=1, column=4, sticky="w", padx=6)
        hint_row = ttk.Frame(gps_box)
        hint_row.grid(row=1, column=5, sticky="w", padx=(10, 6), pady=6)
        ttk.Label(hint_row, text="N value + elevation from an altimeter app (W not needed). Blank both to clear.").pack(side="left")
        self._mf_link = ttk.Label(hint_row, text="Enter MF directly", foreground="#1a5fb4",
                                  cursor="hand2", font=("Segoe UI", 9, "underline"))
        self._mf_link.pack(side="left", padx=(12, 0))
        self._mf_link.bind("<Button-1>", lambda _e: self._toggle_mf_row())
        for w in (lat_entry, elev_entry):
            w.bind("<Return>", lambda _e: self._apply_location())

        mf_label = ttk.Label(gps_box, text="MF Override")
        mf_label.grid(row=2, column=0, sticky="w", padx=6, pady=6)
        mf_entry = ttk.Entry(gps_box, textvariable=self.mf_override_var, width=12)
        mf_entry.grid(row=2, column=1, sticky="w", pady=6)
        mf_entry.bind("<Return>", lambda _e: self._apply_mf_override())
        mf_apply = ttk.Button(gps_box, text="Apply", command=self._apply_mf_override)
        mf_apply.grid(row=2, column=4, sticky="w", padx=6)
        mf_hint = ttk.Label(gps_box, text="Overrides the location MF. Clear and Apply to go back.")
        mf_hint.grid(row=2, column=5, sticky="w", padx=(10, 6), pady=6)
        self._mf_row_widgets = (mf_label, mf_entry, mf_apply, mf_hint)
        self._mf_row_visible = True
        self._set_mf_row_visible(False)

        dp_box = ttk.LabelFrame(root, text="Decimal point")
        dp_box.pack(fill="x", **pad)
        ttk.Label(dp_box, text="Decimal places").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Combobox(dp_box, textvariable=self.decimal_point_var, values=DECIMAL_POINT_OPTIONS,
                     state="readonly", width=6).grid(row=0, column=1, sticky="w", pady=6)
        ttk.Label(dp_box, text="When the OCR misses the display's decimal point (98765 for 987.65): "
                               "Auto matches it to the HADI, 0–5 always places it, Off shows it as read."
                  ).grid(row=0, column=2, sticky="w", padx=(10, 6), pady=6)

        webcam_box = ttk.LabelFrame(root, text="Webcam OCR")
        webcam_box.pack(fill="x", **pad)

        ttk.Radiobutton(webcam_box, text="iPhone (UDP stream)",
                        variable=self.ocr_source_var, value="phone",
                        command=self._on_ocr_source_change).grid(
            row=0, column=0, sticky="w", padx=6, pady=4)
        ttk.Radiobutton(webcam_box, text="Webcam (local OCR)",
                        variable=self.ocr_source_var, value="webcam",
                        command=self._on_ocr_source_change).grid(
            row=0, column=1, sticky="w", padx=6, pady=4)

        ttk.Label(webcam_box, text="Camera").grid(row=1, column=0, sticky="w", padx=6, pady=6)
        self.settings_cam_combo = ttk.Combobox(
            webcam_box, textvariable=self.webcam_cam_var,
            values=["0", "1", "2", "3"],
            state="readonly", width=24, font=("Segoe UI", 9))
        self.settings_cam_combo.grid(row=1, column=1, sticky="w", pady=6)
        self.settings_cam_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_camera_selected())
        ttk.Button(webcam_box, text="Refresh Cameras", command=self._reconnect_to_default).grid(
            row=1, column=2, sticky="w", padx=6)

        ttk.Label(webcam_box, text="Cam Latency (ms)").grid(row=2, column=0, sticky="w", padx=6, pady=6)
        latency_entry = ttk.Entry(webcam_box, textvariable=self.webcam_latency_var, width=8)
        latency_entry.grid(row=2, column=1, sticky="w", pady=6)
        latency_entry.bind("<Return>", lambda _e: self._apply_webcam_latency())
        latency_entry.bind("<FocusOut>", lambda _e: self._apply_webcam_latency())
        ttk.Button(webcam_box, textvariable=self.webcam_start_btn_var,
                   command=self._toggle_webcam_ocr).grid(row=2, column=2, sticky="w", padx=6)

        webcam_status_frame = ttk.Frame(webcam_box)
        webcam_status_frame.grid(row=3, column=0, columnspan=4, sticky="w", padx=6, pady=(6, 4))
        ttk.Label(webcam_status_frame, textvariable=self.webcam_status_var).pack(side="left")
        ttk.Label(webcam_status_frame, text="  |  OCR:").pack(side="left", padx=(12, 2))
        ttk.Label(webcam_status_frame, textvariable=self.webcam_ms_var,
                  font=("Consolas", 10)).pack(side="left")
        ttk.Label(webcam_status_frame, text="  |  Reading:").pack(side="left", padx=(12, 2))
        ttk.Label(webcam_status_frame, textvariable=self.webcam_reading_var,
                  font=("Consolas", 12, "bold")).pack(side="left")
        ttk.Label(webcam_box, textvariable=self.sync_status_var).grid(
            row=4, column=0, columnspan=4, sticky="w", padx=6, pady=(0, 6))

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

        med_box = ttk.LabelFrame(root, text="Capture Median")
        med_box.pack(fill="x", **pad)
        ttk.Label(med_box, text="Half-window (seconds, default 0.5)").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(med_box, textvariable=self.manual_median_window_var, width=8).grid(row=0, column=1, sticky="w", pady=6)
        ttk.Label(med_box, text="Samples ±this many seconds around the keypress").grid(
            row=0, column=2, sticky="w", padx=(10, 6), pady=6)

        req_box = ttk.LabelFrame(root, text="Capture Requirements")
        req_box.pack(fill="x", **pad)
        ttk.Checkbutton(req_box, text="Require both HADI and OCR to capture", variable=self.require_both_var).grid(
            row=0, column=0, sticky="w", padx=6, pady=6)

        ac_box = ttk.LabelFrame(root, text="Experimental")
        ac_box.pack(fill="x", **pad)
        ttk.Checkbutton(ac_box, text="Enable auto-capture at target", variable=self.auto_capture_enabled,
                         command=self._reset_auto_capture_state).grid(row=0, column=0, columnspan=4, sticky="w", padx=6, pady=6)
        ttk.Label(ac_box, text="Dwell (seconds)").grid(row=1, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(ac_box, textvariable=self.auto_capture_dwell_var, width=8).grid(row=1, column=1, sticky="w", pady=6)
        ttk.Label(ac_box, text="Hold in range this long before capturing").grid(
            row=1, column=2, sticky="w", padx=(10, 6), pady=6)
        ttk.Checkbutton(ac_box, text="Voice announcements (announces target and capture)",
                         variable=self.auto_capture_voice_var).grid(row=2, column=0, columnspan=4, sticky="w", padx=6, pady=6)
        self.auto_capture_status_var = tk.StringVar(value="")

    def _build_load_cells_tab(self, root):
        pad = {"padx": 10, "pady": 8}

        top = ttk.LabelFrame(root, text="Saved Load Cells")
        top.pack(fill="x", **pad)

        ttk.Label(
            top, textvariable=self.in_use_cell_var, font=("Segoe UI", 10, "bold")
        ).grid(row=1, column=0, columnspan=6, sticky="w", pady=(8, 2))

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
        # Move cells between computers/versions: a small JSON file.
        ttk.Button(top, text="Export…", command=self._export_load_cells).grid(row=0, column=5, padx=(18, 6))
        ttk.Button(top, text="Import…", command=self._import_load_cells).grid(row=0, column=6, padx=6)

        # Packed (before the coefficients form) at the bottom of the tab, with a
        # clear gap above it, so it's set apart from editing the saved cells.
        bz = ttk.LabelFrame(root, text="Better Zero")
        bz.pack(side="bottom", fill="x", padx=10, pady=(40, 14))
        # Off every launch; while ticked, every load cell's B0 is treated as 0.
        bz_check = ttk.Checkbutton(bz, text="Better Zero", variable=self.zero_b0_var,
                                   command=self._on_zero_b0_toggle)
        bz_check.grid(row=0, column=0, sticky="w", padx=6, pady=6)
        bz_text = ttk.Label(bz, text="When on, whichever load cell is selected has its B0 set to 0 (tension and "
                                     "compression) until you uncheck this. Temporary: the saved coefficients "
                                     "above are not changed, and it's off again next time the app opens.",
                            wraplength=600)
        bz_text.grid(row=0, column=1, sticky="w", padx=(10, 6), pady=6)
        bz.columnconfigure(1, weight=1)
        # Wrap to whatever width the window has, so nothing is cut off.
        bz.bind("<Configure>", lambda e: bz_text.configure(
            wraplength=max(200, e.width - bz_check.winfo_reqwidth() - 50)))

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

    def _build_converter_tab(self, root):
        pad = {"padx": 10, "pady": 8}
        units = ["LBF", "KGF", "N", "kN", "gF", "t"]

        box = ttk.LabelFrame(root, text="Unit Converter")
        box.pack(fill="x", **pad)

        self.conv_input_var = tk.StringVar(value="")
        self.conv_from_var = tk.StringVar(value="LBF")
        self.conv_to_var = tk.StringVar(value="kN")

        ttk.Label(box, text="Value", font=("Segoe UI", 11)).grid(row=0, column=0, sticky="w", padx=6, pady=8)
        conv_entry = ttk.Entry(box, textvariable=self.conv_input_var, width=20, font=("Consolas", 14))
        conv_entry.grid(row=0, column=1, sticky="w", padx=6, pady=8)
        conv_entry.bind("<KeyRelease>", lambda _e: self._update_converter())

        ttk.Label(box, text="From").grid(row=0, column=2, sticky="w", padx=(20, 4), pady=8)
        from_combo = ttk.Combobox(box, textvariable=self.conv_from_var, values=units, state="readonly", width=8)
        from_combo.grid(row=0, column=3, sticky="w", pady=8)
        from_combo.bind("<<ComboboxSelected>>", lambda _e: self._update_converter())

        ttk.Label(box, text="→", font=("Segoe UI", 14)).grid(row=0, column=4, padx=10, pady=8)

        ttk.Label(box, text="To").grid(row=0, column=5, sticky="w", padx=(0, 4), pady=8)
        to_combo = ttk.Combobox(box, textvariable=self.conv_to_var, values=units, state="readonly", width=8)
        to_combo.grid(row=0, column=6, sticky="w", pady=8)
        to_combo.bind("<<ComboboxSelected>>", lambda _e: self._update_converter())

        self.conv_result_var = tk.StringVar(value="")
        ttk.Label(box, textvariable=self.conv_result_var, font=("Consolas", 20, "bold")).grid(
            row=1, column=0, columnspan=7, sticky="w", padx=6, pady=(4, 12))

        all_box = ttk.LabelFrame(root, text="All Conversions")
        all_box.pack(fill="x", **pad)
        self.conv_all_labels: dict[str, tk.StringVar] = {}
        for i, unit in enumerate(units):
            ttk.Label(all_box, text=f"{unit}:", font=("Segoe UI", 10, "bold")).grid(
                row=i, column=0, sticky="w", padx=6, pady=3)
            var = tk.StringVar(value="")
            self.conv_all_labels[unit] = var
            ttk.Label(all_box, textvariable=var, font=("Consolas", 12)).grid(
                row=i, column=1, sticky="w", padx=10, pady=3)

    def _update_converter(self):
        raw = self.conv_input_var.get().strip()
        if not raw:
            self.conv_result_var.set("")
            for v in self.conv_all_labels.values():
                v.set("")
            return
        try:
            value = float(raw)
        except ValueError:
            self.conv_result_var.set("Invalid number")
            for v in self.conv_all_labels.values():
                v.set("")
            return

        from_unit = self.conv_from_var.get()
        to_unit = self.conv_to_var.get()
        from_factor = HADI_UNIT_FACTORS.get(from_unit, 1.0)
        to_factor = HADI_UNIT_FACTORS.get(to_unit, 1.0)

        converted = value * (to_factor / from_factor)
        self.conv_result_var.set(f"{converted:.10g} {to_unit}")

        for unit, var in self.conv_all_labels.items():
            factor = HADI_UNIT_FACTORS.get(unit, 1.0)
            result = value * (factor / from_factor)
            var.set(f"{result:.10g}")

    def _build_report_tab(self, root):
        pad = {"padx": 10, "pady": 8}

        top = ttk.Frame(root)
        top.pack(fill="x", **pad)
        self.report_summary_var = tk.StringVar(value="No runs saved yet")
        ttk.Label(top, textvariable=self.report_summary_var, font=("Segoe UI", 11, "bold")).pack(side="left")
        ttk.Button(top, text="Export S/N", command=self._export_report).pack(side="right", padx=(10, 0))
        ttk.Label(top, text="S/N:").pack(side="right", padx=(10, 2))
        self.report_sn_var = tk.StringVar(value="")
        self.report_sn_combo = ttk.Combobox(top, textvariable=self.report_sn_var, state="readonly", width=14)
        self.report_sn_combo.pack(side="right")
        self.report_sn_combo.bind("<<ComboboxSelected>>", lambda _e: self._filter_report_by_sn())
        ttk.Button(top, text="Clear Report", command=self._clear_report).pack(side="right", padx=(0, 10))

        tree_frame = ttk.Frame(root)
        tree_frame.pack(fill="both", expand=True, **pad)

        cols = ("run", "mode", "sn", "load_cell", "points", "date")
        self.report_tree = ttk.Treeview(tree_frame, columns=cols, show="tree headings", height=8, selectmode="browse")
        self.report_tree.heading("#0", text="")
        self.report_tree.column("#0", width=30, stretch=False)
        self.report_tree.heading("run", text="Run")
        self.report_tree.column("run", width=60, anchor="center", stretch=False)
        self.report_tree.heading("mode", text="Mode")
        self.report_tree.column("mode", width=110, anchor="center", stretch=False)
        self.report_tree.heading("sn", text="S/N")
        self.report_tree.column("sn", width=100, anchor="center", stretch=False)
        self.report_tree.heading("load_cell", text="Load Cell")
        self.report_tree.column("load_cell", width=220, anchor="w")
        self.report_tree.heading("points", text="Points")
        self.report_tree.column("points", width=60, anchor="center", stretch=False)
        self.report_tree.heading("date", text="Date")
        self.report_tree.column("date", width=160, anchor="center")

        scrollbar = ttk.Scrollbar(tree_frame, orient="vertical", command=self.report_tree.yview)
        self.report_tree.configure(yscrollcommand=scrollbar.set)
        self.report_tree.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def _add_to_report(self):
        if not self._has_capture_data():
            return

        mode = self.mode_var.get()
        sn = self.serial_number_var.get().strip()
        load_cell = self.selected_load_cell_name.get()
        units = self.hadi_units_var.get()
        decimals = self.hadi_decimals_var.get()
        timestamp = datetime.now().isoformat(timespec="seconds")

        for run in (1, 2):
            run_data = []
            for i, point_row in enumerate(self.capture_rows):
                rr = point_row.get(f"run{run}")
                if rr and rr.get("ocr") not in (None, ""):
                    run_data.append(dict(rr))
            if not run_data:
                continue

            entry = {
                "run": run,
                "mode": mode,
                "sn": sn,
                "load_cell": load_cell,
                "units": units,
                "decimals": decimals,
                "timestamp": timestamp,
                "point_count": len(run_data),
                "rows": run_data,
            }
            self.report_entries.insert(0, entry)

        self._refresh_report_tree()

    def _refresh_report_tree(self):
        all_sns = list(dict.fromkeys(e.get("sn", "") or "(no S/N)" for e in self.report_entries))
        self.report_sn_combo["values"] = all_sns
        if all_sns and not self.report_sn_var.get():
            self.report_sn_var.set(all_sns[0])
        elif self.report_sn_var.get() not in all_sns:
            self.report_sn_var.set(all_sns[0] if all_sns else "")
        self._filter_report_by_sn()

    def _filter_report_by_sn(self):
        selected_sn = self.report_sn_var.get()
        self.report_tree.delete(*self.report_tree.get_children())

        filtered = [e for e in self.report_entries if (e.get("sn", "") or "(no S/N)") == selected_sn]

        for i, entry in enumerate(filtered):
            parent_id = self.report_tree.insert(
                "", "end",
                iid=f"report_{i}",
                text="+",
                values=(
                    f"R{entry['run']}",
                    entry["mode"],
                    entry.get("sn", ""),
                    entry["load_cell"],
                    str(entry["point_count"]),
                    entry["timestamp"],
                ),
            )
            for j, row in enumerate(entry["rows"]):
                hadi_text = row.get("hadi_text", "")
                ocr_text = self._ocr_text_for_row(row, show_plus=False)
                pct = ""
                if row.get("percent_error_na"):
                    pct = "NA"
                elif row.get("percent_error") is not None:
                    try:
                        pct = f"{float(row['percent_error']):+.2f}%"
                    except Exception:
                        pass
                self.report_tree.insert(
                    parent_id, "end",
                    iid=f"report_{i}_row_{j}",
                    text="",
                    values=(
                        f"P{row.get('point', j + 1)}",
                        "",
                        "",
                        f"HADI: {hadi_text}",
                        f"OCR: {ocr_text}",
                        pct,
                    ),
                )

        total = len(self.report_entries)
        shown = len(filtered)
        if total == 0:
            self.report_summary_var.set("No runs saved yet")
        else:
            self.report_summary_var.set(f"{shown} run{'s' if shown != 1 else ''} for S/N: {selected_sn}  ({total} total)")

    def _clear_report(self):
        if not self.report_entries:
            return
        if not messagebox.askyesno("Clear report?", "Remove all runs from the report?"):
            return
        self.report_entries.clear()
        self.report_sn_var.set("")
        self.report_sn_combo["values"] = []
        self.report_tree.delete(*self.report_tree.get_children())
        self.report_summary_var.set("No runs saved yet")

    def _export_report(self):
        selected_sn = self.report_sn_var.get()
        if not selected_sn:
            messagebox.showinfo("Nothing to export", "Select a S/N to export.")
            return

        entries = [e for e in self.report_entries if (e.get("sn", "") or "(no S/N)") == selected_sn]
        if not entries:
            messagebox.showinfo("Nothing to export", f"No runs for S/N: {selected_sn}")
            return

        sn_clean = selected_sn.replace(" ", "_").replace("/", "-")
        cap = self.customer_capacity_var.get().strip()
        units = self.hadi_units_var.get()
        parts = ["report"]
        if cap:
            parts.append(f"{cap}{units}")
        parts.append(sn_clean)
        parts.append(datetime.now().strftime("%y%m%d_%H%M"))
        default_name = "_".join(parts) + ".csv"

        path = filedialog.asksaveasfilename(
            title=f"Export Report — S/N: {selected_sn}",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return

        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)

                writer.writerow(["CALIBRATION REPORT"])
                writer.writerow(["S/N", selected_sn])
                load_cells = set(e["load_cell"] for e in entries if e.get("load_cell"))
                if load_cells:
                    writer.writerow(["Load Cell", ", ".join(load_cells)])
                units_set = set(e.get("units", "") for e in entries if e.get("units"))
                if units_set:
                    writer.writerow(["Units", ", ".join(units_set)])
                writer.writerow(["Exported", datetime.now().isoformat(timespec="seconds")])
                writer.writerow([])

                def _find_entry(mode, run):
                    for e in entries:
                        if e["mode"] == mode and e["run"] == run:
                            return e
                    return None

                slots = [
                    ("Tension", 1), ("Tension", 2),
                    ("Compression", 1), ("Compression", 2),
                ]
                sections = []
                for mode, run in slots:
                    entry = _find_entry(mode, run)
                    if entry is not None:
                        sections.append((mode, run, entry))

                if not sections:
                    writer.writerow(["No data"])
                else:
                    header = ["#"]
                    for mode, run, _entry in sections:
                        label = f"{mode} R{run}"
                        header.extend([f"{label} HADI", f"{label} OCR", f"{label} %"])
                    writer.writerow(header)

                    max_points = max(len(s[2]["rows"]) for s in sections)
                    for i in range(max_points):
                        csv_row = [str(i + 1)]
                        for _mode, _run, entry in sections:
                            if i < len(entry["rows"]):
                                row = entry["rows"][i]
                                hadi_text = row.get("hadi_text", "")
                                ocr_text = self._ocr_text_for_row(row, show_plus=False)
                                pct = ""
                                if row.get("percent_error_na"):
                                    pct = "NA"
                                elif row.get("percent_error") is not None:
                                    try:
                                        pct = f"{float(row['percent_error']):+.2f}%"
                                    except Exception:
                                        pass
                                csv_row.extend([hadi_text, ocr_text, pct])
                            else:
                                csv_row.extend(["", "", ""])
                        writer.writerow(csv_row)

            messagebox.showinfo("Exported", f"Report for S/N: {selected_sn} saved to:\n{path}")
        except Exception as exc:
            messagebox.showerror("Export failed", str(exc))

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

        ttk.Label(card, textvariable=var, style="BigValue.TLabel", width=14, anchor="w").grid(row=1, column=0, sticky="w")

    def _error_live_label(self, parent, title: str, var: tk.StringVar, row: int):
        card = ttk.Frame(parent)
        card.grid(row=row, column=0, sticky="ew", padx=12, pady=(14, 0))
        card.columnconfigure(0, weight=1)
        header = ttk.Frame(card)
        header.pack(anchor="w", fill="x")
        ttk.Label(header, text=title, style="LiveTitle.TLabel").pack(side="left")
        flip_cb = ttk.Checkbutton(header, text="Flip ±", variable=self.flip_sign_var, takefocus=False)
        flip_cb.pack(side="left", padx=(12, 0))
        self.live_error_label = ttk.Label(card, textvariable=var, style="ErrorValue.TLabel", width=14, anchor="w")
        self.live_error_label.pack(anchor="w")
        parent.columnconfigure(0, weight=1)

    def _small_label(self, parent, title: str, var: tk.StringVar, row: int):
        card = ttk.Frame(parent)
        card.grid(row=row, column=0, sticky="ew", padx=12, pady=(8, 0))
        ttk.Label(card, text=title, font=("Segoe UI", 10, "bold")).pack(anchor="w")
        ttk.Label(card, textvariable=var, style="SmallValue.TLabel").pack(anchor="w")
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

    # A row's OCR reading is kept in two layers once units start changing:
    #   basis    (ocr_original*): what the current display is derived from.
    #            Convert re-derives from it; Don't convert re-labels it.
    #   captured (ocr_cap*):      the reading as captured/typed. A units change
    #            never touches it; "Restore original" rebuilds from it.
    _OCR_BASIS_KEYS = ("ocr_original", "ocr_text_original", "ocr_original_units")
    _OCR_CAPTURED_KEYS = ("ocr_cap", "ocr_text_cap", "ocr_cap_units")
    # Cleared when a reading is edited or recaptured: the new value is the
    # new "as captured" in whatever units are showing.
    _OCR_ORIGINAL_KEYS = _OCR_BASIS_KEYS + _OCR_CAPTURED_KEYS

    def _on_hadi_units_selected(self):
        """Convert captured OCR values along with HADI to the new units.

        Both columns scale by the same factor, so % error is unchanged. The
        reading as captured is set aside on the row, and switching back to
        those units restores it exactly — the conversion never loses the
        original. mV/V isn't a force unit, so OCR isn't converted to/from it.
        """
        new_units = self.hadi_units_var.get()
        old_units = self._applied_hadi_units
        if new_units == old_units:
            return
        has_data = self._has_capture_data()
        old_capacity = self.customer_capacity_var.get().strip()
        if not has_data and not old_capacity:
            # Nothing captured or typed: nothing to restore later either.
            self._units_original = None
            self._capacity_captured = None
        convert = True
        if has_data or old_capacity:
            # Once units have been changed, Restore is offered in every units
            # dialog, whichever units were picked: it always goes back to the
            # original units with readings as captured and Capacity as typed.
            restore_units = self._units_original
            if self._units_original is None:
                self._units_original = old_units
            choice = self._ask_units_choice(old_units, new_units, restore_units)
            if choice is None:
                self.hadi_units_var.set(old_units)
                return
            if choice == "restore":
                self._restore_original(old_units)
                return
            convert = choice == "convert"
        self._applied_hadi_units = new_units
        rows = self._rows_with_ocr()
        # Remember what was captured/typed before anything changes it.
        self._record_capacity_captured(old_units)
        for row in rows:
            self._record_row_captured(row, old_units)
        if convert:
            capacity_text, capacity_state = self._capacity_in_units(old_units, new_units)
            if old_capacity:
                self._capacity_state = capacity_state
                self.customer_capacity_var.set(capacity_text)
            for row in rows:
                self._convert_row_ocr_units(row, old_units, new_units)
            toast = f"Converted to {new_units}"
            if old_capacity and capacity_text != old_capacity:
                toast += f" — Capacity {old_capacity} {old_units} → {capacity_text} {new_units}"
        else:
            # The numbers on screen were really in the new units all along
            # (e.g. Units was set wrong while capturing): keep them, and
            # re-judge everything against the HADI in the new units. Only the
            # basis is re-labelled; "as captured" is kept for Restore.
            self._capacity_state = ({"text": old_capacity, "units": new_units, "shown": old_capacity}
                                    if old_capacity else None)
            for row in rows:
                row["ocr_original"] = row["ocr"]
                row["ocr_text_original"] = row.get("ocr_text", "")
                row["ocr_original_units"] = new_units
                self._recalculate_row_error(row)
            if old_capacity:
                self._regenerate_targets()
            toast = f"Units now {new_units} — numbers kept, % error recalculated"
            if old_capacity:
                toast = f"Units now {new_units} — Capacity kept at {old_capacity} {new_units}, targets updated"
        self._set_hadi_display_options()
        if has_data or old_capacity:
            self._show_capacity_toast(toast)
        if rows:
            self._mark_data_changed()

    def _rows_with_ocr(self) -> list:
        return [point_row[k] for point_row in self.capture_rows for k in ("run1", "run2")
                if point_row.get(k) and point_row[k].get("ocr") not in (None, "")]

    def _record_row_captured(self, row: dict, units: str) -> None:
        """First units change a reading goes through: remember it as captured."""
        if "ocr_cap" not in row:
            row["ocr_cap"] = row["ocr"]
            row["ocr_text_cap"] = row.get("ocr_text", "")
            row["ocr_cap_units"] = units

    def _record_capacity_captured(self, units: str) -> None:
        """Remember the Capacity as typed. If the box was edited since the last
        units change, the new text is the new 'as typed' value (in `units`)."""
        text = self.customer_capacity_var.get().strip()
        if not text:
            return
        state = self._capacity_state
        edited = state is None or state.get("shown") != text
        if self._capacity_captured is None or edited:
            self._capacity_captured = {"text": text, "units": units}

    def _restore_original(self, current_units: str) -> None:
        """Put every reading back as captured and the Capacity back as typed,
        in the units used before the first units change. Points captured
        since then are kept, converted to those units (same force)."""
        orig = self._units_original
        self._record_capacity_captured(current_units)
        for row in self._rows_with_ocr():
            self._record_row_captured(row, current_units)   # captured since a change
            row["ocr_original"] = row["ocr_cap"]
            row["ocr_text_original"] = row["ocr_text_cap"]
            row["ocr_original_units"] = row["ocr_cap_units"]
            self._convert_row_ocr_units(row, current_units, orig)
        self._applied_hadi_units = orig
        self.hadi_units_var.set(orig)
        for row in self._rows_with_ocr():
            self._recalculate_row_error(row)
        cap = self._capacity_captured
        if cap is not None:
            self._capacity_state = dict(cap, shown=self.customer_capacity_var.get().strip())
            text, state = self._capacity_in_units(current_units, orig)
            self._capacity_state = state
            self.customer_capacity_var.set(text)
            self._regenerate_targets()
        self._set_hadi_display_options()
        self._show_capacity_toast(f"Restored — readings and Capacity back to the original {orig} values")
        self._mark_data_changed()

    def _ask_units_choice(self, old_units: str, new_units: str, restore_units: Optional[str]):
        """Units-change dialog. Returns 'restore', 'convert', 'keep' or None
        (cancel). A custom dialog because it can have four choices."""
        dlg = tk.Toplevel(self)
        dlg.title(f"Change units to {new_units}?")
        dlg.transient(self)
        dlg.resizable(False, False)
        result = {"value": None}

        def choose(value):
            result["value"] = value
            dlg.destroy()

        ttk.Label(dlg, text="Units should match what the device's display shows.",
                  padding=(14, 12, 14, 6)).pack(anchor="w")

        def option(label, desc, value):
            row = ttk.Frame(dlg, padding=(14, 4))
            row.pack(fill="x")
            ttk.Button(row, text=label, width=20, command=lambda: choose(value)).pack(side="left")
            ttk.Label(row, text=desc, wraplength=380).pack(side="left", padx=(10, 0))

        if restore_units:
            option(f"Restore original ({restore_units})",
                   f"back to {restore_units}: every reading as captured and Capacity as typed, "
                   "exactly as before any units change", "restore")
        option(f"Convert to {new_units}", "same force: readings, Capacity and targets convert; "
                                          "% error unchanged", "convert")
        option("Don't convert", f"numbers stay as they are and are used as {new_units}; "
                                "HADI and % error are recalculated", "keep")
        option("Cancel", f"keep {old_units}", None)
        ttk.Frame(dlg, height=8).pack()
        dlg.bind("<Escape>", lambda _e: choose(None))
        dlg.protocol("WM_DELETE_WINDOW", lambda: choose(None))
        dlg.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - dlg.winfo_width()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - dlg.winfo_height()) // 3
        dlg.geometry(f"+{max(0, x)}+{max(0, y)}")
        dlg.grab_set()
        dlg.focus_set()
        self.wait_window(dlg)
        return result["value"]

    def _show_capacity_toast(self, text: str, ms: int = 2500) -> None:
        """Briefly show what a units change did, just above the Capacity box."""
        toast = getattr(self, "_capacity_toast", None)
        if toast is None:
            toast = self._capacity_toast = tk.Label(
                self, bg="#1a5fb4", fg="white", font=("Segoe UI", 9, "bold"), padx=8, pady=3)
        toast.configure(text=text)
        toast.place(in_=self._capacity_entry, relx=0.0, rely=0.0, anchor="sw", y=-4)
        toast.lift()
        if getattr(self, "_capacity_toast_after", None):
            self.after_cancel(self._capacity_toast_after)
        self._capacity_toast_after = self.after(ms, toast.place_forget)

    def _convert_row_ocr_units(self, row: dict, old_units: str, new_units: str) -> bool:
        if "ocr_original" not in row:
            row["ocr_original"] = row["ocr"]
            row["ocr_text_original"] = row.get("ocr_text", "")
            row["ocr_original_units"] = old_units
        orig_units = row["ocr_original_units"]
        if new_units == orig_units:
            row["ocr"] = row["ocr_original"]
            row["ocr_text"] = row["ocr_text_original"]
            return True
        f_from = HADI_UNIT_FACTORS.get(orig_units)
        f_to = HADI_UNIT_FACTORS.get(new_units)
        if f_from is None or f_to is None:
            # mV/V on either side: show the reading as captured.
            row["ocr"] = row["ocr_original"]
            row["ocr_text"] = row["ocr_text_original"]
            return True
        try:
            value = float(row["ocr_original"]) / f_from * f_to
        except (TypeError, ValueError):
            return False
        # Keep the display's resolution: 987.65 lbf -> 4393.29 N, 4.39329 kN.
        ratio = f_to / f_from
        decimals = self._decimal_places(row["ocr_text_original"]) + math.ceil(round(-math.log10(ratio), 9))
        row["ocr"] = value
        row["ocr_text"] = self._fixed_decimals(value, max(0, decimals))
        return True

    def _capacity_in_units(self, old_units: str, new_units: str):
        """The Capacity field converted to `new_units` (same physical force).

        Returns (text, state). Converts from the value as originally typed,
        so 1000 LBF -> 4448.22 N -> 1000 LBF comes back exactly; if the field
        was edited since the last conversion, what's there now is the new
        original. mV/V isn't a force unit, so it's left as typed.
        """
        text = self.customer_capacity_var.get().strip()
        state = getattr(self, "_capacity_state", None)
        if state is None or state["shown"] != text:
            state = {"text": text, "units": old_units}
        if not text:
            return "", None
        if new_units == state["units"]:
            # Keep the state (not None): its units may differ from the
            # as-typed value after a Don't convert, and Restore needs that.
            return state["text"], dict(state, shown=state["text"])
        f_from = HADI_UNIT_FACTORS.get(state["units"])
        f_to = HADI_UNIT_FACTORS.get(new_units)
        try:
            value = float(state["text"]) / f_from * f_to
        except (TypeError, ValueError):
            # mV/V on either side (or not a number): leave it as typed, but
            # remember its real units so LBF -> mV/V -> N still converts.
            return text, dict(state, shown=text)
        new_text = f"{value:.6g}"
        return new_text, dict(state, shown=new_text)

    def _carry_ocr_original(self, old_row: dict, new_row: dict) -> None:
        """Keep the as-captured OCR reading when a row is rebuilt (W rows)."""
        if "ocr_original" in old_row and new_row.get("ocr") not in (None, ""):
            for key in self._OCR_ORIGINAL_KEYS:
                new_row[key] = old_row[key]

    def _set_hadi_display_options(self):
        units = self.hadi_units_var.get()
        self.hadi_title_var.set("HADI Response (mV/V)" if units == "mV/V" else f"HADI Force ({units})")
        if hasattr(self, "capacity_label_var"):
            self.capacity_label_var.set(f"Capacity ({units}):")
        self._refresh_hadi_mode_badge()
        if self.latest_hadi:
            self.hadi_lbf_var.set(self._format_hadi_display_value(self.latest_hadi))
            self._update_capacity_warning(self.latest_hadi)
        else:
            self._update_capacity_warning(None)
        self._update_target_force_display()
        if hasattr(self, "_ac_units_var"):
            self._ac_units_var.set(units)
        if hasattr(self, "run1_tree"):
            self._redraw_point_table()

    def _refresh_hadi_mode_badge(self):
        mode = self.mode_var.get()
        self.mode_badge_var.set(f"↓ {mode.upper()}" if mode == "Compression" else f"↑ {mode.upper()}")
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
            return f"{float(value):+.10g}"
        except Exception:
            text = str(value).strip()
            return text if text.startswith(("+", "-")) else f"+{text}"

    @staticmethod
    def _decimal_places(text) -> int:
        """Digits shown after the decimal point in a numeric string.

        Preserves the instrument's displayed precision (e.g. '0.0000' -> 4)
        so a value can be reformatted after negation/median without losing
        trailing zeros the way '%g' would (0.0000 -> 0, 1.50 -> 1.5).
        """
        if text in (None, ""):
            return 0
        parts = str(text).strip().split(".", 1)
        if len(parts) < 2:
            return 0
        digits = ""
        for ch in parts[1]:
            if ch.isdigit():
                digits += ch
            else:
                break
        return len(digits)

    @staticmethod
    def _fixed_decimals(value, decimals) -> str:
        """Format value with exactly `decimals` places, avoiding '-0.0000'."""
        if value == 0:
            value = 0.0
        return f"{value:.{decimals}f}"

    def _format_ocr_text(self, raw_text, value=None, show_plus: bool = True) -> str:
        text = "" if raw_text in (None, "") else str(raw_text).strip()
        if not text and value not in (None, ""):
            text = f"{float(value):.10g}"
        if not text:
            return ""
        if show_plus and not text.startswith(("+", "-")):
            return "+" + text
        if not show_plus and text.startswith("+"):
            return text[1:]
        return text

    def _ocr_text_for_row(self, run_row, show_plus: bool = True) -> str:
        if not run_row or run_row.get("ocr") in (None, ""):
            return ""
        return self._format_ocr_text(run_row.get("ocr_text"), run_row.get("ocr"), show_plus=show_plus)

    def _hadi_lbf_to_display(self, lbf_value: float, raw_response: float = None) -> float:
        units = self.hadi_units_var.get()
        if units == "mV/V":
            return raw_response if raw_response is not None else lbf_value
        return lbf_value * HADI_UNIT_FACTORS.get(units, 1.0)

    def _display_to_lbf(self, display_value: float) -> float:
        units = self.hadi_units_var.get()
        if units == "mV/V":
            return display_value
        return display_value / HADI_UNIT_FACTORS.get(units, 1.0)

    def _format_hadi_display_value(self, reading: HADIReading) -> str:
        units = self.hadi_units_var.get()
        decimals = self._selected_hadi_decimals()
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
        decimals = self._selected_hadi_decimals()
        return f"{force_lbf * factor:.{decimals}f} {units}"

    def _selected_hadi_decimals(self, ocr_text=None) -> int:
        """Number of decimal places for HADI display/capture output.

        Dec = Auto shows the HADI with one more decimal than the OCR reading:
        a captured row uses its own OCR text, the live readout the latest OCR
        reading (or the last one seen if the camera has lost the number).
        Never more than 5 decimals, and 5 before any OCR reading.
        """
        step = self.hadi_decimals_var.get()
        if step != "Auto":
            return _decimals_from_step(step)
        if ocr_text not in (None, ""):
            ocr_decimals = self._decimal_places(ocr_text)
        else:
            ocr_decimals = self._last_ocr_decimals
            if ocr_decimals is None:
                return self.HADI_MAX_DECIMALS
        return min(ocr_decimals + 1, self.HADI_MAX_DECIMALS)

    # The finest HADI resolution offered (Dec 0.00001); Auto never goes past it.
    HADI_MAX_DECIMALS = 5

    def _format_hadi_lbf_text(self, value, show_plus: bool = False, ocr_text=None) -> str:
        """Format captured HADI LBF with exactly the selected decimal count.

        Capture rows keep this text so the table, autosave, and manual CSV export
        all preserve the same decimal precision the operator selected at capture
        time instead of later shortening/reformatting the float. `ocr_text` is
        the row's OCR reading, which sets the precision when Dec = Auto.
        """
        if value in (None, ""):
            return ""
        try:
            decimals = self._selected_hadi_decimals(ocr_text)
            sign = "+" if show_plus else ""
            v = float(value)
            return f"{v:{sign}.{decimals}f}"
        except Exception:
            return str(value)

    def _set_row_hadi_text(self, row: dict) -> None:
        """Store the fixed-decimal HADI text used by table and CSV export."""
        if row is not None:
            hadi_lbf = row.get("hadi_lbf")
            if hadi_lbf not in (None, ""):
                try:
                    _raw = row.get("hadi_raw")
                    _raw_f = float(_raw) if _raw not in (None, "") else None
                    display_val = self._hadi_lbf_to_display(float(hadi_lbf), raw_response=_raw_f)
                    row["hadi_text"] = self._format_hadi_lbf_text(
                        display_val, show_plus=False, ocr_text=row.get("ocr_text"))
                except (TypeError, ValueError):
                    row["hadi_text"] = ""
            else:
                row["hadi_text"] = ""

    def _update_capacity_warning(self, reading: Optional[HADIReading] = None) -> None:
        """Check HADI load cell overload (105% of cell capacity in lbf)."""
        capacity_lbf = self._capacity_limit_lbf()
        if capacity_lbf is None or reading is None:
            self._hadi_overload = False
            self._hadi_overload_text = ""
            return
        force_lbf = abs(float(reading.force_lbf))
        limit_lbf = capacity_lbf * 1.05
        if force_lbf >= limit_lbf:
            self._hadi_overload = True
            self._hadi_overload_text = (
                f"HADI OVERLOAD  {self._format_force_for_current_units(force_lbf)} / "
                f"{self._format_force_for_current_units(limit_lbf)} max"
            )
        else:
            self._hadi_overload = False
            self._hadi_overload_text = ""

    def _update_device_overload(self) -> None:
        """Device under test past 105% of its Capacity.

        Judged from the HADI force: the reference cell and the device are in
        series, so they carry the same force, and the HADI value is exact.
        Units, a camera misread, a missed decimal point, or a lost/frozen
        display can't hide an overload or fake one. Only when there's no live
        HADI reading (e.g. weight rows) does it fall back to the camera value.
        """
        self._device_overload = False
        try:
            capacity = float(self.customer_capacity_var.get().strip())
        except ValueError:
            return
        if capacity <= 0:
            return
        limit = capacity * 1.05
        units = self.hadi_units_var.get()
        reading = self.latest_hadi          # None once HADI readings go stale
        if reading is not None and self.selected_load_cell_name.get():
            if units == "mV/V":
                self._device_overload = abs(reading.raw_response) >= limit
            else:
                self._device_overload = abs(reading.force_lbf) >= self._display_to_lbf(limit)
            return
        with self.ocr_lock:
            ocr = self.latest_ocr
        if ocr is not None:
            self._device_overload = abs(float(ocr.value)) >= limit

    def _update_overload_badge(self) -> None:
        parts = []
        if getattr(self, "_hadi_overload", False):
            parts.append("HADI OVERLOAD")
        if getattr(self, "_device_overload", False):
            parts.append("DEVICE OVERLOAD")

        if parts:
            self.overload_badge_var.set("  |  ".join(parts))
            # Sit to the left of the mode badge, on the same row: anchor the
            # banner's right edge to the mode bar's left edge so it grows into
            # the open space without covering the Tension/Compression badge.
            self.overload_badge.place(in_=self._mode_bar, relx=0.0, rely=0.5, anchor="e", x=-10)
            self.overload_badge.lift()
            if self._overload_flash_id is None:
                self._overload_flash_tick()
        else:
            self.overload_badge_var.set("")
            self.overload_badge.place_forget()
            if self._overload_flash_id is not None:
                self.after_cancel(self._overload_flash_id)
                self._overload_flash_id = None
            self._overload_flash_visible = True

    def _overload_flash_tick(self) -> None:
        self._overload_flash_visible = not self._overload_flash_visible
        style = "OverloadBadge.TLabel" if self._overload_flash_visible else "OverloadBadgeHidden.TLabel"
        self.overload_badge.configure(style=style)
        self._overload_flash_id = self.after(500, self._overload_flash_tick)

    def _label_for_camera_name(self, name: str) -> Optional[str]:
        for value in self.settings_cam_combo["values"]:
            if value.split(":", 1)[-1].strip() == name:
                return value
        return None

    def _auto_connect_on_launch(self, attempt: int = 1):
        if WebcamOCR is not None and _HAS_WEBCAM_DEPS:
            saved = self._camera_prefs.get("camera_name")
            preferred = saved or DEFAULT_CAMERA_NAME
            label = self._label_for_camera_name(preferred)
            available = list(self.settings_cam_combo["values"])
            if label is None and not saved and attempt >= 3:
                # First run on a new computer: nobody has chosen a camera yet,
                # so there's no choice to override. One camera -> use it;
                # several -> ask, rather than guess (it could be the laptop's
                # built-in camera pointing at the operator).
                if len(available) == 1:
                    label = available[0]
                else:
                    self.webcam_status_var.set(
                        "Choose the camera pointed at the display: Settings → Webcam OCR → Camera"
                        if available else "No camera found — plug one in, then press Refresh Cameras.")
                    self._kick_auto_scan()
                    return
            if label is not None:
                self.webcam_cam_var.set(label)
                # Quiet on launch: a busy camera shows in the status label
                # instead of popping an error dialog before the UI settles.
                self._start_webcam_ocr(show_error=False)
            elif attempt < 3:
                # USB cameras can enumerate late after boot/replug, so rescan
                # before giving up on the saved default.
                self.webcam_status_var.set(
                    f"Looking for camera '{preferred}' (scan {attempt + 1}/3)…" if saved
                    else f"Looking for cameras (scan {attempt + 1}/3)…")
                self.after(3000, lambda: self._refresh_camera_list(
                    then=lambda: self._auto_connect_on_launch(attempt + 1)))
            else:
                # Never silently connect to a different camera than the saved
                # default — that reads as "it ignored my selection".
                self.webcam_status_var.set(
                    f"Default camera '{preferred}' not found — check its USB "
                    "connection, then press Refresh, or pick a camera in Settings.")
        elif not self.ocr:
            self._start_ocr(show_error=False)
        self._kick_auto_scan()

    def _set_hadi_light(self, color: str):
        self.hadi_status_light.itemconfig(self._hadi_light_id, fill=color)

    def _update_hadi_light(self, now_pc: float):
        """HADI light: flashing red while trying to connect, yellow when
        connected but no data yet, green while readings are flowing."""
        if not self.hadi.is_connected():
            # A disconnect ends any yellow spell; time spent disconnected
            # must not be reported as yellow on the next reading.
            self._hadi_went_stale = None
            if self._auto_scan_active:
                flash_on = int(now_pc / 0.4) % 2 == 0
                self._set_hadi_light("#cc0000" if flash_on else "#4d0000")
            else:
                self._set_hadi_light("#cc0000")
        elif self._hadi_is_fresh(now_pc):
            if self._hadi_went_stale is not None:
                log_freeze(f"HADI readings resumed after "
                           f"{now_pc - self._hadi_went_stale:.1f} s yellow")
                self._hadi_went_stale = None
            self._set_hadi_light("#00aa00")
        else:
            if self._hadi_went_stale is None and self.hadi_last_pc_time is not None:
                self._hadi_went_stale = now_pc
                log_freeze(f"HADI yellow: connected but no reading for "
                           f"{now_pc - self.hadi_last_pc_time:.1f} s")
            self._set_hadi_light("#ccaa00")

    def _set_ocr_light(self, color: str):
        self.ocr_status_light.itemconfig(self._ocr_light_id, fill=color)

    def _on_ocr_heartbeat(self):
        self._ocr_last_packet_pc = time.perf_counter()

    def _update_ocr_light(self, now_pc: float):
        if not self.ocr and not self.webcam_ocr:
            self._set_ocr_light("#cc0000")
            return
        stale = 3.0
        reading_gap = 1.5
        last_reading = self.ocr_last_pc_time
        last_packet = self._ocr_last_packet_pc
        has_reading = last_reading is not None and (now_pc - last_reading) <= reading_gap
        has_packet = last_packet is not None and (now_pc - last_packet) <= stale
        if has_reading:
            self._set_ocr_light("#00aa00")
        elif has_packet:
            self._set_ocr_light("#ccaa00")
        else:
            self._set_ocr_light("#cc0000")

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
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = 19200
        self._scan_thread = threading.Thread(
            target=self._background_scan, args=(baud,), daemon=True)
        self._scan_thread.start()

    def _background_scan(self, baudrate: int):
        if list_ports is None:
            return
        # The port that just errored may still be closing; probing it before
        # that finishes fails with "Access is denied" and delays reconnect.
        self.hadi.wait_closed(3.0)
        ports = [p.device for p in list_ports.comports()]
        if self._last_good_port and self._last_good_port in ports:
            ports.remove(self._last_good_port)
            ports.insert(0, self._last_good_port)
        for port in ports:
            if self.hadi.is_connected():
                return
            if HADIWorker.probe_port(port, baudrate):
                self.after(0, lambda p=port, ps=ports: self._auto_connect_port(p, ps))
                return
        self.after(0, self._on_scan_failed)

    def _on_scan_failed(self):
        self.status_var.set("HADI: no device found, retrying...")

    def _auto_connect_port(self, port: str, ports: list[str]):
        if self.hadi.is_connected():
            return
        self.port_var.set(port)
        # Reuse the scan thread's port list: enumerating COM ports here
        # (list_ports.comports) can take hundreds of ms on the Tk thread.
        self.port_combo["values"] = ports
        try:
            self.hadi.connect(port, int(self.baud_var.get()))
            self._select_load_cell()
            self._set_mode()
            self._apply_poll_rate()
            self.hadi_button_var.set("Disconnect HADI")
            self._last_good_port = port
            self.status_var.set(f"HADI connected on {port}")
            log_freeze(f"HADI auto-connected on {port}")
        except Exception:
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
            self.status_var.set(f"HADI connected on {self.port_var.get()}")
        except Exception as exc:
            self.status_var.set(f"HADI waiting: {exc}")
            if show_error:
                messagebox.showerror("HADI connection failed", str(exc))

    def _disconnect_hadi(self):
        self.hadi.disconnect(wait=False)
        self.hadi_button_var.set("Connect HADI")
        self.status_var.set("HADI disconnected")
        self.latest_hadi = None
        self.hadi_last_pc_time = None
        self.hadi_lbf_var.set("---")
        self.hadi_raw_var.set("WAITING")
        self.raw_text_var.set("")
        self._update_live_percent_error()

    def _start_ocr(self, show_error=True):
        try:
            self._stop_ocr()
            self.ocr = OCRReceiver(port=int(self.ocr_port_var.get()))
            self.ocr.on_reading = self._on_ocr_reading
            self.ocr.on_gps = self._on_gps_fix
            self.ocr.on_heartbeat = self._on_ocr_heartbeat
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
        self._ocr_last_packet_pc = None
        self.ocr_value_var.set("---")
        self._set_ocr_light("#cc0000")
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
            self.hadi.load_cell = self._effective_load_cell(cell)
            zeroed = " (Better Zero on: B0 = 0)" if self.zero_b0_var.get() else ""
            self.status_var.set(f"Using load cell: {cell['name']}{zeroed}")
            self._update_capacity_warning(self.latest_hadi)

    def _effective_load_cell(self, cell: dict) -> dict:
        """The coefficients the HADI force is computed with: the saved cell, or
        with "Better Zero" ticked, a temporary copy with B0 = 0 on both sides.
        The saved cell (and load_cells.json) is never changed."""
        if not self.zero_b0_var.get():
            return cell
        temp = dict(cell)
        for side in ("compression", "tension"):
            temp[side] = dict(cell.get(side) or {}, B0=0.0)
        return temp

    def _on_zero_b0_toggle(self):
        if self.selected_load_cell_name.get():
            self._select_load_cell()
        else:
            self.status_var.set("Better Zero on: B0 = 0 for any load cell selected (temporary)"
                                if self.zero_b0_var.get() else "Better Zero off: B0 back to the saved values")

    def _sync_load_cell_controls(self):
        names = [c["name"] for c in self.load_cells]
        if hasattr(self, "load_cell_combo"):
            self.load_cell_combo["values"] = names
        if hasattr(self, "editor_combo"):
            self.editor_combo["values"] = names

        if names:
            current = self.selected_load_cell_name.get()
            if current and current not in names:
                self.selected_load_cell_name.set("")
            if not self.editor_select_var.get() or self.editor_select_var.get() not in names:
                self.editor_select_var.set(names[0])
            if self.selected_load_cell_name.get():
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

    def _on_tab_changed(self, _event=None):
        try:
            current = self.notebook.nametowidget(self.notebook.select())
        except Exception:
            return
        if current is self.load_cells_tab:
            self._show_active_load_cell()

    def _show_active_load_cell(self):
        """Opening the Load Cells tab previews the cell currently in use on the
        Capture tab (or the first saved cell when none is selected) so its
        coefficients are visible at a glance."""
        names = [c["name"] for c in self.load_cells]
        if not names:
            self.in_use_cell_var.set("No load cells saved yet — create one below.")
            return
        # Don't clobber a new, unsaved cell the user is in the middle of typing.
        if not self.editor_select_var.get() and self.cell_name_var.get().strip():
            return
        active = self.selected_load_cell_name.get()
        if active and active in names:
            self.in_use_cell_var.set(f"Currently using:  {active}")
        else:
            active = names[0]
            self.in_use_cell_var.set(f"Currently using:  (none selected) — showing {active}")
        self.editor_select_var.set(active)
        self._load_cell_into_editor()

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
        # Calibration coefficients can't be recovered once deleted, so ask
        # (default button: No).
        if not messagebox.askyesno(
                "Delete load cell?",
                f"Delete the load cell '{name}' and its calibration coefficients?\n\n"
                "This can't be undone. Tip: Export… it first if you might need it again.",
                icon="warning", default="no"):
            return
        self.load_cells = [c for c in self.load_cells if c["name"] != name]
        save_load_cells(self.load_cells)
        self.selected_load_cell_name.set(self.load_cells[0]["name"])
        self.editor_select_var.set(self.load_cells[0]["name"])
        self._sync_load_cell_controls()

    def _export_load_cells(self):
        """Save load cells to a small JSON file (same format as load_cells.json)."""
        if not self.load_cells:
            messagebox.showinfo("Export load cells", "There are no load cells to export.")
            return
        selected = self.editor_select_var.get()
        cells = self.load_cells
        if selected and len(self.load_cells) > 1:
            choice = messagebox.askyesnocancel(
                "Export load cells",
                f"Export all {len(self.load_cells)} load cells?\n\n"
                f"Yes — all {len(self.load_cells)}\nNo — only '{selected}'\nCancel — don't export")
            if choice is None:
                return
            if choice is False:
                cells = [c for c in self.load_cells if c["name"] == selected]
        if len(cells) == 1:
            stem = re.sub(r'[\\/:*?"<>|]+', "_", cells[0]["name"]).strip() or "load_cell"
        else:
            stem = f"load_cells_{datetime.now():%y%m%d}"
        path = filedialog.asksaveasfilename(
            title="Export load cells", defaultextension=".json", initialfile=f"{stem}.json",
            filetypes=[("Load cell file", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            Path(path).write_text(json.dumps(cells, indent=2), encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("Export failed", str(exc))
            return
        messagebox.showinfo("Exported", f"Exported {len(cells)} load cell{'s' if len(cells) != 1 else ''} to:\n{path}")

    @staticmethod
    def _normalize_load_cell(raw) -> dict:
        """Validate one imported load cell; raise ValueError if it isn't one."""
        if not isinstance(raw, dict):
            raise ValueError("not a load cell entry")
        name = str(raw.get("name", "")).strip()
        if not name:
            raise ValueError("missing name")
        capacity = raw.get("capacity_lbf")
        if capacity not in (None, ""):
            capacity = float(capacity)
            if not math.isfinite(capacity) or capacity <= 0:
                raise ValueError(f"'{name}': capacity must be a positive number")
        else:
            capacity = None
        cell = {"name": name, "capacity_lbf": capacity}
        for side in ("compression", "tension"):
            coeffs = raw.get(side) or {}
            if not isinstance(coeffs, dict):
                raise ValueError(f"'{name}': {side} coefficients are not valid")
            cell[side] = {}
            for key in ("B0", "B1", "B2", "B3", "B4", "B5"):
                value = float(coeffs.get(key, 0.0) or 0.0)
                if not math.isfinite(value):
                    raise ValueError(f"'{name}': {side} {key} is not a number")
                cell[side][key] = value
        if not any(cell["compression"].values()) and not any(cell["tension"].values()):
            raise ValueError(f"'{name}': no coefficients")
        return cell

    def _has_identical_load_cell(self, cell: dict) -> bool:
        existing = self._find_load_cell(cell["name"])
        if existing is None:
            return False
        try:
            return self._normalize_load_cell(existing) == cell
        except (ValueError, TypeError):
            return False   # an incomplete cell here can't be identical

    def _marked_import_copy(self, cell: dict) -> Optional[dict]:
        """'<name> (IMPORTED - DIFFERENT)', or '... DIFFERENT 2)' etc. if that
        name is taken by something else. None if an identical marked copy is
        already here."""
        n = 1
        while True:
            suffix = " (IMPORTED - DIFFERENT)" if n == 1 else f" (IMPORTED - DIFFERENT {n})"
            candidate = dict(cell, name=cell["name"] + suffix)
            if self._find_load_cell(candidate["name"]) is None:
                return candidate
            if self._has_identical_load_cell(candidate):
                return None
            n += 1

    def _import_load_cells(self):
        """Add load cells from an exported file. Cells already here with the
        same name and identical numbers are skipped; a cell with the same name
        but different numbers is added as '<name> (IMPORTED - DIFFERENT)', so
        existing calibration coefficients are never overwritten."""
        path = filedialog.askopenfilename(
            title="Import load cells",
            filetypes=[("Load cell file", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            messagebox.showerror("Import failed", f"Couldn't read that file as a load cell file.\n\n{exc}")
            return
        if isinstance(data, dict):
            data = data.get("load_cells", [data])
        if not isinstance(data, list):
            messagebox.showerror("Import failed", "That file doesn't contain load cells.")
            return
        cells, problems = [], []
        for raw in data:
            try:
                cells.append(self._normalize_load_cell(raw))
            except (ValueError, TypeError) as exc:
                problems.append(str(exc))
        if not cells:
            messagebox.showerror("Import failed", "No valid load cells found."
                                 + ("\n\n" + "\n".join(problems[:5]) if problems else ""))
            return

        new, same, renamed = [], [], []
        for cell in cells:
            if self._has_identical_load_cell(cell):
                same.append(cell)
            elif self._find_load_cell(cell["name"]) is None:
                self.load_cells.append(cell)
                new.append(cell)
            else:
                # Same name, different numbers: never overwrite calibration
                # coefficients. Add it alongside as a clearly marked copy.
                marked = self._marked_import_copy(cell)
                if marked is None:
                    same.append(cell)          # that exact copy was imported before
                else:
                    self.load_cells.append(marked)
                    renamed.append(marked)

        save_load_cells(self.load_cells)
        changed = new + renamed
        if changed:
            self.editor_select_var.set(changed[0]["name"])
        self._sync_load_cell_controls()

        lines = [f"Added {len(new)} new load cell{'s' if len(new) != 1 else ''}."]
        if renamed:
            lines.append(f"{len(renamed)} had the same name as one of yours but different numbers, so "
                         f"{'it was' if len(renamed) == 1 else 'they were'} added as a separate copy "
                         "(yours is unchanged):")
            lines += [f"  • {c['name']}" for c in renamed[:8]]
        if same:
            lines.append(f"Skipped {len(same)} already here (identical).")
        if problems:
            lines.append(f"Ignored {len(problems)} invalid entr{'ies' if len(problems) != 1 else 'y'}: "
                         + "; ".join(problems[:3]))
        messagebox.showinfo("Import complete", "\n".join(lines))

    def _prune_buffers(self, now_pc: Optional[float] = None):
        if now_pc is None:
            now_pc = time.perf_counter()
        cutoff = now_pc - self.buffer_keep_seconds
        while self.hadi_buffer and self.hadi_buffer[0].pc_time < cutoff:
            self.hadi_buffer.popleft()
        with self.ocr_lock:
            self._prune_ocr_buffer_locked(now_pc)

    def _prune_ocr_buffer_locked(self, now_pc: float):
        """Drop expired OCR samples. Caller must hold ocr_lock."""
        cutoff = now_pc - self.buffer_keep_seconds
        while self.ocr_buffer and self.ocr_buffer[0].pc_time < cutoff:
            self.ocr_buffer.popleft()

    def _on_decimal_point_change(self):
        mode = self.decimal_point_var.get()
        self.decimal_fixer.mode = mode
        save_decimal_point_mode(mode)

    def _refresh_decimal_point_hadi(self):
        """Snapshot the HADI force for Auto decimal placement (Tk thread).

        None when there's nothing trustworthy to match against: no HADI, no
        cell selected, or load under 1% of the cell's capacity, where a small
        offset becomes a huge ratio and the remembered decimals do better.
        """
        reading = self.latest_hadi
        if (reading is None or not self.hadi.is_connected()
                or not self.selected_load_cell_name.get()):
            self._dp_hadi = None
            return
        capacity_lbf = self._capacity_limit_lbf()
        if capacity_lbf is not None and abs(reading.force_lbf) < 0.01 * capacity_lbf:
            self._dp_hadi = None
            return
        force = self._hadi_lbf_to_display(reading.force_lbf, raw_response=reading.raw_response)
        self._dp_hadi = (force, reading.pc_time)

    def _on_ocr_reading(self, reading):
        now_pc = time.perf_counter()
        # Use frame capture time if available (webcam OCR), otherwise use
        # arrival time (iPhone UDP). This aligns the OCR reading with when
        # the display was actually photographed, not when OCR finished.
        effective_pc = getattr(reading, "capture_pc", None) or now_pc
        raw_text = getattr(reading, "raw_text", "") or f"{reading.value:g}"
        value = reading.value
        # Restore a decimal point the OCR missed (Settings -> Decimal point).
        # Everything downstream — live display, capture median, % error,
        # CSV — sees the fixed-up text and value.
        hadi = self._dp_hadi
        hadi_force = hadi[0] if hadi and (now_pc - hadi[1]) <= 2.0 else None
        fixed_text = self.decimal_fixer.fix(raw_text, hadi_force)
        if fixed_text != raw_text:
            try:
                value = float(fixed_text)
                raw_text = fixed_text
            except ValueError:
                pass
        self._last_ocr_decimals = self._decimal_places(raw_text)
        sample = OCRTimedReading(
            value=value,
            pc_time=effective_pc,
            wall_time=reading.received_at,
            phone_time=reading.timestamp,
            raw_text=raw_text,
        )
        with self.ocr_lock:
            self.latest_ocr = sample
            self.ocr_buffer.append(sample)
            # Prune only the OCR buffer here: this runs on the OCR thread, and
            # popping from hadi_buffer while the Tk thread iterates it raised
            # "deque mutated during iteration", silently aborting UI ticks.
            self._prune_ocr_buffer_locked(now_pc)
        self.ocr_last_pc_time = now_pc
        self._ocr_last_packet_pc = now_pc

    def _on_gps_fix(self, gps):
        self._ocr_last_packet_pc = time.perf_counter()
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

        self._refresh_gravity_status()

    def _refresh_gravity_status(self):
        ctx = self._gravity_context()
        mf = ctx["gravity_factor"]
        if ctx["source"] == "standard":
            self.gps_status_var.set("No location — W rows use standard gravity (MF 1.0)")
            # Always show the MF, so a forgotten location is obvious before
            # any weight rows are made.
            self.w_button_var.set(f"SELECT / UNSELECT W    MF {mf:.7f} (standard)")
            return
        self.gps_status_var.set(f"{ctx['note']} | MF {mf:.7f}")
        self.w_button_var.set(f"SELECT / UNSELECT W    MF {mf:.7f}")

    def _set_mf_row_visible(self, visible: bool):
        if visible == self._mf_row_visible:
            return
        self._mf_row_visible = visible
        for w in self._mf_row_widgets:
            if visible:
                w.grid()
            else:
                w.grid_remove()
        self._mf_link.configure(text="Hide MF override" if visible else "Enter MF directly")

    def _toggle_mf_row(self):
        # An active override always stays on screen so it can't be forgotten.
        if self._mf_row_visible and self._mf_override is not None:
            self.gps_status_var.set("Clear the MF override and press Apply before hiding it")
            self.after(3000, self._refresh_gravity_status)
            return
        self._set_mf_row_visible(not self._mf_row_visible)
        if self._mf_row_visible:
            self._mf_row_widgets[1].focus_set()

    def _apply_mf_override(self):
        raw = self.mf_override_var.get().strip()
        if raw:
            try:
                new_override = float(raw)
            except ValueError:
                self.gps_status_var.set("Invalid MF value — enter a number or leave blank")
                return
        else:
            new_override = None
        prev_override, prev_text = self._mf_override, self._mf_override_applied_text

        def apply():
            self._mf_override = new_override

        def revert():
            self._mf_override = prev_override
            self.mf_override_var.set(prev_text)

        if not self._commit_gravity_change(apply, revert):
            return
        self._mf_override_applied_text = raw
        if new_override is not None:
            self._set_mf_row_visible(True)

    def _apply_location(self):
        # Session-only, like the MF override: every launch starts from
        # standard gravity rather than wherever the last session was.
        lat_raw = self.loc_lat_var.get().strip()
        elev_raw = self.loc_elev_var.get().strip()
        unit = self.loc_elev_unit_var.get()
        if not lat_raw and not elev_raw:
            new_location = None
        else:
            try:
                lat = float(lat_raw)
                elev = float(elev_raw) if elev_raw else 0.0
                if not -90.0 <= lat <= 90.0:
                    raise ValueError
            except ValueError:
                self.gps_status_var.set("Invalid location — latitude must be −90 to 90 and elevation a number")
                return
            altitude_m = elev * 0.3048 if unit == "ft" else elev
            mf = astm_multiplying_factor(lat, altitude_m)
            new_location = {
                "latitude": lat,
                "altitude_m": altitude_m,
                "elevation_text": f"{elev:g} {unit}",
                "gravity_factor": mf,
                "gravity_m_s2": mf * STANDARD_GRAVITY,
            }
        prev_location, prev_text = self._manual_location, self._location_applied_text

        def apply():
            self._manual_location = new_location

        def revert():
            self._manual_location = prev_location
            self.loc_lat_var.set(prev_text[0])
            self.loc_elev_var.set(prev_text[1])
            self.loc_elev_unit_var.set(prev_text[2])

        if self._commit_gravity_change(apply, revert):
            self._location_applied_text = (lat_raw, elev_raw, unit)

    def _commit_gravity_change(self, apply, revert) -> bool:
        """Apply a location/MF change; if existing W rows were made with a
        different MF, ask what to do with them. Returns False if cancelled.

        A W row's reference force is nominal weight x the MF in effect when
        the row was made, so correcting the location afterwards used to leave
        those rows silently on the old MF while new rows used the new one.
        """
        apply()
        ctx = self._gravity_context()
        new_mf = ctx["gravity_factor"]
        stale = []
        for point_row in self.capture_rows:
            for run_key in ("run1", "run2"):
                row = point_row.get(run_key)
                if (row and row.get("nominal_weight_lbf") not in (None, "")
                        and row.get("gravity_factor") not in (None, "")
                        and abs(float(row["gravity_factor"]) - new_mf) > 1e-12):
                    stale.append(row)
        if stale:
            old = sorted({f"{float(r['gravity_factor']):.7f}" for r in stale})
            n = len(stale)
            choice = messagebox.askyesnocancel(
                "Gravity changed",
                f"{n} W row{'s' if n != 1 else ''} use{'s' if n == 1 else ''} MF {', '.join(old)}. "
                f"The new MF is {new_mf:.7f}.\n\n"
                f"Yes — update {'those rows' if n != 1 else 'that row'} to the new MF "
                f"(OCR readings kept, % error recalculated)\n"
                f"No — keep {'them' if n != 1 else 'it'} as {'they are' if n != 1 else 'it is'}; "
                f"only new W rows use the new MF\n"
                f"Cancel — don't change the gravity setting",
            )
            if choice is None:
                revert()
                self._refresh_gravity_status()
                return False
            if choice:
                for row in stale:
                    self._update_weight_row_gravity(row, ctx)
                self._redraw_point_table()
                self._mark_data_changed()
        self._refresh_gravity_status()
        return True

    def _update_weight_row_gravity(self, row: dict, ctx: dict) -> None:
        """Re-base a W row on a new MF, keeping its weight and OCR reading."""
        mf = ctx["gravity_factor"]
        row["gravity_factor"] = mf
        row["gravity_m_s2"] = ctx["gravity_m_s2"]
        row["gps_latitude"] = ctx["gps_latitude"]
        row["gps_longitude"] = ctx["gps_longitude"]
        row["gps_altitude_m"] = ctx.get("gps_altitude_m", "")
        row["hadi_lbf"] = float(row["nominal_weight_lbf"]) * mf
        self._set_row_hadi_text(row)
        if row.get("ocr") not in (None, ""):
            self._recalculate_row_error(row)

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

    @staticmethod
    def _corr(xs, ys):
        """Pearson correlation of two equal-length numpy arrays."""
        if len(xs) < 8:
            return None
        dx = xs - xs.mean()
        dy = ys - ys.mean()
        vx = float(dx @ dx)
        vy = float(dy @ dy)
        if vx <= 1e-9 or vy <= 1e-9:
            return None
        return float(dx @ dy) / (vx * vy) ** 0.5

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
            self.sync_status_var.set(f"Sync: waiting for data | lag {self.sync_lag_seconds*1000:+.0f} ms")
            return

        # Use recent data only.
        recent_start = now_pc - self.sync_window_seconds
        hadi = [s for s in hadi if s.pc_time >= recent_start]
        ocr = [s for s in ocr if s.pc_time >= recent_start]
        if len(hadi) < 8 or len(ocr) < 8:
            return

        # Both signals must actually change, or there's nothing to line up.
        # This used to demand a 5 lbf swing, which steady-load calibration
        # (and any small cell) never produces, so the lag was never measured.
        # Any variation works — even at-rest flicker of a count or two locks
        # on (corr ~0.97 in testing) — and the correlation threshold below
        # rejects windows where the two don't genuinely track each other.
        h_vals = [s.force_lbf for s in hadi]
        o_vals = [s.value for s in ocr]
        if max(h_vals) == min(h_vals) or max(o_vals) == min(o_vals):
            self.sync_status_var.set(f"Sync: no change to track | lag {self.sync_lag_seconds*1000:+.0f} ms")
            return

        best_lag = None
        best_corr = -2.0

        # Webcam: refine around the current lag (seeded from Cam Latency).
        # Otherwise scan the full ±500 ms range.
        if self.webcam_ocr:
            center_ms = int(round(self.sync_lag_seconds * 1000.0))
            search_range = range(center_ms - 250, center_ms + 251, 10)
        else:
            search_range = range(-500, 501, 25)

        # Vectorised: this runs once a second on the Tk thread, and the
        # pure-Python version took 30-40 ms per run while the load moved —
        # a visible stutter every second. numpy does it in about a millisecond.
        hadi_t = np.fromiter((s.pc_time for s in hadi), float)
        ocr_t = np.fromiter((s.pc_time for s in ocr), float)
        h_arr = np.asarray(h_vals, float)
        o_arr = np.asarray(o_vals, float)

        for lag_ms in search_range:
            lag = lag_ms / 1000.0
            t0 = max(hadi_t[0], ocr_t[0] - lag)
            t1 = min(hadi_t[-1], ocr_t[-1] - lag)
            if t1 - t0 < 1.0:
                continue

            grid = np.arange(t0, t1 + 1e-9, 0.05)
            xs = np.interp(grid, hadi_t, h_arr)
            ys = np.interp(grid + lag, ocr_t, o_arr)
            c = self._corr(xs, ys)
            if c is not None and c > best_corr:
                best_corr = c
                best_lag = lag

        if best_lag is None:
            return

        if best_corr >= 0.65 and self._lag_estimate_plausible(best_lag):
            # Converge quickly while far off, then settle: once within 20 ms,
            # take only small corrections so the per-second estimate noise
            # (about ±7 ms in testing) doesn't move the pairing between
            # captures. Settled lag ended up 2-3x steadier with no loss of
            # capture accuracy in replays of 600 real captures.
            diff = best_lag - self.sync_lag_seconds
            weight = 0.20 if abs(diff) > 0.020 else 0.05
            self.sync_lag_seconds += weight * diff
            self.sync_confidence = best_corr
            # Persist at most every 10 s. This runs once a second on the Tk
            # thread, and while the lag is converging it drifts by more than a
            # couple of ms each tick, so a change-triggered save meant a file
            # write (and a possible disk/antivirus hiccup) every second.
            last_saved = getattr(self, "_last_sync_save", None)
            lag_ms = self.sync_lag_seconds * 1000.0
            if last_saved is None or now_pc - last_saved[0] >= 10.0:
                save_sync_state(lag_ms, self.sync_confidence)
                self._last_sync_save = (now_pc, lag_ms)

        self.sync_status_var.set(
            f"Sync lag {self.sync_lag_seconds*1000:+.0f} ms | conf {self.sync_confidence:.2f}"
            + (" | ignoring an outlier" if self._lag_jump_candidates else "")
        )

    # A webcam frame can't arrive before the display shows it, and even slow
    # USB/4K cameras measure well under this. Estimates outside are glitches.
    WEBCAM_LAG_LIMITS = (0.0, 0.8)
    # Once settled, an estimate this far from the current lag must be
    # repeated (3 in a row, agreeing within 30 ms) before it's believed.
    LAG_JUMP_SECONDS = 0.060
    LAG_JUMP_CONFIRMATIONS = 3

    def _lag_estimate_plausible(self, best_lag: float) -> bool:
        """Reject lag estimates a glitch could produce (a camera stalling or
        delivering a burst of stale frames for a moment), while still letting
        a genuine, persistent change through after a few seconds."""
        if self.webcam_ocr:
            lo, hi = self.WEBCAM_LAG_LIMITS
            if not lo <= best_lag <= hi:
                return False
        diff = abs(best_lag - self.sync_lag_seconds)
        if not self._lag_settled or diff <= self.LAG_JUMP_SECONDS:
            self._lag_jump_candidates = []
            # Settled = 5 estimates in a row within 20 ms of the current lag.
            self._lag_good_streak = self._lag_good_streak + 1 if diff <= 0.020 else 0
            if self._lag_good_streak >= 5:
                self._lag_settled = True
            return True
        # Settled and this estimate jumps: hold it until it repeats.
        cands = self._lag_jump_candidates
        if cands and abs(best_lag - cands[-1]) > 0.030:
            cands.clear()
        cands.append(best_lag)
        if len(cands) < self.LAG_JUMP_CONFIRMATIONS:
            return False
        # Real change: follow it at the fast rate and re-settle from there.
        self._lag_jump_candidates = []
        self._lag_settled = False
        self._lag_good_streak = 0
        return True

    def _reset_lag_tracking(self):
        self._lag_settled = False
        self._lag_good_streak = 0
        self._lag_jump_candidates: list[float] = []

    def _corrected_ocr_time(self, sample):
        return sample.pc_time - self.sync_lag_seconds

    @staticmethod
    def _middle_sample(samples, attr):
        """The real sample at the median of `attr` (lower middle when even).

        statistics.median averages the two middle values of an even count,
        recording a number neither instrument ever showed (0.02318 and
        0.04637 -> 0.03478). Picking an actual sample also keeps a HADI
        reading's force and raw mV/V from the same poll.
        """
        ordered = sorted(samples, key=lambda s: getattr(s, attr))
        return ordered[(len(ordered) - 1) // 2]

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
        self.hadi_lbf_var.set("---")
        self.hadi_raw_var.set("WAITING")
        self.raw_text_var.set("")
        self._update_capacity_warning(None)

    def _set_ocr_waiting_display(self):
        with self.ocr_lock:
            self.latest_ocr = None
        self.ocr_value_var.set("---")

    def _ui_tick(self):
        # This loop drives every live display update. If an exception ever
        # escaped it, the `after` reschedule would never run and the whole UI
        # would freeze permanently, so the body is guarded and the reschedule
        # is in a finally block.
        self._ui_heartbeat = time.perf_counter()
        try:
            self._ui_tick_body()
            self._last_tick_error = None  # recovered; log it again if it recurs
        except Exception as exc:
            # Log each distinct failure once per streak, not 20x a second.
            msg = f"{type(exc).__name__}: {exc}"
            if msg != self._last_tick_error:
                self._last_tick_error = msg
                log_freeze(f"UI tick error: {msg}\n{traceback.format_exc()}")
        finally:
            self.after(50, self._ui_tick)

    def _start_freeze_watchdog(self):
        """Log the Tk thread's stack whenever it stops ticking for >0.5 s.

        The busy cursor / yellow HADI light means the Tk thread is stuck in
        something; this records exactly what, in freeze_log.txt.
        """
        main_id = threading.main_thread().ident

        def watch():
            stalled_from = None
            while not getattr(self, "_closing", False):
                time.sleep(0.1)
                beat = self._ui_heartbeat
                gap = time.perf_counter() - beat
                if stalled_from is None and gap >= 0.5:
                    stalled_from = beat
                    frame = sys._current_frames().get(main_id)
                    stack = "".join(traceback.format_stack(frame)) if frame else "(no frame)\n"
                    log_freeze(f"UI STALLED (no tick for {gap * 1000:.0f} ms) — Tk thread is in:\n{stack}")
                elif stalled_from is not None and beat != stalled_from:
                    log_freeze(f"UI recovered after {(beat - stalled_from) * 1000:.0f} ms\n")
                    stalled_from = None

        threading.Thread(target=watch, daemon=True, name="FreezeWatchdog").start()

    def _ui_tick_body(self):
        now_pc = time.perf_counter()
        got_hadi = False

        while True:
            try:
                item = self.hadi.out.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, Exception):
                self.status_var.set(f"HADI error: {item}")
                log_freeze(f"HADI error: {type(item).__name__}: {item}")
                self.hadi_last_pc_time = None
                self._set_hadi_waiting_display()
                if self._auto_scan_active:
                    self.hadi.disconnect(wait=False)
                    self.hadi_button_var.set("Connect HADI")
                continue

            got_hadi = True
            self.latest_hadi = item
            self.hadi_last_pc_time = item.pc_time
            self.hadi_buffer.append(item)
            self._prune_buffers(item.pc_time)
            self.hadi_raw_var.set(f"{item.raw_response:+.5f}")
            if self.selected_load_cell_name.get():
                self.hadi_lbf_var.set(self._format_hadi_display_value(item))
                self._update_capacity_warning(item)
            else:
                self.hadi_lbf_var.set("↑ Select Cell")
            self.raw_text_var.set(item.raw_text)

        if not got_hadi and not self._hadi_is_fresh(now_pc):
            self._set_hadi_waiting_display()
        self._refresh_decimal_point_hadi()

        if self.ocr or self.webcam_ocr:
            with self.ocr_lock:
                r = self.latest_ocr
            if r is None or not self._ocr_is_fresh(now_pc):
                self._set_ocr_waiting_display()
            else:
                live_text = "" if r.raw_text in (None, "") else str(r.raw_text).strip()
                if not live_text:
                    live_text = f"{r.value:g}"
                if not live_text.startswith(("+", "-")):
                    live_text = "+" + live_text
                self.ocr_value_var.set(live_text)
        else:
            self._set_ocr_waiting_display()

        self._update_hadi_light(now_pc)
        self._update_ocr_light(now_pc)

        self._update_device_overload()
        self._update_overload_badge()
        self._estimate_sync_lag()
        self._update_live_percent_error()
        self._check_auto_capture()

        if self._auto_scan_active and not self.hadi.is_connected():
            self._kick_auto_scan()

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
        capacity_lbf = self._display_to_lbf(capacity)
        count = len(self.capture_rows)
        self.target_forces = self._generate_targets(capacity_lbf, count)
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
        capacity_lbf = self._display_to_lbf(capacity)
        count = len(self.capture_rows)
        self.target_forces = self._generate_targets(capacity_lbf, count)
        needed = len(self.target_forces)
        if needed > count:
            self._resize_point_rows(needed, keep_existing=True)
            if needed in (11, 20):
                self.point_count_var.set(str(needed))
            else:
                self.point_count_var.set("Custom")
                self.custom_point_count_var.set(str(needed))
        self._update_target_force_display()

        first_empty = self._first_empty_from(0, self.capture_target_run)
        if first_empty is not None:
            self._select_target(*first_empty)
        self._speak_greeting(capacity, count)

    def _speak_greeting(self, capacity: float, points: int):
        greeting = "Hello, "
        mode = self.mode_var.get().lower()
        units = self.hadi_units_var.get()
        cell = self._find_load_cell(self.selected_load_cell_name.get())
        if cell:
            cell_cap = cell.get("capacity_lbf", "")
            greeting += f"you have your {cell_cap} pound calibration load cell in {mode} "
        else:
            greeting += f"you have your calibration load cell in {mode} "
        greeting += (f"and are calibrating a {capacity:g} {units} capacity, "
                     f"{points} points. Don't forget to preload. Let's begin. ")
        first_target = self._target_speech_text()
        if first_target:
            greeting += first_target
        self._speak(greeting)

    def _update_target_force_display(self):
        idx = self.capture_target_index
        if not self.target_forces or idx is None or idx >= len(self.target_forces):
            self.target_force_var.set("")
            return
        val = self.target_forces[idx]
        units = self.hadi_units_var.get()
        if units == "mV/V":
            self.target_force_var.set(f"▶ {val:g} LBF")
        else:
            factor = HADI_UNIT_FACTORS.get(units, 1.0)
            converted = val * factor
            self.target_force_var.set(f"▶ {converted:.10g} {units}")

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
        hadi_text = ""
        if run_row and run_row.get("hadi_lbf") is not None:
            try:
                _raw = run_row.get("hadi_raw")
                _raw_f = float(_raw) if _raw not in (None, "") else None
                display_val = self._hadi_lbf_to_display(float(run_row["hadi_lbf"]), raw_response=_raw_f)
                hadi_text = self._format_hadi_lbf_text(display_val, show_plus=False,
                                                       ocr_text=run_row.get("ocr_text"))
            except (TypeError, ValueError):
                hadi_text = ""
        return (
            point_label,
            hadi_text,
            self._ocr_text_for_row(run_row),
            self._format_run_percent(run_row),
        )

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

    def _calculate_percent_error(self, ocr_value, hadi_value, ocr_text=None):
        """Return (percent_error, is_na).

        A zero OCR/reference value is still a valid capture value, but percent
        error cannot be computed because the denominator would be zero. Store it
        as NA instead of rejecting the row. `ocr_text` is the OCR reading as
        shown; it sets the HADI's displayed precision when Dec = Auto.
        """
        try:
            ocr = float(ocr_value)
            hadi = float(hadi_value)
        except Exception:
            return None, False
        # Compare against the HADI value as shown (rounded to the Dec
        # setting), not its full float precision — otherwise identical-looking
        # rows report a nonzero error, which near zero load gets large. Never
        # round coarser than the OCR reading itself, though: with Dec = 1 a
        # 2.49925 lb reference would become 2 against a 2.50 reading.
        if ocr_text in (None, ""):
            ocr_text = repr(ocr) if "e" not in repr(ocr) else f"{ocr:.10f}"
        ocr_decimals = self._decimal_places(ocr_text)
        hadi = round(hadi, max(self._selected_hadi_decimals(ocr_text), ocr_decimals))
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
        if next_empty is not None:
            self._select_target(*next_empty)
        else:
            if next_index < len(self.capture_rows):
                self._select_target(next_index, run)
            else:
                self._select_target(0, run)

    def _on_run_tree_select(self, _event, run: int):
        if self.suppress_next_tree_select:
            self.suppress_next_tree_select = False
            return
        if self.ocr_edit_entry is not None:
            return
        self._update_copy_w_button()

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
        hadi_text = ""
        if run_row:
            if kind == "hadi":
                # Stored values are lbf; the entry must show the selected display
                # unit, because commit converts what was typed from that unit.
                try:
                    if run_row.get("nominal_weight_lbf") not in (None, ""):
                        hadi_display = self._hadi_lbf_to_display(float(run_row["nominal_weight_lbf"]))
                    elif run_row.get("hadi_lbf") not in (None, ""):
                        _raw = run_row.get("hadi_raw")
                        _raw_f = float(_raw) if _raw not in (None, "") else None
                        hadi_display = self._hadi_lbf_to_display(float(run_row["hadi_lbf"]), raw_response=_raw_f)
                    else:
                        hadi_display = None
                    if hadi_display is not None:
                        hadi_text = self._format_hadi_lbf_text(hadi_display, show_plus=False,
                                                               ocr_text=run_row.get("ocr_text"))
                except (TypeError, ValueError):
                    hadi_text = ""
            else:
                current_value = run_row.get("ocr_text", run_row.get("ocr", ""))

        entry = ttk.Entry(tree)
        if kind == "hadi":
            entry.insert(0, hadi_text)
        elif current_value != "":
            try:
                entry.insert(0, self._format_ocr_text(current_value, run_row.get("ocr"), show_plus=False))
            except Exception:
                entry.insert(0, str(current_value).replace("+", ""))
        self.edit_original_text = entry.get().strip()
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
        unchanged = raw == self.edit_original_text.replace("+", "").replace("%", "")

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

        if not raw:
            return "break"
        try:
            new_value = float(raw)
        except ValueError:
            label = "weight" if kind == "hadi" else "OCR value"
            messagebox.showerror("Invalid value", f"Enter a numeric {label}.")
            return "break"

        if row_index is None or run not in (1, 2) or not (0 <= row_index < len(self.capture_rows)):
            return "break"

        if unchanged:
            # Re-committing the displayed text would round the captured value
            # to the shown decimals (and an OCR cell would lose its as-captured
            # original and get flagged edited), so leave the row as captured.
            pass
        elif kind == "hadi":
            existing = self.capture_rows[row_index].get(f"run{run}") or {}
            was_weight = existing.get("nominal_weight_lbf") not in (None, "")

            if was_weight:
                # Editing a W row means "change the nominal weight", not
                # "turn W off". Keep the W marker, apply GPS gravity again,
                # preserve OCR if it already exists, and recalc % error.
                existing_ocr = existing.get("ocr")
                nominal_lbf = self._display_to_lbf(new_value)
                row = self._make_manual_weight_row(
                    nominal_lbf,
                    ocr_value=existing_ocr if existing_ocr not in ("", None) else None,
                    ocr_text=existing.get("ocr_text", ""),
                )
                row["point"] = row_index + 1
                row["run"] = run
                row["pre_w_hadi_lbf"] = nominal_lbf
                row["ocr_edited"] = existing.get("ocr_edited", False)
                self._carry_ocr_original(existing, row)
                self.capture_rows[row_index][f"run{run}"] = row
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
                    "hadi_lbf": self._display_to_lbf(new_value),
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
        else:
            run_row = self.capture_rows[row_index].get(f"run{run}")
            if run_row is None:
                return "break"

            run_row["ocr"] = new_value
            run_row["ocr_text"] = raw
            run_row["ocr_edited"] = True
            # Typed in the current units, so this is the reading now.
            for key in self._OCR_ORIGINAL_KEYS:
                run_row.pop(key, None)

            try:
                _raw = run_row.get("hadi_raw")
                _raw_f = float(_raw) if _raw not in (None, "") else None
                hadi_value = self._hadi_lbf_to_display(float(run_row.get("hadi_lbf")), raw_response=_raw_f)
            except Exception:
                hadi_value = None

            if hadi_value is not None:
                err, is_na = self._calculate_percent_error(new_value, hadi_value, ocr_text=raw)
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
        stamp = datetime.now().strftime("%y%m%d_%H%M")
        sn = self.serial_number_var.get().strip().replace(" ", "_").replace("/", "-")
        cap = self.customer_capacity_var.get().strip()
        prefix = "autosave_" if autosave else ""
        parts = []
        if cap:
            units = self.hadi_units_var.get()
            parts.append(f"{cap}{units}")
        if sn:
            parts.append(sn)
        parts.append(prefix + mode)
        parts.append(runs)
        parts.append(stamp)
        return "_".join(parts) + ".csv"

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

    def _csv_row_for_point(self, run_key: str, point_row: dict):
        run_row = point_row.get(run_key)
        if not run_row:
            return ["", "", ""]
        hadi_text = ""
        hadi_lbf = run_row.get("hadi_lbf")
        if hadi_lbf not in (None, ""):
            try:
                _raw = run_row.get("hadi_raw")
                _raw_f = float(_raw) if _raw not in (None, "") else None
                display_val = self._hadi_lbf_to_display(float(hadi_lbf), raw_response=_raw_f)
                hadi_text = self._format_hadi_lbf_text(display_val, show_plus=False,
                                                       ocr_text=run_row.get("ocr_text"))
            except (TypeError, ValueError):
                pass
        return [
            self._ocr_text_for_row(run_row, show_plus=False),
            hadi_text,
            self._csv_fmt_error(run_row.get("percent_error"), run_row.get("percent_error_na", False)),
        ]

    # Height (data rows) of a full 20-point run block. An 11-point run reuses
    # this height: its leading points stay at the top and its final
    # (return-to-zero) point drops to the last row, matching a 20-point layout.
    _CSV_REFERENCE_ROWS = 20

    def _run_block_rows(self, run_key: str):
        """Data rows for one run, padded so the last point lands on the same
        row a 20-point save would use. Returns (rows, filled_count)."""
        point_rows = [self._csv_row_for_point(run_key, point_row) for point_row in self.capture_rows]
        filled = sum(1 for r in point_rows if r != ["", "", ""])
        n = len(point_rows)
        reference = max(self._CSV_REFERENCE_ROWS, n)
        if n == 0 or n >= reference:
            return point_rows, filled
        # Leading points at the top, final point on the reference's last row,
        # empty rows skipped in between.
        leading = point_rows[:-1]
        last = point_rows[-1]
        pad = reference - n  # empty rows between the leading block and the last point
        rows = leading + [["", "", ""] for _ in range(pad)] + [last]
        return rows, filled

    def _write_capture_csv(self, path) -> int:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)

        has_run1 = self._run_has_data("run1")
        has_run2 = self._run_has_data("run2")
        units = self.hadi_units_var.get()
        fields = self._capture_csv_fields()

        blocks = []
        if has_run1:
            blocks.append(("Run 1", "run1"))
        if has_run2:
            blocks.append(("Run 2", "run2"))

        total_rows = 0
        with path.open("w", newline="") as f:
            writer = csv.writer(f)

            for block_idx, (label, run_key) in enumerate(blocks):
                if block_idx > 0:
                    writer.writerow([])  # one blank row between runs
                writer.writerow([f"{label} ({units})"])
                writer.writerow(fields)
                rows, filled = self._run_block_rows(run_key)
                for row in rows:
                    writer.writerow(row)
                total_rows += filled

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

        if self.flip_sign_var.get() and ocr_value is not None:
            ocr_value = -ocr_value
            ocr_text = self._fixed_decimals(ocr_value, self._decimal_places(ocr_text))

        try:
            hadi_lbf = float(run_row.get("hadi_lbf"))
        except Exception:
            messagebox.showwarning("Missing weight", "This W row does not have a HADI/weight value.")
            return False

        hadi_raw_val = run_row.get("hadi_raw")
        try:
            hadi_raw_val = float(hadi_raw_val) if hadi_raw_val not in (None, "") else None
        except (TypeError, ValueError):
            hadi_raw_val = None
        hadi_display = self._hadi_lbf_to_display(hadi_lbf, raw_response=hadi_raw_val)
        percent_error, percent_error_na = self._calculate_percent_error(ocr_value, hadi_display, ocr_text=ocr_text)

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
        for key in self._OCR_ORIGINAL_KEYS:  # fresh reading in current units
            run_row.pop(key, None)
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
        hadi_fresh = self._hadi_is_fresh(now_pc)
        ocr_fresh = self._ocr_is_fresh(now_pc)
        if not hadi_fresh or not ocr_fresh:
            self.live_error_var.set("--")
            self._set_live_error_color(None)
            return

        try:
            hadi_lbf = self.latest_hadi.force_lbf if self.latest_hadi else None
            hadi_raw = self.latest_hadi.raw_response if self.latest_hadi else None
            with self.ocr_lock:
                ocr = self.latest_ocr.value if self.latest_ocr else None
                ocr_text = self.latest_ocr.raw_text if self.latest_ocr else None
            if hadi_lbf is None or ocr is None:
                raise ValueError
            # Mirror the Flip ± toggle so the live % error matches what a
            # capture would record (capture negates OCR the same way).
            if self.flip_sign_var.get():
                ocr = -ocr
            hadi = self._hadi_lbf_to_display(hadi_lbf, raw_response=hadi_raw)
            err, is_na = self._calculate_percent_error(ocr, hadi, ocr_text=ocr_text)
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

        need_both = self.require_both_var.get()

        if need_both and not self._hadi_is_fresh(now_pc):
            raise RuntimeError("Waiting for fresh HADI readings.")
        if need_both and not self._ocr_is_fresh(now_pc):
            raise RuntimeError("Waiting for fresh OCR packets.")
        if not need_both:
            if not self._hadi_is_fresh(now_pc) and not self._ocr_is_fresh(now_pc):
                raise RuntimeError("No HADI or OCR data available.")

        hadi_samples = [s for s in self.hadi_buffer if (now_pc - s.pc_time) <= self.hadi_wait_seconds]
        with self.ocr_lock:
            ocr_samples = [s for s in self.ocr_buffer if (now_pc - s.pc_time) <= self.ocr_wait_seconds]

        if need_both and len(hadi_samples) < 2:
            raise RuntimeError("Need at least two fresh HADI readings.")
        if need_both and len(ocr_samples) < 2:
            raise RuntimeError("Need at least two fresh OCR readings.")

        hadi_lbf = None
        hadi_raw = None
        ocr_value = None
        ocr_text = ""
        target_t = now_pc

        if hadi_samples:
            latest_hadi_t = hadi_samples[-1].pc_time
            target_t = min(target_t, latest_hadi_t)
        if ocr_samples:
            latest_ocr_corrected_t = self._corrected_ocr_time(ocr_samples[-1])
            target_t = min(target_t, latest_ocr_corrected_t)

        if hadi_samples:
            hadi_lbf = self._interp(hadi_samples, target_t, "force_lbf")
            hadi_raw = self._interp(hadi_samples, target_t, "raw_response")

        if ocr_samples:
            ocr_sample = self._nearest_sample(ocr_samples, target_t, corrected_ocr=True)
            ocr_value = None if ocr_sample is None else ocr_sample.value
            ocr_text = "" if ocr_sample is None else ocr_sample.raw_text

        if need_both and hadi_lbf is None:
            raise RuntimeError("Could not interpolate HADI reading.")
        if need_both and ocr_value is None:
            raise RuntimeError("Could not match OCR reading.")

        return now_pc, time.time(), target_t, hadi_lbf, hadi_raw, ocr_value, ocr_text

    def _gravity_context(self):
        mf = getattr(self, "_mf_override", None)
        if mf is not None:
            return {
                "gravity_factor": mf,
                "gravity_m_s2": mf * STANDARD_GRAVITY,
                "gps_latitude": "",
                "gps_longitude": "",
                "gps_altitude_m": "",
                "note": "MF override",
                "using_gps": False,
                "source": "override",
            }
        loc = getattr(self, "_manual_location", None)
        if loc is not None:
            return {
                "gravity_factor": loc["gravity_factor"],
                "gravity_m_s2": loc["gravity_m_s2"],
                "gps_latitude": loc["latitude"],
                "gps_longitude": "",
                "gps_altitude_m": loc["altitude_m"],
                "note": f"Location {loc['latitude']:.4f}°, {loc['elevation_text']}",
                "using_gps": True,
                "source": "location",
            }
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
                "source": "standard",
            }
        return {
            "gravity_factor": gps.gravity_factor,
            "gravity_m_s2": gps.gravity_m_s2,
            "gps_latitude": gps.latitude,
            "gps_longitude": gps.longitude,
            "gps_altitude_m": gps.altitude_m if gps.altitude_m is not None else "",
            "note": (
                f"Gravity locked (GPS) {gps.latitude:.4f}°, {gps.altitude_m:.1f} m"
                if gps.altitude_m is not None
                else f"Gravity locked (GPS) {gps.latitude:.4f}°, alt 0 m"
            ),
            "using_gps": True,
            "source": "gps",
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

        reference_display = self._hadi_lbf_to_display(reference_lbf)
        percent_error, percent_error_na = self._calculate_percent_error(ocr_value, reference_display, ocr_text=ocr_text)

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
        hadi_lbf = row.get("hadi_lbf")
        hadi_raw = row.get("hadi_raw")
        try:
            raw_val = float(hadi_raw) if hadi_raw not in (None, "") else None
        except (TypeError, ValueError):
            raw_val = None
        try:
            hadi_display = self._hadi_lbf_to_display(float(hadi_lbf), raw_response=raw_val) if hadi_lbf not in (None, "") else None
        except (TypeError, ValueError):
            hadi_display = None
        err, is_na = self._calculate_percent_error(row.get("ocr"), hadi_display, ocr_text=row.get("ocr_text"))
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
        self._carry_ocr_original(existing, row)
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

    def _selected_run1_w_indexes(self) -> list[int]:
        selected = self.run1_tree.selection()
        if len(selected) < 2:
            return []
        children = list(self.run1_tree.get_children())
        indexes = []
        for item_id in selected:
            try:
                idx = children.index(item_id)
            except ValueError:
                continue
            run1 = self.capture_rows[idx].get("run1")
            if run1 and run1.get("nominal_weight_lbf") not in (None, ""):
                indexes.append(idx)
        return indexes

    def _update_copy_w_button(self):
        if not hasattr(self, "copy_w_btn"):
            return
        if self._selected_run1_w_indexes():
            self.copy_w_btn.pack(side="right")
        else:
            self.copy_w_btn.pack_forget()

    def _copy_weights_to_run2(self):
        indexes = self._selected_run1_w_indexes()
        if not indexes:
            return

        has_run2 = any(
            self.capture_rows[i].get("run2") is not None for i in indexes
        )
        if has_run2:
            if not messagebox.askyesno(
                "Overwrite Run 2?",
                f"Some selected rows already have Run 2 data. Copy {len(indexes)} W rows to Run 2?",
            ):
                return

        copied = 0
        for i in indexes:
            run1 = self.capture_rows[i]["run1"]
            nominal = float(run1["nominal_weight_lbf"])
            row = self._make_manual_weight_row(nominal)
            row["point"] = self.capture_rows[i]["point"]
            row["run"] = 2
            row["pre_w_hadi_lbf"] = nominal
            self.capture_rows[i]["run2"] = row
            copied += 1

        self._redraw_point_table()
        self._mark_data_changed()
        self.status_var.set(f"Copied {copied} W rows to Run 2")

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

        conventional_lbf = hadi_lbf / gravity_factor if gravity_factor else hadi_lbf
        nominal_weight_lbf = nearest_standard_weight_lbf(conventional_lbf)
        reference_lbf = nominal_weight_lbf * gravity_factor

        reference_display = self._hadi_lbf_to_display(reference_lbf)
        percent_error, percent_error_na = self._calculate_percent_error(ocr_value, reference_display, ocr_text=ocr_text)
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
                row.get("hadi_text", self._format_hadi_lbf_text(self._hadi_lbf_to_display(reference_lbf))),
                self._format_ocr_text(ocr_text, ocr_value),
                "NA" if percent_error_na else f"{percent_error:+.2f}%",
            ),
        )
        self._update_point_count_label()

    def _update_ac_indicator_visibility(self):
        if self.auto_capture_enabled.get():
            self._ac_indicator_frame.place(relx=0.5, rely=0.5, anchor="w", x=80)
            self._ac_gauge.pack(fill="x", padx=10, pady=(0, 2),
                                before=self._ac_display_frame)
        else:
            self._ac_indicator_frame.place_forget()
            self._ac_gauge.pack_forget()

    _TTS_SCRIPT = (
        "Add-Type -AssemblyName System.Speech;"
        "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer;"
        "$s.Rate = 2; $s.Speak('%s')"
    )

    def _speak(self, text: str):
        if not self.auto_capture_voice_var.get():
            return
        safe = text.replace("'", "''")
        cmd = self._TTS_SCRIPT % safe
        threading.Thread(
            target=lambda: subprocess.run(
                ["powershell", "-NoProfile", "-Command", cmd],
                creationflags=0x08000000,
            ),
            daemon=True,
        ).start()

    def _target_speech_text(self) -> str:
        idx = self.capture_target_index
        if idx is None or not self.target_forces or idx >= len(self.target_forces):
            return ""
        val = self.target_forces[idx]
        units = self.hadi_units_var.get()
        if units == "mV/V":
            return f"target {val:g} pounds"
        factor = HADI_UNIT_FACTORS.get(units, 1.0)
        converted = val * factor
        return f"target {converted:.10g} {units}"

    def _speak_capture_and_next(self):
        next_text = self._target_speech_text()
        if next_text:
            self._speak(f"captured. {next_text}")
        else:
            self._speak("captured")

    def _redraw_ac_gauge(self):
        c = self._ac_gauge
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 20:
            return

        tol = self._ac_gauge_tolerance
        if tol <= 0:
            tol = 1.0
        error = self._ac_gauge_error
        view_range = tol * 3.0
        cx = w / 2
        half = w / 2

        tol_px = (tol / view_range) * half
        green_l = cx - tol_px
        green_r = cx + tol_px

        c.create_rectangle(0, 0, w, h, fill="#fecaca", outline="")
        c.create_rectangle(green_l, 0, green_r, h, fill="#bbf7d0", outline="")
        c.create_line(green_l, 0, green_l, h, fill="#16a34a", width=2)
        c.create_line(green_r, 0, green_r, h, fill="#16a34a", width=2)
        c.create_line(cx, 2, cx, h - 2, fill="#d1d5db", width=1, dash=(2, 2))

        if self._ac_gauge_active:
            needle_px = (error / view_range) * half
            nx = cx + max(-(half - 4), min(half - 4, needle_px))
            c.create_polygon(nx - 5, 0, nx + 5, 0, nx, 7, fill="#0f172a")
            c.create_polygon(nx - 5, h, nx + 5, h, nx, h - 7, fill="#0f172a")
            c.create_line(nx, 3, nx, h - 3, fill="#0f172a", width=2)

    def _reset_auto_capture_state(self):
        self._auto_capture_in_range_since = None
        self._auto_capture_last_index = None
        self._auto_capture_last_run = None
        self.auto_capture_status_var.set("")
        self._ac_status_label_var.set("")
        self._ac_gauge_active = False
        self._redraw_ac_gauge()
        self._update_ac_indicator_visibility()
        if self.auto_capture_enabled.get():
            text = self._target_speech_text()
            if text:
                self._speak(text)

    def _check_auto_capture(self):
        if not self.auto_capture_enabled.get():
            return
        if not self.target_forces:
            return

        self._ac_units_var.set(self.hadi_units_var.get())

        idx = self.capture_target_index
        run = self.capture_target_run
        if idx is None or not (0 <= idx < len(self.target_forces)):

            self._ac_status_label_var.set("")
            self._ac_gauge_active = False
            self._redraw_ac_gauge()
            return

        target_lbf = self.target_forces[idx]
        if target_lbf == 0:
            return

        target_display = self._hadi_lbf_to_display(target_lbf)
        if target_display == 0:
            return

        now_pc = time.perf_counter()
        if not self._ocr_is_fresh(now_pc):
            self._auto_capture_in_range_since = None

            self._ac_status_label_var.set("waiting")
            self._ac_gauge_active = False
            self._redraw_ac_gauge()
            return

        with self.ocr_lock:
            ocr = self.latest_ocr
        if ocr is None:
            self._auto_capture_in_range_since = None
            return

        try:
            tolerance = float(self.auto_capture_tolerance_var.get())
            dwell_seconds = float(self.auto_capture_dwell_var.get())
        except ValueError:
            return

        ocr_val = ocr.value
        signed_error = ocr_val - target_display
        abs_error = abs(signed_error)

        self._ac_gauge_error = signed_error
        self._ac_gauge_tolerance = tolerance
        self._ac_gauge_active = True
        self._redraw_ac_gauge()

        if idx != self._auto_capture_last_index or run != self._auto_capture_last_run:
            self._auto_capture_in_range_since = None
            self._auto_capture_last_index = idx
            self._auto_capture_last_run = run

        if abs_error <= tolerance:

            if self._auto_capture_in_range_since is None:
                self._auto_capture_in_range_since = now_pc
            elapsed = now_pc - self._auto_capture_in_range_since
            remaining = dwell_seconds - elapsed
            if remaining <= 0:
                self._auto_capture_in_range_since = None
                self._fire_auto_capture(idx, run, dwell_seconds)
            else:
                self._ac_status_label_var.set(f"{remaining:.1f}s")
        else:
            self._auto_capture_in_range_since = None

            self._ac_status_label_var.set(f"off by {abs_error:.1f}")

    def _fire_auto_capture(self, idx: int, run: int, dwell_seconds: float):
        now_pc = time.perf_counter()
        now_wall = time.time()
        window_start = now_pc - dwell_seconds

        hadi_samples = [s for s in self.hadi_buffer if s.pc_time >= window_start]
        with self.ocr_lock:
            ocr_samples = [s for s in self.ocr_buffer
                           if self._corrected_ocr_time(s) >= window_start]

        if not hadi_samples or not ocr_samples:
            self.auto_capture_status_var.set("Auto-capture: not enough samples")
            return

        pairs = []
        for ocr_s in ocr_samples:
            ocr_t = self._corrected_ocr_time(ocr_s)
            nearest_hadi = min(hadi_samples, key=lambda h: abs(h.pc_time - ocr_t))
            gap = abs(nearest_hadi.pc_time - ocr_t)
            pairs.append((gap, nearest_hadi, ocr_s))

        if not pairs:
            self.auto_capture_status_var.set("Auto-capture: no matched pairs")
            return

        pairs.sort(key=lambda p: p[0])
        best_gap = pairs[0][0]
        threshold = max(best_gap * 2, 0.060)
        tight = [(h, o) for g, h, o in pairs if g <= threshold]
        if len(tight) < 3:
            tight = [(h, o) for _, h, o in pairs[:max(3, len(pairs))]]

        mid_hadi = self._middle_sample([h for h, _ in tight], "force_lbf")
        hadi_lbf, hadi_raw = mid_hadi.force_lbf, mid_hadi.raw_response
        ocr_value = self._middle_sample([o for _, o in tight], "value").value
        ocr_decimals = max((self._decimal_places(o.raw_text) for _, o in tight), default=0)
        ocr_text = self._fixed_decimals(ocr_value, ocr_decimals)

        if self.flip_sign_var.get():
            ocr_value = -ocr_value
            ocr_text = self._fixed_decimals(ocr_value, self._decimal_places(ocr_text))

        hadi_display = self._hadi_lbf_to_display(hadi_lbf, raw_response=hadi_raw)
        percent_error, percent_error_na = self._calculate_percent_error(ocr_value, hadi_display, ocr_text=ocr_text)

        row = {
            "point": idx + 1,
            "time": datetime.fromtimestamp(now_wall).isoformat(timespec="milliseconds"),
            "load_cell": self.selected_load_cell_name.get(),
            "mode": self.mode_var.get(),
            "method": f"auto_median_{len(tight)}_pairs",
            "sync_mode": "auto",
            "sync_lag_ms": self.sync_lag_seconds * 1000.0,
            "sync_confidence": self.sync_confidence,
            "target_delay_ms": "",
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
                row.get("hadi_text", self._format_hadi_lbf_text(hadi_display)),
                self._format_ocr_text(ocr_text, ocr_value),
                "NA" if percent_error_na else f"{percent_error:+.2f}%",
            ),
        )
        self._update_point_count_label()
        self._ac_status_label_var.set(f"captured P{idx + 1}")
        self._ac_gauge_active = False
        self._redraw_ac_gauge()
        self._speak_capture_and_next()

    def _capture_from_key(self, _event=None):
        if self.ocr_edit_entry is not None:
            return "break"
        if self.notebook.index(self.notebook.select()) != 0:
            return
        # Key auto-repeat (holding space) would otherwise queue a capture per
        # repeat event, stamping the same reading into several rows.
        if getattr(self, "_manual_capture_pending", False):
            return "break"
        self._manual_capture_pending = True
        self.focus_set()
        try:
            half_window = float(self.manual_median_window_var.get())
        except ValueError:
            half_window = 0.5
        self._manual_median_press_time = time.perf_counter()
        # Camera frames arrive `sync_lag` after the moment they show, so the
        # frames for the end of the window are still in flight at
        # press + half_window. Wait for them (plus a little OCR time), or the
        # window silently loses its last ~lag of camera samples.
        delay_ms = int((half_window + max(0.0, self.sync_lag_seconds) + 0.05) * 1000)
        self.after(delay_ms, self._capture_manual_median)
        return "break"

    def _capture_manual_median(self):
        self._manual_capture_pending = False
        press_pc = getattr(self, "_manual_median_press_time", None)
        if press_pc is None:
            return
        try:
            half_window = float(self.manual_median_window_var.get())
        except ValueError:
            half_window = 0.5

        target_index, target_run, weight_row = self._target_weight_row()
        if weight_row is not None:
            self._capture_into_weight_row(target_index, target_run, weight_row)
            return

        window_start = press_pc - half_window
        window_end = press_pc + half_window

        hadi_samples = [s for s in self.hadi_buffer if window_start <= s.pc_time <= window_end]
        with self.ocr_lock:
            ocr_samples = [s for s in self.ocr_buffer
                           if window_start <= self._corrected_ocr_time(s) <= window_end]

        if not hadi_samples or not ocr_samples:
            self._capture()
            return

        # Pair each OCR sample with its closest HADI sample in time,
        # keep all pairs with a tight gap, then median to filter misreads.
        pairs = []
        for ocr_s in ocr_samples:
            ocr_t = self._corrected_ocr_time(ocr_s)
            nearest_hadi = min(hadi_samples, key=lambda h: abs(h.pc_time - ocr_t))
            gap = abs(nearest_hadi.pc_time - ocr_t)
            pairs.append((gap, nearest_hadi, ocr_s))

        if not pairs:
            self._capture()
            return

        pairs.sort(key=lambda p: p[0])
        # Use all pairs within 2x of the best gap (at least 3 pairs)
        best_gap = pairs[0][0]
        threshold = max(best_gap * 2, 0.060)
        tight = [(h, o) for g, h, o in pairs if g <= threshold]
        if len(tight) < 3:
            tight = [(h, o) for _, h, o in pairs[:max(3, len(pairs))]]

        now_wall = time.time()
        mid_hadi = self._middle_sample([h for h, _ in tight], "force_lbf")
        hadi_lbf, hadi_raw = mid_hadi.force_lbf, mid_hadi.raw_response
        ocr_value = self._middle_sample([o for _, o in tight], "value").value
        ocr_decimals = max((self._decimal_places(o.raw_text) for _, o in tight), default=0)
        ocr_text = self._fixed_decimals(ocr_value, ocr_decimals)

        if self.flip_sign_var.get():
            ocr_value = -ocr_value
            ocr_text = self._fixed_decimals(ocr_value, self._decimal_places(ocr_text))

        hadi_display = self._hadi_lbf_to_display(hadi_lbf, raw_response=hadi_raw)
        percent_error, percent_error_na = self._calculate_percent_error(ocr_value, hadi_display, ocr_text=ocr_text)

        self.status_var.set(
            f"Capture: {len(tight)} pairs (gap<={threshold*1000:.0f}ms) | "
            f"HADI={hadi_display:.5g} OCR={ocr_value:.5g}"
        )

        row = {
            "point": (self.capture_target_index + 1) if self.capture_target_index is not None else "",
            "time": datetime.fromtimestamp(now_wall).isoformat(timespec="milliseconds"),
            "load_cell": self.selected_load_cell_name.get(),
            "mode": self.mode_var.get(),
            "method": f"median_{len(tight)}_pairs",
            "sync_mode": "manual" if self.sync_manual else "auto",
            "sync_lag_ms": self.sync_lag_seconds * 1000.0,
            "sync_confidence": self.sync_confidence,
            "target_delay_ms": "",
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
                row.get("hadi_text", self._format_hadi_lbf_text(hadi_display)),
                self._format_ocr_text(ocr_text, ocr_value),
                "NA" if percent_error_na else f"{percent_error:+.2f}%",
            ),
        )
        self._update_point_count_label()

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

        if self.flip_sign_var.get() and ocr_value is not None:
            ocr_value = -ocr_value
            ocr_text = self._fixed_decimals(ocr_value, self._decimal_places(ocr_text))

        hadi_display = self._hadi_lbf_to_display(hadi_lbf, raw_response=hadi_raw) if hadi_lbf is not None else None
        percent_error, percent_error_na = self._calculate_percent_error(ocr_value, hadi_display, ocr_text=ocr_text)
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
            "hadi_raw": hadi_raw if hadi_raw is not None else "",
            "hadi_lbf": hadi_lbf if hadi_lbf is not None else "",
            "ocr": ocr_value if ocr_value is not None else "",
            "ocr_text": ocr_text,
            "ocr_edited": False,
            "percent_error": percent_error,
            "percent_error_na": percent_error_na,
            "captured_hadi_lbf": hadi_lbf if hadi_lbf is not None else "",
            "conventional_lbf_from_hadi": "",
            "nominal_weight_lbf": "",
            "gravity_factor": "",
            "gravity_m_s2": "",
            "gps_latitude": "",
            "gps_longitude": "",
            "gps_altitude_m": "",
        }
        self._set_row_hadi_text(row)

        hadi_fallback = ""
        if hadi_display is not None:
            hadi_fallback = self._format_hadi_lbf_text(hadi_display)
        self._insert_or_replace_capture_row(
            row,
            (
                row.get("hadi_text", hadi_fallback),
                self._format_ocr_text(ocr_text, ocr_value) if ocr_value is not None else "",
                "NA" if percent_error_na else (f"{percent_error:+.2f}%" if percent_error is not None else ""),
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
        self._add_to_report()
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

        count = len(self.capture_rows) if self.capture_rows else 11
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

        self._closing = True  # shutdown stalls are expected; stop logging them
        self.hadi.disconnect()
        self._stop_ocr()
        self._stop_webcam_ocr()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
