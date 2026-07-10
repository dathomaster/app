"""
webcam_ocr.py — Local webcam OCR for reading numbers off instrument displays.

Replaces the iPhone UDP stream with a local webcam + OCR engine.
Provides the same callback interface as OCRReceiver so the main app
can swap between iPhone-UDP and webcam-local seamlessly.
"""
from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable

import cv2
import numpy as np

try:
    import comtypes
    from pygrabber.dshow_graph import FilterGraph
except ImportError:
    comtypes = None
    FilterGraph = None

NUM_RE = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)")


@dataclass
class Reading:
    value: float
    timestamp: float
    received_at: float
    raw_text: str = ""


@dataclass
class ROI:
    x1: int = 0
    y1: int = 0
    x2: int = 80
    y2: int = 25


class WebcamOCR:
    """Webcam-based OCR that mimics the OCRReceiver interface."""

    # Suspicious integer-jump readings must repeat (within tolerance) this many
    # times before being accepted as a real value change rather than a glitch.
    JUMP_CONFIRM_FRAMES = 2
    # ...but never withhold a reading longer than this, so a persistent (even if
    # not exactly repeating) jump can't get stuck rejecting forever.
    JUMP_MAX_HOLD_SECONDS = 0.4

    def __init__(self, cam_index: int = 0,
                 value_range: Optional[tuple] = None,
                 reject_integer_jumps: bool = True):
        """
        :param value_range: optional (min, max) tuple. Readings outside this range
            are silently dropped.
        :param reject_integer_jumps: if True, a reading with no fractional part that
            differs from the previous accepted reading by more than 10x is treated
            as a likely missing-decimal-point misread (e.g. ".034" recognized as
            "34") and held rather than accepted immediately. It's only dropped if
            it fails to repeat within JUMP_CONFIRM_FRAMES frames/JUMP_MAX_HOLD_SECONDS
            — a genuine fast load change still gets through quickly, while a
            one-off OCR glitch (which rarely repeats identically) gets filtered.
        """
        self.cam_index = cam_index
        self.roi = ROI()
        self.value_range = value_range
        self.reject_integer_jumps = reject_integer_jumps
        self._cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._capture_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        # Latest-frame handoff between the capture thread and the OCR thread.
        # The capture thread drains the camera continuously so frames never
        # queue up in the driver buffer; each frame carries its own arrival
        # timestamps so OCR readings are stamped with when the frame was
        # captured, not when OCR happened to finish.
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_pc: float = 0.0
        self._latest_frame_wall: float = 0.0
        self._frame_count: int = 0
        self.capture_fps: float = 0.0
        self._latest: Optional[Reading] = None
        self._lock = threading.Lock()
        self.on_reading: Optional[Callable] = None
        self.last_elapsed_ms: float = 0.0
        self.last_frame: Optional[np.ndarray] = None
        self.last_crop: Optional[np.ndarray] = None
        self._ocr_engine = None
        self.rejected_count = 0
        self._pending_value: Optional[float] = None
        self._pending_count: int = 0
        self._pending_since: float = 0.0
        # Camera latency estimation: measures the full pipeline delay
        # (camera internal buffer + USB transfer + grab call blocking time).
        # This is subtracted from the timestamp so the OCR reading aligns
        # with when the display actually showed that number.
        self.camera_latency_ms: float = 190.0

    def _init_engine(self):
        from rapidocr_onnxruntime import RapidOCR
        ocr = RapidOCR()
        ocr.use_text_det = False
        ocr.use_angle_cls = False
        self._ocr_engine = ocr

    def _run_ocr(self, img: np.ndarray) -> str:
        result, _ = self._ocr_engine(img)
        if not result:
            return ""
        texts = [text for _, text, _ in result]
        return " ".join(texts)

    @staticmethod
    def _fix_dropped_decimal(raw: str) -> str:
        """Reconstruct a number whose decimal point was dropped by OCR.

        No real display renders an integer with a redundant leading zero
        (e.g. "0345") — the only way that text comes out of OCR is if the
        original was "0.345" and the dot was lost. Detecting this from the
        digit pattern itself is unambiguous, unlike guessing from history.
        "0.12" is untouched since it already has its dot.
        """
        sign = ""
        digits = raw
        if digits and digits[0] in "+-":
            sign = "-" if digits[0] == "-" else ""
            digits = digits[1:]
        if "." not in raw and len(digits) > 1 and digits[0] == "0":
            return f"{sign}0.{digits[1:]}"
        return raw

    @staticmethod
    def preprocess(crop: np.ndarray) -> np.ndarray:
        h, w = crop.shape[:2]
        if max(h, w) > 640:
            scale = 640 / max(h, w)
            crop = cv2.resize(crop, None, fx=scale, fy=scale,
                              interpolation=cv2.INTER_AREA)
        elif max(h, w) < 100:
            scale = 100 / max(h, w)
            crop = cv2.resize(crop, None, fx=scale, fy=scale,
                              interpolation=cv2.INTER_CUBIC)
        return crop

    def start(self) -> "WebcamOCR":
        if self._thread and self._thread.is_alive():
            return self
        self._init_engine()
        self._cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW)
        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(self.cam_index)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open camera {self.cam_index}")
        # MJPG first: raw YUY2 at 1280x720 saturates USB2 at ~10 fps, while
        # MJPG reaches 30 fps — three times fresher frames for the same camera.
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self._cap.set(cv2.CAP_PROP_FPS, 30)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # A busy or wedged device can "open" successfully yet never deliver a
        # frame (endless MSMF grab errors). Verify one frame arrives before
        # declaring the camera connected, so the failure surfaces as an error
        # instead of a silently dead stream.
        deadline = time.time() + 5.0
        got_frame = False
        while time.time() < deadline:
            ret, frame = self._cap.read()
            if ret and frame is not None:
                got_frame = True
                break
            time.sleep(0.1)
        if not got_frame:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
            raise RuntimeError(
                f"Camera {self.cam_index} opened but is not delivering frames — "
                "it may be in use by another app or wedged. Unplug/replug or "
                "close other apps using it, then press Refresh.")
        self._stop.clear()
        self._latest_frame = None
        self._frame_count = 0
        self._capture_thread = threading.Thread(target=self._capture_loop,
                                                daemon=True, name="WebcamCapture")
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="WebcamOCR")
        self._capture_thread.start()
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._thread = None
        if self._capture_thread:
            self._capture_thread.join(timeout=2.0)
        self._capture_thread = None
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def _capture_loop(self):
        """Drain the camera as fast as it delivers frames.

        Because a read is always pending, the driver buffer never accumulates
        stale frames — so a frame's read-return time is an honest capture
        timestamp. The old single-loop design let frames queue during the
        100-300 ms OCR step and then stamped an old frame as 'now', adding a
        variable error that made pairing OCR with HADI samples jittery.
        """
        fail_count = 0
        last_pc = None
        while not self._stop.is_set():
            cap = self._cap
            if cap is None or not cap.isOpened():
                time.sleep(0.5)
                continue
            ret, frame = cap.read()
            if not ret or frame is None:
                fail_count += 1
                time.sleep(min(0.5, 0.05 * fail_count))
                continue
            fail_count = 0
            now_pc = time.perf_counter()
            now_wall = time.time()
            if last_pc is not None:
                dt = now_pc - last_pc
                if dt > 0:
                    # EMA of frame rate, for the status display.
                    self.capture_fps = 0.9 * self.capture_fps + 0.1 * (1.0 / dt) \
                        if self.capture_fps else 1.0 / dt
            last_pc = now_pc
            with self._frame_lock:
                self._latest_frame = frame
                self._latest_frame_pc = now_pc
                self._latest_frame_wall = now_wall
                self._frame_count += 1
            self.last_frame = frame

    def _loop(self):
        last_processed = 0
        while not self._stop.is_set():
            with self._frame_lock:
                frame = self._latest_frame
                frame_id = self._frame_count
                capture_pc = self._latest_frame_pc
                capture_wall = self._latest_frame_wall

            if frame is None or frame_id == last_processed:
                # No new frame yet — OCR is faster than the camera right now.
                time.sleep(0.005)
                continue
            last_processed = frame_id

            r = self.roi
            h, w = frame.shape[:2]
            x1 = max(0, min(r.x1, w))
            y1 = max(0, min(r.y1, h))
            x2 = max(0, min(r.x2, w))
            y2 = max(0, min(r.y2, h))

            if (x2 - x1) < 10 or (y2 - y1) < 10:
                time.sleep(0.05)
                continue

            crop = frame[y1:y2, x1:x2].copy()
            self.last_crop = crop
            processed = self.preprocess(crop)

            t0 = time.perf_counter()
            text = self._run_ocr(processed)
            self.last_elapsed_ms = (time.perf_counter() - t0) * 1000

            numbers = NUM_RE.findall(text)
            if numbers:
                try:
                    raw_num = self._fix_dropped_decimal(numbers[0])
                    val = float(raw_num)

                    if self.value_range is not None:
                        lo, hi = self.value_range
                        if val < lo or val > hi:
                            self.rejected_count += 1
                            continue

                    suspicious = False
                    if self.reject_integer_jumps and val == int(val):
                        prev = self._latest.value if self._latest else None
                        if prev is not None and prev != 0 and abs(val) > abs(prev) * 10:
                            suspicious = True

                    if suspicious:
                        now = time.perf_counter()
                        tol = max(0.5, abs(self._pending_value or 0) * 0.01)
                        if self._pending_value is not None and abs(val - self._pending_value) <= tol:
                            self._pending_count += 1
                        else:
                            self._pending_value = val
                            self._pending_count = 1
                            self._pending_since = now

                        confirmed = (self._pending_count >= self.JUMP_CONFIRM_FRAMES or
                                     (now - self._pending_since) >= self.JUMP_MAX_HOLD_SECONDS)
                        if not confirmed:
                            self.rejected_count += 1
                            continue
                        self._pending_value = None
                        self._pending_count = 0
                    else:
                        self._pending_value = None
                        self._pending_count = 0

                    reading = Reading(
                        value=val,
                        timestamp=capture_wall,
                        received_at=capture_wall,
                        raw_text=raw_num,
                    )
                    reading.capture_pc = capture_pc
                    with self._lock:
                        self._latest = reading
                    if self.on_reading:
                        try:
                            self.on_reading(reading)
                        except Exception:
                            pass
                except ValueError:
                    pass

    def latest(self) -> Optional[float]:
        with self._lock:
            return self._latest.value if self._latest else None

    def latest_reading(self) -> Optional[Reading]:
        with self._lock:
            return self._latest

    @staticmethod
    def list_cameras(max_check: int = 5) -> list[dict]:
        """Return info about available cameras.

        When pygrabber is available, device names come straight from a
        DirectShow enumeration (no camera is opened), which is fast and safe
        to call even while another index is actively capturing. Without it,
        we fall back to probing each index by actually opening it via OpenCV
        — slower, and this fallback path must not run while a camera is
        already open, since opening the same DirectShow/MSMF device from two
        places at once is what wedges the driver.
        """
        if FilterGraph is not None:
            # COM must be initialized on whichever thread calls this. The
            # main thread usually gets this for free from other libraries,
            # but a plain background thread doesn't — without it, the very
            # first call here throws and silently falls through to the slow
            # per-index probe below.
            needs_co_uninit = False
            try:
                comtypes.CoInitialize()
                needs_co_uninit = True
            except OSError:
                pass
            try:
                device_names = FilterGraph().get_input_devices()
                return [{"index": i, "name": name, "resolution": ""}
                        for i, name in enumerate(device_names)]
            except Exception:
                pass
            finally:
                if needs_co_uninit:
                    comtypes.CoUninitialize()

        cameras = []
        for i in range(max_check):
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if cap.isOpened():
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                cameras.append({"index": i, "name": f"Camera {i}", "resolution": f"{w}x{h}"})
                cap.release()
        return cameras
