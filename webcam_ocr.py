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

NUM_RE = re.compile(r"[-+]?\d+\.?\d*")


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
    x2: int = 160
    y2: int = 50


class WebcamOCR:
    """Webcam-based OCR that mimics the OCRReceiver interface."""

    def __init__(self, cam_index: int = 0):
        self.cam_index = cam_index
        self.roi = ROI()
        self._cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._latest: Optional[Reading] = None
        self._lock = threading.Lock()
        self.on_reading: Optional[Callable] = None
        self.last_elapsed_ms: float = 0.0
        self.last_frame: Optional[np.ndarray] = None
        self.last_crop: Optional[np.ndarray] = None
        self._ocr_engine = None
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
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="WebcamOCR")
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._thread = None
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def _loop(self):
        fail_count = 0
        while not self._stop.is_set():
            if self._cap is None or not self._cap.isOpened():
                time.sleep(0.5)
                continue

            ret, frame = self._cap.read()
            if not ret or frame is None:
                fail_count += 1
                time.sleep(min(0.5, 0.05 * fail_count))
                continue
            fail_count = 0

            capture_wall = time.time()
            capture_pc = time.perf_counter()

            self.last_frame = frame

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
                    val = float(numbers[0])
                    reading = Reading(
                        value=val,
                        timestamp=capture_wall,
                        received_at=capture_wall,
                        raw_text=numbers[0],
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

            time.sleep(0.01)

    def latest(self) -> Optional[float]:
        with self._lock:
            return self._latest.value if self._latest else None

    def latest_reading(self) -> Optional[Reading]:
        with self._lock:
            return self._latest

    @staticmethod
    def list_cameras(max_check: int = 5) -> list[dict]:
        """Probe camera indices and return info about available ones."""
        cameras = []
        for i in range(max_check):
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if cap.isOpened():
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                cameras.append({"index": i, "name": f"Camera {i}", "resolution": f"{w}x{h}"})
                cap.release()
        return cameras
