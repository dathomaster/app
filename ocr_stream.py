"""
ocr_stream.py — Receive numeric OCR readings from the iPhone app over UDP.

Three ways to use this file:

1) As a library (drop into any project):

    from ocr_stream import OCRReceiver
    rx = OCRReceiver(port=9999).start()
    while True:
        value = rx.latest()          # most recent float, or None
        # or:
        value = rx.get(timeout=1.0)  # block until next reading arrives
        print(value)

2) As a CLI stream printer:
    python ocr_stream.py

3) As an HTTP server (so any app on any language can poll for the current number):
    python ocr_stream.py --http 8080
    # then: curl http://localhost:8080/latest
    # returns: {"value": 0.1234, "timestamp": 1716412345.123456, "age_ms": 12}

Wire protocol: newline-delimited UTF-8 packets of "<timestamp>\t<number>\n".
Stale frames are simply overwritten — latest() always returns the newest reading.
"""
from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, HTTPServer
from queue import Queue, Empty
from typing import Optional


@dataclass
class Reading:
    value: float
    timestamp: float       # phone-side unix time when OCR'd
    received_at: float     # local unix time when received
    raw_text: str = ""     # exact numeric text received from the phone


@dataclass
class GPSReading:
    timestamp: float       # phone-side unix time when GPS fix was sent
    latitude: float
    longitude: float
    altitude_m: Optional[float]
    received_at: float     # local unix time when received


class OCRReceiver:
    """UDP listener for the iPhone OCR streamer.

    Thread-safe. Call .start() to begin, .latest() to peek, .get() to block
    for the next reading, .stop() to clean up.
    """

    def __init__(self,
                 port: int = 9999,
                 host: str = "0.0.0.0",
                 queue_size: int = 256,
                 value_range: Optional[tuple] = None,
                 reject_integer_jumps: bool = False):
        """
        :param port: UDP port to listen on.
        :param host: bind address.
        :param queue_size: max queued readings before oldest are dropped.
        :param value_range: optional (min, max) tuple. Readings outside this range
            are silently dropped. Useful when you know your instrument only reads,
            e.g., -1.0 to 1.0 — catches OCR errors like a missing decimal point
            ("0.1359" misread as "1359") before they pollute your data.
        :param reject_integer_jumps: if True, a reading with no fractional part that
            differs from the previous reading by more than 10x is treated as a
            likely missing-decimal-point error and dropped. Belt-and-suspenders
            on top of value_range.
        """
        self.port = port
        self.host = host
        self.value_range = value_range
        self.reject_integer_jumps = reject_integer_jumps
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._latest: Optional[Reading] = None
        self._latest_gps: Optional[GPSReading] = None
        self._latest_lock = threading.Lock()
        self._queue: "Queue[Reading]" = Queue(maxsize=queue_size)
        self.on_reading = None  # optional callback: fn(Reading) -> None
        self.on_gps = None      # optional callback: fn(GPSReading) -> None
        self.on_heartbeat = None  # optional callback: fn() -> None
        self.last_packet_time = 0.0
        # Stats so you can see how aggressive the filtering is
        self.rejected_count = 0

    def start(self) -> "OCRReceiver":
        if self._thread and self._thread.is_alive():
            return self
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        s.bind((self.host, self.port))
        s.settimeout(0.5)
        self._sock = s
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="OCRReceiver")
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self):
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                data, _addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            self.last_packet_time = time.time()

            # A packet may contain one or more "ts\tnumber\n" lines
            for line in data.decode("utf-8", errors="ignore").splitlines():
                line = line.strip()
                if not line:
                    continue
                parts = line.split("\t")

                if parts and parts[0] == "HB":
                    if self.on_heartbeat is not None:
                        try:
                            self.on_heartbeat()
                        except Exception:
                            pass
                    continue

                # GPS packet from the iPhone app:
                # GPS\t<unix_timestamp>\t<latitude>\t<longitude>[\t<altitude_m>]
                if parts and parts[0] == "GPS":
                    try:
                        if len(parts) >= 4:
                            altitude_m = None
                            if len(parts) >= 5 and parts[4].strip() != "":
                                try:
                                    altitude_m = float(parts[4])
                                except ValueError:
                                    altitude_m = None
                            gps = GPSReading(
                                timestamp=float(parts[1]),
                                latitude=float(parts[2]),
                                longitude=float(parts[3]),
                                altitude_m=altitude_m,
                                received_at=time.time(),
                            )
                            with self._latest_lock:
                                self._latest_gps = gps
                            if self.on_gps is not None:
                                try:
                                    self.on_gps(gps)
                                except Exception:
                                    pass
                    except ValueError:
                        pass
                    continue

                try:
                    if len(parts) == 2:
                        ts = float(parts[0])
                        val = float(parts[1])
                        raw_str = parts[1]
                    else:
                        # tolerate "just a number" payload
                        ts = time.time()
                        val = float(parts[-1])
                        raw_str = parts[-1]
                except ValueError:
                    continue

                # Filter: value out of expected range?
                if self.value_range is not None:
                    lo, hi = self.value_range
                    if val < lo or val > hi:
                        self.rejected_count += 1
                        continue

                # Filter: suspicious integer jump (likely missing decimal point)?
                # "Effectively integer" = val equals its truncation. This catches
                # both "1359" and "1359.0" (which is how some OCR outputs render).
                if self.reject_integer_jumps and val == int(val):
                    prev = self._latest.value if self._latest else None
                    if prev is not None and prev != 0 and abs(val) > abs(prev) * 10:
                        self.rejected_count += 1
                        continue

                r = Reading(value=val, timestamp=ts, received_at=time.time(), raw_text=raw_str)
                with self._latest_lock:
                    self._latest = r
                try:
                    self._queue.put_nowait(r)
                except Exception:
                    # queue full — drop oldest, keep newest
                    try:
                        self._queue.get_nowait()
                        self._queue.put_nowait(r)
                    except Exception:
                        pass
                if self.on_reading is not None:
                    try:
                        self.on_reading(r)
                    except Exception:
                        pass

    # --- consumer API ---

    def latest(self) -> Optional[float]:
        """Most recent numeric reading, or None if none received yet."""
        with self._latest_lock:
            return self._latest.value if self._latest else None

    def latest_reading(self) -> Optional[Reading]:
        with self._latest_lock:
            return self._latest

    def latest_gps(self) -> Optional[GPSReading]:
        with self._latest_lock:
            return self._latest_gps

    def get(self, timeout: Optional[float] = None) -> Optional[float]:
        """Block until the next reading arrives (or timeout). Returns the float."""
        try:
            r = self._queue.get(timeout=timeout)
            return r.value
        except Empty:
            return None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


# ---------------------------------------------------------------------------
# Optional HTTP bridge — exposes the latest reading at GET /latest as JSON.
# Lets you integrate from any language: curl, JS fetch, LabVIEW, Excel, etc.
# ---------------------------------------------------------------------------

def _make_http_handler(rx: OCRReceiver):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args, **kwargs):
            return  # silence default access log
        def do_GET(self):
            if self.path.rstrip("/") in ("/latest", ""):
                r = rx.latest_reading()
                if r is None:
                    body = json.dumps({"value": None}).encode()
                else:
                    body = json.dumps({
                        "value": r.value,
                        "timestamp": r.timestamp,
                        "age_ms": int((time.time() - r.received_at) * 1000),
                    }).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404); self.end_headers()
    return Handler


def serve_http(rx: OCRReceiver, http_port: int):
    server = HTTPServer(("0.0.0.0", http_port), _make_http_handler(rx))
    t = threading.Thread(target=server.serve_forever, daemon=True, name="OCRHttp")
    t.start()
    return server


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _print_local_ips():
    """Print non-loopback IPv4s so the user knows what to type into the phone."""
    ips = set()
    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ips.add(info[4][0])
    except socket.gaierror:
        pass
    # Fallback: a UDP "connect" trick to get the outbound interface IP
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    ips.discard("127.0.0.1")
    if ips:
        print("Enter one of these IPs into the iPhone app:")
        for ip in sorted(ips):
            print(f"  {ip}")


def main():
    ap = argparse.ArgumentParser(description="iPhone numeric OCR UDP receiver")
    ap.add_argument("--port", type=int, default=9999, help="UDP port to listen on (default 9999)")
    ap.add_argument("--http", type=int, default=0, help="Also serve latest reading on this HTTP port")
    ap.add_argument("--capacity", type=float, default=None,
                    help="Load cell capacity. Auto-sets filter to +/-120%% of this value. "
                         "Use this for normal calibration work — handles 1%% to 100%% testing on any cell.")
    ap.add_argument("--min", type=float, default=None, help="Reject readings below this value (overrides --capacity)")
    ap.add_argument("--max", type=float, default=None, help="Reject readings above this value (overrides --capacity)")
    ap.add_argument("--reject-int-jumps", action="store_true",
                    help="Drop bare-integer readings that jump >10x — catches missed decimal points")
    ap.add_argument("--quiet", action="store_true", help="Don't print every reading")
    args = ap.parse_args()

    # Resolve the value range from --capacity and/or explicit --min/--max
    value_range = None
    if args.capacity is not None:
        if args.capacity <= 0:
            ap.error("--capacity must be a positive number")
        limit = args.capacity * 1.2
        lo, hi = -limit, limit
        # explicit --min/--max override the capacity-derived range
        if args.min is not None: lo = args.min
        if args.max is not None: hi = args.max
        value_range = (lo, hi)
        print(f"Cell capacity: {args.capacity}  →  filter range [{lo}, {hi}]")
    elif args.min is not None or args.max is not None:
        lo = args.min if args.min is not None else float("-inf")
        hi = args.max if args.max is not None else float("inf")
        value_range = (lo, hi)
        print(f"Filtering readings to range [{lo}, {hi}]")

    _print_local_ips()
    print(f"Listening on UDP :{args.port} ...  (Ctrl-C to quit)")

    rx = OCRReceiver(port=args.port,
                     value_range=value_range,
                     reject_integer_jumps=args.reject_int_jumps).start()
    if args.http:
        serve_http(rx, args.http)
        print(f"HTTP: GET http://localhost:{args.http}/latest")

    try:
        last_printed = None
        while True:
            v = rx.get(timeout=1.0)
            if v is None:
                continue
            if not args.quiet and v != last_printed:
                print(f"{v}")
                last_printed = v
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop()


if __name__ == "__main__":
    main()
