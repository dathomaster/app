"""
gps_receiver.py - Receives GPS coordinates sent from the iPhone OCR app.

Packet formats:
    GPS\t<unix_timestamp>\t<latitude>\t<longitude>\n
    GPS\t<unix_timestamp>\t<latitude>\t<longitude>\t<altitude_m>\n

Altitude is optional. If absent, altitude is None and the main app assumes
the 0 m / sea-level MF.
"""
import socket, time, argparse
from typing import Optional

class GPSReceiver:
    def __init__(self, port: int = 9999, host: str = "0.0.0.0"):
        self.port = port
        self._latest: Optional[dict] = None
        import threading
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while True:
            try:
                data, _ = self._sock.recvfrom(256)
                for line in data.decode("utf-8", errors="ignore").splitlines():
                    if not line.startswith("GPS\t"):
                        continue
                    parts = line.split("\t")
                    if len(parts) < 4:
                        continue
                    fix = {
                        "timestamp": float(parts[1]),
                        "latitude": float(parts[2]),
                        "longitude": float(parts[3]),
                        "altitude": None,
                        "received_at": time.time(),
                    }
                    if len(parts) >= 5 and parts[4].strip() != "":
                        try:
                            fix["altitude"] = float(parts[4])
                        except ValueError:
                            fix["altitude"] = None
                    self._latest = fix
            except socket.timeout:
                continue
            except Exception:
                break

    def latest(self) -> Optional[dict]:
        return self._latest

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9999)
    args = ap.parse_args()

    rx = GPSReceiver(port=args.port)
    print(f"Listening for GPS packets on UDP :{args.port} ...")
    try:
        seen = None
        while True:
            fix = rx.latest()
            if fix and fix != seen:
                alt = fix.get("altitude")
                alt_str = f"  alt={alt:.1f}m" if alt is not None else "  alt=None"
                print(f"lat={fix['latitude']:.8f}  lon={fix['longitude']:.8f}{alt_str}  (ts {fix['timestamp']:.0f})")
                seen = fix
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
