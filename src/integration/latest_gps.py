"""Latest-only MORAI NMEA GPS receiver (no multiprocessing or ctypes).

Reads valid $GPRMC/$GNRMC sentences on a background thread. Timestamp is
based on packet arrival, not coordinate changes: stationary GPS is valid.
"""
import socket
import threading
import time


def _degrees(value, hemisphere, degree_digits):
    degrees = int(value[:degree_digits])
    minutes = float(value[degree_digits:])
    result = degrees + minutes / 60.0
    return -result if hemisphere in ("S", "W") else result


class LatestGpsReceiver:
    def __init__(self, ip, port):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self._sock.bind((ip, int(port)))
        self._sock.settimeout(0.2)
        self._lock = threading.Lock()
        self._latest = None
        self._updated = None
        self._count = 0
        self._closed = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._closed:
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                line = data.decode("ascii", errors="ignore").strip()
                fields = line.split("*", 1)[0].split(",")
                if fields[0] not in ("$GPRMC", "$GNRMC"):
                    continue
                if len(fields) < 7 or fields[2] != "A":
                    continue
                if not fields[3] or not fields[5]:
                    continue
                lat = _degrees(fields[3], fields[4], 2)
                lon = _degrees(fields[5], fields[6], 3)
                if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                    continue
            except (ValueError, IndexError):
                continue
            with self._lock:
                self._latest = (lat, lon)
                self._updated = time.monotonic()
                self._count += 1

    def read(self):
        with self._lock:
            if self._latest is None:
                return None, float("inf"), self._count
            return self._latest, time.monotonic() - self._updated, self._count

    def close(self):
        self._closed = True
        self._sock.close()
        self._thread.join(timeout=0.5)
