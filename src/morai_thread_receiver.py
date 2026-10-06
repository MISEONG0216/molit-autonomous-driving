#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Thread-based MORAI UDP receiver.

This is an experimental replacement for the original MORAI NetworkModule
Receiver on Windows/Python 3.13.

Key difference:
- Original: socket + multiprocessing.Process + ctypes object + Queue
- This:     socket + one background Thread + Lock

The original third_party/MORAI-NetworkModule/lib/network/UDP.py is NOT changed.
"""

import ctypes
import socket
import threading


class ThreadReceiver:
    def __init__(self, ip, port, data_type):
        self.ip = ip
        self.port = int(port)
        self.data_type_class = type(data_type)
        self.data_size = ctypes.sizeof(data_type)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)
        self.socket.bind((self.ip, self.port))
        self.socket.settimeout(0.5)

        self._lock = threading.Lock()
        self._latest = None
        self._packet_count = 0
        self._last_error = None
        self._running = True

        self._thread = threading.Thread(
            target=self._recv_loop,
            name=f"ThreadReceiver-{self.port}",
            daemon=True,
        )
        self._thread.start()

    def _recv_loop(self):
        while self._running:
            try:
                raw_data, _ = self.socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:
                if self._running:
                    self._last_error = repr(exc)
                break

            # Never copy beyond either the ctypes structure or received packet.
            copy_size = min(len(raw_data), self.data_size)
            if copy_size <= 0:
                continue

            try:
                parsed = self.data_type_class()
                ctypes.memmove(ctypes.addressof(parsed), raw_data, copy_size)

                try:
                    parsed.parsing()
                except Exception:
                    # Keep behavior compatible with the original MORAI module:
                    # some data types require parsing(), others do not.
                    pass

                with self._lock:
                    self._latest = parsed
                    self._packet_count += 1
                    self._last_error = None

            except Exception as exc:
                self._last_error = repr(exc)

    def get_data(self):
        with self._lock:
            return self._latest

    def get_stats(self):
        with self._lock:
            return {
                "packet_count": self._packet_count,
                "last_error": self._last_error,
                "data_size": self.data_size,
            }

    def close(self):
        self._running = False
        try:
            self.socket.close()
        except OSError:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
