#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Local, size-checked MORAI UDP receiver for static avoidance module.

Avoids third_party UDP.py's fixed-size memmove on short datagrams.
No multiprocessing; one background thread per sensor.
"""
import ctypes
import socket
import threading


class Receiver:
    def __init__(self, ip, port, data_type):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)
        self.socket.bind((ip, int(port)))
        self.socket.settimeout(0.2)
        self.data_size = ctypes.sizeof(data_type)
        self.data_class = type(data_type)
        self.parsed_data = data_type
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self.recv_udp_data, daemon=True)
        self.thread.start()

    def recv_udp_data(self):
        while self.running:
            try:
                raw_data, _ = self.socket.recvfrom(self.data_size + 1)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(raw_data) != self.data_size:
                print(f'[UDP WARNING] expected={self.data_size}, received={len(raw_data)}')
                continue
            obj = self.data_class()
            ctypes.memmove(ctypes.addressof(obj), raw_data, len(raw_data))
            with self.lock:
                self.parsed_data = obj

    def get_data(self):
        with self.lock:
            return self.parsed_data

    def close(self):
        self.running = False
        self.socket.close()
        if self.thread.is_alive():
            self.thread.join(timeout=0.5)
