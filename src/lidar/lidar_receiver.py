import socket
import threading
import time
import numpy as np

from lidar_parser import parse_buffer
from lidar_preprocess import preprocess_points


# ============================================================
# MORAI VLP16 LiDAR Receiver
#
# Module #1
#
# Pipeline:
#   MORAI UDP
#       ↓
#   Packet validation
#       ↓
#   Azimuth scan segmentation
#       ↓
#   Scan integrity check
#       ↓
#   VLP16 parsing
#       ↓
#   Invalid point removal
#       ↓
#   ROI filtering
#       ↓
#   Latest Point Cloud
#
# Output:
#   np.ndarray shape = (N, 4)
#   [x, y, z, intensity]
#
# Coordinate:
#   +X = Right
#   -X = Left
#   +Y = Forward
#   -Y = Backward
#   +Z = Up
#   -Z = Down
#
# NOT performed here:
#   - Ground removal
#   - Clustering
#   - Object detection
#   - Tracking
#   - Vehicle control
# ============================================================


IP = "127.0.0.1"
PORT = 1116

PACKET_SIZE = 1206
PAYLOAD_SIZE = 1200


# ============================================================
# Scan quality settings
#
# IMPORTANT:
# 75 packets/scan is an EMPIRICAL observation from the
# current MORAI VLP16 configuration.
#
# It is NOT used for scan segmentation.
# Scan segmentation is always based on azimuth wrap.
#
# This range is only used as a diagnostic integrity check.
# ============================================================

EXPECTED_PACKETS = 75

MIN_SCAN_PACKETS = 70
MAX_SCAN_PACKETS = 80


# Safety cap only.
#
# If an azimuth wrap is not detected for an abnormally long
# period, discard the partial scan and resynchronize instead
# of allowing scan_payloads to grow indefinitely.
#
# This is NOT a LiDAR specification and is NOT used to define
# a normal scan.
MAX_ACCUMULATED_PACKETS = 200


# Current empirical scan:
# start ~ 2.70 deg
# end   ~ 357.90 deg
#
# Use generous diagnostic limits.
MIN_START_AZIMUTH = 0.0
MAX_START_AZIMUTH = 15.0

MIN_END_AZIMUTH = 345.0
MAX_END_AZIMUTH = 360.0


# ============================================================
# Freshness
#
# Current simulator is heavily lagging and one scan currently
# takes roughly 1.4~1.5 s wall-clock time.
#
# Therefore the stale threshold must currently be larger than
# that observed interval.
#
# This is a software safety/diagnostic setting, not an official
# LiDAR specification.
# ============================================================

STALE_TIMEOUT_SEC = 3.0


# ============================================================
# TEST ROI
#
# Development/tuning value.
# NOT an official competition fixed value.
# ============================================================

ROI = {
    "x_min": -10.0,
    "x_max": 10.0,

    "y_min": -5.0,
    "y_max": 30.0,

    "z_min": -2.5,
    "z_max": 2.0,
}


# ============================================================
# Utility
# ============================================================

def get_first_azimuth(packet):

    low = packet[2]
    high = packet[3]

    raw = (
        float(low)
        + 256.0 * float(high)
    )

    return raw / 100.0


def crossed_zero(previous_azimuth, current_azimuth):

    if previous_azimuth is None:
        return False

    return (
        previous_azimuth
        - current_azimuth
    ) > 180.0


# ============================================================
# Receiver
# ============================================================

class LidarReceiver:

    def __init__(
        self,
        ip=IP,
        port=PORT,
        roi=ROI
    ):

        self.ip = ip
        self.port = port
        self.roi = roi

        self.sock = None

        # ----------------------------------------------------
        # Scan segmentation state
        # ----------------------------------------------------

        self.pending_packet = None
        self.pending_azimuth = None

        self.synchronized = False


        # ----------------------------------------------------
        # Background receiver
        # ----------------------------------------------------

        self.thread = None
        self.running = False


        # ----------------------------------------------------
        # Shared latest data
        # ----------------------------------------------------

        self.lock = threading.Lock()

        self.latest_points = None
        self.latest_stats = None


        # ----------------------------------------------------
        # Statistics
        # ----------------------------------------------------

        self.scan_count = 0

        self.valid_scan_count = 0
        self.dropped_scan_count = 0

        self.invalid_packet_count = 0
        self.socket_timeout_count = 0

        self.last_frame_time = None


    # ========================================================
    # Socket
    # ========================================================

    def connect(self):

        if self.sock is not None:
            return

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_RCVBUF,
            4 * 1024 * 1024
        )

        # Allows stop() to terminate the receiver thread.
        self.sock.settimeout(0.5)

        self.sock.bind(
            (
                self.ip,
                self.port
            )
        )

        print(
            f"[LiDAR] UDP bind: "
            f"{self.ip}:{self.port}"
        )


    def close(self):

        if self.sock is not None:

            try:
                self.sock.close()

            except OSError:
                pass

            self.sock = None


    # ========================================================
    # Packet receive
    # ========================================================

    def receive_valid_packet(self):

        while self.running:

            try:

                data, _ = self.sock.recvfrom(
                    2048
                )

            except socket.timeout:

                self.socket_timeout_count += 1
                continue

            except OSError:

                return None


            if len(data) != PACKET_SIZE:

                self.invalid_packet_count += 1
                continue


            return data


        return None


    # ========================================================
    # Initial synchronization
    # ========================================================

    def initial_synchronize(self):

        previous_azimuth = None

        discarded_packets = 0


        while self.running:

            packet = self.receive_valid_packet()

            if packet is None:
                return None


            current_azimuth = (
                get_first_azimuth(packet)
            )


            if crossed_zero(
                previous_azimuth,
                current_azimuth
            ):

                self.synchronized = True

                return (
                    packet,
                    current_azimuth,
                    discarded_packets
                )


            previous_azimuth = (
                current_azimuth
            )

            discarded_packets += 1


        return None


    # ========================================================
    # One continuous scan
    # ========================================================

    def receive_one_scan(self):

        # ----------------------------------------------------
        # Scan start
        # ----------------------------------------------------

        if not self.synchronized:

            result = (
                self.initial_synchronize()
            )

            if result is None:
                return None


            (
                first_packet,
                start_azimuth,
                discarded_packets
            ) = result


        else:

            if self.pending_packet is None:
                return None


            first_packet = (
                self.pending_packet
            )

            start_azimuth = (
                self.pending_azimuth
            )

            discarded_packets = 0


        # Current scan consumes pending packet.
        self.pending_packet = None
        self.pending_azimuth = None


        scan_payloads = [
            first_packet[:PAYLOAD_SIZE]
        ]

        packet_count = 1

        previous_azimuth = (
            start_azimuth
        )

        end_azimuth = (
            start_azimuth
        )


        start_time = time.perf_counter()


        # ----------------------------------------------------
        # Receive until next azimuth wrap
        # ----------------------------------------------------

        while self.running:

            packet = self.receive_valid_packet()

            if packet is None:
                return None


            current_azimuth = (
                get_first_azimuth(packet)
            )


            if crossed_zero(
                previous_azimuth,
                current_azimuth
            ):

                # First packet of next scan.
                self.pending_packet = (
                    packet
                )

                self.pending_azimuth = (
                    current_azimuth
                )

                break


            scan_payloads.append(
                packet[:PAYLOAD_SIZE]
            )

            packet_count += 1

            end_azimuth = (
                current_azimuth
            )

            previous_azimuth = (
                current_azimuth
            )


            # ------------------------------------------------
            # Safety cap
            #
            # Normal scan segmentation is still determined
            # only by packet-level azimuth wrap.
            #
            # This cap exists solely to prevent an abnormal
            # stream from growing scan_payloads indefinitely.
            # ------------------------------------------------

            if (
                packet_count
                >= MAX_ACCUMULATED_PACKETS
            ):

                print(
                    "[LiDAR WARNING] "
                    "scan accumulation safety cap reached; "
                    "resynchronizing"
                )

                self.synchronized = False

                self.pending_packet = None
                self.pending_azimuth = None

                self.dropped_scan_count += 1

                return None


        if not self.running:
            return None


        receive_ms = (
            time.perf_counter()
            - start_time
        ) * 1000.0


        return {
            "buffer":
                b"".join(scan_payloads),

            "packets":
                packet_count,

            "start_azimuth":
                start_azimuth,

            "end_azimuth":
                end_azimuth,

            "receive_ms":
                receive_ms,

            "initial_drop":
                discarded_packets,
        }


    # ========================================================
    # Scan integrity
    # ========================================================

    def check_scan_integrity(self, scan):
        """
        Scan segmentation itself is based on azimuth.

        This function performs only a diagnostic integrity
        check on the completed scan.

        return:
            valid_scan
            quality
            reasons
        """

        reasons = []


        packet_count = (
            scan["packets"]
        )

        start_azimuth = (
            scan["start_azimuth"]
        )

        end_azimuth = (
            scan["end_azimuth"]
        )


        # ----------------------------------------------------
        # Packet count
        # ----------------------------------------------------

        if (
            packet_count < MIN_SCAN_PACKETS
            or
            packet_count > MAX_SCAN_PACKETS
        ):

            reasons.append(
                "packet_count_out_of_range"
            )


        # ----------------------------------------------------
        # Start azimuth
        # ----------------------------------------------------

        if not (
            MIN_START_AZIMUTH
            <= start_azimuth
            <= MAX_START_AZIMUTH
        ):

            reasons.append(
                "abnormal_start_azimuth"
            )


        # ----------------------------------------------------
        # End azimuth
        # ----------------------------------------------------

        if not (
            MIN_END_AZIMUTH
            <= end_azimuth
            <= MAX_END_AZIMUTH
        ):

            reasons.append(
                "abnormal_end_azimuth"
            )


        # ----------------------------------------------------
        # Quality label
        #
        # EXACT:
        # current observed expected packet count
        #
        # ACCEPTABLE:
        # scan is structurally usable but packet count differs
        # slightly from current observation
        #
        # INVALID:
        # outside diagnostic limits
        # ----------------------------------------------------

        if len(reasons) > 0:

            return (
                False,
                "INVALID",
                reasons
            )


        if packet_count == EXPECTED_PACKETS:

            quality = "EXACT"

        else:

            quality = "ACCEPTABLE"


        return (
            True,
            quality,
            reasons
        )


    # ========================================================
    # Parse + preprocess
    # ========================================================

    def process_scan(self, scan):

        # ----------------------------------------------------
        # Parse
        # ----------------------------------------------------

        t0 = time.perf_counter()


        raw_points = parse_buffer(
            scan["buffer"]
        )


        parse_ms = (
            time.perf_counter()
            - t0
        ) * 1000.0


        # ----------------------------------------------------
        # Preprocess
        # ----------------------------------------------------

        t0 = time.perf_counter()


        points, pre_stats = (
            preprocess_points(
                raw_points,
                roi=self.roi
            )
        )


        preprocess_ms = (
            time.perf_counter()
            - t0
        ) * 1000.0


        return (
            points,
            pre_stats,
            parse_ms,
            preprocess_ms
        )


    # ========================================================
    # Background loop
    # ========================================================

    def _receive_loop(self):

        while self.running:

            scan = self.receive_one_scan()

            if scan is None:
                continue


            self.scan_count += 1


            # ------------------------------------------------
            # Integrity check
            # ------------------------------------------------

            (
                valid_scan,
                quality,
                reasons
            ) = self.check_scan_integrity(
                scan
            )


            if not valid_scan:

                self.dropped_scan_count += 1

                print(
                    f"[LiDAR WARNING] "
                    f"scan={self.scan_count} "
                    f"dropped | "
                    f"packets={scan['packets']} | "
                    f"az="
                    f"{scan['start_azimuth']:.2f}"
                    f"->"
                    f"{scan['end_azimuth']:.2f} | "
                    f"reason={','.join(reasons)}"
                )

                continue


            # ------------------------------------------------
            # Parse + preprocess only usable scans
            # ------------------------------------------------

            try:

                (
                    points,
                    pre_stats,
                    parse_ms,
                    preprocess_ms
                ) = self.process_scan(scan)


            except Exception as error:

                self.dropped_scan_count += 1

                print(
                    f"[LiDAR ERROR] "
                    f"scan={self.scan_count} "
                    f"processing failed: "
                    f"{error}"
                )

                continue


            self.valid_scan_count += 1

            frame_time = time.monotonic()


            stats = {

                "scan":
                    self.scan_count,

                "valid_scan":
                    self.valid_scan_count,

                "quality":
                    quality,

                "packets":
                    scan["packets"],

                "packet_delta":
                    scan["packets"]
                    - EXPECTED_PACKETS,

                "start_azimuth":
                    scan["start_azimuth"],

                "end_azimuth":
                    scan["end_azimuth"],

                "raw":
                    pre_stats["raw"],

                "valid":
                    pre_stats["valid"],

                "roi":
                    pre_stats["roi"],

                "receive_ms":
                    scan["receive_ms"],

                "parse_ms":
                    parse_ms,

                "preprocess_ms":
                    preprocess_ms,

                "initial_drop":
                    scan["initial_drop"],

                "dropped_scans":
                    self.dropped_scan_count,

                "invalid_packets":
                    self.invalid_packet_count,

                "socket_timeouts":
                    self.socket_timeout_count,

                "timestamp":
                    frame_time,
            }


            # ------------------------------------------------
            # Publish latest complete frame
            # ------------------------------------------------

            with self.lock:

                self.latest_points = points
                self.latest_stats = stats

                self.last_frame_time = (
                    frame_time
                )


    # ========================================================
    # Public API
    # ========================================================

    def start(self):

        if self.running:
            return


        self.connect()


        # Reset runtime segmentation state.
        self.pending_packet = None
        self.pending_azimuth = None

        self.synchronized = False


        # ----------------------------------------------------
        # Reset previous-session latest frame
        #
        # start -> stop -> start 시 이전 세션의 Point Cloud를
        # 새 데이터로 오인하지 않도록 초기화한다.
        # ----------------------------------------------------

        with self.lock:

            self.latest_points = None
            self.latest_stats = None
            self.last_frame_time = None


        self.running = True


        self.thread = threading.Thread(
            target=self._receive_loop,
            name="MORAI-LiDAR-Receiver",
            daemon=True
        )


        self.thread.start()


        print(
            "[LiDAR] background receiver started"
        )


    def stop(self):

        if not self.running:
            return


        self.running = False


        if self.thread is not None:

            self.thread.join(
                timeout=2.0
            )

            self.thread = None


        self.close()


        print(
            "[LiDAR] background receiver stopped"
        )


    def get_latest(self, copy=True):
        """
        Non-blocking.

        return:
            points
            stats

        Before first valid frame:
            None, None
        """

        with self.lock:

            if self.latest_points is None:

                return (
                    None,
                    None
                )


            if copy:

                points = (
                    self.latest_points.copy()
                )

            else:

                points = (
                    self.latest_points
                )


            # Stats are always copied.
            #
            # age_sec / fresh are consumer-time values and
            # must never modify the receiver's internal
            # latest_stats dictionary.
            stats = dict(
                self.latest_stats
            )


        # Age is calculated when consumer requests data.
        stats["age_sec"] = (
            time.monotonic()
            - stats["timestamp"]
        )

        stats["fresh"] = (
            stats["age_sec"]
            <= STALE_TIMEOUT_SEC
        )


        return (
            points,
            stats
        )


    def get_status(self):
        """
        Receiver 전체 상태 확인.

        Point Cloud 자체가 필요하지 않은 모듈에서도
        LiDAR 상태를 확인할 수 있다.
        """

        with self.lock:

            if self.last_frame_time is None:

                age_sec = None
                fresh = False

            else:

                age_sec = (
                    time.monotonic()
                    - self.last_frame_time
                )

                fresh = (
                    age_sec
                    <= STALE_TIMEOUT_SEC
                )


            return {

                "running":
                    self.running,

                "synchronized":
                    self.synchronized,

                "scan_count":
                    self.scan_count,

                "valid_scan_count":
                    self.valid_scan_count,

                "dropped_scan_count":
                    self.dropped_scan_count,

                "invalid_packet_count":
                    self.invalid_packet_count,

                "socket_timeout_count":
                    self.socket_timeout_count,

                "age_sec":
                    age_sec,

                "fresh":
                    fresh,
            }


# ============================================================
# Standalone verification
# ============================================================

def main():

    receiver = LidarReceiver()

    receiver.start()


    print()

    print(
        "======================================"
    )

    print(
        " MORAI VLP16 Receiver Verification"
    )

    print(
        "======================================"
    )

    print(
        "Segmentation : azimuth wrap"
    )

    print(
        "Integrity    : packet/azimuth check"
    )

    print(
        "Output       : latest (N,4)"
    )

    print(
        "Ctrl+C로 종료"
    )

    print()


    last_scan = 0


    try:

        while True:

            points, stats = (
                receiver.get_latest()
            )


            if points is None:

                time.sleep(0.05)
                continue


            if stats["scan"] != last_scan:

                last_scan = (
                    stats["scan"]
                )


                freshness = (
                    "FRESH"
                    if stats["fresh"]
                    else "STALE"
                )


                print(
                    f"[SCAN {stats['scan']:04d}] "
                    f"{stats['quality']:10s} | "
                    f"packets="
                    f"{stats['packets']:3d} "
                    f"({stats['packet_delta']:+d}) | "
                    f"az="
                    f"{stats['start_azimuth']:6.2f}"
                    f"->"
                    f"{stats['end_azimuth']:6.2f} | "
                    f"points={len(points):5d} | "
                    f"raw={stats['raw']:5d} | "
                    f"recv="
                    f"{stats['receive_ms']:7.1f}ms | "
                    f"parse="
                    f"{stats['parse_ms']:4.1f}ms | "
                    f"pre="
                    f"{stats['preprocess_ms']:4.1f}ms | "
                    f"{freshness} | "
                    f"drop="
                    f"{stats['dropped_scans']}"
                )


            time.sleep(0.02)


    except KeyboardInterrupt:

        print()


    finally:

        receiver.stop()


        status = (
            receiver.get_status()
        )


        print()

        print(
            "========== FINAL STATUS =========="
        )

        print(
            f"Total scans     : "
            f"{status['scan_count']}"
        )

        print(
            f"Valid scans     : "
            f"{status['valid_scan_count']}"
        )

        print(
            f"Dropped scans   : "
            f"{status['dropped_scan_count']}"
        )

        print(
            f"Invalid packets : "
            f"{status['invalid_packet_count']}"
        )

        print(
            f"Socket timeouts : "
            f"{status['socket_timeout_count']}"
        )


if __name__ == "__main__":
    main()
