import time

from morai_common import ensure_morai_import_path, load_network_config

ensure_morai_import_path()

from lib.define.GPS import GPS
from lib.network.UDP import Receiver


def main():
    config = load_network_config()["gps"]
    receiver = Receiver(config["ip"], int(config["port"]), GPS())

    print(f"GPS 수신 대기: {config['ip']}:{config['port']}")
    print("Ctrl+C로 종료")

    while True:
        gps = receiver.get_data()
        gps.parsing()
        print(
            f"GPRMC lat={gps.gprmc.lat:.6f}, lon={gps.gprmc.lon:.6f} | "
            f"GPGGA lat={gps.gpgga.lat:.6f}, lon={gps.gpgga.lon:.6f}, alt={gps.gpgga.alt:.3f}"
        )
        time.sleep(0.1)


if __name__ == "__main__":
    main()
