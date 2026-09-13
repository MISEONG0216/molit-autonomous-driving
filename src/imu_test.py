import time

from morai_common import ensure_morai_import_path, load_network_config

ensure_morai_import_path()

from lib.define.IMU import IMU
from lib.network.UDP import Receiver


def main():
    config = load_network_config()["imu"]
    receiver = Receiver(config["ip"], int(config["port"]), IMU())

    print(f"IMU 수신 대기: {config['ip']}:{config['port']}")
    print("Ctrl+C로 종료")

    while True:
        imu = receiver.get_data()
        print(
            "orientation "
            f"w={imu.ori_w:.6f}, x={imu.ori_x:.6f}, "
            f"y={imu.ori_y:.6f}, z={imu.ori_z:.6f} | "
            f"ang_vel_z={imu.ang_vel_z:.6f}"
        )
        time.sleep(0.1)


if __name__ == "__main__":
    main()
