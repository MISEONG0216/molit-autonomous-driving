import time

from morai_common import ensure_morai_import_path, load_network_config

ensure_morai_import_path()

from lib.define.EgoCtrlCmd import EgoCtrlCmd
from lib.network.UDP import Sender


def main():
    config = load_network_config()
    control = config["control"]
    target_speed = float(config["pure_pursuit"]["target_speed_kmh"])

    sender = Sender(control["ip"], int(control["port"]))
    cmd = EgoCtrlCmd()
    cmd.ctrl_mode = 2
    cmd.gear = 4
    cmd.cmd_type = 2
    cmd.velocity = target_speed
    cmd.steer = 0.0

    print(f"CtrlCmd 송신: {control['ip']}:{control['port']}")
    print(f"{target_speed:.1f} km/h 직진 명령을 3초간 송신합니다.")
    print("MORAI Ego Controller가 AV - ExternalCtrl 상태인지 확인하세요.")

    start = time.time()
    while time.time() - start < 3.0:
        sender.send(cmd)
        time.sleep(0.1)

    cmd.velocity = 0.0
    cmd.steer = 0.0
    print("정지 명령 송신")
    for _ in range(20):
        sender.send(cmd)
        time.sleep(0.1)

    print("CtrlCmd 테스트 종료")


if __name__ == "__main__":
    main()
