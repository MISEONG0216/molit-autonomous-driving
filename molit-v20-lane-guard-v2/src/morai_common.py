from __future__ import annotations

import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MORAI_DIR = PROJECT_ROOT / "third_party" / "MORAI-NetworkModule"
CONFIG_PATH = PROJECT_ROOT / "config" / "network.json"


def ensure_morai_import_path() -> None:
    if not MORAI_DIR.exists():
        raise RuntimeError(
            "MORAI Network Module이 없습니다. 프로젝트 루트에서 ./setup.sh를 먼저 실행하세요."
        )
    sys.path.insert(0, str(MORAI_DIR))


def load_network_config() -> dict:
    with CONFIG_PATH.open("r", encoding="utf-8") as f:
        return json.load(f)
