import json
from pathlib import Path

PATH_DIR = Path(__file__).resolve().parent / "parking_paths"

def load_path(spot_name: str) -> dict:
    file_path = PATH_DIR / f"{spot_name}.json"
    with open(file_path, "r", encoding="utf-8") as f:
        return json.load(f)