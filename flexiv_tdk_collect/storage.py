from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


def save_arrays(directory: Path, values: dict[str, list[Any]]) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for name, samples in values.items():
        array = np.asarray(samples) if name.startswith("timestamp") else _stack(samples)
        np.save(directory / f"{name}.npy", array)


def save_image(directory: Path, index: int, image: Any) -> None:
    import cv2

    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{index:06d}.png"
    if not cv2.imwrite(str(path), np.asarray(image)):
        raise OSError(f"failed to write image: {path}")


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")


def _stack(samples: list[Any]) -> np.ndarray:
    if not samples:
        return np.empty((0,), dtype=np.float64)
    return np.stack([np.asarray(value) for value in samples], axis=0)
