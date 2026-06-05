from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "configs" / "train_special_target.yaml"
DEFAULT_WEIGHTS = REPO_ROOT / "runs" / "special_target" / "train" / "weights" / "best.pt"


def load_config(path: str | Path = DEFAULT_CONFIG) -> dict[str, Any]:
    config_path = resolve_path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    return config


def prepare_dataset_yaml(data_yaml: str | Path, project: str | Path) -> Path:
    source_yaml = resolve_path(data_yaml)
    if not source_yaml.exists():
        raise FileNotFoundError(f"Dataset YAML not found: {source_yaml}")

    with source_yaml.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    dataset_root = source_yaml.parent
    raw_path = data.get("path")
    if raw_path:
        raw_path = Path(raw_path)
        dataset_root = raw_path if raw_path.is_absolute() else (source_yaml.parent / raw_path)

    normalized = dict(data)
    normalized["path"] = str(dataset_root.resolve())

    output_dir = resolve_path(project)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_yaml = output_dir / "dataset_resolved.yaml"
    with output_yaml.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(normalized, handle, sort_keys=False, allow_unicode=True)
    return output_yaml


def resolve_path(path: str | Path) -> Path:
    value = Path(path)
    if value.is_absolute():
        return value
    return (REPO_ROOT / value).resolve()


def as_output_dict(data: dict[str, Any]) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


def require_cuda_if_requested(device: str | int | None) -> None:
    if device is None or str(device).lower() in {"", "none", "cpu"}:
        return

    import torch

    if not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA is not available, but the config requests GPU device "
            f"{device!r}. Set device: cpu in the config or fix the GPU environment."
        )
