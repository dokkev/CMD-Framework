# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Build a weighted grasp-start cache from ordinary and post-rotation grasps."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

_COMPATIBILITY_KEYS = (
    "hand",
    "joint_names",
    "cube_size_m",
    "grasp_mcp_deg",
    "grasp_pip_deg",
    "pad_crown_mm",
    "pad_crown_layout",
)
_ARRAY_KEYS = ("q", "pos", "quat", "target")


def _load(path: Path) -> dict:
    data = torch.load(path, map_location="cpu", weights_only=True)
    missing = [key for key in (*_COMPATIBILITY_KEYS, "q", "pos", "quat") if key not in data]
    if missing:
        raise ValueError(f"{path} is missing cache fields: {', '.join(missing)}")
    data.setdefault("target", data["q"].clone())
    n = len(data["q"])
    if n == 0 or data["pos"].shape != (n, 3) or data["quat"].shape != (n, 4) or data["target"].shape != data["q"].shape:
        raise ValueError(f"{path} must contain matching nonempty q, target, pos and quat arrays")
    if not all(torch.isfinite(data[key]).all() for key in _ARRAY_KEYS):
        raise ValueError(f"{path} contains non-finite grasp values")
    return data


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, required=True, help="ordinary stable-grasp cache")
    parser.add_argument("--curriculum", type=Path, required=True, help="post-rotation grasp cache")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fraction", type=float, default=0.2, help="desired post-rotation rows in the combined cache")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    if not 0.0 < args.fraction < 1.0:
        parser.error("--fraction must lie strictly between 0 and 1")
    base_path, curriculum_path = args.base.resolve(strict=True), args.curriculum.resolve(strict=True)
    if args.output.resolve() in (base_path, curriculum_path):
        parser.error("--output must differ from both input caches")
    if args.output.exists():
        parser.error(f"destination already exists: {args.output}")
    base, curriculum = _load(base_path), _load(curriculum_path)
    for key in _COMPATIBILITY_KEYS:
        if base[key] != curriculum[key]:
            raise ValueError(f"grasp caches differ in {key}: {base[key]!r} != {curriculum[key]!r}")

    n_base = len(base["q"])
    n_curriculum = round(args.fraction * n_base / (1.0 - args.fraction))
    n_unique = len(curriculum["q"])
    generator = torch.Generator(device="cpu").manual_seed(args.seed)
    cycles, remainder = divmod(n_curriculum, n_unique)
    indices = [torch.randperm(n_unique, generator=generator) for _ in range(cycles)]
    if remainder:
        indices.append(torch.randperm(n_unique, generator=generator)[:remainder])
    curriculum_indices = torch.cat(indices)
    arrays = {key: torch.cat((base[key], curriculum[key][curriculum_indices]), dim=0) for key in _ARRAY_KEYS}
    permutation = torch.randperm(n_base + n_curriculum, generator=generator)
    arrays = {key: value[permutation] for key, value in arrays.items()}
    metadata = {key: value for key, value in base.items() if key not in _ARRAY_KEYS}
    metadata.update(
        {
            "q": arrays["q"],
            "target": arrays["target"],
            "pos": arrays["pos"],
            "quat": arrays["quat"],
            "source_base_cache": str(base_path),
            "source_curriculum_cache": str(curriculum_path),
            "requested_curriculum_fraction": args.fraction,
            "actual_curriculum_fraction": n_curriculum / (n_base + n_curriculum),
            "curriculum_states_unique": n_unique,
            "curriculum_rows": n_curriculum,
            "mix_seed": args.seed,
        }
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as stream:
        torch.save(metadata, stream)
    print(
        json.dumps(
            {
                "base_rows": n_base,
                "curriculum_unique_rows": n_unique,
                "curriculum_rows": n_curriculum,
                "actual_curriculum_fraction": metadata["actual_curriculum_fraction"],
                "output": str(args.output.resolve()),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
