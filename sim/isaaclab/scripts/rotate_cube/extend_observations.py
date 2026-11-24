"""Warm-start an RSL-RL checkpoint when observation features are appended."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
import torch.nn.functional as functional


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--features", required=True, type=int, help="number of appended features")
    args = parser.parse_args()
    if args.features < 1:
        parser.error("--features must be positive")
    if args.output.exists():
        parser.error(f"destination already exists: {args.output}")
    checkpoint = torch.load(args.source, map_location="cpu", weights_only=True)
    for key in ("actor_state_dict", "critic_state_dict"):
        model = checkpoint[key]
        first_layer = model["mlp.0.weight"]
        model["mlp.0.weight"] = functional.pad(first_layer, (0, args.features))
        for stat, value in (("_mean", 0.0), ("_var", 1.0), ("_std", 1.0)):
            name = f"obs_normalizer.{stat}"
            if name in model:
                model[name] = functional.pad(model[name], (0, args.features), value=value)
    checkpoint.pop("optimizer_state_dict", None)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as output:
        torch.save(checkpoint, output)
    print(f"Wrote {args.output}. Load with a fresh optimizer.")


if __name__ == "__main__":
    main()
