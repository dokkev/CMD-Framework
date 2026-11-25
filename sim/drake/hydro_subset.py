"""Run a task module's own analysis on the first N randomized draws only (the point side of the point-vs-hydro
rerun, docs/experimental/drake_hydro.md): the module's loader is wrapped to drop draws >= N, then its main() runs
with the remaining arguments. No task module is edited.

    uv run python hydro_subset.py 32 slider_rail analyze --tag e40 --noise_tag e40n --out logs/e40_slider_first32.json
"""
import importlib
import sys

LOADERS = {"slider_rail": "_load", "ball_roll": "_load", "box_pivot": "_load", "sphere_grasp": "_load",
           "coin_drag": "load", "analyze_roll": "tip_load"}


def _draw(k):
    return k[1] if isinstance(k, tuple) else k


if __name__ == "__main__":
    n, name = int(sys.argv[1]), sys.argv[2]
    mod = importlib.import_module(name)
    fn = getattr(mod, LOADERS[name])

    def loader(*a, **kw):
        d = fn(*a, **kw)
        return None if d is None else {k: v for k, v in d.items() if _draw(k) < n}
    setattr(mod, LOADERS[name], loader)
    sys.argv = [f"{name}.py"] + sys.argv[3:]
    raise SystemExit(mod.main())
