"""Run a fingertip-task module's own command line with hydroelastic contact (hydro.enable(); experimental, the
point-vs-hydro rerun). The module is imported and its main() called with the remaining arguments; Pool workers fork
from this process and inherit the patch. No task module is edited.

    uv run python hydro_run.py slider_rail run --hands O --draws logs/slider_e40.json --tag e40h --only 32 \
        --config '{"time_step": 0.001}'
"""
import importlib
import sys

import hydro

if __name__ == "__main__":
    hydro.enable()
    mod = importlib.import_module(sys.argv[1])
    sys.argv = [f"{sys.argv[1]}.py"] + sys.argv[2:]
    raise SystemExit(mod.main())
