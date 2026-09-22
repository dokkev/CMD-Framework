#!/usr/bin/env python3
"""Show three 2D fingertip views and one interactive 3D view.

Run without arguments to open a window, or use --output to save the figure.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cmd.visualization.visualize_fingertip import main


if __name__ == "__main__":
    raise SystemExit(main())
