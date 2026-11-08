"""Run every CMD-Framework test from design/, whatever directory pytest is started in.

Some tests and configs use paths relative to the code root (``scripts/optimize/...``). Before the monorepo
layout the code root was the repository root; it is now ``design/``. Changing the working directory once
collection starts (after pytest has resolved its arguments and ``testpaths``) keeps those paths valid when
pytest is started from the repository root (``pytest.ini``) or from ``design/``.
"""

import os
from pathlib import Path

import pytest

DESIGN = Path(__file__).resolve().parents[1]


@pytest.hookimpl(tryfirst=True)
def pytest_collection(session):
    os.chdir(DESIGN)
