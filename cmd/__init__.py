"""Command-framework source package.

The package name is also used by Python's standard-library ``cmd`` module.
Re-exporting its ``Cmd`` class keeps standard-library consumers such as
``pdb`` working while allowing ``cmd.morphology`` to be imported from this
repository.
"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sysconfig


_stdlib_cmd_path = Path(sysconfig.get_path("stdlib")) / "cmd.py"
_stdlib_cmd_spec = spec_from_file_location("_python_stdlib_cmd", _stdlib_cmd_path)
if _stdlib_cmd_spec is None or _stdlib_cmd_spec.loader is None:
    raise ImportError(f"could not load the standard-library cmd module from {_stdlib_cmd_path}")

_stdlib_cmd = module_from_spec(_stdlib_cmd_spec)
_stdlib_cmd_spec.loader.exec_module(_stdlib_cmd)
Cmd = _stdlib_cmd.Cmd

__all__ = ["Cmd"]
