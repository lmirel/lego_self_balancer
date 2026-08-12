"""Thin subprocess transport for the proven pybricksdev BLE workflow."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
from typing import Callable


LineHandler = Callable[[str], None]


def run_ble_program(program: Path, on_line: LineHandler, hub_name: str | None = None) -> int:
    """Run one hub program, forwarding each output line live and to ``on_line``."""
    executable = Path(sys.executable).with_name("pybricksdev")
    command = [str(executable), "run", "ble"]
    if hub_name:
        command.extend(["--name", hub_name])
    command.append(program.name)

    environment = os.environ.copy()
    environment["PYTHONUNBUFFERED"] = "1"

    # Work around the Homebrew Python/Expat linker mismatch observed on this Mac.
    expat_lib = Path("/opt/homebrew/opt/expat/lib")
    if sys.platform == "darwin" and expat_lib.is_dir():
        existing = environment.get("DYLD_LIBRARY_PATH")
        environment["DYLD_LIBRARY_PATH"] = (
            f"{expat_lib}:{existing}" if existing else str(expat_lib)
        )

    process = subprocess.Popen(
        command,
        cwd=program.parent,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    assert process.stdout is not None
    try:
        for line in process.stdout:
            print(line, end="", flush=True)
            on_line(line)
        return process.wait()
    except KeyboardInterrupt:
        # Let pybricksdev handle SIGINT first so it can stop/disconnect cleanly.
        process.send_signal(signal.SIGINT)
        try:
            return process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            return process.wait(timeout=5)
