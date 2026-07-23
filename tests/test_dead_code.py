"""Repository-wide dead-code enforcement tests."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def test_vulture_reports_no_dead_code() -> None:
    """Reject unreviewed dead Python code across source and tests."""
    repository_root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [sys.executable, "-m", "vulture"],
        cwd=repository_root,
        capture_output=True,
        text=True,
        check=False,
    )
    output = "\n".join(part for part in (result.stdout, result.stderr) if part)
    assert result.returncode == 0, output
