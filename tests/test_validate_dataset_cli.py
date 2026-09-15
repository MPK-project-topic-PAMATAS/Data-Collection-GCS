"""Command-line tests for the dataset validator."""

from pathlib import Path
import subprocess
import sys


def test_help_exits_successfully():
    """The CLI parser should handle --help before validating a path."""
    script = Path(__file__).parents[1] / "validate_dataset.py"
    result = subprocess.run(
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0
    assert "Validate a stereo dataset" in result.stdout
