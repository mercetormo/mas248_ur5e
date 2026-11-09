#!/usr/bin/env python3
"""Check live planning mode, the expected calibration YAML and a complete drawing plan."""
import sys

from run_picture_real import main


if __name__ == "__main__":
    sys.exit(main(preflight=True))
