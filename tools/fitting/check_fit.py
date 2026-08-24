#!/usr/bin/env python3
"""Fail when the held-out Garmin carry target is not met."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    after = report["fitted_metrics"]["validation"]
    carry_mae = after["carry_mae_yd"]
    if carry_mae > 5.0:
        print(f"validation Garmin fit did not meet the target: carry_mae_yd: {carry_mae:.3f} yd (must be <= 5.000 yd)")
        return 1
    print(f"validation Garmin carry MAE is {carry_mae:.3f} yd (target <= 5.000 yd)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
