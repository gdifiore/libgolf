#!/usr/bin/env python3
"""Fetch the public MIT-licensed Garmin R50 dataset used by the aero fitter.

The data is intentionally not committed to libgolf. It is a third-party asset
that can be refreshed independently while the source URL and extraction rule
remain reproducible here.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
import urllib.request
import zipfile
from pathlib import Path


SOURCE = {
    "url": "https://www.kaggle.com/api/v1/datasets/download/jamieb122/golf-swing-and-trajectory-data",
    "member": "golf_trajectory.csv",
    "output": "garmin_r50.csv",
    "license": "MIT",
    "citation": "Jamie Blummer, Golf Swing and Trajectory Data (Kaggle, 2025).",
}


def download(url: str, destination: Path) -> None:
    request = urllib.request.Request(url, headers={"User-Agent": "libgolf-data-fetch/1"})
    with urllib.request.urlopen(request) as response, destination.open("wb") as out:
        shutil.copyfileobj(response, out)


def extract_member(archive: Path, expected_name: str, destination: Path) -> None:
    with zipfile.ZipFile(archive) as zf:
        members = [name for name in zf.namelist() if name == expected_name or name.endswith("/" + expected_name)]
        if len(members) != 1:
            raise RuntimeError(f"expected exactly one {expected_name} in {archive}, found {members}")
        with zf.open(members[0]) as source, destination.open("wb") as out:
            shutil.copyfileobj(source, out)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent / "data" / "raw")
    parser.add_argument("--refresh", action="store_true", help="replace an existing extracted CSV")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    output = args.output_dir / SOURCE["output"]
    with tempfile.TemporaryDirectory(prefix="libgolf-fit-") as tmp:
        if output.exists() and not args.refresh:
            print(f"kept {output}")
        else:
            archive = Path(tmp) / "garmin_r50.zip"
            print("downloading garmin_r50")
            download(SOURCE["url"], archive)
            extract_member(archive, SOURCE["member"], output)
            print(f"wrote {output}")

    (args.output_dir / "manifest.json").write_text(json.dumps({"source": SOURCE}, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
