#!/usr/bin/env python3

import argparse
import os
import sys


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pid", required=True)
    parser.add_argument("--admittance", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    for path in (args.pid, args.admittance):
        if not os.path.exists(path):
            raise SystemExit(f"bag does not exist: {path}")

    raise SystemExit(
        "analyze_mujoco_cutting_comparison.py is intentionally disabled: "
        "the previous implementation wrote a placeholder summary without reading either bag. "
        "Replace it with a real bag-comparison pipeline before using it."
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
