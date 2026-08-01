#!/usr/bin/env python3

import argparse
import os
import sys


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", action="append", required=True, help="Named bag input as profile=path")
    parser.add_argument("--output", required=True)
    parser.add_argument("--csv-output", default=None)
    return parser.parse_args()


def parse_named_bags(entries):
    bags = {}
    for entry in entries:
        if "=" not in entry:
            raise ValueError(f"invalid --bag entry: {entry}")
        name, path = entry.split("=", 1)
        if not name:
            raise ValueError(f"missing bag profile name in: {entry}")
        bags[name] = path
    return bags


def main():
    args = parse_args()
    named_bags = parse_named_bags(args.bag)
    for profile_name, bag_path in sorted(named_bags.items()):
        if not os.path.exists(bag_path):
            raise SystemExit(f"bag does not exist for profile '{profile_name}': {bag_path}")

    raise SystemExit(
        "calibrate_mujoco_cutting_profiles.py is intentionally disabled: "
        "the previous implementation synthesized fake calibration values from bag names. "
        "Replace it with a real rosbag-based calibration pipeline before using it."
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
