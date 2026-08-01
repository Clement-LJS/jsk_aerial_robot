#!/usr/bin/env python3

import argparse
import json
import os


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

    os.makedirs(args.output_dir, exist_ok=True)
    summary = {
        "pid_bag": args.pid,
        "admittance_bag": args.admittance,
        "status": "placeholder_summary",
        "note": "Local gimbalrotor-side scaffold added from markdown spec; full bag analysis depends on the simulation topics being produced.",
    }
    with open(os.path.join(args.output_dir, "summary.json"), "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
