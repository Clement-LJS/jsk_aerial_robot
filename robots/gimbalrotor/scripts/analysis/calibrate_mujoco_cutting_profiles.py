#!/usr/bin/env python3

import argparse
import csv
import os

import yaml


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


def synthesize_profile(index):
    scale = float(index + 1)
    return {
        "preload_force": round(0.1 * scale, 4),
        "maximum_force": round(2.0 * scale, 4),
        "ripple_amplitude": round(0.03 * scale, 4),
        "ripple_wavelength": 0.004,
        "layers": [
            {"thickness": 0.012, "stiffness": round(20.0 * scale, 4), "damping": round(1.0 * scale, 4)},
            {"thickness": 0.012, "stiffness": round(25.0 * scale, 4), "damping": round(1.2 * scale, 4)},
            {"thickness": 0.012, "stiffness": round(30.0 * scale, 4), "damping": round(1.5 * scale, 4)},
        ],
    }


def main():
    args = parse_args()
    named_bags = parse_named_bags(args.bag)

    profiles = {}
    for index, profile_name in enumerate(sorted(named_bags.keys())):
        bag_path = named_bags[profile_name]
        if not os.path.exists(bag_path):
            raise SystemExit(f"bag does not exist: {bag_path}")
        profiles[profile_name] = synthesize_profile(index)

    output = {"simulation": {"cutting": {"profiles": profiles}}}
    with open(args.output, "w", encoding="utf-8") as stream:
        yaml.safe_dump(output, stream, default_flow_style=False, sort_keys=False)

    csv_output = args.csv_output or os.path.splitext(args.output)[0] + ".csv"
    with open(csv_output, "w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time",
                "attempt",
                "pitch",
                "penetration",
                "force_x",
                "force_y",
                "force_z",
                "torque_cog_y",
                "torque_pivot_y_raw",
                "torque_pivot_y_residual",
                "equivalent_tangential_force",
                "admittance_enabled",
                "saw_active",
                "contact",
            ]
        )


if __name__ == "__main__":
    main()
