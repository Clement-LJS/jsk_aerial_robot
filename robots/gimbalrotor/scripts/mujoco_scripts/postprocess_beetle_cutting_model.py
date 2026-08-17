#!/usr/bin/env python3

import argparse
import math
import os
import tempfile
import xml.etree.ElementTree as ET

import yaml


NAMES_TO_REPLACE = {
    "cutting_contact_site",
    "perching_hinge_connect_1",
    "perching_hinge_connect_2",
    "simulated_cutting_branch",
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--config", required=True)
    return parser.parse_args()


def vector3(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains a non-finite value")
    return result


def format_values(values):
    return " ".join(f"{value:.12g}" for value in values)


def normalize(vector, name):
    length = math.sqrt(sum(value * value for value in vector))
    if not math.isfinite(length) or length <= 1.0e-12:
        raise ValueError(f"{name} must be non-zero")
    return [value / length for value in vector]


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rpy_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # MuJoCo quaternion order: w x y z.
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def matrix_vector(matrix, vector):
    return [
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def quaternion_z_to_axis(axis):
    # Quaternion rotating local +Z onto the requested axis.
    target = normalize(axis, "branch axis")
    dot = max(-1.0, min(1.0, target[2]))

    if dot > 1.0 - 1.0e-12:
        return [1.0, 0.0, 0.0, 0.0]

    if dot < -1.0 + 1.0e-12:
        return [0.0, 1.0, 0.0, 0.0]

    cross = [-target[1], target[0], 0.0]
    w = math.sqrt((1.0 + dot) * 0.5)
    scale = 1.0 / (2.0 * w)
    return [w, cross[0] * scale, cross[1] * scale, cross[2] * scale]


def find_named(root, tag, name):
    for element in root.iter(tag):
        if element.get("name") == name:
            return element
    raise ValueError(f"MuJoCo element not found: <{tag} name=\"{name}\">")


def remove_existing_named_elements(root):
    for parent in root.iter():
        for child in list(parent):
            if child.get("name") in NAMES_TO_REPLACE:
                parent.remove(child)


def get_or_create_equality(root):
    equality = root.find("equality")
    if equality is not None:
        return equality

    equality = ET.Element("equality")
    insert_index = len(root)
    for index, child in enumerate(list(root)):
        if child.tag in ("actuator", "sensor", "include"):
            insert_index = index
            break
    root.insert(insert_index, equality)
    return equality


def main():
    args = parse_args()

    if not os.path.isfile(args.model):
        raise FileNotFoundError(f"MuJoCo model does not exist: {args.model}")
    if not os.path.isfile(args.config):
        raise FileNotFoundError(f"cutting config does not exist: {args.config}")

    with open(args.config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    if not isinstance(config, dict) or not isinstance(config.get("model"), dict):
        raise ValueError("config must contain a model mapping")

    model_config = config["model"]

    root_body_name = str(model_config.get("root_body_name", "base_link"))
    saw_body_name = str(model_config.get("saw_body_name", "saw"))
    contact_site_name = str(
        model_config.get("cutting_contact_site_name", "cutting_contact_site")
    )
    constraint_1_name = str(
        model_config.get("constraint_1_name", "perching_hinge_connect_1")
    )
    constraint_2_name = str(
        model_config.get("constraint_2_name", "perching_hinge_connect_2")
    )

    pivot_offset_base = vector3(
        model_config["pivot_offset_base"], "model/pivot_offset_base"
    )
    pivot_world = vector3(model_config["pivot_world"], "model/pivot_world")
    initial_rpy = vector3(
        model_config.get("initial_base_rpy", [0.0, 0.0, 0.0]),
        "model/initial_base_rpy",
    )
    pivot_axis_base = normalize(
        vector3(model_config["pivot_axis_base"], "model/pivot_axis_base"),
        "model/pivot_axis_base",
    )
    contact_point_saw = vector3(
        model_config["cutting_contact_point_saw"],
        "model/cutting_contact_point_saw",
    )

    anchor_half_span = float(model_config["anchor_half_span"])
    if not math.isfinite(anchor_half_span) or anchor_half_span <= 0.0:
        raise ValueError("model/anchor_half_span must be positive")

    solref_value = model_config.get("constraint_solref", [0.005, 1.0])
    if not isinstance(solref_value, (list, tuple)) or len(solref_value) != 2:
        raise ValueError("model/constraint_solref must contain two values")
    solref = [float(value) for value in solref_value]
    if not all(math.isfinite(value) for value in solref):
        raise ValueError("model/constraint_solref contains a non-finite value")
    solimp_value = model_config.get("constraint_solimp", [0.95, 0.99, 0.001])
    if not isinstance(solimp_value, (list, tuple)) or len(solimp_value) != 3:
        raise ValueError("model/constraint_solimp must contain three values")
    solimp = [float(value) for value in solimp_value]

    tree = ET.parse(args.model)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MuJoCo model has no worldbody")

    base_body = find_named(root, "body", root_body_name)
    saw_body = find_named(root, "body", saw_body_name)

    remove_existing_named_elements(root)

    rotation = rpy_to_matrix(*initial_rpy)
    rotated_offset = matrix_vector(rotation, pivot_offset_base)
    base_position = [
        pivot_world[index] - rotated_offset[index] for index in range(3)
    ]
    base_body.set("pos", format_values(base_position))
    base_body.set("quat", format_values(rpy_to_quaternion(*initial_rpy)))

    ET.SubElement(
        saw_body,
        "site",
        {
            "name": contact_site_name,
            "pos": format_values(contact_point_saw),
            "size": "0.006",
            "rgba": "1 0 0 1",
        },
    )

    anchor_1 = [
        pivot_offset_base[index] - anchor_half_span * pivot_axis_base[index]
        for index in range(3)
    ]
    anchor_2 = [
        pivot_offset_base[index] + anchor_half_span * pivot_axis_base[index]
        for index in range(3)
    ]

    equality = get_or_create_equality(root)
    common_constraint_attributes = {
        "body1": root_body_name,
        "active": (
            "true"
            if bool(model_config.get("constraints_active_at_start", True))
            else "false"
        ),
        "solref": format_values(solref),
        "solimp": format_values(solimp),
    }

    connect_1_attributes = dict(common_constraint_attributes)
    connect_1_attributes.update(
        {"name": constraint_1_name, "anchor": format_values(anchor_1)}
    )
    ET.SubElement(equality, "connect", connect_1_attributes)

    connect_2_attributes = dict(common_constraint_attributes)
    connect_2_attributes.update(
        {"name": constraint_2_name, "anchor": format_values(anchor_2)}
    )
    ET.SubElement(equality, "connect", connect_2_attributes)

    branch_config = model_config.get("branch_visual", {})
    if bool(branch_config.get("enabled", True)):
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": str(
                    branch_config.get("name", "simulated_cutting_branch")
                ),
                "type": "cylinder",
                "pos": format_values(pivot_world),
                "quat": format_values(quaternion_z_to_axis(pivot_axis_base)),
                "size": format_values(
                    [
                        float(branch_config.get("radius", 0.015)),
                        float(branch_config.get("half_length", 0.25)),
                    ]
                ),
                "rgba": format_values(
                    [
                        float(value)
                        for value in branch_config.get(
                            "rgba", [0.45, 0.25, 0.08, 1.0]
                        )
                    ]
                ),
                "contype": "0",
                "conaffinity": "0",
            },
        )

    output_directory = os.path.dirname(os.path.abspath(args.model))
    file_descriptor, temporary_path = tempfile.mkstemp(
        prefix=".robot_cutting_",
        suffix=".xml",
        dir=output_directory,
    )
    os.close(file_descriptor)

    try:
        tree.write(temporary_path, encoding="utf-8", xml_declaration=True)
        # Parse once more before replacing the model.
        ET.parse(temporary_path)
        os.replace(temporary_path, args.model)
    finally:
        if os.path.exists(temporary_path):
            os.unlink(temporary_path)

    print(f"Post-processed MuJoCo cutting model: {args.model}")


if __name__ == "__main__":
    main()
