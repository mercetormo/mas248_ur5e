#!/usr/bin/env python3
"""Render and check group 19's calibrated URDF without opening a connection."""
import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


def load_common():
    launch = Path(__file__).resolve().parents[1] / "launch"
    if not (launch / "_real_common.py").is_file():
        from ament_index_python.packages import get_package_share_directory

        launch = Path(get_package_share_directory("mas248_ur5e_real")) / "launch"
    sys.path.insert(0, str(launch))
    import _real_common

    return _real_common


def check_numbers(value, expected, label):
    actual = tuple(float(item) for item in value.split())
    if len(actual) != len(expected) or any(
        not math.isclose(a, b, rel_tol=0.0, abs_tol=1e-8)
        for a, b in zip(actual, expected)
    ):
        raise ValueError(f"{label}: expected {expected}, got {actual}.")


def check_fixed_joint(root, parent, child, xyz, rpy=(0, 0, 0)):
    matches = [
        joint for joint in root.findall("joint")
        if joint.find("child") is not None and joint.find("child").get("link") == child
    ]
    if len(matches) != 1:
        raise ValueError(f"Expected one joint mounting {child}.")
    joint = matches[0]
    if joint.get("type") != "fixed" or joint.find("parent").get("link") != parent:
        raise ValueError(f"Expected a fixed joint from {parent} to {child}.")
    origin = joint.find("origin")
    if origin is None:
        raise ValueError(f"Missing origin for {child}.")
    check_numbers(origin.get("xyz", "0 0 0"), xyz, f"{child} xyz")
    check_numbers(origin.get("rpy", "0 0 0"), rpy, f"{child} rpy")


def check_measured_workcell(xml):
    root = ET.fromstring(xml)
    check_fixed_joint(root, "world", "pedestal_link", (0, 0, 0))
    check_fixed_joint(root, "pedestal_link", "base_link", (0, 0, 0.303))
    check_fixed_joint(root, "tool0", "spindle", (0, 0, 0))
    check_fixed_joint(root, "spindle", "tcp", (0, -0.089, 0.052), (math.pi / 2, 0, 0))
    for category in ("visual", "collision"):
        part = root.find(f"./link[@name='pedestal_link']/{category}")
        cylinder = part.find("geometry/cylinder") if part is not None else None
        if cylinder is None or part.find("origin") is None:
            raise ValueError(f"Missing pedestal {category} cylinder or origin.")
        check_numbers(part.find("origin").get("xyz", ""), (0, 0, 0.1515),
                      f"pedestal {category} xyz")
        check_numbers(cylinder.get("radius", ""), (0.1025,), f"pedestal {category} radius")
        check_numbers(cylinder.get("length", ""), (0.303,), f"pedestal {category} height")
    return root


def read_calibration_hash(path):
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))
    kin = data.get("kinematics") if isinstance(data, dict) else None
    value = kin.get("hash") if isinstance(kin, dict) else None
    if not isinstance(value, str) or not value.startswith("calib_"):
        raise ValueError("Invalid UR calibration hash.")
    for name in ("shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"):
        joint = kin.get(name)
        if not isinstance(joint, dict) or any(
            isinstance(joint.get(axis), bool)
            or not isinstance(joint.get(axis), (int, float))
            or not math.isfinite(joint[axis])
            for axis in ("x", "y", "z", "roll", "pitch", "yaw")
        ):
            raise ValueError(f"Invalid calibration entry: {name}.")
    return value


def hardware_parameters(root):
    return {
        element.get("name"): (element.text or "").strip()
        for element in root.findall("./ros2_control/hardware/param")
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--reverse-ip", required=True)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--output", type=Path, help="Optional path for the rendered URDF")
    args = parser.parse_args()

    try:
        common = load_common()
        expected_hash = read_calibration_hash(args.calibration)
        reverse_ip = common.validate_ip(args.reverse_ip)
        settings = {**common.HARDWARE_DEFAULTS, "reverse_ip": reverse_ip}
        xml = common.render_description(args.robot_ip, str(args.calibration), **settings)
        root = check_measured_workcell(xml)
        params = hardware_parameters(root)
        for key, expected in {
            "robot_ip": args.robot_ip,
            "reverse_ip": reverse_ip,
            "kinematics/hash": expected_hash,
        }.items():
            if params.get(key) != expected:
                raise ValueError(f"Rendered hardware parameter {key} does not match.")
        semantic = common.moveit_parameters(xml, "scaled_joint_trajectory_controller")
        srdf = ET.fromstring(semantic["robot_description_semantic"])
        links = {link.get("name") for link in root.findall("link")}
        for pair in srdf.findall("disable_collisions"):
            if {pair.get("link1"), pair.get("link2")} - links:
                raise ValueError(f"SRDF references unknown links: {pair.attrib}")
        if args.output is not None:
            args.output.write_text(xml, encoding="utf-8")
    except Exception as error:
        print(f"Description validation failed: {error}", file=sys.stderr)
        return 1

    print("URDF and SRDF checked. Base: 303 mm. TCP in tool0: (0, -89, 52) mm.")
    print(f"Calibration hash in model: {expected_hash}. Verify it against the driver log in the lab.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
