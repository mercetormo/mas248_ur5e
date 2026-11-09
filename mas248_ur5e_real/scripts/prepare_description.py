#!/usr/bin/env python3
"""Generate the calibrated-driver Xacro from the team's workcell at build time."""
import argparse
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

import yaml


MODEL_FILES = ("joint_limits.yaml", "physical_parameters.yaml", "visual_parameters.yaml")
CONFIG_ARGUMENTS = {
    "joint_limits_parameters_file": "joint_limit_params",
    "kinematics_parameters_file": "kinematics_params",
    "physical_parameters_file": "physical_params",
    "visual_parameters_file": "visual_params",
}
DRIVER_FILES = ("script_filename", "input_recipe_filename", "output_recipe_filename")


def generate(source, output, config_output, ur_description, ur_driver):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.parse(source, parser=parser).getroot()
    if root.tag != "robot" or root.get("name") != "mas248_ur5e":
        raise ValueError("Expected the canonical mas248_ur5e workcell.")
    match = re.search(r'xmlns:xacro=["\']([^"\']+)', source.read_text(encoding="utf-8"))
    if match is None:
        raise ValueError("Canonical Xacro has no xacro namespace.")
    namespace = match.group(1)
    ET.register_namespace("xacro", namespace)
    xacro = "{" + namespace + "}"
    instances = root.findall(xacro + "ur_robot")
    if len(instances) != 1 or root.find(".//ros2_control") is not None:
        raise ValueError("Expected one UR robot macro and no explicit ros2_control system.")
    robot = instances[0]

    settings_file = Path(__file__).resolve().parents[1] / "config" / "real_robot_defaults.yaml"
    hardware = yaml.safe_load(settings_file.read_text(encoding="utf-8"))["hardware"]
    vendor = ET.parse(ur_description / "urdf" / "ur_macro.xacro").getroot()
    vendor_macro = next(
        (element for element in vendor if element.tag.endswith("}macro")
         and element.get("name") == "ur_robot"),
        None,
    )
    if vendor_macro is None:
        raise ValueError("Installed ur_description does not define ur_robot.")
    macro_parameters = vendor_macro.get("params", "")
    optional = {}
    if re.search(r"(?<!\S)use_currents_as_efforts(?=[:=\s]|$)", macro_parameters):
        optional["use_currents_as_efforts"] = "true"

    defaults = {
        **dict.fromkeys(CONFIG_ARGUMENTS.values(), ""),
        **dict.fromkeys(DRIVER_FILES, ""),
        **hardware,
        **optional,
        "robot_ip": "0.0.0.0",
        "ur_type": "ur5e",
        "tf_prefix": "",
        "name": "ur5e",
        "use_fake_hardware": "false",
        "fake_sensor_commands": "false",
    }
    existing = {}
    for arg in root.findall(xacro + "arg"):
        name = arg.get("name")
        if name in existing:
            raise ValueError(f"Duplicate canonical Xacro argument: {name}")
        existing[name] = arg
    for name, value in defaults.items():
        arg = existing.get(name)
        if arg is None:
            arg = ET.Element(xacro + "arg", {"name": name})
            root.insert(0, arg)
        arg.set("default", str(value).lower() if isinstance(value, bool) else str(value))

    forwarded = {
        **CONFIG_ARGUMENTS,
        **{name: name for name in (*DRIVER_FILES, *hardware, *optional)},
        **{name: name for name in ("name", "ur_type", "tf_prefix", "robot_ip")},
    }
    for parameter, arg in forwarded.items():
        robot.set(parameter, f"$(arg {arg})")
    robot.set("generate_ros2_control_tag", "true")
    robot.set("use_fake_hardware", "false")
    robot.set("fake_sensor_commands", "false")

    model_dir = ur_description / "config" / "ur5e"
    files = [model_dir / name for name in MODEL_FILES]
    update_rate = ur_driver / "config" / "ur5e_update_rate.yaml"
    for path in (*files, update_rate):
        if not path.is_file():
            raise FileNotFoundError(path)
    config_output.mkdir(parents=True, exist_ok=True)
    for path in files:
        shutil.copyfile(path, config_output / path.name)
    shutil.copyfile(update_rate, config_output.parent / update_rate.name)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config-output", required=True, type=Path)
    args = parser.parse_args()

    from ament_index_python.packages import get_package_share_directory

    try:
        generate(
            args.source, args.output, args.config_output,
            Path(get_package_share_directory("ur_description")),
            Path(get_package_share_directory("ur_robot_driver")),
        )
    except (OSError, RuntimeError, ValueError, ET.ParseError, yaml.YAMLError) as error:
        parser.exit(1, f"Could not generate real workcell: {error}\n")
    print(f"Generated real workcell from {args.source}: {args.output}")


if __name__ == "__main__":
    main()
