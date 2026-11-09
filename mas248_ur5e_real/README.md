# MAS248 Lecture 10 — Group 19 planning-only bringup

This reconstruction adds the five Python scripts missing from the course ZIP.
The course's driver and MoveIt launch files use the calibrated description
generated from the group's canonical workcell. The group MoveIt SRDF refers to
`spindle`, so the generated description keeps that link name.

## Placement and offline check

Extract the archive into the root of the group's Git repository. The resulting
`mas248_ur5e_real/` directory belongs alongside the existing `mas248_ur5e/`,
`mas248_ur5e_demos/` and `mas248_ur5e_moveit_config/` packages. If you already
have a copy of `mas248_ur5e_real/`, compare it before replacing it and keep only
one discoverable package of that name in the workspace.

Use the measured model from `main` and the course UR5e calibration YAML at
`~/ur5e_calibration.yaml`. The build expects the Humble UR packages installed
for the lab. From the workspace root, with the ROS underlay sourced:

```bash
source /opt/ros/humble/setup.bash
cd ~/mas248_ws
colcon build --packages-up-to mas248_ur5e_real
source install/setup.bash
ros2 run mas248_ur5e_real validate_description.py \
  --robot-ip 192.168.1.101 --reverse-ip 192.168.1.102 \
  --calibration "$HOME/ur5e_calibration.yaml"
```

`validate_description.py` renders the URDF and verifies the measured pedestal,
the TCP transform, the selected calibration and the SRDF link names. The IP
arguments are rendered into the model for inspection. This script performs no
network operations.

## Planning in the lab

Use the course's two separate launches with `enable_execution:=false` in both.
Use the lab-assigned ROS domain in every terminal. Confirm the physical robot's
calibration in the driver's log, then follow the instructor's procedure for
External Control on the pendant.

`validate_stack.py` checks the live planning-only setup. Use
`run_picture_real.py --targets <file> --validate-only` to check a JSON offline.
The preflight below checks the stack and plans the selected contour from the
current arm state. Replace the target path if the circle was generated on
another PC or stored elsewhere.

```bash
ros2 run mas248_ur5e_real preflight.py \
  --calibration "$HOME/ur5e_calibration.yaml" \
  --targets "$HOME/mas248_ws/src/mas248_ur5e/mas248_ur5e_real/examples/circle/targets.json" \
  --max-strokes 1
```

The drawing script requests plans and publishes an RViz preview. This student
workflow ends at planning. Contact height, paper placement, physical calibration
and any motion on hardware are checked separately with the instructor.
