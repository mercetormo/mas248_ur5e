#!/usr/bin/env python3
"""Check and plan a MAS248 drawing against the calibrated real workcell."""
import argparse
from pathlib import Path
import sys
import time

from validate_stack import check_ready


def main(*, preflight=False):
    parser = argparse.ArgumentParser(
        description="MAS248 Lecture 10 student planning-only drawing workflow."
    )
    parser.add_argument("--targets", required=True, type=Path, help="JSON in the world frame")
    parser.add_argument("--calibration", required=preflight, type=Path,
                        help="Compare the loaded model hash to this YAML")
    if not preflight:
        parser.add_argument("--validate-only", action="store_true",
                            help="Check target JSON offline")
    parser.add_argument("--max-strokes", type=int)
    args = parser.parse_args()
    if args.max_strokes is not None and args.max_strokes < 1:
        parser.error("--max-strokes must be at least 1.")

    try:
        from scripts.run_picture import load_targets, plan_picture, ros_pose

        data = load_targets(args.targets)
        strokes = data["strokes"]
        if args.max_strokes is not None:
            strokes = strokes[:args.max_strokes]
        count = sum(len(stroke["poses"]) for stroke in strokes)
        print(f"Validated {len(strokes)} selected stroke(s), {count} target pose(s).", flush=True)
        if getattr(args, "validate_only", False):
            return 0

        import rclpy
        from moveit_msgs.msg import DisplayTrajectory
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from scripts.common import cartesian, move_group

        rclpy.init(args=[])
        try:
            node = Node("mas248_run_picture_real")
            try:
                check_ready(node, calibration=args.calibration)
                publisher = node.create_publisher(
                    DisplayTrajectory, "/display_planned_path",
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                )
                state, segments = plan_picture(
                    node, strokes, lift_m=0.03, max_step=0.003, pose_factory=ros_pose,
                    move_group=move_group, cartesian=cartesian,
                )
                display = DisplayTrajectory()
                display.model_id = "mas248_ur5e"
                display.trajectory_start = state
                display.trajectory = [segment.trajectory for segment in segments if segment.moves]
                publisher.publish(display)
                until = time.monotonic() + 2.0
                while rclpy.ok() and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.1)
                node.get_logger().info(
                    f"PLAN ONLY complete: {len(strokes)} stroke(s). "
                    "Every Cartesian path reached its final waypoint with collision checking enabled."
                )
            finally:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
    except (Exception, KeyboardInterrupt) as error:
        print(f"Drawing preflight failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
