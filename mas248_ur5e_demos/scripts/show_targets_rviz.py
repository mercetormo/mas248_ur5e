#!/usr/bin/env python3
"""Replacement for the missing MAS248 Lecture 9 target viewer.

Reconstructed from the supplied JSON format, CLI and orientation tests.
This is not the original course file. It publishes visualization markers and
the image-frame transform; it does not plan or execute robot movements.
ROS imports stay inside functions so the quaternion helpers work offline.
"""

import argparse
import json
import math
from pathlib import Path


def _vector(values, size, name):
    result = tuple(float(v) for v in values)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(f'{name} must contain {size} finite numbers')
    return result


def _unit_quaternion(quaternion):
    q = _vector(quaternion, 4, 'Quaternion (qx, qy, qz, qw)')
    norm = math.hypot(*q)
    if norm < 1.e-12:
        raise ValueError('A zero quaternion does not define an orientation')
    return tuple(v / norm for v in q)


def rotate_vector_xyzw(quaternion, vector):
    """Rotate a vector by a quaternion in ROS (x, y, z, w) order."""
    x, y, z, w = _unit_quaternion(quaternion)
    vx, vy, vz = _vector(vector, 3, 'Vector')
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def target_axes(quaternion, length):
    """Return the TCP X, Y and Z axis vectors, expressed in its parent frame."""
    length = float(length)
    if not math.isfinite(length) or length <= 0:
        raise ValueError('Axis length must be positive and finite')
    return tuple(rotate_vector_xyzw(quaternion, axis) for axis in
                 ((length, 0., 0.), (0., length, 0.), (0., 0., length)))


def load_targets(filename):
    """Read the converter's JSON; preserve its existing world positions."""
    data = json.loads(Path(filename).expanduser().read_text(encoding='utf-8'))
    transform = data['frame_to_world']
    parent, child = transform['parent'], transform['child']
    if not all(isinstance(f, str) and f and not f.startswith('/')
               for f in (parent, child)) or parent == child:
        raise ValueError('The parent and child frame names must be distinct and valid')
    if data['drawing_frame'] != child:
        raise ValueError('drawing_frame must match the transform child frame')
    transform['xyz'] = _vector(transform['xyz'], 3, 'Image-frame translation')
    transform['quaternion_xyzw'] = _unit_quaternion(transform['quaternion_xyzw'])
    for key in ('width_m', 'height_m'):
        value = float(data['image'][key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'image.{key} must be positive and finite')
        data['image'][key] = value
    if not isinstance(data['strokes'], list) or not data['strokes']:
        raise ValueError('The JSON must contain at least one contour')
    for stroke in data['strokes']:
        poses = stroke['poses']
        if not isinstance(poses, list) or len(poses) < 2:
            raise ValueError('Each contour must contain at least two world poses')
        for pose in poses:
            position = _vector((pose[k] for k in ('x', 'y', 'z')), 3, 'TCP position')
            q = _unit_quaternion(pose[k] for k in ('qx', 'qy', 'qz', 'qw'))
            pose.update(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), position + q))
    return data


def build_markers(data, axis_every=10, axis_length_m=0.018,
                  axis_lift_m=0.0, show_orientations=True):
    """Build the documented MarkerArray using world poses from the JSON."""
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    if axis_every < 1 or not isinstance(axis_every, int):
        raise ValueError('axis_every must be a positive integer')
    if not math.isfinite(axis_lift_m):
        raise ValueError('Axis lift must be finite')
    target_axes((0., 0., 0., 1.), axis_length_m)
    parent = data['frame_to_world']['parent']
    child = data['frame_to_world']['child']
    colors = ((1., 0.1, 0.1), (0.1, 1., 0.1), (0.1, 0.3, 1.))
    palette = ((1., 0.65, 0.1), (0.2, 0.85, 1.), (0.9, 0.35, 0.9),
               (0.4, 1., 0.45), (1., 0.4, 0.25))
    markers = []

    def point(xyz):
        return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

    def marker(namespace, index, kind, frame, color):
        m = Marker()
        m.header.frame_id = frame  # Zero timestamp requests the latest TF.
        m.ns, m.id, m.type, m.action = namespace, index, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        markers.append(m)
        return m

    def arrow(namespace, index, frame, start, direction, color):
        m = marker(namespace, index, Marker.ARROW, frame, color)
        end = tuple(a + b for a, b in zip(start, direction))
        m.points = [point(start), point(end)]
        length = math.hypot(*direction)
        m.scale.x, m.scale.y, m.scale.z = 0.05 * length, 0.12 * length, 0.22 * length

    # Clear markers from an earlier run, including contours no longer present.
    clear = Marker()
    clear.header.frame_id = parent
    clear.action = Marker.DELETEALL
    markers.append(clear)

    # The rectangle is the input-image extent at the drawing plane, not paper.
    half_w = data['image']['width_m'] / 2.0
    half_h = data['image']['height_m'] / 2.0
    bounds = marker('MAS248_image_bounds', 0, Marker.LINE_STRIP, child, (0.8, 0.8, 0.8))
    bounds.scale.x = 0.001
    bounds.points = [point((x, y, 0.)) for x, y in
                     ((-half_w, -half_h), (half_w, -half_h),
                      (half_w, half_h), (-half_w, half_h), (-half_w, -half_h))]
    for i, direction in enumerate(target_axes((0., 0., 0., 1.), 0.04)):
        arrow('MAS248_image_frame_axes', i, child, (0., 0., 0.), direction, colors[i])

    axis_id = 0
    for stroke_id, stroke in enumerate(data['strokes']):
        poses = stroke['poses']
        positions = [(p['x'], p['y'], p['z']) for p in poses]
        color = palette[stroke_id % len(palette)]
        path = marker('MAS248_contour_paths', stroke_id, Marker.LINE_STRIP, parent, color)
        path.scale.x = 0.0015
        path.points = [point(p) for p in positions]
        dots = marker('MAS248_TCP_waypoints', stroke_id, Marker.POINTS, parent, color)
        dots.scale.x = dots.scale.y = 0.003
        dots.points = [point(p) for p in positions]
        start = marker('MAS248_stroke_start', stroke_id, Marker.SPHERE, parent, (0.1, 1., 0.1))
        start.pose.position = point(positions[0])
        start.scale.x = start.scale.y = start.scale.z = 0.006
        if show_orientations:
            for pose in poses[::axis_every]:
                origin = (pose['x'], pose['y'], pose['z'] + axis_lift_m)
                q = [pose[k] for k in ('qx', 'qy', 'qz', 'qw')]
                for i, vector in enumerate(target_axes(q, axis_length_m)):
                    arrow('MAS248_TCP_orientation_' + 'XYZ'[i], axis_id,
                          parent, origin, vector, colors[i])
                axis_id += 1
    return MarkerArray(markers=markers)


def get_parser():
    parser = argparse.ArgumentParser(description='Display MAS248 image targets in RViz; no robot motion')
    parser.add_argument('--json', default='/tmp/mas248_targets.json')
    parser.add_argument('--axis-every', type=int, default=10)
    parser.add_argument('--axis-length-mm', type=float, default=18.0)
    parser.add_argument('--axis-lift-mm', type=float, default=0.0,
                        help='Visual world-Z offset of TCP arrows only, in millimetres')
    parser.add_argument('--no-orientations', action='store_true',
                        help='Hide TCP orientation arrows; keep the image-frame axes')
    return parser


def main(argv=None):
    parser = get_parser()
    args = parser.parse_args(argv)
    try:
        if args.axis_every < 1:
            raise ValueError('--axis-every must be at least 1')
        target_axes((0., 0., 0., 1.), args.axis_length_mm / 1000.0)
        if not math.isfinite(args.axis_lift_mm):
            raise ValueError('--axis-lift-mm must be finite')
        data = load_targets(args.json)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(f'Invalid targets or options: {exc}')

    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        from visualization_msgs.msg import MarkerArray
    except ImportError as exc:
        parser.exit(2, f'ROS 2 dependencies unavailable: {exc}. Source ROS Humble and the workspace.\n')

    rclpy.init(args=[])
    node = Node('mas248_show_targets_rviz')
    try:
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        publisher = node.create_publisher(MarkerArray, '/mas248/drawing_targets', qos)
        broadcaster = StaticTransformBroadcaster(node)
        spec = data['frame_to_world']
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = spec['parent']
        transform.child_frame_id = spec['child']
        t, q = transform.transform.translation, transform.transform.rotation
        t.x, t.y, t.z = spec['xyz']
        q.x, q.y, q.z, q.w = spec['quaternion_xyzw']
        broadcaster.sendTransform(transform)
        markers = build_markers(data, args.axis_every, args.axis_length_mm / 1000.0,
                                args.axis_lift_mm / 1000.0, not args.no_orientations)
        publisher.publish(markers)
        node.create_timer(1.0, lambda: publisher.publish(markers))
        count = sum(len(stroke['poses']) for stroke in data['strokes'])
        node.get_logger().info(
            f'{len(data["strokes"])} contours / {count} TCP targets. '
            'RViz: Fixed Frame world, MarkerArray /mas248/drawing_targets. '
            'Visualization only; Ctrl+C to stop.')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
