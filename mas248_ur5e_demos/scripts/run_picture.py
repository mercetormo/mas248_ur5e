"""Lecture 9: plan the complete drawing before optional MOCK execution.

Extends the JSON validator contributed in commit 610a57b. ROS imports are
delayed so file validation and the planning-sequence tests work offline.
"""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


DEFAULT_TARGETS = Path('/tmp/mas248_targets.json')
JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
POSE_FIELDS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')


def load_targets(path):
    """Load and validate every stroke before contacting ROS."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f'Fichier de cibles introuvable : {path}')
    with path.open(encoding='utf-8') as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError('Le JSON doit contenir un objet.')
    for field in ('frame_id', 'frame'):
        if field in data and data[field] != 'world':
            raise ValueError('Les cibles doivent être exprimées dans world.')
    strokes = data.get('strokes')
    if not isinstance(strokes, list) or not strokes:
        raise ValueError('Aucun trait trouvé dans le fichier JSON.')
    for index, stroke in enumerate(strokes, 1):
        if not isinstance(stroke, dict):
            raise ValueError(f'Le trait {index} est invalide.')
        poses = stroke.get('poses')
        if not isinstance(poses, list) or not poses:
            raise ValueError(f'Le trait {index} ne contient aucune pose.')
        for pose_index, pose in enumerate(poses, 1):
            where = f'trait {index}, pose {pose_index}'
            if not isinstance(pose, dict):
                raise ValueError(f'Pose invalide : {where}.')
            for key in POSE_FIELDS:
                value = pose.get(key)
                if (isinstance(value, bool) or
                        not isinstance(value, (int, float)) or
                        not math.isfinite(value)):
                    raise ValueError(f'Champ {key} absent ou invalide : {where}.')
            norm = math.hypot(*(pose[key] for key in ('qx', 'qy', 'qz', 'qw')))
            if not math.isfinite(norm) or norm < 1e-12:
                raise ValueError(f'Quaternion nul ou invalide : {where}.')
    return data


def lifted(pose, lift_m):
    result = dict(pose)
    result['z'] = pose['z'] + lift_m
    return result


def stroke_waypoints(stroke, lift_m):
    """Transfer above the next start, descend, draw, then retract vertically."""
    poses = stroke['poses']
    return [lifted(poses[0], lift_m), *poses, lifted(poses[-1], lift_m)]


def ordered_positions(names, positions):
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError('État articulaire mal formé.')
    values = dict(zip(names, positions))
    if not set(JOINTS).issubset(values):
        raise ValueError('Les six articulations UR5e sont requises.')
    result = [float(values[name]) for name in JOINTS]
    if not all(math.isfinite(value) for value in result):
        raise ValueError('Position articulaire non finie.')
    return result


def check_trajectory(trajectory, start, *, cartesian=False):
    """Check continuity and timing; return endpoint and whether it moves."""
    jt = trajectory.joint_trajectory
    if len(jt.joint_names) != 6 or set(jt.joint_names) != set(JOINTS):
        raise ValueError('La trajectoire doit commander les six articulations UR5e.')
    if not jt.points:
        raise ValueError('Trajectoire vide.')
    positions = []
    timestamps = []
    kept_points = []
    for point in jt.points:
        values = ordered_positions(jt.joint_names, point.positions)
        for field in ('velocities', 'accelerations'):
            derivatives = getattr(point, field)
            if derivatives and (len(derivatives) != 6 or not all(math.isfinite(v) for v in derivatives)):
                raise ValueError('Vitesses ou accélérations invalides.')
        duration = point.time_from_start
        if duration.sec < 0 or not 0 <= duration.nanosec < 1_000_000_000:
            raise ValueError('Horodatage de trajectoire invalide.')
        timestamp = duration.sec * 1_000_000_000 + duration.nanosec
        if (positions and timestamp == timestamps[-1] and
                max(abs(a - b) for a, b in zip(values, positions[-1])) < 1e-12 and
                point.velocities == kept_points[-1].velocities and
                point.accelerations == kept_points[-1].accelerations):
            # An already-reached first waypoint may duplicate the start sample.
            continue
        positions.append(values)
        timestamps.append(timestamp)
        kept_points.append(point)
    jt.points = kept_points
    if max(abs(a - b) for a, b in zip(positions[0], start)) > 0.02:
        raise ValueError('Début de trajectoire différent de la fin du segment précédent.')
    moves = any(max(abs(a - b) for a, b in zip(point, start)) > 1e-9
                for point in positions)
    if moves and (timestamps[-1] <= 0 or
                  any(b <= a for a, b in zip(timestamps, timestamps[1:]))):
        raise ValueError('Trajectoire sans horodatage strictement croissant ; exécution refusée.')
    if cartesian:
        for before, after in zip(positions, positions[1:]):
            if max(abs(a - b) for a, b in zip(before, after)) > 0.5:
                raise ValueError('Saut articulaire supérieur à 0,5 rad dans le trajet cartésien.')
    return positions[-1], moves


@dataclass
class Segment:
    label: str
    trajectory: object
    moves: bool


def plan_picture(node, strokes, *, lift_m, max_step, pose_factory,
                 move_group, cartesian):
    """All requests are PLAN ONLY; chain their joint states explicitly."""
    approach = move_group(node, pose=pose_factory(lifted(strokes[0]['poses'][0], lift_m)),
                          execute=False)
    state = approach.trajectory_start.joint_state
    start = ordered_positions(state.name, state.position)
    start, moves = check_trajectory(approach.planned_trajectory, start)
    segments = [Segment('Approche', approach.planned_trajectory, moves)]
    for index, stroke in enumerate(strokes, 1):
        waypoints = [pose_factory(pose) for pose in stroke_waypoints(stroke, lift_m)]
        result = cartesian(node, waypoints, max_step=max_step,
                           start_joint_positions=start, jump_threshold=0.0,
                           avoid_collisions=True)
        if result.error_code.val != 1:
            raise RuntimeError(f'Trait {index} : erreur MoveIt {result.error_code.val}.')
        if not math.isfinite(result.fraction) or result.fraction < 1.0 - 1e-9:
            raise RuntimeError(f'Trait {index} incomplet ({result.fraction:.6f}) ; '
                               'aucune trajectoire ne sera exécutée.')
        start, moves = check_trajectory(result.solution, start, cartesian=True)
        segments.append(Segment(f'Trait {index}', result.solution, moves))
    return approach.trajectory_start, segments


def check_mock_description(description):
    """Fail closed unless controller_manager describes the supplied mock system."""
    root = ET.fromstring(description)
    systems = root.findall('.//ros2_control')
    if root.tag != 'robot' or len(systems) != 1:
        raise RuntimeError('Un seul système ros2_control simulé est attendu.')
    system = systems[0]
    plugin = system.findtext('hardware/plugin', default='').strip()
    names = [joint.get('name') for joint in system.findall('joint')]
    if (system.get('name') != 'MAS248MockHardware' or
            system.get('type') != 'system' or
            plugin != 'mock_components/GenericSystem' or
            len(names) != 6 or set(names) != set(JOINTS)):
        raise RuntimeError('Exécution refusée : le contrôleur MAS248 simulé n’est pas confirmé.')


def require_mock(node):
    from rcl_interfaces.srv import GetParameters
    from .common import wait_future
    client = node.create_client(GetParameters, '/controller_manager/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Impossible de vérifier le contrôleur simulé.')
        request = GetParameters.Request()
        request.names = ['robot_description']
        response = wait_future(node, client.call_async(request), 15.0)
        if len(response.values) != 1 or response.values[0].type != 4:
            raise RuntimeError('robot_description du contrôleur indisponible.')
        check_mock_description(response.values[0].string_value)
    finally:
        node.destroy_client(client)


def execute_segments(node, segments, *, confirm_mock, execute):
    """Called only after the whole plan succeeds, never from the planning loop."""
    confirm_mock(node)
    for segment in segments:
        if segment.moves:
            node.get_logger().info(f'Exécution simulée : {segment.label}')
            execute(node, segment.trajectory)


def ros_pose(values):
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x = float(values['x'])
    pose.position.y = float(values['y'])
    pose.position.z = float(values['z'])
    norm = math.hypot(*(values[key] for key in ('qx', 'qy', 'qz', 'qw')))
    for key, axis in zip(('qx', 'qy', 'qz', 'qw'), ('x', 'y', 'z', 'w')):
        setattr(pose.orientation, axis, values[key] / norm)
    return pose


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('Une valeur positive et finie est requise.')
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Planifier une image ; --execute réservé au contrôleur MAS248 simulé.')
    parser.add_argument('image_targets', nargs='?', type=Path)
    parser.add_argument('--targets', type=Path, help='JSON exprimé dans world')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--validate-only', action='store_true', help='Vérifier le JSON sans ROS')
    modes.add_argument('--execute', action='store_true', help='Exécuter après planification complète, MOCK uniquement')
    parser.add_argument('--max-strokes', type=int, help='Limiter le premier essai à N traits')
    parser.add_argument('--lift-m', type=positive_float, default=0.03,
                        help='Remontée verticale entre traits, en mètres (0.03)')
    parser.add_argument('--eef-step', type=positive_float, default=0.003,
                        help='Pas cartésien maximal, en mètres (0.003)')
    args = parser.parse_args(argv)
    if args.targets is not None and args.image_targets is not None:
        parser.error('Utiliser le chemin positionnel ou --targets, pas les deux.')
    if args.max_strokes is not None and args.max_strokes < 1:
        parser.error('--max-strokes doit être au moins 1.')
    path = args.targets or args.image_targets or DEFAULT_TARGETS
    try:
        data = load_targets(path)
        strokes = data['strokes']
        count = sum(len(stroke['poses']) for stroke in strokes)
        print(f'Cibles validées : {len(strokes)} traits / {count} poses : {path}', flush=True)
        if args.validate_only:
            print('Vérification uniquement : aucun mouvement commandé.')
            return 0
        if args.max_strokes is not None:
            strokes = strokes[:args.max_strokes]
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from moveit_msgs.msg import DisplayTrajectory
        from .common import move_group, cartesian, execute_trajectory
        rclpy.init(args=[])
        node = Node('mas248_run_picture')
        try:
            if args.execute:
                require_mock(node)
            publisher = node.create_publisher(
                DisplayTrajectory, '/display_planned_path',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            state, segments = plan_picture(
                node, strokes, lift_m=args.lift_m, max_step=args.eef_step,
                pose_factory=ros_pose, move_group=move_group, cartesian=cartesian)
            message = DisplayTrajectory()
            message.model_id = 'mas248_ur5e'
            message.trajectory_start = state
            message.trajectory = [segment.trajectory for segment in segments if segment.moves]
            publisher.publish(message)
            node.get_logger().info(f'Plan complet validé : {len(strokes)} traits, remontée {args.lift_m:.3f} m.')
            until = time.monotonic() + 2.0
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.1)
            if args.execute:
                execute_segments(node, segments, confirm_mock=require_mock,
                                 execute=execute_trajectory)
                node.get_logger().info('Dessin simulé terminé ; TCP relevé.')
            else:
                node.get_logger().info('PLAN ONLY : aucune commande de mouvement envoyée.')
            return 0
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except (Exception, KeyboardInterrupt) as error:
        print(f'Erreur : {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
