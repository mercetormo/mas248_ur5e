"""Offline regression tests for whole-picture planning and mock-only execution."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from scripts import run_picture as runner


def pose(x=0.55, y=0.0, z=0.003):
    return dict(x=x, y=y, z=z, qx=1.0, qy=0.0, qz=0.0, qw=0.0)


def trajectory(start, end, *, names=runner.JOINTS):
    points = []
    for seconds, values in enumerate((start, end)):
        mapping = dict(zip(runner.JOINTS, values))
        points.append(NS(positions=[mapping[name] for name in names],
                         velocities=[], accelerations=[],
                         time_from_start=NS(sec=seconds, nanosec=0)))
    return NS(joint_trajectory=NS(joint_names=list(names), points=points))


def mock_xml(plugin='mock_components/GenericSystem', name='MAS248MockHardware'):
    joints = ''.join(f'<joint name="{joint}"/>' for joint in runner.JOINTS)
    return (f'<robot><ros2_control name="{name}" type="system">'
            f'<hardware><plugin>{plugin}</plugin></hardware>{joints}'
            '</ros2_control></robot>')


class RunPictureTests(unittest.TestCase):
    def setUp(self):
        self.strokes = [dict(poses=[pose(), pose(y=0.01)]),
                        dict(poses=[pose(x=0.56), pose(x=0.56, y=0.01)])]

    def load(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'targets.json'
            path.write_text(json.dumps(data), encoding='utf-8')
            return runner.load_targets(path)

    def test_valid_targets_and_offline_cli(self):
        data = dict(strokes=self.strokes)
        self.assertEqual(self.load(data), data)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'targets.json'
            path.write_text(json.dumps(data), encoding='utf-8')
            self.assertEqual(runner.main([str(path), '--validate-only']), 0)

    def test_bad_targets_fail_before_ros(self):
        for value in (True, float('nan'), float('inf'), '0.55', None):
            data = dict(strokes=copy.deepcopy(self.strokes))
            data['strokes'][0]['poses'][0]['x'] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.load(data)
        for invalid in ({}, {'strokes': []}, {'strokes': [{'poses': []}]},
                        {'frame_id': 'base_link', 'strokes': self.strokes}):
            with self.subTest(data=invalid), self.assertRaises(ValueError):
                self.load(invalid)

    def test_zero_quaternion_rejected(self):
        data = dict(strokes=copy.deepcopy(self.strokes))
        data['strokes'][0]['poses'][0]['qx'] = 0.0
        with self.assertRaises(ValueError):
            self.load(data)

    def test_all_segments_are_chained_from_planned_endpoints(self):
        original = copy.deepcopy(self.strokes)
        requests = []
        def move(node, *, pose, execute):
            self.assertFalse(execute)
            self.assertAlmostEqual(pose['z'], 0.033)
            return NS(trajectory_start=NS(joint_state=NS(name=list(runner.JOINTS),
                                                       position=[0.0] * 6)),
                      planned_trajectory=trajectory([0.0] * 6, [0.1] * 6))
        def path(node, waypoints, **kwargs):
            start = kwargs['start_joint_positions']
            requests.append(copy.deepcopy((waypoints, kwargs)))
            end = [value + 0.1 for value in start]
            return NS(error_code=NS(val=1), fraction=1.0,
                      solution=trajectory(start, end, names=tuple(reversed(runner.JOINTS))))
        _, segments = runner.plan_picture(
            None, self.strokes, lift_m=0.03, max_step=0.003,
            pose_factory=lambda value: value, move_group=move, cartesian=path)
        self.assertEqual(len(segments), 3)
        self.assertEqual(self.strokes, original)
        self.assertEqual(requests[0][1]['start_joint_positions'], [0.1] * 6)
        self.assertEqual(requests[1][1]['start_joint_positions'], [0.2] * 6)
        for waypoints, kwargs in requests:
            self.assertTrue(kwargs['avoid_collisions'])
            self.assertEqual(kwargs['max_step'], 0.003)
            self.assertAlmostEqual(waypoints[0]['z'], 0.033)
            self.assertAlmostEqual(waypoints[-1]['z'], 0.033)
            self.assertAlmostEqual(waypoints[1]['z'], 0.003)
            self.assertAlmostEqual(waypoints[-2]['z'], 0.003)

    def test_partial_or_failed_cartesian_path_aborts(self):
        for fraction, code in ((0.9, 1), (float('nan'), 1), (1.0, -1)):
            seen = []
            def move(node, **kwargs):
                return NS(trajectory_start=NS(joint_state=NS(name=list(runner.JOINTS), position=[0.] * 6)),
                          planned_trajectory=trajectory([0.] * 6, [0.1] * 6))
            def path(node, waypoints, **kwargs):
                seen.append('planned')
                return NS(fraction=fraction, error_code=NS(val=code),
                          solution=trajectory([0.1] * 6, [0.2] * 6))
            with self.subTest(fraction=fraction, code=code), self.assertRaises(RuntimeError):
                runner.plan_picture(None, self.strokes, lift_m=.03, max_step=.003,
                                    pose_factory=lambda value: value,
                                    move_group=move, cartesian=path)
            self.assertEqual(seen, ['planned'])

    def test_discontinuous_or_jumping_path_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Début'):
            runner.check_trajectory(trajectory([0.1] * 6, [0.2] * 6), [0.] * 6)
        with self.assertRaisesRegex(ValueError, 'Saut'):
            runner.check_trajectory(trajectory([0.] * 6, [0.6] * 6), [0.] * 6, cartesian=True)

    def test_moving_path_requires_strict_timing(self):
        for seconds in (0, -1):
            path = trajectory([0.] * 6, [0.1] * 6)
            path.joint_trajectory.points[-1].time_from_start.sec = seconds
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                runner.check_trajectory(path, [0.] * 6)

    def test_no_motion_does_not_require_a_positive_duration(self):
        path = trajectory([0.] * 6, [0.] * 6)
        path.joint_trajectory.points = path.joint_trajectory.points[:1]
        endpoint, moves = runner.check_trajectory(path, [0.] * 6)
        self.assertFalse(moves)
        self.assertEqual(endpoint, [0.] * 6)

    def test_duplicate_start_sample_can_be_removed(self):
        path = trajectory([0.] * 6, [0.1] * 6)
        path.joint_trajectory.points.insert(1, copy.deepcopy(path.joint_trajectory.points[0]))
        endpoint, moves = runner.check_trajectory(path, [0.] * 6)
        self.assertEqual(len(path.joint_trajectory.points), 2)
        self.assertTrue(moves)
        self.assertEqual(endpoint, [0.1] * 6)

    def test_nonfinite_joint_trajectory_rejected(self):
        path = trajectory([0.] * 6, [0.1] * 6)
        path.joint_trajectory.points[-1].positions[2] = float('nan')
        with self.assertRaises(ValueError):
            runner.check_trajectory(path, [0.] * 6)

    def test_mock_description_guard(self):
        runner.check_mock_description(mock_xml())
        for xml in (mock_xml(plugin='ur_robot_driver/URPositionHardwareInterface'),
                    mock_xml(name='OtherHardware'), '<robot/>',
                    mock_xml().replace('</robot>', mock_xml() + '</robot>')):
            with self.subTest(xml=xml), self.assertRaises(RuntimeError):
                runner.check_mock_description(xml)

    def test_execution_order_and_no_motion_skip(self):
        events = []
        node = NS(get_logger=lambda: NS(info=lambda message: None))
        segments = [runner.Segment('noop', 'noop', False),
                    runner.Segment('first', 'first', True),
                    runner.Segment('second', 'second', True)]
        runner.execute_segments(node, segments,
                                confirm_mock=lambda node: events.append('guard'),
                                execute=lambda node, path: events.append(path))
        self.assertEqual(events, ['guard', 'first', 'second'])

    def test_guard_failure_prevents_every_execution(self):
        called = []
        def fail(node):
            raise RuntimeError('not mock')
        with self.assertRaises(RuntimeError):
            runner.execute_segments(None, [runner.Segment('one', object(), True)],
                                    confirm_mock=fail, execute=lambda *args: called.append(args))
        self.assertEqual(called, [])

    def test_execution_failure_stops_following_segments(self):
        called = []
        node = NS(get_logger=lambda: NS(info=lambda message: None))
        def fail(node, path):
            called.append(path)
            raise RuntimeError('controller error')
        with self.assertRaises(RuntimeError):
            runner.execute_segments(node, [runner.Segment('one', 'one', True),
                                           runner.Segment('two', 'two', True)],
                                    confirm_mock=lambda node: None, execute=fail)
        self.assertEqual(called, ['one'])


if __name__ == '__main__':
    unittest.main()
