import math
import time
import unittest
from unittest.mock import patch

import numpy as np

import app
from fleet.registration import (register_planar, rotation, register_planar_layout,
                                layout_bounds, layout_position, row_conflicts)


def scene():
    rng = np.random.default_rng(7)
    xy = np.vstack([
        np.column_stack((np.linspace(-3, 4, 200), np.full(200, -2))),
        np.column_stack((np.full(180, 4), np.linspace(-2, 3, 180))),
        np.column_stack((np.linspace(-3, 1.5, 160), np.full(160, 3))),
        np.column_stack((rng.normal(.7, .12, 150), rng.normal(.4, .12, 150))),
        np.column_stack((np.full(120, -1.8), np.linspace(-.8, 1.7, 120))),
    ])
    return np.column_stack((xy, rng.uniform(0, 2, len(xy))))


class RegistrationTests(unittest.TestCase):
    def test_layout_uses_robot_pose_instead_of_odometry_origin(self):
        target = scene()
        pose = dict(x=1.1, y=-.5, yaw=2.1)
        for rank in [-2, 2]:
            yaw = -pose['yaw']
            body = np.array([.05, rank*.65])
            translation = body - rotation(yaw) @ [pose['x'], pose['y']]
            source = target.copy()
            source[:, :2] = (target[:, :2] - translation) @ rotation(yaw)
            result = register_planar_layout(source, target, pose, layout_bounds(rank))
            self.assertTrue(result['accepted'], result)
            actual = layout_position(result['transform'], pose)
            self.assertLess(abs(actual['y'] - body[1]), .1)
            self.assertLess(abs(actual['x'] - body[0]), .1)
            self.assertLess(abs(actual['yaw']), .05)

    def test_wrong_side_does_not_pass_by_using_layout_as_fake_position(self):
        target = scene()
        source = target.copy()
        source[:, 1] += 1.3  # True body position is on the right.
        result = register_planar_layout(source, target, dict(x=0, y=0, yaw=0), layout_bounds(2))
        self.assertFalse(result['accepted'], result)

    def test_row_spacing_conflict_detected(self):
        results = []
        for number, rank, y in [(1, 1, .8), (2, 2, 1.0)]:
            results.append(dict(number=number, accepted=True, bounds=layout_bounds(rank),
                                local_pose=dict(x=0,y=0,yaw=0), transform=dict(x=0,y=y,yaw=0)))
        self.assertEqual(row_conflicts(results), {1, 2})

    def test_unknown_rotation_translation_and_partial_overlap(self):
        target = scene()
        for yaw, translation in [(.73, [1.8, -.9]), (-2.4, [-2, 1.5])]:
            with self.subTest(yaw=yaw):
                source = target.copy()
                source[:, :2] = (target[:, :2] - translation) @ rotation(yaw)
                source = source[np.arange(len(source)) % 5 != 0]
                source[:, :2] += np.random.default_rng(3).normal(0, .01, source[:, :2].shape)
                result = register_planar(source, target)
                self.assertTrue(result['accepted'], result)
                self.assertLess(np.linalg.norm(np.array([result['transform']['x'], result['transform']['y']]) - translation), .12)
                self.assertLess(abs(result['transform']['yaw'] - yaw), .06)

    def test_floor_only_rejected(self):
        points = scene()
        points[:, 2] = 0
        with self.assertRaises(ValueError):
            register_planar(points, points)

    def test_symmetric_scene_not_automatically_accepted(self):
        theta = np.tile(np.linspace(0, 2 * math.pi, 300, endpoint=False), 4)
        points = np.column_stack((3*np.cos(theta), 3*np.sin(theta), np.repeat([0, .5, 1, 1.5], 300)))
        result = register_planar(points, points)
        self.assertFalse(result['accepted'], result)


class AlignmentServiceTests(unittest.TestCase):
    def setUp(self):
        self.service = app.MultiGo2Service()
        self.target = scene()
        self.numbers = []
        for ip in ['192.168.31.191', '192.168.31.120', '192.168.31.103']:
            number = self.service.add_device(ip)['number']
            self.numbers.append(number)
            self.service.devices[number].connected = True
            self.service.devices[number].cloud_enabled = True
            self.service.connections[number] = object()
            frame = self.service.frames[number]
            frame.local_pose = dict(x=0, y=0, z=0, yaw=0)
            frame.last_pose_time = frame.cloud_time = time.time()
            frame.raw_points = self.target.copy()
        self.client_patch = patch.object(app, 'service', self.service)
        self.client_patch.start()
        self.client = app.app.test_client()

    def tearDown(self):
        self.client_patch.stop()
        self.service.connections.clear()
        self.service.close()

    def preview(self):
        response = self.client.post('/api/frame/auto-align', json={
            'targets': self.numbers, 'reference_ip': '192.168.31.191', 'alignment_mode': 'free'})
        self.assertEqual(response.status_code, 200, response.json)
        token = response.json['token']
        deadline = time.time() + 10
        while time.time() < deadline:
            result = self.client.get('/api/frame/auto-align/' + token).json
            if result['done']:
                self.assertFalse(result['error'])
                return result
            time.sleep(.02)
        self.fail('alignment worker timed out')

    def refresh_times(self):
        for frame in self.service.frames.values():
            frame.last_pose_time = frame.cloud_time = time.time()

    def test_preview_isolated_no_side_effects_and_atomic_apply(self):
        reference, other, stale = self.numbers
        self.service.frames[stale].last_pose_time = time.time() - 565
        # Reference robot is not at its own local origin.
        self.service.frames[reference].local_pose = dict(x=1, y=2, z=.3, yaw=.4)
        result = self.preview()
        self.assertEqual(result['reference'], reference)
        self.assertFalse(any(frame.aligned for frame in self.service.frames.values()))
        failure = next(r for r in result['results'] if r['number'] == stale)
        self.assertNotIn('transform', failure)
        self.refresh_times()
        response = self.client.post('/api/frame/auto-align/apply', json={'token': result['token']})
        self.assertEqual(response.status_code, 200, response.json)
        self.assertEqual(set(response.json['aligned']), {reference, other})
        self.assertFalse(self.service.frames[stale].aligned)
        pose = self.service.frames[reference].map_pose
        for key in ['x', 'y', 'z', 'yaw']:
            self.assertAlmostEqual(pose[key], 0, places=4)

    def test_stale_reference_and_reconnected_device_rejected(self):
        reference, other, _ = self.numbers
        self.service.frames[reference].last_pose_time = time.time() - 5
        self.assertEqual(self.client.post('/api/frame/auto-align', json={
            'targets': self.numbers, 'reference_ip': '192.168.31.191'}).status_code, 400)
        self.refresh_times()
        result = self.preview()
        self.refresh_times()
        self.service.frames[other].session = 'reconnected'
        response = self.client.post('/api/frame/auto-align/apply', json={'token': result['token']})
        self.assertEqual(response.status_code, 400)
        self.assertFalse(any(f.aligned for f in self.service.frames.values()))

    def test_native_cloud_cached_before_alignment_and_connection_isolation(self):
        first, second, _ = self.numbers
        message = {'topic': app.RTC_TOPIC['ULIDAR_ARRAY'], 'data': {'data': {'points': self.target}}}
        self.service.frames[first].raw_points = np.empty((0, 3))
        self.service.frames[second].raw_points = np.empty((0, 3))
        self.service.devices[first].cloud_enabled = True
        async def receive():
            self.service._telemetry_callback(first, app.RTC_TOPIC['ULIDAR_ARRAY'], message)
            await self.service.cloud_tasks[first]
        self.service.submit(receive())
        self.assertEqual(len(self.service.frames[first].raw_points), len(self.target))
        self.assertEqual(len(self.service.frames[second].raw_points), 0)
        self.assertEqual(self.service.frames[first].map_points, [])
        response = self.client.get('/api/telemetry')
        self.assertEqual(response.status_code, 200)
        self.assertNotIn('raw_points', response.json['robots'][0])

    def test_manual_alignment_rejects_stale_pose(self):
        number = self.numbers[0]
        self.service.frames[number].last_pose_time = time.time() - 5
        with self.assertRaises(ValueError):
            self.service.align_robot(number, dict(x=0, y=0, z=0, yaw=0))

    def test_known_layout_endpoint_and_apply_rejects_wrong_side(self):
        config = dict(targets=self.numbers, layout_enabled=True, reference_ip='192.168.31.191',
                      left_ips='214,120,103', right_ips='142,113,232', min_spacing=.5, max_spacing=.8)
        for number, rank in zip(self.numbers, [0, 2, 3]):
            self.service.frames[number].raw_points[:, 1] -= rank*.65
        response = self.client.post('/api/frame/auto-align', json=config)
        self.assertEqual(response.status_code, 200, response.json)
        token = response.json['token']
        deadline = time.time() + 15
        while time.time() < deadline:
            result = self.client.get('/api/frame/auto-align/' + token).json
            if result['done']:
                break
            time.sleep(.02)
        self.assertTrue(result['done'])
        self.assertFalse(result['error'])
        other = next(r for r in result['results'] if r['number'] == self.numbers[1])
        self.assertEqual(other['bounds']['rank'], 2)
        self.assertTrue(other['accepted'], other)
        self.refresh_times()
        bad = dict(other['transform'], y=-2)
        response = self.client.post('/api/frame/auto-align/apply', json={'token':token, 'adjustments':{str(other['number']):bad}})
        self.assertEqual(response.status_code, 400)
        self.assertFalse(any(f.aligned for f in self.service.frames.values()))
        response = self.client.post('/api/frame/auto-align/apply', json={'token':token})
        self.assertEqual(response.status_code, 200, response.json)
        self.assertAlmostEqual(self.service.frames[self.numbers[1]].map_pose['y'], 1.3, delta=.1)

    def test_invalid_layout_configuration_rejected(self):
        base = dict(targets=self.numbers, layout_enabled=True, left_ips='120,103', right_ips='142')
        for changes in [dict(right_ips='120'), dict(min_spacing=.9, max_spacing=.5), dict(left_ips='191')]:
            response = self.client.post('/api/frame/auto-align', json={**base, **changes})
            self.assertEqual(response.status_code, 400, response.json)


if __name__ == '__main__':
    unittest.main()
