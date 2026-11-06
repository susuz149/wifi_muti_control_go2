import asyncio
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

import numpy as np

import app
from fleet.registration import rotation
from test_alignment import scene


class PersistenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'private' / 'fleet.json'
        self.services = []

    def new_service(self):
        service = app.MultiGo2Service(self.path)
        self.services.append(service)
        return service

    def tearDown(self):
        for service in self.services:
            service.connections.clear()
            service.close()
        self.temp.cleanup()

    def saved_scene(self):
        service = self.new_service()
        number = service.add_device('192.168.31.191', sn='test-sn')['number']
        service.devices[number].connected = True
        frame = service.frames[number]
        frame.local_pose = dict(x=0, y=0, z=0, yaw=0)
        frame.last_pose_time = frame.cloud_time = time.time()
        frame.raw_points = scene()
        service.align_robot(number, dict(x=0, y=0, z=0, yaw=0))
        return service, number

    def test_roundtrip_key_numbers_preferences_and_permissions(self):
        service = self.new_service()
        key = '0123456789abcdef'*2
        a = service.add_device('192.168.31.191', aes_128_key=key)['number']
        b = service.add_device('192.168.31.120')['number']
        service.submit(service.disconnect([b]))
        restored = self.new_service()
        self.assertEqual(restored.devices[a].aes_128_key, key)
        self.assertFalse(restored.devices[b].auto_connect)
        self.assertFalse(restored.devices[a].connected)
        self.assertNotIn(key, json.dumps(restored.list_devices()))
        self.assertEqual(os.stat(self.path).st_mode & 0o777, 0o600)
        self.assertEqual(os.stat(self.path.parent).st_mode & 0o777, 0o700)
        self.assertEqual(restored.next_number, b+1)
        restored.set_aes_key(a, '')
        self.assertEqual(self.new_service().devices[a].aes_128_key, '')

    def test_auto_connect_only_saved_enabled_devices(self):
        service = self.new_service()
        a = service.add_device('192.168.31.191')['number']
        b = service.add_device('192.168.31.120')['number']
        service.devices[b].auto_connect = False
        with patch.object(service, 'connect', new_callable=AsyncMock) as connect:
            future = service.start_saved_connections()
            future.result(timeout=3)
            connect.assert_awaited_once_with([a])
            service.auto_connect_enabled = False
            self.assertIsNone(service.start_saved_connections())

    def test_connect_and_normal_shutdown_keep_startup_preference(self):
        service = self.new_service()
        number = service.add_device('192.168.31.191')['number']
        noop = lambda *args: None
        conn = SimpleNamespace(connect=AsyncMock(), disconnect=AsyncMock(),
                               video=SimpleNamespace(add_track_callback=noop),
                               datachannel=SimpleNamespace(channel=SimpleNamespace(readyState='open'), set_decoder=noop, disableTrafficSaving=AsyncMock(),
                               switchVideoChannel=noop, pub_sub=SimpleNamespace(subscribe=noop, unsubscribe=noop)))
        with patch('app.UnitreeWebRTCConnection', return_value=conn), \
             patch.object(service, '_enable_obstacle_avoid', new_callable=AsyncMock, return_value=True), \
             patch.object(service, 'command', new_callable=AsyncMock):
            result = service.start_saved_connections().result(timeout=3)
            self.assertTrue(result[0]['ok'])
            self.assertTrue(service.devices[number].connected)
            service.submit(service.stop_and_disconnect_all())
        self.assertTrue(self.new_service().devices[number].auto_connect)

    def test_calibration_not_trusted_until_new_cloud_matches(self):
        _, number = self.saved_scene()
        restored = self.new_service()
        frame = restored.frames[number]
        self.assertFalse(frame.aligned)
        self.assertIsNone(frame.map_pose)
        # The odometry frame has reset: both translation and rotation differ.
        yaw, translation = .6, np.array([1.2, -.7])
        points = scene()
        points[:, :2] = (points[:, :2]-translation) @ rotation(yaw)
        frame.raw_points = points
        frame.local_pose = dict(x=0, y=0, z=0, yaw=0)
        frame.cloud_time = frame.last_pose_time = time.time()
        restored.devices[number].connected = True
        restored.submit(restored._restore_alignment(number))
        self.assertTrue(frame.aligned, frame.restore_status)
        self.assertAlmostEqual(frame.map_pose['x'], translation[0], delta=.1)
        self.assertAlmostEqual(frame.map_pose['y'], translation[1], delta=.1)
        self.assertAlmostEqual(frame.map_pose['yaw'], yaw, delta=.06)

    def test_failed_or_cancelled_verification_does_not_restore(self):
        service, number = self.saved_scene()
        frame = service.frames[number]
        frame.aligned = False
        frame.map_pose = frame.map_to_local = None
        with patch('app.register_planar', return_value={'accepted':False,'overlap':.1,'rmse':.2}):
            service.submit(service._restore_alignment(number))
        self.assertFalse(frame.aligned)
        self.assertIn('需要重新对齐', frame.restore_status)

        def cancelled(*args):
            frame.restore_epoch += 1
            return dict(accepted=True, overlap=1, rmse=0, transform=dict(x=0,y=0,yaw=0))
        with patch('app.register_planar', side_effect=cancelled):
            service.submit(service._restore_alignment(number))
        self.assertFalse(frame.aligned)

    def test_clear_alignment_persists(self):
        service, number = self.saved_scene()
        service.clear_alignment([number])
        restored = self.new_service()
        self.assertEqual(restored.saved_alignments, {})

    def test_settings_preference_persists_and_storage_errors_are_visible(self):
        service = self.new_service()
        with patch.object(app, 'service', service):
            client = app.app.test_client()
            self.assertEqual(client.post('/api/settings', json={'auto_connect_enabled':False}).status_code, 200)
            self.assertFalse(self.new_service().auto_connect_enabled)
            with patch('app.write_state', side_effect=OSError('disk full')):
                response = client.post('/api/settings', json={'auto_connect_enabled':True})
                self.assertEqual(response.status_code, 400)
                self.assertIn('保存失败', client.get('/api/devices').json['storage_error'])


if __name__ == '__main__':
    unittest.main()
