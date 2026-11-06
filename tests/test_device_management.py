import asyncio
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

import app
from test_transport import connection


class DeviceManagementTests(unittest.TestCase):
    def test_metadata_delete_and_number_survive_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'fleet.json'
            s = app.MultiGo2Service(path)
            try:
                n = s.add_device('192.0.2.1', aes_128_key='a'*32)['number']
                with patch.object(app, 'service', s):
                    client = app.app.test_client()
                    response = client.patch(f'/api/devices/{n}', json={'name':' 一号 ', 'group':' A组 '})
                    self.assertEqual(response.status_code, 200)
                    self.assertEqual(response.json['device']['name'], '一号')
                    self.assertNotIn('aes_128_key', response.json['device'])
                    restored = app.MultiGo2Service(path)
                    try:
                        self.assertEqual(restored.devices[n].group, 'A组')
                    finally:
                        restored.close()
                    for data in ([], {'name':1}, {'group':'x'*41}):
                        self.assertEqual(client.patch(f'/api/devices/{n}', json=data).status_code, 400)
                    for status in ('connected', 'busy'):
                        setattr(s.devices[n], status, True)
                        self.assertEqual(client.delete(f'/api/devices/{n}').status_code, 400)
                        setattr(s.devices[n], status, False)
                    s.saved_alignments['192.0.2.1'] = {'points':[]}
                    s.return_points[n] = {'saved':True}
                    self.assertEqual(client.delete(f'/api/devices/{n}').status_code, 200)
                    self.assertNotIn(n, s.frames)
                    self.assertNotIn(n, s.return_points)
                    self.assertNotIn('192.0.2.1', s.saved_alignments)
                restored = app.MultiGo2Service(path)
                try:
                    self.assertEqual(restored.list_devices(), [])
                    self.assertGreater(restored.add_device('192.0.2.2')['number'], n)
                finally:
                    restored.close()
            finally:
                s.close()

    def test_disconnect_clears_video_and_unknown_frame_does_not_create_state(self):
        s = app.MultiGo2Service()
        try:
            n = s.add_device('192.0.2.1')['number']
            s.videos[n] = app.VideoState(enabled=True, frame=b'old', frame_time=1.)
            self.assertIsNone(s.video_frame(n))
            s.submit(s.disconnect([n]))
            self.assertIsNone(s.videos[n].frame)
            self.assertEqual(s.videos[n].frame_time, 0.)
            self.assertIsNone(s.video_frame(999))
            self.assertNotIn(999, s.videos)
        finally:
            s.close()

    def test_missing_channel_is_not_usable(self):
        conn = connection()
        conn.datachannel.channel = None
        self.assertFalse(app.MultiGo2Service._transport_open(conn))

    def test_manual_disconnect_and_delete_cancel_pending_retry(self):
        for operation in ('disconnect', 'delete'):
            with self.subTest(operation=operation):
                s = app.MultiGo2Service()
                try:
                    n = s.add_device('192.0.2.1')['number']
                    async def exercise():
                        task = asyncio.create_task(s._retry_connection(n))
                        s.reconnect_tasks[n] = task
                        await asyncio.sleep(0)
                        if operation == 'delete':
                            await s.delete_device(n)
                        else:
                            await s.disconnect([n])
                        self.assertTrue(task.cancelled())
                        self.assertNotIn(n, s.reconnect_tasks)
                    s.submit(exercise())
                finally:
                    s.close()


class ReconnectTests(unittest.IsolatedAsyncioTestCase):
    async def test_retry_is_bounded_and_does_not_start_motion(self):
        s = app.MultiGo2Service.__new__(app.MultiGo2Service)
        s.devices = {1: app.Device(1, '192.0.2.1')}
        s.closed = False
        s.auto_connect_enabled = True
        s.reconnect_tasks = {}
        s._connect_one = AsyncMock(return_value={'ok':False})
        with patch('app.asyncio.sleep', new_callable=AsyncMock) as sleep:
            await s._retry_connection(1)
        self.assertEqual(s._connect_one.await_count, 3)
        self.assertEqual([c.args[0] for c in sleep.await_args_list], [2.,4.,8.])
        self.assertEqual(s.devices[1].connection_health['status'], 'failed')
        s._connect_one.reset_mock()
        s._connect_one.return_value = {'ok':True}
        with patch('app.asyncio.sleep', new_callable=AsyncMock):
            await s._retry_connection(1)
        s._connect_one.assert_awaited_once()
        s._connect_one.reset_mock()
        s.devices[1].auto_connect = False
        await s._retry_connection(1)
        s._connect_one.assert_not_awaited()

    async def test_deleted_device_waiting_to_connect_is_rejected(self):
        s = app.MultiGo2Service.__new__(app.MultiGo2Service)
        s.devices = {}
        result = await s._connect_one_locked(app.Device(1, '192.0.2.1'))
        self.assertFalse(result['ok'])
