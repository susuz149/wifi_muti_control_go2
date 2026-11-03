import asyncio
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

import app


class MoveTransportTests(unittest.TestCase):
    def test_move_uses_sport_request_envelope(self):
        import json
        pub_sub = SimpleNamespace(channel=SimpleNamespace(readyState='open'),
                                  publish_without_callback=Mock())
        conn = SimpleNamespace(datachannel=SimpleNamespace(pub_sub=pub_sub))
        app.MultiGo2Service._move_without_reply(conn, .1, -.02, .15)
        call = pub_sub.publish_without_callback.call_args
        self.assertEqual(call.args[0], app.RTC_TOPIC['SPORT_MOD'])
        self.assertEqual(call.kwargs['msg_type'], app.DATA_CHANNEL_TYPE['REQUEST'])
        self.assertEqual(call.args[1]['header']['identity']['api_id'], app.SPORT_CMD['Move'])
        self.assertEqual(json.loads(call.args[1]['parameter']), {'x': .1, 'y': -.02, 'z': .15})

    def test_closed_channel_fails_instead_of_silently_dropping_move(self):
        for channel in (None, SimpleNamespace(readyState='closed')):
            with self.subTest(channel=channel):
                pub_sub = SimpleNamespace(channel=channel, publish_without_callback=Mock())
                conn = SimpleNamespace(datachannel=SimpleNamespace(pub_sub=pub_sub))
                with self.assertRaisesRegex(RuntimeError, '数据通道未打开'):
                    app.MultiGo2Service._move_without_reply(conn, .1, 0, 0)
                pub_sub.publish_without_callback.assert_not_called()


class MotionAPITests(unittest.TestCase):
    def setUp(self):
        self.client = app.app.test_client()
        self.fake = SimpleNamespace(
            start_motion=AsyncMock(return_value={'status': 'running'}),
            motion_status=AsyncMock(return_value={'status': 'idle'}),
            motion=SimpleNamespace(cancel=AsyncMock(return_value={'status': 'cancelled'})),
            submit=lambda coro, timeout: asyncio.run(coro),
        )
        self.patch = patch('app.service', self.fake)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def test_start_requires_both_confirmations_and_valid_program(self):
        response = self.client.post('/api/motion', json={'program': {}})
        self.assertEqual(response.status_code, 400)
        self.fake.start_motion.assert_not_called()
        response = self.client.post('/api/motion', json={'confirm_motion': True,
            'confirm_formation': True, 'program': {'steps': []}})
        self.assertEqual(response.status_code, 400)
        self.fake.start_motion.assert_not_called()

    def test_start_status_and_stop(self):
        response = self.client.post('/api/motion', json={'confirm_motion': True,
            'confirm_formation': True, 'program': {'steps': [{'targets': {'1': {'x': .3, 'y': 0, 'yaw': 0}}}]}})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(self.fake.start_motion.call_args.args[0]['steps'][0]['targets'][1]['x'], .3)
        self.assertEqual(self.client.get('/api/motion').json['motion']['status'], 'idle')
        response = self.client.post('/api/motion/stop')
        self.assertEqual(response.json['motion']['status'], 'cancelled')
        self.fake.motion.cancel.assert_awaited_once()


class CommandOwnershipTests(unittest.IsolatedAsyncioTestCase):
    async def test_manual_commands_rejected_and_stop_cancels_entire_group(self):
        service = app.MultiGo2Service.__new__(app.MultiGo2Service)
        service.pending_motion_commands = {}
        service.motion = SimpleNamespace(active=True, participants={1, 2}, cancel=AsyncMock())
        service.connected_connections = lambda numbers: [(n, None) for n in numbers]
        service._command_one = AsyncMock(return_value={'ok': True})
        with self.assertRaisesRegex(ValueError, '先停止'):
            await service.command([1], 'move', {})
        service._command_one.assert_not_called()
        await service.command([1], 'stop', {})
        service.motion.cancel.assert_awaited_once()

    async def test_start_rejects_a_pending_manual_move(self):
        service = app.MultiGo2Service.__new__(app.MultiGo2Service)
        service.pending_motion_commands = {1: 1}
        service.connected_connections = lambda numbers: [(1, None)]
        with self.assertRaisesRegex(ValueError, '手动动作'):
            await service.start_motion({'steps': [{'targets': {1: {}}}]})


if __name__ == '__main__':
    unittest.main()
