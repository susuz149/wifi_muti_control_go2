"""Review regressions; all connection and motion operations are simulated."""
import asyncio
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

import app
from test_transport import connection


class FormationRouteTests(unittest.TestCase):
    def test_line_route_reaches_execution_without_positions_in_request(self):
        fake = SimpleNamespace(formation=AsyncMock(return_value=[{'number': 1, 'ok': True}]),
                               list_devices=lambda: [],
                               submit=lambda coro, timeout: asyncio.run(coro))
        with patch.object(app, 'service', fake):
            result = app.app.test_client().post('/api/formation', json={
                'targets': [1, 2], 'pattern': 'line', 'spacing': .8,
                'confirm_motion': True, 'confirm_formation': True})
        self.assertEqual(result.status_code, 200, result.json)
        fake.formation.assert_awaited_once()


class StopDeliveryTests(unittest.IsolatedAsyncioTestCase):
    async def test_each_stop_method_is_attempted_after_prior_failure(self):
        for failed in ('wireless', 'velocity', 'request', 'all', None):
            with self.subTest(failed=failed):
                s = app.MultiGo2Service.__new__(app.MultiGo2Service)
                s.joystick_deadmen = {}
                s.set_status = Mock()
                def failure(name):
                    return RuntimeError(name) if failed in (name, 'all') else None
                s._publish_wireless_controller = Mock(side_effect=failure('wireless'))
                s._move_without_reply = Mock(side_effect=failure('velocity'))
                s._sport_request = AsyncMock(return_value=None, side_effect=failure('request'))
                result = await s._command_one(1, None, 'stop', {})
                s._publish_wireless_controller.assert_called_once()
                s._move_without_reply.assert_called_once()
                s._sport_request.assert_awaited_once()
                self.assertEqual(result['ok'], failed is None)


class AlignmentGuardTests(unittest.TestCase):
    def test_manual_alignment_and_clear_reject_active_control_without_changes(self):
        for kind in ('closed_loop', 'manual', 'stop', 'action', 'joystick'):
            for operation in ('align', 'clear'):
                with self.subTest(kind=kind, operation=operation):
                    s = app.MultiGo2Service.__new__(app.MultiGo2Service)
                    s.lock = threading.Lock()
                    s.devices = {1: app.Device(1, '192.0.2.1', connected=True)}
                    original = {'x': 0., 'y': 0., 'z': 0., 'yaw': 0.}
                    s.frames = {1: app.FrameState(local_pose=dict(original),
                        map_to_local=dict(original), aligned=True, last_pose_time=time.time())}
                    s.saved_alignments = {'192.0.2.1': {'saved': True}}
                    s.motion = SimpleNamespace(active=kind == 'closed_loop')
                    s.pending_motion_commands = {1: 1} if kind == 'manual' else {}
                    s.joystick_deadmen = {1: object()} if kind == 'joystick' else {}
                    s.stopping_numbers = {1: 1} if kind == 'stop' else {}
                    s.action_until = {1: time.monotonic()+10} if kind == 'action' else {}
                    s._persist_locked = Mock()
                    with self.assertRaisesRegex(ValueError, '先停止运动'):
                        if operation == 'align':
                            s.align_robot(1, dict(original, x=2.))
                        else:
                            s.clear_alignment([1])
                    self.assertEqual(s.frames[1].map_to_local, original)
                    self.assertTrue(s.frames[1].aligned)
                    self.assertIn('192.0.2.1', s.saved_alignments)
                    s._persist_locked.assert_not_called()


class StrictConfirmationTests(unittest.TestCase):
    def test_non_boolean_confirmation_never_dispatches_motion(self):
        fake = SimpleNamespace(command=AsyncMock(), formation=AsyncMock())
        for value in ('false', 'true', 1, None, False):
            with self.subTest(value=value), patch.object(app, 'service', fake):
                response = app.app.test_client().post('/api/command', json={
                    'command': 'move', 'targets': [1], 'direction': 'forward',
                    'confirm_motion': value})
                self.assertEqual(response.status_code, 400)
                response = app.app.test_client().post('/api/formation', json={
                    'targets': [1, 2], 'confirm_motion': True, 'confirm_formation': value})
                self.assertEqual(response.status_code, 400)
        fake.command.assert_not_called()
        fake.formation.assert_not_called()

    def test_high_risk_requires_boolean_true(self):
        for value in ('false', 'true', 1, None, False):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, '单独确认'):
                app.sport_action_parameters({'action': 'Damp', 'confirm_high_risk': value})


class DisconnectStopTests(unittest.TestCase):
    def test_disconnect_stops_before_closing_even_after_stop_failure(self):
        for failure in (False, True):
            with self.subTest(failure=failure):
                s = app.MultiGo2Service()
                n = s.add_device('192.0.2.1')['number']
                conn = connection()
                s.connections[n] = conn
                s.devices[n].connected = True
                events = []
                s._publish_wireless_controller = Mock(side_effect=lambda *_: events.append('wireless'))
                s._move_without_reply = Mock(side_effect=lambda *_: events.append('velocity'))
                async def stop(*_):
                    events.append('stop')
                    self.assertIn(n, s.stopping_numbers)
                    if failure:
                        raise RuntimeError('stop failed')
                async def close():
                    events.append('disconnect')
                s._sport_request = stop
                conn.disconnect.side_effect = close
                try:
                    result = s.submit(s.disconnect([n]))[0]
                    self.assertEqual(events, ['wireless', 'velocity', 'stop', 'disconnect'])
                    self.assertEqual(result['ok'], not failure)
                    self.assertNotIn(n, s.stopping_numbers)
                    self.assertFalse(s.devices[n].connected)
                finally:
                    s.close()


class ConnectionDiagnosticsTests(unittest.TestCase):
    def test_initialization_failure_shows_stage_and_cleans_peer(self):
        for stage in ('traffic', 'avoid'):
            with self.subTest(stage=stage):
                s = app.MultiGo2Service()
                n = s.add_device('192.0.2.1')['number']
                conn = connection()
                conn.connect = AsyncMock()
                conn.datachannel.set_decoder = Mock()
                conn.video = SimpleNamespace(add_track_callback=Mock())
                if stage == 'traffic':
                    conn.datachannel.disableTrafficSaving.side_effect = asyncio.TimeoutError()
                avoid = AsyncMock(side_effect=asyncio.TimeoutError() if stage == 'avoid' else None)
                try:
                    with patch.object(app, 'UnitreeWebRTCConnection', return_value=conn), \
                            patch.object(s, '_enable_obstacle_avoid', avoid):
                        result = s.submit(s.connect([n]))[0]
                    self.assertFalse(result['ok'])
                    self.assertIn('设置点云流量节省' if stage == 'traffic' else '开启并确认避障', result['error'])
                    self.assertIn('TimeoutError', result['error'])
                    self.assertNotIn(n, s.connections)
                    self.assertFalse(s.devices[n].busy)
                    conn.disconnect.assert_awaited_once()
                finally:
                    s.close()
