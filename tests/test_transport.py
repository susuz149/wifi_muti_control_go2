import asyncio
import json
import struct
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

from pyee import EventEmitter
import app
from fleet.transport import ResponsiveConnection, ResponsiveDataChannel, DeferredVideoChannel, bounded_local_request


class Channel(EventEmitter):
    readyState = 'open'

    def send(self, message):
        pass


def connection():
    channel = Channel()
    return SimpleNamespace(
        pc=SimpleNamespace(connectionState='connected', iceConnectionState='completed'),
        disconnect=AsyncMock(),
        datachannel=SimpleNamespace(channel=channel,
            last_heartbeat_monotonic=time.monotonic(), last_rx_monotonic=time.monotonic(),
            set_lidar_enabled=Mock(), switchVideoChannel=Mock(),
            disableTrafficSaving=AsyncMock(return_value=True),
            pub_sub=SimpleNamespace(channel=channel, subscribe=Mock(), unsubscribe=Mock())),
    )


class ThreadedAsyncTestCase(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        # This workspace's restricted selector sometimes misses cross-thread
        # wakeups, including the executor-shutdown callback. Match app's timer.
        loop = asyncio.get_event_loop()
        def tick():
            loop.call_later(.02, tick)
        loop.call_soon(tick)


class SignalingTests(ThreadedAsyncTestCase):
    async def test_signaling_does_not_block_other_robots(self):
        entered, release = threading.Event(), threading.Event()

        def signaling(*args, **kwargs):
            entered.set()
            if not release.wait(1):
                raise RuntimeError('event loop could not release signaling worker')
            return 'answer'

        conn = ResponsiveConnection(app.WebRTCConnectionMethod.LocalSTA, ip='192.0.2.1')
        pc = SimpleNamespace(localDescription=SimpleNamespace(sdp='offer', type='offer'))
        with patch('fleet.transport.webrtc_driver.send_sdp_to_local_peer', side_effect=signaling):
            task = asyncio.create_task(conn.get_answer_from_local_peer(pc, conn.ip))
            try:
                for _ in range(100):
                    if entered.is_set():
                        break
                    await asyncio.sleep(.005)
                self.assertTrue(entered.is_set())
                self.assertFalse(task.done())
            finally:
                release.set()
            self.assertEqual(await task, 'answer')

    def test_lan_http_has_connect_and_read_deadlines(self):
        response = SimpleNamespace(status_code=200, raise_for_status=Mock())
        with patch('fleet.transport.requests.post', return_value=response) as post:
            self.assertIs(bounded_local_request('http://192.0.2.1'), response)
            self.assertEqual(post.call_args.kwargs['timeout'], (3, 5))


class ReceiverTests(ThreadedAsyncTestCase):
    async def asyncSetUp(self):
        self.channel = Channel()
        pc = SimpleNamespace(createDataChannel=lambda name: self.channel)
        self.receiver = ResponsiveDataChannel(SimpleNamespace(), pc)

    async def asyncTearDown(self):
        self.receiver.stop_readers()
        if self.receiver.cloud_task:
            await asyncio.gather(self.receiver.cloud_task, return_exceptions=True)
        await asyncio.gather(*list(self.receiver.message_tasks), return_exceptions=True)

    def voxel_message(self):
        header = json.dumps({'type': 'msg', 'topic': app.RTC_TOPIC['ULIDAR_ARRAY']}).encode()
        return struct.pack('<HHII', 2, 0, len(header), 0) + header + b'voxel'

    async def test_disabled_cloud_is_not_decoded_but_heartbeat_is_received(self):
        self.receiver.deal_array_buffer = Mock()
        self.receiver.receive_message(self.voxel_message())
        self.receiver.receive_message(json.dumps({'type': app.DATA_CHANNEL_TYPE['HEARTBEAT'], 'topic': ''}))
        await asyncio.gather(*list(self.receiver.message_tasks))
        self.receiver.deal_array_buffer.assert_not_called()
        self.assertIsNotNone(self.receiver.last_heartbeat_monotonic)

    async def test_slow_voxel_decoder_cannot_block_heartbeat_and_queue_is_bounded(self):
        entered, release = threading.Event(), threading.Event()

        def decode(message):
            entered.set()
            release.wait(1)
            return {'type': 'msg', 'topic': app.RTC_TOPIC['ULIDAR_ARRAY']}

        self.receiver.deal_array_buffer = decode
        self.receiver.set_lidar_enabled(True)
        self.receiver.receive_message(self.voxel_message())
        try:
            for _ in range(100):
                if entered.is_set():
                    break
                await asyncio.sleep(.005)
            self.assertTrue(entered.is_set())
            for _ in range(50):
                self.receiver.receive_message(self.voxel_message())
            self.assertEqual(self.receiver.cloud_queue.qsize(), 1)
            await self.receiver._process_message(json.dumps({'type': app.DATA_CHANNEL_TYPE['HEARTBEAT'], 'topic': ''}))
            self.assertIsNotNone(self.receiver.last_heartbeat_monotonic)
            self.receiver.set_lidar_enabled(False)
            self.assertEqual(self.receiver.cloud_queue.qsize(), 0)
        finally:
            release.set()

    async def test_late_reply_to_cancelled_request_does_not_break_dispatch(self):
        future = asyncio.get_running_loop().create_future()
        self.receiver.pub_sub.future_resolver.save_resolve('msg', 'test', future, None)
        future.cancel()
        callback = Mock()
        self.receiver.pub_sub.subscriptions['test'] = callback
        await self.receiver._dispatch({'type': 'msg', 'topic': 'test'})
        callback.assert_called_once()

    async def test_early_video_track_waits_for_callback_registration(self):
        pc = SimpleNamespace(addTransceiver=Mock())
        video = DeferredVideoChannel(pc, SimpleNamespace(channel=self.channel))
        track = object()
        reader = AsyncMock()
        task = asyncio.create_task(video.track_handler(track))
        await asyncio.sleep(0)
        self.assertFalse(task.done())
        video.add_track_callback(reader)
        await asyncio.wait_for(task, timeout=1.)
        reader.assert_awaited_once_with(track)


class ConnectionServiceTests(unittest.TestCase):
    def setUp(self):
        self.service = app.MultiGo2Service()
        self.number = self.service.add_device('192.0.2.1')['number']
        self.conn = connection()
        self.service.connections[self.number] = self.conn
        self.service.devices[self.number].connected = True

    def tearDown(self):
        self.service.submit(self.service.disconnect([self.number], preserve_auto=True))
        self.service.close()

    def test_cloud_disabled_by_default_and_only_explicit_enable_subscribes(self):
        self.assertFalse(self.service.devices[self.number].cloud_enabled)
        self.service._subscribe_telemetry(self.number, self.conn)
        topics = [call.args[0] for call in self.conn.datachannel.pub_sub.subscribe.call_args_list]
        self.assertNotIn(app.RTC_TOPIC['ULIDAR_ARRAY'], topics)
        result = self.service.submit(self.service.point_cloud([self.number], True))
        self.assertTrue(result[0]['ok'])
        self.assertTrue(self.service.devices[self.number].cloud_enabled)
        self.assertEqual(self.conn.datachannel.pub_sub.subscribe.call_args.args[0], app.RTC_TOPIC['ULIDAR_ARRAY'])
        self.service.submit(self.service.point_cloud([self.number], False))
        self.assertFalse(self.service.devices[self.number].cloud_enabled)
        self.assertEqual(self.service.frames[self.number].cloud_time, 0.)
        self.assertEqual(self.conn.datachannel.pub_sub.unsubscribe.call_args.args[0], app.RTC_TOPIC['ULIDAR_ARRAY'])

    def test_health_uses_heartbeat_even_when_pose_and_cloud_are_missing(self):
        self.service.frames[self.number].last_pose_time = self.service.frames[self.number].cloud_time = 0.
        now = time.monotonic()
        self.conn.datachannel.last_heartbeat_monotonic = now-1
        self.assertEqual(self.service._connection_health(self.conn, now-20, now)['status'], 'healthy')
        self.conn.datachannel.last_heartbeat_monotonic = now-7
        self.assertEqual(self.service._connection_health(self.conn, now-20, now)['status'], 'warning')
        self.conn.datachannel.last_heartbeat_monotonic = now-13
        self.assertEqual(self.service._connection_health(self.conn, now-20, now)['status'], 'timeout')

    def test_no_initial_heartbeat_also_times_out(self):
        self.conn.datachannel.last_heartbeat_monotonic = None
        self.assertEqual(self.service._connection_health(self.conn, 0, 13)['status'], 'timeout')

    def test_timed_out_connection_is_cleaned_and_error_remains_visible(self):
        self.conn.datachannel.last_heartbeat_monotonic = time.monotonic()-20
        self.service.submit(self.service._watch_connection(self.number, self.conn))
        self.assertNotIn(self.number, self.service.connections)
        device = self.service.list_devices()[0]
        self.assertFalse(device['connected'])
        self.assertEqual(device['connection_health']['status'], 'retrying')
        self.assertIn(self.number, self.service.reconnect_tasks)
        self.assertIn('心跳', device['error'])
        self.conn.disconnect.assert_awaited_once()
        self.assertTrue(device['auto_connect'])

    def test_closed_wireless_channel_raises_instead_of_silent_success(self):
        self.conn.datachannel.channel.readyState = 'closed'
        with self.assertRaisesRegex(RuntimeError, '数据通道未打开'):
            self.service._publish_wireless_controller(self.conn)

    def test_point_cloud_api_and_health_are_available_without_map_polling(self):
        with patch.object(app, 'service', self.service):
            client = app.app.test_client()
            self.assertEqual(client.post('/api/point-cloud', json={'targets': [self.number], 'enabled': True}).status_code, 200)
            response = client.get('/api/devices').json['devices'][0]
            self.assertIn('connection_health', response)
            self.assertTrue(response['cloud_enabled'])
            self.assertEqual(client.post('/api/point-cloud', json={'targets': [999], 'enabled': True}).status_code, 400)

    def test_cloud_enable_rejection_rolls_back_local_subscription(self):
        self.conn.datachannel.disableTrafficSaving.return_value = False
        result = self.service.submit(self.service.point_cloud([self.number], True))
        self.assertFalse(result[0]['ok'])
        self.assertFalse(self.service.devices[self.number].cloud_enabled)
        self.conn.datachannel.pub_sub.subscribe.assert_not_called()
        self.conn.datachannel.set_lidar_enabled.assert_called_with(False)

    def test_disconnect_closes_peer_even_if_traffic_saving_request_fails(self):
        self.conn.datachannel.disableTrafficSaving.side_effect = RuntimeError('lost response')
        self.service.pending_motion_commands[self.number] = 1
        result = self.service.submit(self.service.disconnect([self.number], preserve_auto=True))
        self.assertFalse(result[0]['ok'])
        self.conn.disconnect.assert_awaited_once()
        self.assertNotIn(self.number, self.service.connections)
        self.assertNotIn(self.number, self.service.pending_motion_commands)
        self.assertFalse(self.service.devices[self.number].connected)

    def test_connect_replaces_stale_closed_connection(self):
        old = self.conn
        old.datachannel.channel.readyState = 'closed'
        new = connection()
        new.connect = AsyncMock()
        new.datachannel.set_decoder = Mock()
        new.video = SimpleNamespace(add_track_callback=Mock())
        with patch('app.UnitreeWebRTCConnection', return_value=new), \
                patch.object(self.service, '_enable_obstacle_avoid', new_callable=AsyncMock, return_value=True):
            result = self.service.submit(self.service.connect([self.number]))
        self.assertTrue(result[0]['ok'])
        old.disconnect.assert_awaited_once()
        new.connect.assert_awaited_once()
        self.assertIs(self.service.connections[self.number], new)
        self.assertFalse(self.service.devices[self.number].cloud_enabled)

    def test_http_wait_timeout_cancels_background_coroutine(self):
        cancelled = threading.Event()
        async def work():
            try:
                await asyncio.sleep(60)
            finally:
                cancelled.set()
        with self.assertRaisesRegex(TimeoutError, '已取消'):
            self.service.submit(work(), timeout=.1)
        self.assertTrue(cancelled.wait(1))

    def test_video_reader_drains_frames_while_disabled_without_encoding(self):
        track = SimpleNamespace(recv=AsyncMock(side_effect=[object(), object(), RuntimeError('end')]))
        with patch.object(self.service, '_encode_video_frame', new_callable=AsyncMock) as encode:
            self.service.submit(self.service._video_track_reader(self.number, track, self.conn))
            encode.assert_not_called()
        self.assertEqual(track.recv.await_count, 3)


if __name__ == '__main__':
    unittest.main()
