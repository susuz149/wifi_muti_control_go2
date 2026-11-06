"""Compatibility fixes for the pinned unitree_webrtc_connect 2.1.2 driver.

Keep these fixes in the project rather than editing site-packages. The driver
shares one asyncio loop with robot control, so signaling and voxel decoding
must not block it.
"""
import asyncio
import json
import logging
import struct
import time

import requests
from unitree_webrtc_connect import unitree_auth, webrtc_driver
from unitree_webrtc_connect.constants import DATA_CHANNEL_TYPE
from unitree_webrtc_connect.webrtc_datachannel import WebRTCDataChannel
from unitree_webrtc_connect.webrtc_video import WebRTCVideoChannel


def bounded_local_request(path, body=None, headers=None):
    try:
        response = requests.post(path, data=body, headers=headers, timeout=(3, 5))
        response.raise_for_status()
        return response if response.status_code == 200 else None
    except requests.exceptions.RequestException as exc:
        logging.warning("LAN signaling request failed: %s", exc)
        return None


class ResponsiveDataChannel(WebRTCDataChannel):
    CLOUD_INTERVAL = .2

    def __init__(self, conn, pc):
        super().__init__(conn, pc)
        self.lidar_enabled = False
        self.lidar_epoch = 0
        self.last_rx_monotonic = None
        self.last_heartbeat_monotonic = None
        self.cloud_queue = asyncio.Queue(maxsize=1)
        self.cloud_task = None
        self.message_tasks = set()
        self.last_decode_error = ""
        self.channel.remove_all_listeners("message")
        self.channel.on("message", self.receive_message)
        self.channel.on("close", self.stop_readers)

    def set_decoder(self, decoder_type):
        # The app only uses native meter-based voxels; avoid constructing the
        # much heavier WASM decoder during every connection's initialization.
        super().set_decoder("native")

    def set_lidar_enabled(self, enabled):
        self.lidar_enabled = enabled
        self.lidar_epoch += 1
        self.last_decode_error = ''
        while not self.cloud_queue.empty():
            self.cloud_queue.get_nowait()

    def stop_readers(self):
        self.set_lidar_enabled(False)
        if self.cloud_task:
            self.cloud_task.cancel()
        for task in list(self.message_tasks):
            task.cancel()

    @staticmethod
    def binary_topic(message):
        first, second = struct.unpack_from("<HH", message)
        if (first, second) == (2, 0):
            length, = struct.unpack_from("<I", message, 4)
            header = message[12:12 + length]
        else:
            length = first
            header = message[4:4 + length]
        return json.loads(header).get("topic", "")

    def receive_message(self, message):
        self.last_rx_monotonic = time.monotonic()
        try:
            if isinstance(message, bytes) and "utlidar" in self.binary_topic(message):
                if not self.lidar_enabled:
                    return
                if self.cloud_queue.full():
                    self.cloud_queue.get_nowait()
                self.cloud_queue.put_nowait((self.lidar_epoch, message))
                if self.cloud_task is None or self.cloud_task.done():
                    self.cloud_task = asyncio.create_task(self._cloud_reader())
                return
            task = asyncio.create_task(self._process_message(message))
            self.message_tasks.add(task)
            task.add_done_callback(self.message_tasks.discard)
        except Exception:
            logging.exception("Invalid WebRTC message header")

    async def _dispatch(self, parsed):
        if parsed.get("type") == DATA_CHANNEL_TYPE["HEARTBEAT"]:
            self.last_heartbeat_monotonic = time.monotonic()
        # The upstream resolver tries set_result() on cancelled request futures.
        # Remove those first so late replies cannot break telemetry dispatch.
        pending = self.pub_sub.future_resolver.pending_callbacks
        for key, futures in list(pending.items()):
            live = [future for future in futures if not future.done()]
            if live:
                pending[key] = live
            else:
                pending.pop(key, None)
        self.pub_sub.run_resolve(parsed)
        await self.handle_response(parsed)

    async def _process_message(self, message):
        try:
            if not message:
                return
            parsed = json.loads(message) if isinstance(message, str) else self.deal_array_buffer(message)
            await self._dispatch(parsed)
        except Exception:
            logging.exception("Error processing WebRTC message")

    async def _cloud_reader(self):
        next_decode = 0.
        try:
            while self.lidar_enabled:
                await asyncio.sleep(max(0., next_decode - time.monotonic()))
                epoch, message = await self.cloud_queue.get()
                next_decode = time.monotonic() + self.CLOUD_INTERVAL
                try:
                    parsed = await asyncio.to_thread(self.deal_array_buffer, message)
                    if self.lidar_enabled and epoch == self.lidar_epoch:
                        await self._dispatch(parsed)
                        self.last_decode_error = ""
                except Exception as exc:
                    self.last_decode_error = str(exc)
                    logging.exception("Error decoding LiDAR")
        except asyncio.CancelledError:
            return


class ResponsiveConnection(webrtc_driver.UnitreeWebRTCConnection):
    async def get_answer_from_local_peer(self, pc, ip):
        offer = pc.localDescription
        payload = {
            "id": "STA_localNetwork" if self.connectionMethod == webrtc_driver.WebRTCConnectionMethod.LocalSTA else "",
            "sdp": offer.sdp, "type": offer.type, "token": self.token,
        }
        return await asyncio.to_thread(
            webrtc_driver.send_sdp_to_local_peer, ip, json.dumps(payload),
            aes_128_key=self.aes_128_key,
        )


class DeferredVideoChannel(WebRTCVideoChannel):
    """The first video frame can arrive before app.connect registers its reader."""

    def __init__(self, pc, datachannel):
        super().__init__(pc, datachannel)
        self.reader_ready = asyncio.Event()
        datachannel.channel.on('close', self.reader_ready.set)

    def add_track_callback(self, callback):
        super().add_track_callback(callback)
        if self.track_callbacks:
            self.reader_ready.set()

    async def track_handler(self, track):
        await self.reader_ready.wait()
        await super().track_handler(track)


def install_driver_fixes():
    # These names are looked up by the upstream driver/auth modules at runtime.
    unitree_auth.make_local_request = bounded_local_request
    webrtc_driver.WebRTCDataChannel = ResponsiveDataChannel
    webrtc_driver.WebRTCVideoChannel = DeferredVideoChannel
