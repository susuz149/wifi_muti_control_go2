#!/usr/bin/env python3
"""Browser controller for multiple Go2 Air robots on the local Wi-Fi LAN."""

import argparse
import asyncio
import atexit
import ipaddress
import json
import math
import random
import sys
import threading
import time
import types
from dataclasses import dataclass, field

import cv2
import numpy as np
from flask import Flask, jsonify, render_template, request
from flask import Response

# Audio is neither available nor needed on Go2 Air. Avoid sound-device
# enumeration when importing the installed driver.
sys.modules.setdefault("sounddevice", types.ModuleType("sounddevice"))

import unitree_webrtc_connect.webrtc_driver as webrtc_driver
from unitree_webrtc_connect.constants import OBSTACLES_AVOID_API, RTC_TOPIC, SPORT_CMD, SPORT_CMD_MCF
from unitree_webrtc_connect.multicast_scanner import discover_ip_sn


MAX_SPEED = 0.5
MAX_YAW_RATE = 1.0
MAX_DURATION = 1.0
CONTROL_COMMANDS = {"stop", "move", "joystick", "sport_action"}
FORMATION_PATTERNS = {"line", "column", "wedge", "grid"}
MAX_FORMATION_SPACING = 2.0
MAX_FORMATION_SPEED = 0.25
MAX_FORMATION_DISTANCE = 4.0
CLOSED_LOOP_POSITION_TOLERANCE = 0.12
CLOSED_LOOP_YAW_TOLERANCE = 0.18
CLOSED_LOOP_TIMEOUT = 25.0
LIDAR_MAP_POINT_LIMIT = 2400
TELEMETRY_TOPICS = {
    "pose": RTC_TOPIC["ROBOTODOM"],
    "lidar_state": RTC_TOPIC["ULIDAR_STATE"],
    "lidar_voxel": RTC_TOPIC["ULIDAR_ARRAY"],
}


def action(label, category, parameter="none", risk="normal", enabled=True, note=""):
    return {
        "label": label,
        "category": category,
        "parameter": parameter,
        "risk": risk,
        "enabled": enabled,
        "note": note,
    }


NORMAL_ACTIONS = {
    "Damp": action("阻尼/卸力", "姿态", risk="high", note="电机会卸力，机器人可能倒下"),
    "BalanceStand": action("平衡站立", "姿态"),
    "StopMove": action("停止移动", "基础"),
    "StandUp": action("站起", "姿态"),
    "StandDown": action("趴下", "姿态"),
    "RecoveryStand": action("跌倒恢复站立", "姿态", risk="caution"),
    "Euler": action("机身姿态角", "调节", "xyz", risk="caution"),
    "Move": action("连续移动", "基础", enabled=False, note="请使用上方双摇杆"),
    "Sit": action("坐下", "姿态"),
    "RiseSit": action("从坐姿站起", "姿态"),
    "SwitchGait": action("切换步态", "步态", "int"),
    "Trigger": action("Trigger", "高级", "unavailable", enabled=False, note="需要专用触发参数"),
    "BodyHeight": action("身体高度", "调节", "float"),
    "FootRaiseHeight": action("抬脚高度", "调节", "float"),
    "SpeedLevel": action("速度等级", "调节", "int"),
    "Hello": action("挥手", "表演"),
    "Stretch": action("伸展", "表演"),
    "TrajectoryFollow": action("轨迹跟随", "高级", "unavailable", enabled=False, note="需要轨迹数组"),
    "ContinuousGait": action("连续步态", "步态", "bool"),
    "Content": action("开心动作", "表演"),
    "Wallow": action("打滚", "高风险", risk="high"),
    "Dance1": action("舞蹈 1", "表演", risk="caution"),
    "Dance2": action("舞蹈 2", "表演", risk="caution"),
    "GetBodyHeight": action("读取身体高度", "查询", "query"),
    "GetFootRaiseHeight": action("读取抬脚高度", "查询", "query"),
    "GetSpeedLevel": action("读取速度等级", "查询", "query"),
    "SwitchJoystick": action("切换遥控模式", "步态", "bool", risk="caution"),
    "Pose": action("Pose 模式", "姿态", "bool", risk="caution", note="用停止移动退出"),
    "Scrape": action("刨地", "表演"),
    "FrontFlip": action("前空翻", "高风险", "bool", risk="high", note="需要宽阔软质场地"),
    "FrontJump": action("前跳", "高风险", risk="high"),
    "FrontPounce": action("前扑", "高风险", risk="high"),
    "WiggleHips": action("扭臀", "表演"),
    "GetState": action("读取状态", "查询", "query"),
    "EconomicGait": action("经济步态", "步态", "bool"),
    "FingerHeart": action("比心", "表演"),
    "StandOut": action("直立模式", "高风险", "bool", risk="high", note="机器人将以不稳定姿态站立"),
    "Handstand": action("倒立", "高风险", "bool", risk="high", note="可能倾倒或损坏周围物品"),
    "CrossStep": action("交叉步", "步态", "bool"),
    "OnesidedStep": action("单侧步", "步态", "bool"),
    "Bound": action("跳跃跑", "高风险", "bool", risk="high"),
    "MoonWalk": action("太空步", "表演", "bool"),
    "LeftFlip": action("左空翻", "高风险", "bool", risk="high"),
    "RightFlip": action("右空翻", "高风险", "bool", risk="high"),
    "BackFlip": action("后空翻", "高风险", "bool", risk="high"),
    "LeadFollow": action("跟随模式", "高级", "bool", risk="caution"),
    "FreeWalk": action("自由行走", "步态", "bool"),
    "Standup": action("特殊站起", "姿态", "bool", risk="caution"),
    "CrossWalk": action("交叉行走", "步态", "bool"),
}

MCF_ACTIONS = {
    "Damp": action("阻尼/卸力", "姿态", risk="high", note="电机会卸力，机器人可能倒下"),
    "BalanceStand": action("平衡站立", "姿态"),
    "StopMove": action("停止移动", "基础"),
    "StandUp": action("站起", "姿态"),
    "StandDown": action("趴下", "姿态"),
    "RecoveryStand": action("跌倒恢复站立", "姿态", risk="caution"),
    "Euler": action("机身姿态角", "调节", "xyz", risk="caution"),
    "Move": action("连续移动", "基础", enabled=False, note="请使用上方双摇杆"),
    "Sit": action("坐下", "姿态"),
    "RiseSit": action("从坐姿站起", "姿态"),
    "SpeedLevel": action("速度等级", "调节", "int"),
    "Hello": action("挥手", "表演"),
    "Stretch": action("伸展", "表演"),
    "ContinuousGait": action("连续步态", "步态", "int"),
    "Content": action("开心动作", "表演"),
    "Dance1": action("舞蹈 1", "表演", risk="caution"),
    "Dance2": action("舞蹈 2", "表演", risk="caution"),
    "GetSpeedLevel": action("读取速度等级", "查询", "query"),
    "SwitchJoystick": action("切换遥控模式", "步态", "bool", risk="caution"),
    "Pose": action("Pose 模式", "姿态", "bool", risk="caution", note="用停止移动退出"),
    "Scrape": action("刨地", "表演"),
    "FrontFlip": action("前空翻", "高风险", "bool", risk="high"),
    "FrontJump": action("前跳", "高风险", risk="high"),
    "FrontPounce": action("前扑", "高风险", risk="high"),
    "GetState": action("读取状态", "查询", "query"),
    "Heart": action("比心", "表演"),
    "StaticWalk": action("静态行走", "步态", "bool"),
    "TrotRun": action("小跑", "步态", "bool", risk="caution"),
    "EconomicGait": action("经济步态", "步态", "bool"),
    "LeftFlip": action("左空翻", "高风险", "bool", risk="high"),
    "BackFlip": action("后空翻", "高风险", "bool", risk="high"),
    "HandStand": action("倒立", "高风险", "bool", risk="high", note="可能倾倒或损坏周围物品"),
    "FreeWalk": action("自由行走", "步态", "bool"),
    "FreeBound": action("自由跳跃跑", "高风险", "bool", risk="high"),
    "FreeJump": action("自由跳跃", "高风险", "bool", risk="high"),
    "FreeAvoid": action("自由避障", "步态", "bool"),
    "ClassicWalk": action("经典行走", "步态", "bool"),
    "BackStand": action("后足直立", "高风险", "bool", risk="high"),
    "CrossStep": action("交叉步", "步态", "bool"),
    "SetAutoRecovery": action("设置自动恢复", "设置", "bool"),
    "GetAutoRecovery": action("读取自动恢复", "查询", "query"),
    "LeadFollow": action("跟随模式", "高级", "bool", risk="caution"),
    "SwitchAvoidMode": action("切换避障模式", "设置", "bool"),
}


def build_catalog(commands, definitions):
    catalog = {}
    for name, api_id in commands.items():
        metadata = definitions.get(name, action(name, "未分类", "unavailable", enabled=False))
        catalog[name] = {"name": name, "api_id": api_id, **metadata}
    return catalog


SPORT_PROFILES = {
    "normal": {
        "label": "Normal / legacy",
        "note": "适用于普通 sport mode；旧固件通常从此模式开始。",
        "actions": build_catalog(SPORT_CMD, NORMAL_ACTIONS),
    },
    "mcf": {
        "label": "MCF (firmware >= 1.1.7)",
        "note": "机器人已处于 MCF 时选择；空翻与倒立 ID 与 Normal 不同。",
        "actions": build_catalog(SPORT_CMD_MCF, MCF_ACTIONS),
    },
}


class DisabledAudioChannel:
    """Do not advertise or enable an audio channel on Go2 Air."""

    def __init__(self, _pc, _datachannel):
        pass

    def switchAudioChannel(self, _switch):
        raise RuntimeError("Go2 Air 不支持音频通道")


webrtc_driver.WebRTCAudioChannel = DisabledAudioChannel
UnitreeWebRTCConnection = webrtc_driver.UnitreeWebRTCConnection
WebRTCConnectionMethod = webrtc_driver.WebRTCConnectionMethod


@dataclass
class Device:
    number: int
    ip: str
    sn: str = ""
    source: str = "manual"
    verified: bool = False
    connected: bool = False
    avoid_enabled: bool = False
    busy: bool = False
    error: str = ""
    last_seen: float = field(default_factory=time.time)

    def payload(self):
        return {
            "number": self.number,
            "ip": self.ip,
            "sn": self.sn,
            "source": self.source,
            "verified": self.verified,
            "connected": self.connected,
            "avoid_enabled": self.avoid_enabled,
            "busy": self.busy,
            "error": self.error,
            "last_seen": self.last_seen,
        }


@dataclass
class FrameState:
    local_pose: dict | None = None
    map_pose: dict | None = None
    map_to_local: dict | None = None
    aligned: bool = False
    anchor: dict | None = None
    last_pose_time: float = 0.0
    pose_topic: str = ""
    lidar: dict = field(default_factory=dict)
    map_points: list = field(default_factory=list)
    sequence: int = 0
    error: str = ""

    def payload(self, number, device):
        return {
            "number": number,
            "ip": device.ip,
            "sn": device.sn,
            "connected": device.connected,
            "aligned": self.aligned,
            "local_pose": self.local_pose,
            "map_pose": self.map_pose,
            "map_to_local": self.map_to_local,
            "anchor": self.anchor,
            "last_pose_age": round(time.time() - self.last_pose_time, 3) if self.last_pose_time else None,
            "pose_topic": self.pose_topic,
            "lidar": self.lidar,
            "map_points": self.map_points,
            "sequence": self.sequence,
            "error": self.error,
        }


@dataclass
class VideoState:
    enabled: bool = False
    frame: bytes | None = None
    frame_time: float = 0.0
    width: int = 0
    height: int = 0
    frames: int = 0
    error: str = ""

    def payload(self):
        return {
            "enabled": self.enabled,
            "age": round(time.time() - self.frame_time, 3) if self.frame_time else None,
            "width": self.width,
            "height": self.height,
            "frames": self.frames,
            "error": self.error,
        }


class MultiGo2Service:
    def __init__(self):
        self.lock = threading.Lock()
        self.devices = {}
        self.next_number = 1
        self.connections = {}
        self.joystick_deadmen = {}
        self.frames = {}
        self.videos = {}
        self.loop = None
        self.closed = False
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        self.ready.wait(timeout=3.0)
        if self.loop is None:
            raise RuntimeError("无法启动 WebRTC 后端事件循环")

    def _run_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.call_soon(self._keep_loop_responsive)
        self.ready.set()
        self.loop.run_forever()
        self.loop.close()

    def _keep_loop_responsive(self):
        # Some restricted runtime environments do not wake a selector after
        # call_soon_threadsafe(); polling also keeps HTTP-triggered controls responsive.
        if not self.closed:
            self.loop.call_later(0.05, self._keep_loop_responsive)

    def submit(self, coroutine, timeout=40.0):
        if self.closed or self.loop is None:
            coroutine.close()
            raise RuntimeError("WebRTC 后端已经关闭")
        return asyncio.run_coroutine_threadsafe(coroutine, self.loop).result(timeout=timeout)

    def add_device(self, ip, sn="", source="manual"):
        with self.lock:
            existing = next(
                (
                    device
                    for device in self.devices.values()
                    if device.ip == ip or (sn and device.sn == sn)
                ),
                None,
            )
            if existing:
                existing.ip = ip
                existing.sn = sn or existing.sn
                if source in {"multicast", "probe"}:
                    existing.source = source
                existing.last_seen = time.time()
                return existing.payload()
            device = Device(number=self.next_number, ip=ip, sn=sn, source=source)
            self.devices[device.number] = device
            self.frames[device.number] = FrameState()
            self.videos[device.number] = VideoState()
            self.next_number += 1
            return device.payload()

    def list_devices(self):
        with self.lock:
            return [self.devices[number].payload() for number in sorted(self.devices)]

    def require_devices(self, numbers):
        if not numbers:
            raise ValueError("请至少选择一台 Go2")
        with self.lock:
            unknown = [number for number in numbers if number not in self.devices]
            if unknown:
                raise ValueError(f"不存在的编号: {unknown}")
            return [self.devices[number] for number in numbers]

    def set_status(self, number, **updates):
        with self.lock:
            for key, value in updates.items():
                setattr(self.devices[number], key, value)

    async def scan(self, subnet, timeout=2):
        found = await asyncio.to_thread(discover_ip_sn, timeout)
        discovered = [
            self.add_device(ip=ip, sn=sn, source="multicast")
            for sn, ip in sorted(found.items(), key=lambda item: ipaddress.ip_address(item[1]))
        ]
        known_ips = {device["ip"] for device in discovered}
        candidates = await self._scan_signaling_ports(subnet)
        discovered.extend(
            self.add_device(ip=ip, source="probe")
            for ip in candidates
            if ip not in known_ips
        )
        return discovered

    async def _scan_signaling_ports(self, subnet):
        semaphore = asyncio.Semaphore(64)

        async def probe(ip):
            async with semaphore:
                for port in (9991, 8081):
                    try:
                        _reader, writer = await asyncio.wait_for(
                            asyncio.open_connection(str(ip), port), timeout=0.18
                        )
                        writer.close()
                        await writer.wait_closed()
                        return str(ip)
                    except (ConnectionError, OSError, asyncio.TimeoutError):
                        continue
            return None

        results = await asyncio.gather(*(probe(ip) for ip in subnet.hosts()))
        return sorted((ip for ip in results if ip), key=ipaddress.ip_address)

    async def connect(self, numbers):
        devices = self.require_devices(numbers)
        return await asyncio.gather(*(self._connect_one(device) for device in devices))

    async def _connect_one(self, device):
        if device.number in self.connections:
            return {"number": device.number, "ok": True}
        self.set_status(device.number, busy=True, error="")
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=device.ip)
        try:
            await asyncio.wait_for(conn.connect(), timeout=20.0)
            self.connections[device.number] = conn
            await conn.datachannel.disableTrafficSaving(True)
            self.frames.setdefault(device.number, FrameState())
            self.videos.setdefault(device.number, VideoState())
            conn.video.add_track_callback(
                lambda track, number=device.number: self._video_track_reader(number, track)
            )
            self._subscribe_telemetry(device.number, conn)
            avoid_enabled = await self._enable_obstacle_avoid(conn)
            self.set_status(
                device.number,
                connected=True,
                verified=True,
                avoid_enabled=avoid_enabled,
                busy=False,
            )
            return {"number": device.number, "ok": True, "avoid_enabled": avoid_enabled}
        except Exception as exc:
            self.connections.pop(device.number, None)
            try:
                await conn.datachannel.disableTrafficSaving(False)
                await conn.disconnect()
            except Exception:
                pass
            self.set_status(
                device.number, connected=False, avoid_enabled=False, busy=False, error=str(exc)
            )
            return {"number": device.number, "ok": False, "error": str(exc)}

    async def disconnect(self, numbers):
        self.require_devices(numbers)
        results = await asyncio.gather(*(self._disconnect_one(number) for number in numbers))
        return results

    async def _disconnect_one(self, number):
        task = self.joystick_deadmen.pop(number, None)
        if task:
            task.cancel()
        conn = self.connections.pop(number, None)
        if conn:
            try:
                self._unsubscribe_telemetry(conn)
                conn.datachannel.switchVideoChannel(False)
                await conn.datachannel.disableTrafficSaving(False)
                await conn.disconnect()
            except Exception as exc:
                self.set_status(number, connected=False, error=str(exc))
                return {"number": number, "ok": False, "error": str(exc)}
        self.set_status(number, connected=False, avoid_enabled=False, error="")
        with self.lock:
            video = self.videos.setdefault(number, VideoState())
            video.enabled = False
        return {"number": number, "ok": True}

    def _subscribe_telemetry(self, number, conn):
        for name, topic in TELEMETRY_TOPICS.items():
            def callback(message, number=number, topic=topic):
                self._telemetry_callback(number, topic, message)

            conn.datachannel.pub_sub.subscribe(topic, callback)

    @staticmethod
    def _unsubscribe_telemetry(conn):
        for topic in TELEMETRY_TOPICS.values():
            try:
                conn.datachannel.pub_sub.unsubscribe(topic)
            except Exception:
                pass

    def _telemetry_callback(self, number, topic, message):
        with self.lock:
            device = self.devices.get(number)
            frame = self.frames.setdefault(number, FrameState())
            if not device:
                return
            if number not in self.connections:
                return
            received_topic = message.get("topic") if isinstance(message, dict) else None
            if received_topic and received_topic != topic:
                frame.error = f"topic mismatch: expected {topic}, got {received_topic}"
                return
            try:
                if topic == RTC_TOPIC["ROBOTODOM"]:
                    local_pose = extract_pose(message)
                    if not local_pose:
                        frame.error = "无法解析 rt/utlidar/robot_pose"
                        return
                    frame.local_pose = local_pose
                    frame.map_pose = transform_pose(frame.map_to_local, local_pose)
                    frame.last_pose_time = time.time()
                    frame.pose_topic = topic
                    frame.sequence += 1
                    frame.error = ""
                elif topic == RTC_TOPIC["ULIDAR_STATE"]:
                    frame.lidar["state"] = compact_lidar_state(message)
                    frame.lidar["state_topic"] = topic
                    frame.lidar["state_time"] = time.time()
                elif topic == RTC_TOPIC["ULIDAR_ARRAY"]:
                    frame.lidar["voxel"] = compact_voxel_stats(message)
                    frame.lidar["voxel_topic"] = topic
                    frame.lidar["voxel_time"] = time.time()
                    frame.map_points = voxel_map_points(message, frame.map_to_local)
            except Exception as exc:
                frame.error = str(exc)

    def telemetry_payload(self):
        with self.lock:
            return [
                self.frames.setdefault(number, FrameState()).payload(number, self.devices[number])
                for number in sorted(self.devices)
            ]

    def map_payload(self):
        with self.lock:
            robots = []
            points = []
            for number in sorted(self.devices):
                frame = self.frames.setdefault(number, FrameState())
                robot = frame.payload(number, self.devices[number])
                robot.pop("map_points", None)
                robots.append(robot)
                for point in frame.map_points:
                    points.append({"number": number, "ip": self.devices[number].ip, **point})
            return {"robots": robots, "points": points}

    def align_robot(self, number, map_pose):
        with self.lock:
            if number not in self.devices:
                raise ValueError(f"不存在的编号: {number}")
            frame = self.frames.setdefault(number, FrameState())
            if not frame.local_pose:
                raise ValueError(f"#{number} 还没有收到 rt/utlidar/robot_pose，不能标定")
            frame.map_to_local = compute_map_to_local(map_pose, frame.local_pose)
            frame.anchor = dict(map_pose)
            frame.aligned = True
            frame.map_pose = transform_pose(frame.map_to_local, frame.local_pose)
            frame.map_points = []
            frame.error = ""
            return frame.payload(number, self.devices[number])

    def clear_alignment(self, numbers):
        self.require_devices(numbers)
        with self.lock:
            for number in numbers:
                frame = self.frames.setdefault(number, FrameState())
                frame.map_to_local = None
                frame.anchor = None
                frame.aligned = False
                frame.map_pose = None
                frame.map_points = []
            return [
                self.frames.setdefault(number, FrameState()).payload(number, self.devices[number])
                for number in sorted(self.devices)
            ]

    def current_map_pose(self, number):
        frame = self.frames.get(number)
        if not frame or not frame.map_pose:
            return None
        if not frame.last_pose_time or time.time() - frame.last_pose_time > 1.0:
            return None
        return dict(frame.map_pose)

    def connected_connections(self, numbers):
        self.require_devices(numbers)
        missing = [number for number in numbers if number not in self.connections]
        if missing:
            raise ValueError(f"以下 Go2 尚未连接: {missing}")
        return [(number, self.connections[number]) for number in numbers]

    async def command(self, numbers, name, parameters):
        connections = self.connected_connections(numbers)
        results = await asyncio.gather(
            *(self._command_one(number, conn, name, parameters) for number, conn in connections)
        )
        return results

    async def formation(self, numbers, parameters):
        connections = self.connected_connections(numbers)
        ordered_numbers = sorted(numbers)
        offsets = formation_offsets(ordered_numbers, parameters["pattern"], parameters["spacing"])
        leader_number = ordered_numbers[0]
        leader_pose = self.current_map_pose(leader_number)
        if not leader_pose:
            raise ValueError(f"#{leader_number} 没有可用 map_pose，请先完成坐标标定并确认 pose 正在更新")
        with self.lock:
            not_aligned = [
                number
                for number in ordered_numbers
                if not self.frames.get(number) or not self.frames[number].aligned
            ]
        if not_aligned:
            raise ValueError(f"以下 Go2 尚未对齐到 map frame: {not_aligned}")

        targets = {}
        for number, offset in offsets.items():
            dx, dy = rotate_xy(offset["x"], offset["y"], leader_pose["yaw"])
            targets[number] = {
                "x": leader_pose["x"] + dx,
                "y": leader_pose["y"] + dy,
                "z": leader_pose.get("z", 0.0),
                "yaw": leader_pose["yaw"],
            }

        return await asyncio.gather(
            *(
                self._closed_loop_to_target(number, conn, targets[number], parameters)
                for number, conn in connections
            )
        )

    async def _closed_loop_to_target(self, number, conn, target, parameters):
        deadline = time.monotonic() + parameters["timeout"]
        last_error = None
        try:
            while time.monotonic() < deadline:
                pose = self.current_map_pose(number)
                if not pose:
                    raise RuntimeError("map_pose 不可用或已超时")
                dx = target["x"] - pose["x"]
                dy = target["y"] - pose["y"]
                distance = math.hypot(dx, dy)
                yaw_error = normalize_angle(target["yaw"] - pose["yaw"])
                last_error = {"distance": round(distance, 3), "yaw": round(yaw_error, 3)}
                if (
                    distance <= parameters["position_tolerance"]
                    and abs(yaw_error) <= parameters["yaw_tolerance"]
                ):
                    await self._sport_request(conn, SPORT_CMD["StopMove"])
                    return {"number": number, "ok": True, "target": target, "error": last_error}

                local_x, local_y = rotate_xy(dx, dy, -pose["yaw"])
                scale = min(parameters["speed"] / max(distance, 0.001), 1.0)
                vx = clamp(local_x * scale, -parameters["speed"], parameters["speed"])
                vy = clamp(local_y * scale, -parameters["speed"], parameters["speed"])
                wz = clamp(yaw_error * 0.8, -parameters["yaw_rate"], parameters["yaw_rate"])
                self._move_without_reply(conn, vx, vy, wz)
                await asyncio.sleep(0.12)

            await self._sport_request(conn, SPORT_CMD["StopMove"])
            return {"number": number, "ok": False, "error": f"闭环列队超时: {last_error}", "target": target}
        except Exception as exc:
            try:
                await self._sport_request(conn, SPORT_CMD["StopMove"])
            except Exception:
                pass
            self.set_status(number, error=str(exc))
            return {"number": number, "ok": False, "error": str(exc), "target": target}

    async def _command_one(self, number, conn, name, parameters):
        try:
            if name == "move":
                self._move_without_reply(conn, parameters["x"], parameters["y"], parameters["z"])
                await asyncio.sleep(parameters["duration"])
                await self._sport_request(conn, SPORT_CMD["StopMove"])
                response = None
            elif name == "joystick":
                if parameters["mode"] == "sport":
                    self._move_without_reply(conn, parameters["x"], parameters["y"], parameters["z"])
                else:
                    self._publish_wireless_controller(
                        conn, parameters["lx"], parameters["ly"], parameters["rx"]
                    )
                self._arm_joystick_deadman(number, conn)
                response = None
            elif name == "sport_action":
                response = await self._sport_request(
                    conn, parameters["api_id"], parameters.get("parameter")
                )
            else:
                task = self.joystick_deadmen.pop(number, None)
                if task:
                    task.cancel()
                response = await self._sport_request(conn, SPORT_CMD["StopMove"])
            result = {"number": number, "ok": True}
            if parameters.get("query") and response is not None:
                result["response"] = response
            return result
        except Exception as exc:
            self.set_status(number, error=str(exc))
            return {"number": number, "ok": False, "error": str(exc)}

    def _arm_joystick_deadman(self, number, conn):
        task = self.joystick_deadmen.pop(number, None)
        if task:
            task.cancel()
        self.joystick_deadmen[number] = asyncio.create_task(self._joystick_deadman(number, conn))

    async def _joystick_deadman(self, number, conn):
        try:
            await asyncio.sleep(0.35)
            self._publish_wireless_controller(conn)
            await self._sport_request(conn, SPORT_CMD["StopMove"])
        except asyncio.CancelledError:
            return
        except Exception as exc:
            self.set_status(number, error=str(exc))
        finally:
            current = self.joystick_deadmen.get(number)
            if current is asyncio.current_task():
                self.joystick_deadmen.pop(number, None)

    @staticmethod
    async def _sport_request(conn, api_id, parameter=None):
        options = {"api_id": api_id}
        if parameter is not None:
            options["parameter"] = parameter
        return await asyncio.wait_for(
            conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], options),
            timeout=3.0,
        )

    @staticmethod
    def _move_without_reply(conn, x, y, z):
        request_id = int(time.time() * 1000) % 2147483648 + random.randint(0, 1000)
        payload = {
            "header": {
                "identity": {"id": request_id, "api_id": SPORT_CMD["Move"]},
                "policy": {"priority": 0, "noreply": True},
            },
            "parameter": json.dumps({"x": x, "y": y, "z": z}),
            "binary": [],
        }
        conn.datachannel.pub_sub.publish_without_callback(RTC_TOPIC["SPORT_MOD"], payload)

    @staticmethod
    def _publish_wireless_controller(conn, lx=0.0, ly=0.0, rx=0.0, ry=0.0, keys=0):
        conn.datachannel.pub_sub.publish_without_callback(
            RTC_TOPIC["WIRELESS_CONTROLLER"],
            {"lx": lx, "ly": ly, "rx": rx, "ry": ry, "keys": keys},
        )

    async def _enable_obstacle_avoid(self, conn):
        responses = []
        for _attempt in range(2):
            responses.append(
                await self._obstacle_avoid_request(
                    conn, OBSTACLES_AVOID_API["SWITCH_SET"], {"enable": True}
                )
            )
            await asyncio.sleep(0.2)
        code, enabled = await self._query_obstacle_avoid(conn)
        if enabled is not True:
            await self._sport_request(conn, SPORT_CMD_MCF["FreeAvoid"], {"data": True})
            code, enabled = await self._query_obstacle_avoid(conn)
        if enabled is not True:
            raise RuntimeError(f"自动开启自由避障失败，状态={enabled}, code={code}, responses={responses}")
        return True

    @staticmethod
    async def _obstacle_avoid_request(conn, api_id, parameter=None):
        options = {"api_id": api_id}
        if parameter is not None:
            options["parameter"] = parameter
        return await asyncio.wait_for(
            conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["OBSTACLES_AVOID"], options),
            timeout=3.0,
        )

    async def _query_obstacle_avoid(self, conn):
        response = await self._obstacle_avoid_request(conn, OBSTACLES_AVOID_API["SWITCH_GET"])
        code = response.get("data", {}).get("header", {}).get("status", {}).get("code", -1)
        data = response.get("data", {}).get("data", "")
        enabled = None
        if code == 0 and data:
            try:
                enabled = json.loads(data).get("enable")
            except (TypeError, ValueError):
                enabled = None
        return code, enabled

    async def video(self, numbers, enabled):
        connections = self.connected_connections(numbers)
        for number, conn in connections:
            conn.datachannel.switchVideoChannel(enabled)
            with self.lock:
                video = self.videos.setdefault(number, VideoState())
                video.enabled = enabled
                video.error = ""
        return [{"number": number, "ok": True, "enabled": enabled} for number, _conn in connections]

    async def _video_track_reader(self, number, track):
        while True:
            try:
                frame = await track.recv()
                image = frame.to_ndarray(format="bgr24")
                ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if not ok:
                    continue
                with self.lock:
                    video = self.videos.setdefault(number, VideoState())
                    video.frame = encoded.tobytes()
                    video.frame_time = time.time()
                    video.height, video.width = image.shape[:2]
                    video.frames += 1
                    video.error = ""
            except Exception as exc:
                with self.lock:
                    self.videos.setdefault(number, VideoState()).error = str(exc)
                return

    def video_payload(self):
        with self.lock:
            return {
                str(number): self.videos.setdefault(number, VideoState()).payload()
                for number in sorted(self.devices)
            }

    def video_frame(self, number):
        with self.lock:
            return self.videos.setdefault(number, VideoState()).frame

    async def stop_and_disconnect_all(self):
        numbers = list(self.connections)
        if numbers:
            await self.command(numbers, "stop", {})
            await self.disconnect(numbers)

    def close(self):
        if self.closed or self.loop is None:
            return
        try:
            self.submit(self.stop_and_disconnect_all(), timeout=6.0)
        except Exception:
            pass
        self.closed = True
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(timeout=2.0)


app = Flask(__name__, template_folder="wifi_web/templates", static_folder="wifi_web/static")
service = MultiGo2Service()
atexit.register(service.close)


def payload_error(message, status=400):
    return jsonify({"ok": False, "error": message}), status


def nested(value, *keys):
    current = value
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def numeric(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def yaw_from_quaternion(value):
    if not isinstance(value, dict):
        return 0.0
    x = numeric(value.get("x"))
    y = numeric(value.get("y"))
    z = numeric(value.get("z"))
    w = numeric(value.get("w"), 1.0)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def rotate_xy(x, y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return cos_yaw * x - sin_yaw * y, sin_yaw * x + cos_yaw * y


def extract_position(pose):
    if not isinstance(pose, dict):
        return None
    position = pose.get("position") or pose.get("translation") or pose.get("pos")
    if isinstance(position, dict):
        return {
            "x": numeric(position.get("x")),
            "y": numeric(position.get("y")),
            "z": numeric(position.get("z")),
        }
    if all(key in pose for key in ("x", "y")):
        return {"x": numeric(pose.get("x")), "y": numeric(pose.get("y")), "z": numeric(pose.get("z"))}
    return None


def extract_pose(message):
    pose = nested(message, "data", "pose") or nested(message, "data", "data", "pose")
    if not isinstance(pose, dict):
        return None
    position = extract_position(pose)
    if not position:
        return None
    orientation = pose.get("orientation") or pose.get("rotation") or pose.get("quat") or {}
    yaw = numeric(pose.get("yaw"), yaw_from_quaternion(orientation))
    return {
        "x": round(position["x"], 4),
        "y": round(position["y"], 4),
        "z": round(position["z"], 4),
        "yaw": round(normalize_angle(yaw), 5),
    }


def transform_pose(transform, local_pose):
    if not transform or not local_pose:
        return None
    yaw = numeric(transform.get("yaw"))
    rotated_x, rotated_y = rotate_xy(local_pose["x"], local_pose["y"], yaw)
    x = rotated_x + transform["x"]
    y = rotated_y + transform["y"]
    return {
        "x": round(x, 4),
        "y": round(y, 4),
        "z": round(local_pose.get("z", 0.0) + transform.get("z", 0.0), 4),
        "yaw": round(normalize_angle(local_pose.get("yaw", 0.0) + yaw), 5),
    }


def compute_map_to_local(map_pose, local_pose):
    yaw = normalize_angle(map_pose["yaw"] - local_pose["yaw"])
    rotated_local_x, rotated_local_y = rotate_xy(local_pose["x"], local_pose["y"], yaw)
    return {
        "x": round(map_pose["x"] - rotated_local_x, 4),
        "y": round(map_pose["y"] - rotated_local_y, 4),
        "z": round(map_pose.get("z", 0.0) - local_pose.get("z", 0.0), 4),
        "yaw": round(yaw, 5),
    }


def compact_lidar_state(message):
    data = message.get("data", {}) if isinstance(message, dict) else {}
    keys = [
        "cloud_frequency",
        "cloud_packet_loss_rate",
        "cloud_scan_num",
        "cloud_size",
        "dirty_percentage",
        "error_state",
        "firmware_version",
        "imu_frequency",
        "imu_packet_loss_rate",
        "sdk_version",
    ]
    return {key: data.get(key) for key in keys if key in data}


def compact_voxel_stats(message):
    payload = nested(message, "data", "data")
    if isinstance(payload, dict):
        return {
            "point_count": int(payload.get("point_count") or 0),
            "face_count": int(payload.get("face_count") or 0),
            "positions_bytes": int(getattr(payload.get("positions"), "nbytes", 0) or 0),
            "uvs_bytes": int(getattr(payload.get("uvs"), "nbytes", 0) or 0),
            "indices_bytes": int(getattr(payload.get("indices"), "nbytes", 0) or 0),
        }
    return {"raw_type": type(payload).__name__}


def voxel_map_points(message, transform):
    if not transform:
        return []
    payload = nested(message, "data", "data")
    if not isinstance(payload, dict):
        return []
    positions = payload.get("positions")
    if positions is None:
        return []
    try:
        raw = positions.tobytes() if hasattr(positions, "tobytes") else bytes(positions)
        floats = np.frombuffer(raw, dtype=np.float32)
        usable = (floats.size // 3) * 3
        if usable <= 0:
            return []
        points = floats[:usable].reshape((-1, 3))
        if len(points) > LIDAR_MAP_POINT_LIMIT:
            step = max(1, len(points) // LIDAR_MAP_POINT_LIMIT)
            points = points[::step]
        mapped = []
        for x, y, z in points:
            if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                continue
            if abs(float(x)) > 80.0 or abs(float(y)) > 80.0 or abs(float(z)) > 10.0:
                continue
            map_x, map_y = rotate_xy(float(x), float(y), transform["yaw"])
            mapped.append(
                {
                    "x": round(map_x + transform["x"], 3),
                    "y": round(map_y + transform["y"], 3),
                    "z": round(float(z) + transform.get("z", 0.0), 3),
                }
            )
        return mapped[:LIDAR_MAP_POINT_LIMIT]
    except Exception:
        return []


def parse_numbers(data):
    raw = data.get("targets", [])
    if not isinstance(raw, list):
        raise ValueError("targets 必须是编号数组")
    try:
        numbers = sorted(set(int(number) for number in raw))
    except (TypeError, ValueError) as exc:
        raise ValueError("设备编号无效") from exc
    return numbers


def parse_ip(value):
    try:
        parsed = ipaddress.ip_address(str(value).strip())
    except ValueError as exc:
        raise ValueError("IP 地址无效") from exc
    if parsed.version != 4:
        raise ValueError("当前仅支持 IPv4 局域网设备")
    return str(parsed)


def parse_subnet(value):
    try:
        network = ipaddress.ip_network(str(value).strip() or "192.168.12.0/24", strict=False)
    except ValueError as exc:
        raise ValueError("扫描网段无效，例如 192.168.12.0/24") from exc
    if network.version != 4 or network.num_addresses > 1024:
        raise ValueError("扫描仅支持不大于 /22 的 IPv4 网段")
    return network


def bounded_float(data, name, default, minimum, maximum):
    try:
        value = float(data.get(name, default))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是数字") from exc
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} 必须在 {minimum} 到 {maximum} 之间")
    return value


def movement_parameters(data):
    direction = str(data.get("direction", ""))
    speed = bounded_float(data, "speed", 0.15, 0.01, MAX_SPEED)
    yaw_rate = bounded_float(data, "yaw_rate", 0.35, 0.01, MAX_YAW_RATE)
    duration = bounded_float(data, "duration", 0.25, 0.05, MAX_DURATION)
    vectors = {
        "forward": (speed, 0.0, 0.0),
        "backward": (-speed, 0.0, 0.0),
        "left": (0.0, speed, 0.0),
        "right": (0.0, -speed, 0.0),
        "rotate_left": (0.0, 0.0, yaw_rate),
        "rotate_right": (0.0, 0.0, -yaw_rate),
    }
    if direction not in vectors:
        raise ValueError("移动方向无效")
    x, y, z = vectors[direction]
    return {"x": x, "y": y, "z": z, "duration": duration}


def formation_offsets(numbers, pattern, spacing):
    count = len(numbers)
    center = (count - 1) / 2.0
    offsets = {}
    if pattern == "line":
        for index, number in enumerate(numbers):
            offsets[number] = {"x": 0.0, "y": (index - center) * spacing}
    elif pattern == "column":
        for index, number in enumerate(numbers):
            offsets[number] = {"x": -index * spacing, "y": 0.0}
    elif pattern == "wedge":
        offsets[numbers[0]] = {"x": 0.0, "y": 0.0}
        for index, number in enumerate(numbers[1:], start=1):
            side = -1.0 if index % 2 else 1.0
            row = math.ceil(index / 2)
            offsets[number] = {"x": -row * spacing, "y": side * row * spacing}
    elif pattern == "grid":
        columns = max(1, math.ceil(math.sqrt(count)))
        rows = math.ceil(count / columns)
        for index, number in enumerate(numbers):
            row = index // columns
            column = index % columns
            row_width = min(columns, count - row * columns)
            offsets[number] = {
                "x": -row * spacing,
                "y": (column - (row_width - 1) / 2.0) * spacing,
            }
    else:
        raise ValueError("未知队形")
    return offsets


def formation_parameters(data, numbers):
    pattern = str(data.get("pattern", "line"))
    if pattern not in FORMATION_PATTERNS:
        raise ValueError("队形仅支持横队、纵队、楔形和方阵")
    if len(numbers) < 2:
        raise ValueError("自动列队至少需要两台已连接 Go2")
    spacing = bounded_float(data, "spacing", 0.8, 0.3, MAX_FORMATION_SPACING)
    speed = bounded_float(data, "formation_speed", 0.12, 0.03, MAX_FORMATION_SPEED)
    yaw_rate = bounded_float(data, "yaw_rate", 0.35, 0.05, MAX_YAW_RATE)
    timeout = bounded_float(data, "timeout", CLOSED_LOOP_TIMEOUT, 5.0, 60.0)
    position_tolerance = bounded_float(
        data, "position_tolerance", CLOSED_LOOP_POSITION_TOLERANCE, 0.05, 0.5
    )
    yaw_tolerance = bounded_float(data, "yaw_tolerance", CLOSED_LOOP_YAW_TOLERANCE, 0.05, 0.8)
    offsets = formation_offsets(numbers, pattern, spacing)
    max_distance = max(math.hypot(offset["x"], offset["y"]) for offset in offsets.values())
    if max_distance > MAX_FORMATION_DISTANCE:
        raise ValueError(f"队形最大位移 {max_distance:.2f} m 超出 {MAX_FORMATION_DISTANCE:.1f} m 安全限制")
    return {
        "pattern": pattern,
        "spacing": spacing,
        "speed": speed,
        "yaw_rate": yaw_rate,
        "timeout": timeout,
        "position_tolerance": position_tolerance,
        "yaw_tolerance": yaw_tolerance,
        "offsets": offsets,
    }


def map_pose_parameters(data):
    return {
        "x": bounded_float(data, "x", 0.0, -200.0, 200.0),
        "y": bounded_float(data, "y", 0.0, -200.0, 200.0),
        "z": bounded_float(data, "z", 0.0, -20.0, 20.0),
        "yaw": bounded_float(data, "yaw", 0.0, -math.pi * 2.0, math.pi * 2.0),
    }


def joystick_parameters(data):
    x = bounded_float(data, "x", 0.0, -MAX_SPEED, MAX_SPEED)
    y = bounded_float(data, "y", 0.0, -MAX_SPEED, MAX_SPEED)
    z = bounded_float(data, "z", 0.0, -MAX_YAW_RATE, MAX_YAW_RATE)
    mode = str(data.get("joystick_mode", "wireless")).lower()
    if mode not in {"wireless", "sport"}:
        raise ValueError("摇杆通道必须是 wireless 或 sport")
    return {
        "mode": mode,
        "x": x,
        "y": y,
        "z": z,
        "lx": y / MAX_SPEED,
        "ly": x / MAX_SPEED,
        "rx": z / MAX_YAW_RATE,
    }


def bool_value(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() not in {"false", "off", "0", "no"}


def sport_action_parameters(data):
    profile_name = str(data.get("profile", "normal")).lower()
    profile = SPORT_PROFILES.get(profile_name)
    if not profile:
        raise ValueError("未知 Sport API 模式")
    name = str(data.get("action", ""))
    metadata = profile["actions"].get(name)
    if not metadata:
        raise ValueError("该模式下不存在此 Sport API")
    if not metadata["enabled"]:
        raise ValueError(f"{name} 仅展示 SDK 能力，当前页面不直接发送该参数")
    high_risk_activation = metadata["risk"] == "high" and not (
        metadata["parameter"] == "bool" and not bool_value(data.get("value", True))
    )
    if high_risk_activation and not data.get("confirm_high_risk"):
        raise ValueError("空翻、直立、卸力等高风险动作必须单独确认")

    parameter_type = metadata["parameter"]
    parameter = None
    if parameter_type == "bool":
        parameter = {"data": bool_value(data.get("value", True))}
    elif parameter_type == "int":
        try:
            parameter = {"data": int(data.get("value", 1))}
        except (TypeError, ValueError) as exc:
            raise ValueError("该动作参数必须是整数") from exc
    elif parameter_type == "float":
        parameter = {"data": bounded_float(data, "value", 0.0, -1.0, 1.0)}
    elif parameter_type == "xyz":
        parameter = {
            "x": bounded_float(data, "x", 0.0, -1.0, 1.0),
            "y": bounded_float(data, "y", 0.0, -1.0, 1.0),
            "z": bounded_float(data, "z", 0.0, -1.0, 1.0),
        }
    elif parameter_type == "query" and name == "GetState" and profile_name == "mcf":
        parameter = ["state", "bodyHeight", "speedLevel", "gait", "continuousGait", "economicGait"]
    return {
        "api_id": metadata["api_id"],
        "parameter": parameter,
        "query": parameter_type == "query",
        "action": name,
        "profile": profile_name,
    }


@app.get("/")
def index():
    return render_template("index.html")


@app.get("/api/devices")
def devices():
    return jsonify({"ok": True, "devices": service.list_devices(), "audio_enabled": False})


@app.get("/api/sport/catalog")
def sport_catalog():
    return jsonify({"ok": True, "profiles": SPORT_PROFILES})


@app.get("/api/telemetry")
def telemetry():
    return jsonify({"ok": True, "robots": service.telemetry_payload(), "topics": TELEMETRY_TOPICS})


@app.get("/api/map")
def shared_map():
    payload = service.map_payload()
    return jsonify({"ok": True, **payload})


@app.get("/api/video/status")
def video_status():
    return jsonify({"ok": True, "videos": service.video_payload()})


@app.get("/api/video/frame/<int:number>")
def video_frame(number):
    frame = service.video_frame(number)
    if not frame:
        return payload_error("该编号暂无视频帧", 404)
    return Response(frame, mimetype="image/jpeg")


@app.post("/api/video")
def video():
    data = request.get_json(silent=True) or {}
    try:
        results = service.submit(
            service.video(parse_numbers(data), bool_value(data.get("enabled", True))),
            timeout=8.0,
        )
    except (ValueError, TimeoutError) as exc:
        return payload_error(str(exc))
    return jsonify({"ok": True, "results": results, "videos": service.video_payload()})


@app.post("/api/frame/align")
def align_frame():
    data = request.get_json(silent=True) or {}
    try:
        number = int(data.get("number"))
        frame = service.align_robot(number, map_pose_parameters(data))
    except (TypeError, ValueError) as exc:
        return payload_error(str(exc))
    return jsonify({"ok": True, "frame": frame, "robots": service.telemetry_payload()})


@app.post("/api/frame/clear")
def clear_frame():
    data = request.get_json(silent=True) or {}
    try:
        robots = service.clear_alignment(parse_numbers(data))
    except ValueError as exc:
        return payload_error(str(exc))
    return jsonify({"ok": True, "robots": robots})


@app.post("/api/scan")
def scan():
    data = request.get_json(silent=True) or {}
    try:
        subnet = parse_subnet(data.get("subnet", "192.168.12.0/24"))
        discovered = service.submit(service.scan(subnet), timeout=12.0)
        if data.get("auto_connect") and discovered:
            service.submit(
                service.connect([device["number"] for device in discovered]), timeout=45.0
            )
    except Exception as exc:
        return payload_error(f"扫描失败: {exc}")
    return jsonify({"ok": True, "devices": service.list_devices()})


@app.post("/api/devices")
def add_device():
    data = request.get_json(silent=True) or {}
    try:
        device = service.add_device(parse_ip(data.get("ip", "")))
    except ValueError as exc:
        return payload_error(str(exc))
    return jsonify({"ok": True, "device": device, "devices": service.list_devices()})


@app.post("/api/connect")
def connect():
    data = request.get_json(silent=True) or {}
    try:
        results = service.submit(service.connect(parse_numbers(data)), timeout=45.0)
    except (ValueError, TimeoutError) as exc:
        return payload_error(str(exc))
    return jsonify({"ok": True, "results": results, "devices": service.list_devices()})


@app.post("/api/disconnect")
def disconnect():
    data = request.get_json(silent=True) or {}
    try:
        results = service.submit(service.disconnect(parse_numbers(data)), timeout=15.0)
    except (ValueError, TimeoutError) as exc:
        return payload_error(str(exc))
    return jsonify({"ok": True, "results": results, "devices": service.list_devices()})


@app.post("/api/command")
def command():
    data = request.get_json(silent=True) or {}
    name = str(data.get("command", "")).lower()
    try:
        if name not in CONTROL_COMMANDS:
            raise ValueError("不支持的控制命令")
        numbers = parse_numbers(data)
        if name == "move":
            parameters = movement_parameters(data)
        elif name == "joystick":
            parameters = joystick_parameters(data)
        elif name == "sport_action":
            parameters = sport_action_parameters(data)
        else:
            parameters = {}
        requires_motion_confirmation = name not in {"stop"} and not (
            name == "sport_action"
            and (parameters["query"] or parameters["action"] == "StopMove")
        )
        if requires_motion_confirmation and not data.get("confirm_motion"):
            raise ValueError("发送真机动作前必须开启安全确认")
        results = service.submit(service.command(numbers, name, parameters), timeout=12.0)
    except (ValueError, TimeoutError) as exc:
        return payload_error(str(exc))
    return jsonify({"ok": True, "results": results, "devices": service.list_devices()})


@app.post("/api/formation")
def formation():
    data = request.get_json(silent=True) or {}
    try:
        numbers = parse_numbers(data)
        parameters = formation_parameters(data, numbers)
        if not data.get("confirm_motion"):
            raise ValueError("自动列队会发送真机移动指令，请先开启安全确认")
        if not data.get("confirm_formation"):
            raise ValueError("闭环自动列队会驱动真机，必须单独确认队形区域安全")
        results = service.submit(service.formation(numbers, parameters), timeout=45.0)
    except (ValueError, TimeoutError) as exc:
        return payload_error(str(exc))
    return jsonify(
        {
            "ok": True,
            "results": results,
            "offsets": parameters.get("offsets", {}),
            "devices": service.list_devices(),
        }
    )


def main():
    parser = argparse.ArgumentParser(description="Go2 Air Wi-Fi multi-robot browser controller")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8090, type=int)
    args = parser.parse_args()
    try:
        app.run(host=args.host, port=args.port, debug=False, threaded=True)
    finally:
        service.close()


if __name__ == "__main__":
    main()
