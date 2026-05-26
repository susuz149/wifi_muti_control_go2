#!/usr/bin/env python3
"""Browser controller for multiple Go2 Air robots on the local Wi-Fi LAN."""

import argparse
import asyncio
import atexit
import ipaddress
import json
import random
import sys
import threading
import time
import types
from dataclasses import dataclass, field

from flask import Flask, jsonify, render_template, request

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


class MultiGo2Service:
    def __init__(self):
        self.lock = threading.Lock()
        self.devices = {}
        self.next_number = 1
        self.connections = {}
        self.joystick_deadmen = {}
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
            try:
                await conn.disconnect()
            except Exception:
                pass
            self.set_status(device.number, connected=False, avoid_enabled=False, busy=False, error=str(exc))
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
                await conn.disconnect()
            except Exception as exc:
                self.set_status(number, connected=False, error=str(exc))
                return {"number": number, "ok": False, "error": str(exc)}
        self.set_status(number, connected=False, avoid_enabled=False, error="")
        return {"number": number, "ok": True}

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

    async def _command_one(self, number, conn, name, parameters):
        try:
            if name == "move":
                self._move_without_reply(conn, parameters["x"], parameters["y"], parameters["z"])
                await asyncio.sleep(parameters["duration"])
                await self._sport_request(conn, SPORT_CMD["StopMove"])
                response = None
            elif name == "joystick":
                self._publish_wireless_controller(conn, parameters["lx"], parameters["ly"], parameters["rx"])
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
            # Some firmware versions expose MCF FreeAvoid in addition to the
            # obstacle-avoid service. Enabling both keeps the browser default conservative.
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


def joystick_parameters(data):
    x = bounded_float(data, "x", 0.0, -MAX_SPEED, MAX_SPEED)
    y = bounded_float(data, "y", 0.0, -MAX_SPEED, MAX_SPEED)
    z = bounded_float(data, "z", 0.0, -MAX_YAW_RATE, MAX_YAW_RATE)
    return {
        # Obstacle avoidance filters simulated remote-controller commands.
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
