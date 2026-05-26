# Go2 Air Wi-Fi Multi Controller

Browser-based multi-robot controller for Unitree Go2 Air on the same Wi-Fi LAN.

This project uses `unitree_webrtc_connect` over LocalSTA WebRTC data channels. It is intended for Go2 firmware below `1.1.15`, where LAN control does not require a per-device AES key.

## Features

- Scan the current Wi-Fi subnet for Go2 candidates.
- Manually add robots by IP address.
- Assign stable numbers in the browser session.
- Connect and control one robot, all robots, or any selected subset.
- Dual virtual joystick layout similar to a physical remote:
  - Left stick: forward/backward and side movement.
  - Right stick: yaw/turning.
- Deadman safety:
  - Releasing joysticks sends `StopMove`.
  - Window blur/hidden sends `StopMove`.
  - Backend stops joystick motion if refresh commands stop arriving.
- Obstacle avoidance is enabled automatically after each successful WebRTC connection.
- Joystick channel is selectable:
  - `wirelesscontroller` simulated remote input is the default and is filtered by obstacle avoidance.
  - `Sport Move` can be selected when the user deliberately wants direct motion that bypasses obstacle-avoidance filtering.
- Sport SDK action catalog:
  - Normal / legacy profile.
  - MCF profile for firmware modes using MCF API IDs.
  - High-risk actions such as flips, handstand/upright modes, jumps, and motor damping require extra confirmation.
- Go2 Air audio is disabled in the WebRTC driver path.

## Quick Start

See [INSTALL_CN.md](INSTALL_CN.md) for the full Chinese installation guide.

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip portaudio19-dev

git clone https://github.com/Uoghluvm/go2_air_wifi_multi_control.git
cd go2_air_wifi_multi_control

python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

./run.sh --host 0.0.0.0 --port 8090
```

Open:

```text
http://127.0.0.1:8090
```

If opening from another device on the same LAN, replace `127.0.0.1` with this computer's IP address.

## Safety

This software can send real motion commands to robots. Use it only in a clear area with a physical remote or emergency stop available.

The UI deliberately blocks normal motion until the safety checkbox is enabled. High-risk actions require a second checkbox and browser confirmation.

## Notes

- Default scan subnet: `192.168.12.0/24`.
- Known Go2 LocalSTA signaling ports checked by scanner: `9991` and `8081`.
- After connection, the app calls `rt/api/obstacles_avoid/request` with `SWITCH_SET {"enable": true}` and verifies the state with `SWITCH_GET`.
- Default joystick channel is `rt/wirelesscontroller`. The UI can switch to `rt/api/sport/request` `Move` for direct control.
- For Go2 firmware `>= 1.1.15`, Unitree LAN signaling may require a per-device AES key. This project is currently scoped for firmware `< 1.1.15`.

---

# Go2 Air Wi-Fi 多机控制台

这是一个通过浏览器控制多台 Unitree Go2 Air 的局域网 Web 控制台。

项目使用 `unitree_webrtc_connect` 的 LocalSTA WebRTC data channel。当前面向 Go2 固件小于 `1.1.15` 的旧固件流程，这类固件的局域网连接通常不需要 per-device AES key。

完整中文安装步骤见 [INSTALL_CN.md](INSTALL_CN.md)。

## 功能

- 扫描当前 Wi-Fi 网段中的 Go2 候选设备。
- 支持手动输入机器人 IP。
- 在浏览器会话中自动给机器人编号。
- 可控制单台、全部或任意勾选组合。
- 双虚拟摇杆，接近真实遥控器布局：
  - 左摇杆：前进/后退、左右侧移。
  - 右摇杆：左转/右转。
- 安全停止机制：
  - 松开摇杆发送 `StopMove`。
  - 页面失焦或切到后台发送 `StopMove`。
  - 后端在摇杆指令中断后自动停止。
- 每次连接成功后自动开启自由避障。
- 摇杆通道可选择：
  - 默认 `wirelesscontroller` 模拟无线遥控器输入，Go2 避障服务会过滤运动。
  - 可手动切换到 `Sport Move`，用于明确需要绕开避障过滤的直接控制。
- Sport SDK 动作目录：
  - `Normal / legacy`
  - `MCF (firmware >= 1.1.7)`
  - 空翻、倒立/直立、跳跃、卸力等高风险动作需要额外确认。
- Go2 Air 没有音频，本项目已禁用音频通道。

## 快速安装

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip git portaudio19-dev

git clone https://github.com/Uoghluvm/go2_air_wifi_multi_control.git
cd go2_air_wifi_multi_control

python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## 启动

只在本机浏览器访问：

```bash
./run.sh
```

允许局域网内其他设备访问：

```bash
./run.sh --host 0.0.0.0 --port 8090
```

浏览器打开：

```text
http://127.0.0.1:8090
```

如果从手机、平板或另一台电脑访问，把 `127.0.0.1` 换成运行本程序电脑的 IP。

## 使用方法

1. 确认电脑和 Go2 Air 在同一个 Wi-Fi 局域网。
2. 在“设备发现”里使用默认网段 `192.168.12.0/24` 扫描。
3. 如果扫描不到，手动添加 Go2 IP，例如 `192.168.12.189`。
4. 勾选要控制的机器人编号。
5. 点击“连接选中”。
6. 连接成功后页面会显示“自由避障已开启”。
7. 选择摇杆通道：默认“无线遥控器输入（走避障）”；需要绕开避障时手动切到 `Sport Move`。
8. 勾选安全确认后使用双摇杆。
9. 对高风险动作，需要额外勾选高风险确认并通过浏览器弹窗确认。

## 安全说明

本程序会向机器人发送真实运动命令。请只在空旷场地使用，并确保遥控器或急停手段可立即接管。

不要在人、易碎物、台阶、硬质障碍物附近测试空翻、跳跃、倒立、直立或卸力动作。

## 常见问题

### 扫描不到设备

可以手动输入 IP。部分路由器会屏蔽多播，扫描不到不代表不能连接。

### 扫描到候选但连接失败

端口候选只表示目标开放了类似 Go2 信令端口。连接成功后页面才会显示“已验证 Go2”。

### 固件 1.1.15 或更高版本

Go2 新固件可能需要 per-device AES key 才能进行局域网 WebRTC 握手。本项目当前主要面向 `< 1.1.15`。

## 开发检查

```bash
python3 -m py_compile app.py
python3 app.py --help
node --check wifi_web/static/app.js
```
