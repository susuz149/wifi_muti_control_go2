# Go2 Air Wi-Fi 多机控制台安装教程

本文档用于在另一台 Ubuntu 电脑上安装和运行本项目。

## 1. 适用范围

- 机器人：Unitree Go2 Air。
- 连接方式：电脑和 Go2 在同一个 Wi-Fi 局域网，使用 LocalSTA。
- 固件版本：建议 Go2 固件小于 `1.1.15`。
- 操作系统：Ubuntu 20.04/22.04/24.04 均可尝试，推荐 Ubuntu 22.04。
- Python：Python 3.10 或更高版本。

Go2 Air 没有音频，本项目已禁用音频通道，只使用 WebRTC data channel 控制。

## 2. 安装系统依赖

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip git portaudio19-dev
```

说明：

- `python3-venv` 用于创建独立 Python 环境。
- `portaudio19-dev` 是 `unitree_webrtc_connect` 的依赖链中音频包可能需要的系统库；项目运行时不使用音频。

## 3. 获取代码

从 GitHub 克隆：

```bash
git clone <你的 GitHub 仓库地址>
cd go2_air_wifi_multi_control
```

如果是从压缩包复制代码，进入解压后的目录即可。

## 4. 创建 Python 虚拟环境

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

以后每次重新打开终端运行本项目，都需要先进入目录并激活虚拟环境：

```bash
cd go2_air_wifi_multi_control
source .venv/bin/activate
```

## 5. 安装 Python 依赖

```bash
pip install -r requirements.txt
```

当前锁定的主要依赖：

- `unitree_webrtc_connect==2.1.2`
- `Flask>=3.0,<4.0`

如果国内网络下载慢，可以临时使用镜像源：

```bash
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## 6. 连接 Wi-Fi

确保电脑和 Go2 在同一个 Wi-Fi 局域网。

已知你的 Go2 示例地址：

- `192.168.12.14`
- `192.168.12.189`

在新电脑上可以先测试网络：

```bash
ping 192.168.12.189
```

`ping` 不通并不一定代表 WebRTC 不通，但如果完全不在同一网段，控制台也无法连接。

## 7. 启动控制台

只在本机浏览器使用：

```bash
./run.sh
```

让局域网内其他电脑/平板访问：

```bash
./run.sh --host 0.0.0.0 --port 8090
```

浏览器打开：

```text
http://127.0.0.1:8090
```

如果从其他设备访问，把 `127.0.0.1` 换成运行本程序电脑的 IP。

## 8. 页面使用方法

1. 在“设备发现”里保持默认扫描网段 `192.168.12.0/24`，点击“扫描当前 Wi-Fi”。
2. 如果扫描不到，手动输入 Go2 IP，例如 `192.168.12.189`，点击“添加设备”。
3. 在设备列表中勾选要控制的编号。
4. 点击“连接选中”。
5. 勾选“已确认机器人周围安全”，再使用双摇杆控制。
6. 左摇杆控制前进/后退和侧移；右摇杆控制转向。
7. 松开摇杆会自动停止。
8. “全部紧急停止”会对所有已连接设备发送停止。

## 9. Sport SDK 动作

页面下方列出已安装 SDK 暴露的 Sport API：

- `Normal / legacy`
- `MCF (firmware >= 1.1.7)`

如果不确定机器人当前运动模式，先使用 `Normal / legacy` 中的普通动作，例如：

- `BalanceStand`
- `StandUp`
- `StandDown`
- `Hello`
- `Stretch`

空翻、跳跃、倒立、直立、卸力等属于高风险动作，页面会标红，并要求：

1. 勾选普通真机动作确认。
2. 勾选高风险动作确认。
3. 再通过浏览器弹窗确认。

不要在硬地、狭小空间、人或易碎物附近测试高风险动作。

## 10. 常见问题

### 扫描不到设备

可以手动添加 IP。扫描使用 Unitree 多播和 `9991/8081` 信令端口探测，有些网络会屏蔽多播。

### 能扫描到候选，但连接失败

候选只表示端口像 Go2 信令端口，不代表一定是 Go2。连接成功后页面会显示“已验证 Go2”。

### 某台 Go2 IP 连接不上

检查：

- Go2 是否开机。
- 电脑是否在同一 Wi-Fi。
- IP 是否变化。
- Go2 是否正在被其他 App/电脑占用。
- 防火墙是否阻止局域网连接。

### 固件 1.1.15 或更高版本

Go2 新固件可能需要 per-device AES key 才能进行 LAN WebRTC 握手。本项目当前面向 `< 1.1.15` 的旧固件流程。

## 11. 开发验证

```bash
python3 -m py_compile app.py
python3 app.py --help
```

如果安装了 Node.js，也可以检查前端脚本语法：

```bash
node --check wifi_web/static/app.js
```
