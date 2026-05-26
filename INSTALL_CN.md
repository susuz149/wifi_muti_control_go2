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
git clone https://github.com/Uoghluvm/go2_air_wifi_multi_control.git
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
- `numpy`：解析和降采样 LiDAR voxel 点数据。
- `opencv-python-headless`：把每台 WebRTC 视频帧编码为网页可显示的 JPEG。

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
5. 连接成功后程序会自动开启自由避障；页面状态会显示“自由避障已开启”。
6. 勾选“已确认机器人周围安全”，再使用双摇杆控制。
7. 左摇杆控制前进/后退和侧移；右摇杆控制转向。
8. 松开摇杆会自动停止。
9. “全部紧急停止”会对所有已连接设备发送停止。

说明：默认摇杆控制通过 `rt/wirelesscontroller` 模拟无线遥控器输入发送，Go2 的避障服务会拦截这类指令并做安全过滤。程序连接成功后会调用 `rt/api/obstacles_avoid/request` 开启避障并查询确认。页面也可以手动切换到 `Sport Move`，这会绕开避障过滤，只应在完全安全、需要直接控制时使用。

## 9. 共同地图、视频与闭环列队

连接成功后，页面会按机器人编号和 IP 分开显示 `rt/utlidar/robot_pose` 与 LiDAR 数据，不能把两台机器的话题数据混为同一来源。

1. 在“坐标系对齐与 LiDAR 隔离”中逐台选择编号。
2. 输入该机器当前位置在共同 `map frame` 下的 `x/y/z/yaw`，点击标定。
3. 两台标定完成后，“共同 LiDAR 图”会以不同颜色显示每台扫描点和当前位姿。
4. 在“单机视频”中勾选编号并开启视频；建议先开启一台，确认 Wi-Fi 带宽后再开启第二台。
5. 闭环列队必须在所有参与机器均显示“已对齐 map”且区域空旷时执行。

当前闭环列队使用实时 `map_pose` 反馈修正目标误差，但速度输出使用 `Sport Move` 直接控制通道。它不是带全局避障的自动路径规划器，不应在有人员、窄道或复杂动态障碍的场地直接测试。

## 10. Sport SDK 动作

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

## 11. 常见问题

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

## 12. 开发验证

```bash
python3 -m py_compile app.py
python3 app.py --help
```

如果安装了 Node.js，也可以检查前端脚本语法：

```bash
node --check wifi_web/static/app.js
```
