# Go2 Air Wi-Fi Multi Controller

Browser-based multi-robot controller for Unitree Go2 Air on the same Wi-Fi LAN.

This project uses `unitree_webrtc_connect` over LocalSTA WebRTC data channels. Firmware `>= 1.1.15` can use a per-device AES key entered in the device list; older firmware can leave it empty.

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
- Per-robot telemetry isolation for `rt/utlidar/robot_pose`, LiDAR state, and compressed voxel maps.
- Explicit local-to-`map` alignment per robot, with a color-coded shared LiDAR map.
- Closed-loop formation placement using each robot's aligned `map_pose`.
- Per-robot WebRTC camera display in the browser.
- Point clouds are off after every connection; enable selected robots explicitly
  for maps, alignment, or restoring saved alignment. Heartbeat health is always
  shown in the device list, independently of point cloud streaming.

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
- Closed-loop formation uses direct velocity commands and currently does not constitute a full multi-robot path planner; run it only in a verified clear area.
- `ARCHITECTURE_CN.md` documents the intended split for future shared-map localization, automatic planning, and swappable formation algorithms.
- For Go2 firmware `>= 1.1.15`, configure each robot's own AES key before connecting.

---

# Go2 Air Wi-Fi 多机控制台

这是一个通过浏览器控制多台 Unitree Go2 Air 的局域网 Web 控制台。

项目使用 `unitree_webrtc_connect` 的 LocalSTA WebRTC data channel。旧固件通常不需要密钥；固件 `>= 1.1.15` 可在手动添加设备时填写该设备的 32 位十六进制 AES 密钥，也可在扫描后的设备列表中输入并点击“保存密钥”，再连接。每台设备使用各自的密钥；已连接设备需先断开才能修改。设备编号、IP、SN 和密钥自动保存在本机 `.local_state/fleet.json`，重启后自动加载。文件权限为 `0600`，目录为 `0700`，该路径已加入 Git 忽略；密钥是本机明文保存，页面不返回原文。留空保存会清除页面设置的密钥；此前配置的 `GO2_AES_<IP下划线形式>` 环境变量仍作为后备值。

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
- 按机器人编号/IP 隔离 `rt/utlidar/robot_pose`、LiDAR 状态和压缩 voxel 数据。
- 支持逐台设置 `T_map_local`，将多台扫描内容显示在共同 `map frame` 中。
- 支持基于每台实时 `map_pose` 的闭环列队。
- 横队按参考狗朝向下的实时横向位置分配目标，保留当前左右顺序；页面“间距 m”控制相邻目标中心距离。编号最小者的当前位置作为队形中心、朝向作为基准，两台都可能移动。其余队形仍按编号分配。
- 支持网页按编号打开每台 Go2 的视频。

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

## 目标位置闭环编排

页面新增“目标位置闭环编排”：连接机器人并完成公共 map 坐标标定后，勾选参与设备，点击“读取选中位置生成模板”。生成的模板目标就是当前位置，不会自行开始移动。修改目标或在 `steps` 中追加段落，勾选真机动作确认和列队区域确认后点击“执行闭环编排”。JSON 可导入、导出保存，导入不会执行。

每段的 `targets` 用机器人编号作为键，包含 `x`、`y`（map 坐标，米）和 `yaw`（弧度，范围 -π～π）。每段必须包含相同编号；需要等待的狗也填写其保持位置。`timeout` 是单段最长执行时间，不是要求到达的演出时刻。

```json
{
  "speed": 0.12,
  "yaw_rate": 0.35,
  "acceleration": 0.2,
  "position_tolerance": 0.12,
  "yaw_tolerance": 0.18,
  "settle_time": 0.5,
  "min_separation": 0.8,
  "steps": [
    {"timeout": 25, "targets": {"1": {"x": 0.5, "y": 0, "yaw": 0}}},
    {"timeout": 25, "targets": {"1": {"x": 0.5, "y": 0.5, "yaw": 0}}}
  ]
}
```

上例仅用于说明格式，假设 #1 从场地原点出发；实际使用应从当前位置模板修改。每段最多移动 4 米，速度上限 0.25 m/s。控制器约每 0.1 秒根据位置和朝向误差计算机身速度，限制平移速度与速度变化；全体在容差内持续稳定后进入下一段。页面显示最近的实际位置、目标、误差、到位状态及速度指令；速度指令并非实测速度。

同一时间只允许一项闭环任务；参与狗不能同时接收手动移动或表演命令。原有自动列队也使用同一执行器。点击“停止整组编排”，或向任意参与狗发送停止命令，会取消整个编排并尝试向所有参与狗发送零速度和停止指令。断开参与狗前也会先停止整组。停止未确认会单独显示，不能将断连等同于机器人已物理停止。启动应用、读取模板、导入编排和查看状态均不启动闭环移动。

执行前保守检查各参与狗的直线路段最小距离，拒绝路线交叉或距离不足；执行时检查实际机间距离。位置反馈超过 0.6 秒未更新、坐标标定改变、相邻控制周期定位跳变、任何一段超时或参与狗间距不足，都会终止整组。此检查不规划绕行，也不覆盖非参与设备、场地障碍物或机器人的完整机身轮廓。当前使用 Sport Move 直接通道，不能依赖自由避障过滤。

这是“目标点序列＋全体到位同步”的第一版，尚未实现按音乐时钟连续轨迹跟踪、舞蹈完成反馈或绝对定位纠偏。初始标定后的 map_pose 仍可能漂移，不能仅靠该控制器保证长期场地绝对精度。已有模拟反馈和接口测试，实机位置精度、制动距离和多机稳定性仍需验证。

接口：`POST /api/motion` 接收 `{program, confirm_motion: true, confirm_formation: true}`，快速返回启动状态；`GET /api/motion` 查询进度；`POST /api/motion/stop` 取消整组任务，无需动作确认。关闭浏览器不会自动取消后台任务。

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

设备列表分为“已连接”和“历史设备 / 未连接”。每台设备可保存名称和分组；“只选已连接”用于选择在线设备，“按分组选择”会替换当前勾选，包含该组尚未连接的设备，便于批量连接。

未连接设备可点击“删除历史”，确认后删除该设备本机保存的记录、密钥和标定；已连接或正在连接的设备必须先断开。删除不会屏蔽再次扫描发现，已删除编号不会分配给新设备。

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

Go2 新固件需要 per-device AES key 时，可在手动添加设备或扫描后的设备列表中填写，并保存后连接。密钥不自动获取，需使用绑定账号等受支持的方式自行取得。

### 自动对齐坐标

1. 连接设备，选择参考狗 IP；勾选参与机器及参考狗，点击“开启选中点云”，等待新鲜定位和点云。每次连接后点云均默认关闭。
2. 在同一平地保持机器人静止，让它们看到足够多的共同墙面、柱子等静态环境。
   默认使用任意摆放模式；已知同向横排时，可选择横排辅助布局，按参考狗头朝向填写左右顺序和间距。
3. 点击“自动对齐选中设备”。参考狗在采样时的位置和朝向定义为公共原点和 +X。
4. 检查彩色点云预览、重叠比例和匹配误差。选择设备后可在图上拖动点云，或用按钮平移、旋转；人工确认调整结果。
5. 点击“应用预览”。此操作替换全部旧标定；未通过且未经人工确认的设备保持未标定。预览本身不发送运动指令，也不改变现有坐标。

算法使用各台设备独立的米制 native voxel 点云。开启摆放布局后，按实际机身位置和朝向限制 ICP 搜索，并校验左右顺序及相邻间距；不会将不同机器的 local 原点当作机身位置。关闭布局时使用二维占据相关做全局旋转/平移搜索，再用裁剪 ICP 精调。布局估计不能替代点云验证，匹配失败会保持未通过。仅适用于共同平面，不能估计坡度和完整三维姿态。重叠不足、对称或重复环境可能无法可靠对齐，质量分数不代表真实定位误差保证。没有变换候选的设备需恢复数据后重新生成预览，或使用备用手动标定。

定位超过 1 秒、点云超过 3 秒未更新时禁止应用；预览超过 5 分钟或设备重连后失效。应用后仍依赖各台狗自己的持续定位，未实现运行中的持续重定位与漂移校正。当前列队仍不具备多机路径冲突消解能力。

### 启动直连与恢复标定

首次更新后，连接设备并完成一次有效标定（应用预览或手动标定）。程序会保存各台机器的地图点云；正常退出前也会保存最新可用数据。

下次启动程序会自动连接保存的设备，点云保持关闭；点击“开启选中点云”后，用新点云和保存的地图做重新定位，验证通过后恢复到原公共坐标系。即使机器狗的 local 原点发生变化，也会计算新的坐标转换；不会直接信任旧坐标值。重叠不足、场景改变、匹配有歧义或数据过期时，保持未对齐并显示原因，需要重新对齐。启动、重连和恢复均不发送运动指令，也不会自动执行列队。完成标定后可关闭点云，已验证的坐标转换仍保留；闭环运动继续检查定位反馈的新鲜程度。

“启动自动连接；断线后最多重试 3 次”可关闭；`./run.sh --no-auto-connect` 仅跳过本次启动自动连接。程序正常退出保留下次直连偏好，页面主动断开的设备不再参与下次自动连接，手动连接成功后重新启用。启动自动连接只尝试一次；已建立的连接意外中断时，启用自动连接的设备在清理旧连接后按 2、4、8 秒间隔最多重试 3 次。主动断开、删除历史或退出程序会取消等待中的重连。离线设备也可手动连接；机器狗 IP 变化时需要更新地址。浏览器刷新只读取状态，不重复创建连接。

### 在哪里查看连接异常与超时

“机器人编号与选择”中的每台设备显示心跳响应距今时间、最近收包时间、WebRTC/ICE/数据通道状态，以及后端事件循环调度延迟。即使点云关闭，也会继续检查驱动每 2 秒发送的心跳：响应超过 6 秒未更新提示异常，超过 12 秒判定超时；底层连接失败或通道关闭也会被检测。设备列表约每秒刷新，清理旧连接后保留错误原因，可点击“连接选中”重新建立连接。网络恢复不会自动恢复之前的运动。

定位反馈与点云新鲜程度是另外两项检查：定位过期仍在 LiDAR 状态区域显示并阻止依赖定位的移动；点云只有开启时才检查更新时间，关闭显示“点云未开启”，不作为连接超时依据。点云开关控制上传订阅，不关闭机器狗的雷达硬件或避障功能。

项目的 `fleet/transport.py` 为固定版本驱动提供兼容修复：局域网信令 HTTP 在后台线程执行并设置连接/读取超时，voxel 解码在后台线程执行、最多每秒 5 次且只保留一份最新待处理包。应用的视频 JPEG 编码在后台进行、每路最多每秒 5 帧，并限制并发编码数量。启用多台点云或视频仍会占用共享 Wi-Fi 带宽；实际效果需实机验证。

点云接口：`POST /api/point-cloud`，请求为 `{"targets": [1, 2], "enabled": true}`；关闭使用 `false`。连接健康随 `GET /api/devices` 的 `connection_health` 返回，无需轮询地图接口。

## 开发检查

```bash
python3 -m py_compile app.py
python3 app.py --help
node --check wifi_web/static/app.js
python3 -m unittest discover -s tests -v
node tests/test_alignment_ui.js
```
