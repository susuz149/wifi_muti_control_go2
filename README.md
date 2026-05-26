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

git clone <your-repo-url>
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
- For Go2 firmware `>= 1.1.15`, Unitree LAN signaling may require a per-device AES key. This project is currently scoped for firmware `< 1.1.15`.
