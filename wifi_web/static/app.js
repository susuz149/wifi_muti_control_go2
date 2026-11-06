const state = { devices: [], selected: new Set(), profiles: {}, telemetry: [], mapPoints: [], videos: {}, controlBusy: false, joystickPromise: null, joystickTargets: [], joystickInFlight: false };
const sticks = {
  left: { x: 0, y: 0, active: false },
  right: { x: 0, y: 0, active: false },
};
let stickTimer = null;
const aesKeyDrafts = new Map();
let alignmentPreview = null;
let alignmentView = null;
let alignmentDrag = null;
let alignmentRequest = 0;

function previewPose(pose, transform) {
  const c = Math.cos(transform.yaw), s = Math.sin(transform.yaw);
  return { x: pose.x*c-pose.y*s+transform.x, y: pose.x*s+pose.y*c+transform.y,
    z: (pose.z || 0) + (transform.z || 0), yaw: (pose.yaw || 0)+transform.yaw };
}

function previewScene() {
  const results = (alignmentPreview?.results || []).filter((r) => r.transform);
  return {
    points: results.flatMap((r) => r.points.map(([x,y,z]) => ({ number:r.number, ...previewPose({x,y,z},r.transform) }))),
    robots: results.map((r) => ({ number:r.number, map_pose:previewPose(r.local_pose,r.transform) })),
  };
}

function renderAlignmentResults() {
  const container = byId("alignmentResults");
  container.replaceChildren();
  const previous = byId("adjustRobot").value;
  byId("adjustRobot").replaceChildren();
  (alignmentPreview?.results || []).forEach((r) => {
    const line = document.createElement("div");
    line.className = `alignment-result ${r.accepted || r.manualConfirmed ? "accepted" : ""}`;
    const pose = r.transform ? previewPose(r.local_pose, r.transform) : null;
    const slot = r.bounds ? (r.bounds.rank === 0 ? "中心参考" : `${r.bounds.rank > 0 ? "左" : "右"}${Math.abs(r.bounds.rank)}`) : "";
    line.textContent = `[${r.accepted ? "通过" : r.manualConfirmed ? "人工确认" : "未通过"}] #${r.number} ${r.ip}${slot ? ` [${slot}]` : ""} · ${r.manualConfirmed ? "已人工确认" : r.reason}${r.overlap !== undefined ? ` · 重叠 ${(r.overlap*100).toFixed(0)}% / 匹配误差 ${r.rmse}m` : ""}${pose ? ` · 位置(${pose.x.toFixed(2)}, ${pose.y.toFixed(2)})m / 朝向 ${(pose.yaw*180/Math.PI).toFixed(1)}°` : ""}`;
    container.appendChild(line);
    if (r.transform && r.number !== alignmentPreview.reference) {
      const option = document.createElement("option");
      option.value = r.number;
      option.textContent = `#${r.number} ${r.ip}`;
      byId("adjustRobot").appendChild(option);
    }
  });
  if ([...byId("adjustRobot").options].some((o) => o.value === previous)) byId("adjustRobot").value = previous;
  byId("alignmentEditor").hidden = !alignmentPreview?.done || !byId("adjustRobot").options.length;
  byId("applyAutoAlignButton").disabled = !alignmentPreview?.done || !!alignmentPreview?.error ||
    !(alignmentPreview?.results || []).some((r) => r.number !== alignmentPreview.reference && (r.accepted || r.manualConfirmed));
}

function alignmentOptions() {
  const mode = byId("alignmentMode").value;
  if (!["free", "row"].includes(mode)) throw new Error("未知标定模式");
  const options = {alignment_mode:mode, layout_enabled:mode === "row"};
  if (mode === "row") Object.assign(options, {
    left_ips:byId("layoutLeftIps").value, right_ips:byId("layoutRightIps").value,
    min_spacing:byId("layoutMinSpacing").value, max_spacing:byId("layoutMaxSpacing").value,
  });
  return options;
}

function updateAlignmentMode() {
  const row = byId("alignmentMode").value === "row";
  byId("rowLayoutFields").hidden = !row;
  byId("rowLayoutFields").querySelectorAll("input").forEach((input) => { input.disabled = !row; });
  byId("alignmentModeNote").textContent = row
    ? "横排辅助：按参考狗头朝向填写左右顺序。约束前后 ±0.4m、同向 ±20°，实际位置仍须点云验证。任意阵型请选择任意摆放模式。"
    : "任意摆放：仅选择参与机器和参考狗，无需填写左右顺序、间距或朝向。参考狗自动加入；各台分别匹配共同环境；不会驱动机器人。";
}

function updateReferenceCandidates() {
  const list = byId("referenceCandidates");
  list.replaceChildren();
  const connected = state.devices.filter((d) => d.connected);
  connected.forEach((d) => {
    const option = document.createElement("option"); option.value = d.ip; option.label = `#${d.number}`; list.appendChild(option);
  });
  if (!byId("referenceIp").value && connected.length) {
    byId("referenceIp").value = (connected.find((d) => state.selected.has(d.number)) || connected[0]).ip;
  }
}

async function autoAlign() {
  const requestId = ++alignmentRequest;
  byId("autoAlignButton").disabled = true;
  alignmentPreview = null;
  alignmentView = null;
  renderAlignmentResults();
  setMessage(byId("frameMessage"), "正在匹配点云，请保持机器人静止；预览不会改变现有坐标。");
  try {
    if (!(await stopJoystick())) throw new Error("摇杆停止未确认，请检查后再标定");
    const start = await api("/api/frame/auto-align", {method:"POST", body:JSON.stringify({
      targets:selectedNumbers(), reference_ip:byId("referenceIp").value,
      ...alignmentOptions(),
    })});
    while (requestId === alignmentRequest) {
      const result = await api(`/api/frame/auto-align/${start.token}`);
      if (requestId !== alignmentRequest) return;
      alignmentPreview = result;
      renderAlignmentResults();
      drawSharedMap();
      if (result.done) {
        if (result.error) throw new Error(result.error);
        const others = result.results.filter((r) => r.number !== result.reference);
        const passed = others.filter((r) => r.accepted).length;
        setMessage(byId("frameMessage"), `${result.mode === "free" ? "任意摆放" : "横排辅助"}预览完成：${passed}/${others.length} 台非参考机器通过。请核对 X、Y、朝向及墙面重合；失败候选不会自动应用。`, passed === 0);
        break;
      }
      await new Promise((resolve) => setTimeout(resolve, 800));
    }
  } catch (error) {
    if (requestId === alignmentRequest) setMessage(byId("frameMessage"), error.message, true);
  } finally {
    if (requestId === alignmentRequest) byId("autoAlignButton").disabled = false;
  }
}

function adjustPreview(dx, dy, angle=0) {
  const r = alignmentPreview?.results.find((item) => item.number === Number(byId("adjustRobot").value));
  if (!r?.transform) return;
  // Rotate about this robot's current preview position, rather than world origin.
  const pose = previewPose(r.local_pose, r.transform);
  r.transform.yaw += angle;
  const rotated = previewPose(r.local_pose, r.transform);
  r.transform.x += pose.x-rotated.x+dx;
  r.transform.y += pose.y-rotated.y+dy;
  r.adjusted = true;
  r.accepted = false;
  r.manualConfirmed = false;
  r.reason = "已微调，请确认该设备的预览";
  renderAlignmentResults();
  drawSharedMap();
}

async function applyAutoAlign() {
  if (!alignmentPreview?.done) return;
  if (alignmentPreview.results.some((r) => r.adjusted && !r.manualConfirmed)) {
    setMessage(byId("frameMessage"), "有微调尚未确认，请先确认该设备的预览。", true);
    return;
  }
  const adjustments = Object.fromEntries(alignmentPreview.results.filter((r) => r.manualConfirmed).map((r) => [r.number,r.transform]));
  try {
    const result = await api("/api/frame/auto-align/apply", {method:"POST", body:JSON.stringify({token:alignmentPreview.token,adjustments})});
    alignmentPreview = null;
    alignmentView = null;
    renderAlignmentResults();
    state.telemetry = result.robots;
    await refreshSharedMap();
    setMessage(byId("frameMessage"), `已对齐：${result.aligned.map((n) => `#${n}`).join("、")}。其余设备保持未标定，请检查位置后再列队。`);
  } catch (error) { setMessage(byId("frameMessage"), error.message, true); }
}

function byId(id) { return document.getElementById(id); }

function renderMotion(motion) {
  const labels = {idle:"未执行", running:"执行中", completed:"已完成", cancelled:"已停止", failed:"执行失败"};
  setMessage(byId("motionStatus"), `${labels[motion.status] || motion.status} · 第 ${motion.step}/${motion.total_steps || 0} 段${motion.error ? ` · ${motion.error}` : ""}${motion.stop_error ? ` · ${motion.stop_error}` : ""}`,
    motion.status === "failed" || !!motion.stop_error);
  byId("motionStart").disabled = motion.status === "running";
  const body = byId("motionRobots");
  body.replaceChildren();
  const fmt = (v) => Number(v).toFixed(2);
  Object.entries(motion.robots || {}).forEach(([number, robot]) => {
    const row = document.createElement("tr");
    const pose = (p) => `(${fmt(p.x)}, ${fmt(p.y)}) / ${fmt(p.yaw)} rad`;
    [`#${number}`, pose(robot.pose), pose(robot.target),
      `${fmt(robot.distance)} m / ${fmt(robot.yaw_error)} rad`,
      `${fmt(robot.command.vx)}, ${fmt(robot.command.vy)} m/s / ${fmt(robot.command.wz)} rad/s`,
      robot.arrived ? "是" : "否"].forEach((text) => {
        const cell = document.createElement("td"); cell.textContent = text; row.appendChild(cell);
      });
    body.appendChild(row);
  });
}

async function refreshMotion() {
  const result = await api("/api/motion");
  renderMotion(result.motion);
}

async function captureMotion() {
  try {
    const numbers = selectedNumbers();
    if (!numbers.length) throw new Error("请先选择机器人");
    const telemetry = await api("/api/telemetry");
    const targets = {};
    numbers.forEach((n) => {
      const robot = telemetry.robots.find((r) => r.number === n);
      if (!robot?.connected || !robot.aligned || !robot.map_pose || robot.last_pose_age === null || robot.last_pose_age > .6)
        throw new Error(`#${n} 没有新鲜且已标定的位置`);
      targets[n] = {x:robot.map_pose.x, y:robot.map_pose.y, yaw:robot.map_pose.yaw};
    });
    byId("motionProgram").value = JSON.stringify({speed:.12, yaw_rate:.35, acceleration:.2,
      position_tolerance:.12, yaw_tolerance:.18, settle_time:.5, min_separation:.8,
      steps:[{timeout:25, targets}]}, null, 2);
    setMessage(byId("motionMessage"), "已生成当前位置模板。修改目标或复制 steps 中的段落后执行；模板本身仅保持当前位置。");
  } catch (error) { setMessage(byId("motionMessage"), error.message, true); }
}

async function startMotion() {
  try {
    const program = JSON.parse(byId("motionProgram").value);
    if (!(await stopJoystick())) throw new Error("停止未确认，未启动编排");
    const result = await api("/api/motion", {method:"POST", body:JSON.stringify({program,
      confirm_motion:byId("confirmMotion").checked, confirm_formation:byId("confirmFormation").checked})});
    renderMotion(result.motion);
    setMessage(byId("motionMessage"), "闭环编排已启动。");
  } catch (error) { setMessage(byId("motionMessage"), error.message, true); }
}

async function api(path, options = {}) {
  const controller = new AbortController();
  const readOnly = !options.method || options.method === 'GET';
  const timer = setTimeout(() => controller.abort(), readOnly ? 5000 : 75000);
  try {
    const response = await fetch(path, {
      headers: { "Content-Type": "application/json" },
      ...options,
      signal: controller.signal,
    });
    let result;
    try { result = await response.json(); }
    catch (_error) { throw new Error(`服务器响应异常（HTTP ${response.status}），请检查服务是否正常运行。`); }
    if (!response.ok || !result.ok) throw new Error(result.error || `请求失败（HTTP ${response.status}）`);
    return result;
  } catch (error) {
    if (controller.signal.aborted) throw new Error(readOnly
      ? '状态读取超时，请检查页面到服务的网络连接。'
      : '请求超时，操作结果尚未确认，请刷新设备状态后再操作。');
    if (error instanceof TypeError) throw new Error('无法连接后台服务，请检查网络和服务是否已启动。');
    throw error;
  } finally { clearTimeout(timer); }
}

function setMessage(element, text, failed = false) {
  element.textContent = text;
  element.style.color = failed ? "#b63326" : "";
}

function selectedNumbers() {
  return Array.from(state.selected).sort((a, b) => a - b);
}

function renderDevices() {
  updateReferenceCandidates();
  const container = byId("deviceList");
  if (container.contains(document.activeElement) && document.activeElement.type === "password") return;
  const connectedList = byId("connectedDeviceList");
  const historyList = byId("historyDeviceList");
  const connectedCount = state.devices.filter((device) => device.connected).length;
  byId("connectedDeviceCount").textContent = String(connectedCount);
  byId("historyDeviceCount").textContent = String(state.devices.length - connectedCount);
  for (const [list, count, emptyText] of [
    [connectedList, connectedCount, '暂无已连接设备'],
    [historyList, state.devices.length - connectedCount, '暂无历史或待连接设备'],
  ]) {
    list.replaceChildren();
    list.className = count ? 'device-list' : 'device-list empty';
    if (!count) list.textContent = emptyText;
  }
  state.devices.forEach((device) => {
    const row = document.createElement("div");
    row.className = `device ${device.connected ? "connected" : ""}`;
    const health = device.connection_health || {};
    const unhealthy = ["warning", "failed", "timeout"].includes(health.status);
    const status = device.busy ? "处理中" : unhealthy ? (health.status === "warning" ? "连接异常" : "已断线") : device.connected ? "已连接" : "未连接";
    const identity = device.sn
      ? `SN ${device.sn}`
      : device.verified
        ? `已验证 Go2${device.avoid_enabled ? " / 自由避障已开启" : " / 自由避障未确认"}`
        : device.source === "probe"
          ? "信令端口候选，待连接验证"
          : device.source === "manual"
            ? "手动添加，待连接验证"
            : "多播发现";
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.dataset.number = String(device.number);
    checkbox.setAttribute("aria-label", `选择 Go2 #${device.number}`);
    checkbox.checked = state.selected.has(device.number);
    checkbox.addEventListener("change", () => {
      checkbox.checked ? state.selected.add(device.number) : state.selected.delete(device.number);
      drawFormationPreview();
    });
    const number = document.createElement("span");
    number.className = "number";
    number.textContent = `#${device.number}`;
    const information = document.createElement("span");
    const address = document.createElement("span");
    address.className = "address";
    address.textContent = device.ip;
    const detail = document.createElement("div");
    detail.className = "detail";
    detail.textContent = `${identity}${device.error ? ` / ${device.error}` : ""}`;
    information.append(address, detail);
    const connectionDetail = document.createElement("div");
    connectionDetail.className = "detail";
    const seconds = (value) => value == null ? "等待响应" : `${value}s 前`;
    connectionDetail.textContent = `${health.message || status} · 心跳 ${seconds(health.heartbeat_age)} · 收包 ${seconds(health.receive_age)} · 点云${device.cloud_enabled ? "开启" : "关闭"}`;
    if (device.connected || unhealthy) {
      connectionDetail.textContent += ` · WebRTC=${health.peer_state || "?"} / ICE=${health.ice_state || "?"} / 通道=${health.channel_state || "?"} · 后端调度延迟=${health.loop_lag_ms ?? "?"}ms`;
    }
    if (health.decode_error) connectionDetail.textContent += ` · 点云解码异常：${health.decode_error}`;
    if (unhealthy) connectionDetail.style.color = "#b63326";
    information.appendChild(connectionDetail);
    const statusPill = document.createElement("span");
    statusPill.className = "status";
    statusPill.textContent = status;
    if (unhealthy) statusPill.style.color = "#b63326";
    row.append(checkbox, number, information, statusPill);
    const keySettings = document.createElement("div");
    keySettings.className = "device-key";
    const keyInput = document.createElement("input");
    keyInput.type = "password";
    keyInput.autocomplete = "off";
    keyInput.placeholder = device.has_aes_key ? "密钥已配置；输入新密钥可替换" : "AES 密钥（旧固件可留空）";
    keyInput.setAttribute("aria-label", `Go2 #${device.number} AES 密钥`);
    keyInput.value = aesKeyDrafts.get(device.number) || "";
    keyInput.addEventListener("input", () => aesKeyDrafts.set(device.number, keyInput.value));
    const saveKey = document.createElement("button");
    saveKey.textContent = "保存密钥";
    saveKey.disabled = keyInput.disabled = device.connected || device.busy;
    saveKey.addEventListener("click", async () => {
      saveKey.disabled = true;
      try {
        const result = await api(`/api/devices/${device.number}/aes-key`, {
          method: "POST",
          body: JSON.stringify({ aes_128_key: keyInput.value.trim() }),
        });
        aesKeyDrafts.delete(device.number);
        keyInput.value = "";
        state.devices = result.devices;
        saveKey.blur();
        renderDevices();
        setMessage(byId("scanMessage"), `Go2 #${device.number} 密钥已${result.device.has_aes_key ? "配置" : "清除"}，可点击连接选中。`);
      } catch (error) {
        setMessage(byId("scanMessage"), error.message, true);
      } finally {
        saveKey.disabled = false;
      }
    });
    keySettings.append(keyInput, saveKey);
    row.appendChild(keySettings);
    (device.connected ? connectedList : historyList).appendChild(row);
  });
  byId("connectedCount").textContent = state.devices.filter((item) => item.connected).length;
}

async function refresh() {
  let result;
  try { result = await api("/api/devices"); }
  catch (error) {
    state.backendUnavailable = true;
    byId("connectedCount").textContent = "未知";
    setMessage(byId("scanMessage"), `${error.message} 当前设备状态可能已过期。`, true);
    throw error;
  }
  if (state.backendUnavailable) setMessage(byId("scanMessage"), '后台连接已恢复，设备状态已更新。');
  state.backendUnavailable = false;
  state.devices = result.devices;
  renderDevices();
  if (result.storage_error) setMessage(byId("persistenceMessage"), result.storage_error, true);
}

async function loadSavedSettings() {
  const result = await api('/api/settings');
  byId('startupAutoConnect').checked = result.auto_connect_enabled !== false;
  if (result.storage_error) setMessage(byId('persistenceMessage'), result.storage_error, true);
}

function poseText(pose) {
  if (!pose) return "无";
  return `x=${Number(pose.x).toFixed(2)}, y=${Number(pose.y).toFixed(2)}, yaw=${Number(pose.yaw).toFixed(2)}`;
}

function renderTelemetry() {
  const container = byId("frameList");
  if (!state.telemetry.length) {
    container.className = "frame-list empty";
    container.textContent = "连接机器人后显示各自 local/map pose 和 LiDAR 统计。";
    return;
  }
  container.className = "frame-list";
  container.innerHTML = "";
  state.telemetry.forEach((robot) => {
    const card = document.createElement("article");
    card.className = `frame-card ${robot.aligned ? "aligned" : ""}`;
    const title = document.createElement("div");
    title.className = "frame-title";
    title.innerHTML = `<strong>#${robot.number}</strong><span>${robot.ip}</span><b>${robot.aligned ? "已对齐 map" : "未标定"}</b>`;
    const local = document.createElement("p");
    local.textContent = `local(${robot.pose_topic || "rt/utlidar/robot_pose"}): ${poseText(robot.local_pose)}`;
    const map = document.createElement("p");
    map.textContent = `map: ${poseText(robot.map_pose)} · ${robot.restore_status || ""}`;
    const lidar = robot.lidar || {};
    const voxel = lidar.voxel || {};
    const stateData = lidar.state || {};
    const lidarLine = document.createElement("p");
    lidarLine.textContent = robot.cloud_enabled
      ? `点云已开启 · LiDAR: points=${voxel.point_count || 0}, cloud_hz=${stateData.cloud_frequency ?? "?"}, dirty=${stateData.dirty_percentage ?? "?"}`
      : "点云未开启 · 不接收或处理点云；连接健康请查看设备列表";
    const age = document.createElement("p");
    const fresh = robot.connected && robot.last_pose_age !== null && robot.last_pose_age <= 1;
    const cloudFresh = robot.cloud_age != null && robot.cloud_age <= 3;
    const cloudStatus = !robot.cloud_enabled ? "点云未开启" : robot.cloud_age == null ? "等待点云" : `${cloudFresh ? "点云正常" : "点云超时"}（${robot.cloud_age}s 前）`;
    age.textContent = `${fresh ? "定位正常" : "定位过期或未连接"} · pose age=${robot.last_pose_age ?? "?"}s · ${cloudStatus} · seq=${robot.sequence}${robot.error ? `, error=${robot.error}` : ""}`;
    if (!fresh || (robot.cloud_enabled && !cloudFresh)) age.style.color = "#b63326";
    card.append(title, local, map, lidarLine, age);
    container.appendChild(card);
  });
}

function drawSharedMap() {
  const canvas = byId("sharedMapCanvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#17221f";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const preview = alignmentPreview ? previewScene() : null;
  const robots = preview ? preview.robots : state.telemetry.filter((robot) => robot.map_pose);
  const points = preview ? preview.points : state.mapPoints || [];
  const all = [
    ...points.map((point) => ({ x: point.x, y: point.y })),
    ...robots.map((robot) => ({ x: robot.map_pose.x, y: robot.map_pose.y })),
  ];
  if (!all.length) {
    ctx.fillStyle = "#91a09a";
    ctx.font = "18px sans-serif";
    ctx.fillText("标定每台机器到 map frame 后，这里显示共同 LiDAR 扫描内容", 24, 44);
    return;
  }
  const minX = Math.min(...all.map((item) => item.x));
  const maxX = Math.max(...all.map((item) => item.x));
  const minY = Math.min(...all.map((item) => item.y));
  const maxY = Math.max(...all.map((item) => item.y));
  const extent = Math.max(maxX - minX, maxY - minY, 2) * 1.2;
  let centerX = (minX + maxX) / 2;
  let centerY = (minY + maxY) / 2;
  let scale = Math.min(canvas.width, canvas.height) / extent;
  if (alignmentPreview?.done) {
    if (!alignmentView) alignmentView = {centerX, centerY, scale};
    ({centerX, centerY, scale} = alignmentView);
  }
  const toScreen = (x, y) => [
    canvas.width / 2 - (y - centerY) * scale,
    canvas.height / 2 - (x - centerX) * scale,
  ];
  ctx.strokeStyle = "#354542";
  ctx.lineWidth = 1;
  for (let x = 50; x < canvas.width; x += 50) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke(); }
  for (let y = 50; y < canvas.height; y += 50) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke(); }
  const colors = ["#2fe49d", "#ffd15c", "#62a8ff", "#ff7b92", "#d384ff", "#7be0e0", "#ffae70"];
  points.forEach((point) => {
    const [x, y] = toScreen(point.x, point.y);
    ctx.fillStyle = colors[(Number(point.number) - 1) % colors.length];
    ctx.globalAlpha = 0.45;
    ctx.fillRect(x, y, 2, 2);
  });
  ctx.globalAlpha = 1;
  ctx.fillStyle = "#fffefa";
  ctx.font = "14px sans-serif";
  ctx.fillText(`${alignmentPreview ? "对齐预览（尚未应用）" : "当前坐标"} · ↑ +X / ← +Y`, 16, 22);
  robots.forEach((robot) => {
    const [x, y] = toScreen(robot.map_pose.x, robot.map_pose.y);
    ctx.fillStyle = colors[(Number(robot.number) - 1) % colors.length];
    ctx.beginPath();
    ctx.arc(x, y, 10, 0, Math.PI * 2);
    ctx.fill();
    if (alignmentPreview && robot.number === Number(byId("adjustRobot").value)) {
      ctx.strokeStyle = "#fffefa";
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.lineWidth = 1;
    }
    const yaw = robot.map_pose.yaw;
    const [frontX, frontY] = toScreen(robot.map_pose.x + Math.cos(yaw) * 0.35, robot.map_pose.y + Math.sin(yaw) * 0.35);
    ctx.strokeStyle = "#fffefa";
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(frontX, frontY);
    ctx.stroke();
    ctx.fillStyle = "#fffefa";
    ctx.font = "bold 13px sans-serif";
    ctx.fillText(`#${robot.number}`, x + 12, y - 10);
  });
}

async function refreshSharedMap() {
  try {
    const result = await api("/api/map");
    state.telemetry = result.robots;
    state.mapPoints = result.points;
    renderTelemetry();
    drawSharedMap();
    drawFormationPreview();
  } catch (_error) {
    drawSharedMap();
  }
}

async function refreshTelemetry(quiet = true) {
  try {
    const result = await api("/api/telemetry");
    state.telemetry = result.robots;
    renderTelemetry();
    drawSharedMap();
    drawFormationPreview();
  } catch (error) {
    if (!quiet) setMessage(byId("frameMessage"), error.message, true);
  }
}

async function setPointCloud(enabled) {
  const buttons = [byId("pointCloudOnButton"), byId("pointCloudOffButton")];
  buttons.forEach((button) => { button.disabled = true; });
  try {
    const targets = selectedNumbers();
    if (!targets.length) throw new Error("请先选择设备；对齐时也需选择参考狗开启点云");
    const result = await api("/api/point-cloud", {method:"POST", body:JSON.stringify({targets, enabled})});
    state.devices = result.devices;
    state.telemetry = result.robots;
    const failures = result.results.filter((item) => !item.ok);
    renderDevices();
    renderTelemetry();
    await refreshSharedMap();
    setMessage(byId("pointCloudMessage"), failures.length
      ? failures.map((item) => `#${item.number}: ${item.error}`).join("; ")
      : enabled ? "已开启选中点云，等待新鲜数据；如有已保存标定，将尝试恢复。" : "已关闭选中点云；心跳和连接异常仍在设备列表显示。", failures.length > 0);
  } catch (error) { setMessage(byId("pointCloudMessage"), error.message, true); }
  finally { buttons.forEach((button) => { button.disabled = false; }); }
}

async function alignFrame() {
  try {
    const result = await api("/api/frame/align", {
      method: "POST",
      body: JSON.stringify({
        number: byId("alignNumber").value,
        x: byId("alignX").value,
        y: byId("alignY").value,
        z: byId("alignZ").value,
        yaw: byId("alignYaw").value,
      }),
    });
    state.telemetry = result.robots;
    renderTelemetry();
    drawSharedMap();
    setMessage(byId("frameMessage"), `#${result.frame.number} 已标定到 map frame。`);
  } catch (error) {
    setMessage(byId("frameMessage"), error.message, true);
  }
}

async function clearAlignment() {
  try {
    const result = await api("/api/frame/clear", {
      method: "POST",
      body: JSON.stringify({ targets: selectedNumbers() }),
    });
    state.telemetry = result.robots;
    renderTelemetry();
    drawSharedMap();
    setMessage(byId("frameMessage"), "已清除选中机器人的 map 对齐。");
  } catch (error) {
    setMessage(byId("frameMessage"), error.message, true);
  }
}

async function scan() {
  setMessage(byId("scanMessage"), "正在扫描局域网中的 Go2...");
  try {
    const result = await api("/api/scan", {
      method: "POST",
      body: JSON.stringify({
        auto_connect: byId("autoConnect").checked,
        subnet: byId("scanSubnet").value,
      }),
    });
    state.devices = result.devices;
    result.devices.forEach((device) => state.selected.add(device.number));
    renderDevices();
    setMessage(byId("scanMessage"), `扫描完成，当前已登记 ${result.devices.length} 台设备。`);
  } catch (error) {
    setMessage(byId("scanMessage"), error.message, true);
  }
}

async function addManual() {
  try {
    const result = await api("/api/devices", {
      method: "POST",
      body: JSON.stringify({ ip: byId("manualIp").value, aes_128_key: byId("manualAesKey").value.trim() }),
    });
    state.devices = result.devices;
    state.selected.add(result.device.number);
    byId("manualIp").value = "";
    byId("manualAesKey").value = "";
    renderDevices();
    setMessage(byId("scanMessage"), `已登记 Go2 #${result.device.number}。`);
  } catch (error) {
    setMessage(byId("scanMessage"), error.message, true);
  }
}

let connectionBusy = false;
async function connectionAction(path) {
  if (connectionBusy) return;
  connectionBusy = true;
  byId("connectButton").disabled = byId("disconnectButton").disabled = true;
  setMessage(byId("scanMessage"), path === '/api/connect' ? '正在连接，请稍候…' : '正在停止并断开，请稍候…');
  try {
    const result = await api(path, {
      method: "POST",
      body: JSON.stringify({ targets: selectedNumbers() }),
    });
    state.devices = result.devices;
    renderDevices();
    const failures = result.results.filter((item) => !item.ok);
    setMessage(byId("scanMessage"), failures.length ? failures.map((item) => item.error).join("; ") : "连接状态已更新。", failures.length > 0);
  } catch (error) {
    setMessage(byId("scanMessage"), error.message, true);
  } finally {
    connectionBusy = false;
    byId("connectButton").disabled = byId("disconnectButton").disabled = false;
  }
}

async function sendCommand(command, extra = {}, all = false, quiet = false) {
  const targets = all
    ? state.devices.filter((device) => device.connected).map((device) => device.number)
    : selectedNumbers();
  try {
    const result = await api("/api/command", {
      method: "POST",
      body: JSON.stringify({
        targets,
        command,
        confirm_motion: byId("confirmMotion").checked,
        confirm_high_risk: byId("confirmHighRisk").checked,
        speed: byId("speed").value,
        yaw_rate: byId("yawRate").value,
        duration: byId("duration").value,
        joystick_mode: byId("joystickMode").value,
        hold_seconds: Number(byId("actionHoldSeconds").value),
        ...extra,
      }),
    });
    state.devices = result.devices;
    renderDevices();
    const failures = result.results.filter((item) => !item.ok);
    if (!quiet || failures.length) {
      setMessage(byId("controlMessage"), failures.length ? failures.map((item) => `#${item.number}: ${item.error}`).join("; ") : `命令 ${command} 已发送到 ${targets.join(", ")}。`, failures.length > 0);
    }
    return failures.length === 0;
  } catch (error) {
    setMessage(byId("controlMessage"), error.message, true);
    return false;
  }
}

function formationOffsets(numbers, pattern, spacing) {
  const center = (numbers.length - 1) / 2;
  const offsets = {};
  if (pattern === "line") {
    const robots = new Map(state.telemetry.map((robot) => [robot.number, robot]));
    if (numbers.some((n) => {
      const robot = robots.get(n);
      return !robot?.connected || !robot.aligned || !robot.map_pose ||
        robot.last_pose_age == null || robot.last_pose_age > .6;
    })) return null;
    const yaw = robots.get(numbers[0]).map_pose.yaw;
    const lateral = (n) => {
      const pose = robots.get(n).map_pose;
      return -Math.sin(yaw) * pose.x + Math.cos(yaw) * pose.y;
    };
    const ordered = [...numbers].sort((a, b) => lateral(a) - lateral(b) || a - b);
    ordered.forEach((number, index) => { offsets[number] = { x: 0, y: (index - center) * spacing }; });
  } else if (pattern === "column") {
    numbers.forEach((number, index) => { offsets[number] = { x: -index * spacing, y: 0 }; });
  } else if (pattern === "wedge") {
    if (numbers.length) offsets[numbers[0]] = { x: 0, y: 0 };
    numbers.slice(1).forEach((number, index) => {
      const order = index + 1;
      const side = order % 2 ? -1 : 1;
      const row = Math.ceil(order / 2);
      offsets[number] = { x: -row * spacing, y: side * row * spacing };
    });
  } else {
    const columns = Math.max(1, Math.ceil(Math.sqrt(numbers.length)));
    numbers.forEach((number, index) => {
      const row = Math.floor(index / columns);
      const column = index % columns;
      const rowWidth = Math.min(columns, numbers.length - row * columns);
      offsets[number] = { x: -row * spacing, y: (column - (rowWidth - 1) / 2) * spacing };
    });
  }
  return offsets;
}

function drawFormationPreview() {
  const canvas = byId("formationCanvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#17221f";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const numbers = selectedNumbers();
  if (!numbers.length) {
    ctx.fillStyle = "#91a09a";
    ctx.font = "16px sans-serif";
    ctx.fillText("选择至少两台机器人后预览队形", 24, 42);
    return;
  }
  const spacing = Number(byId("formationSpacing").value || 0.8);
  const offsets = formationOffsets(numbers, byId("formationPattern").value, spacing);
  if (!offsets) {
    ctx.fillStyle = "#91a09a";
    ctx.font = "16px sans-serif";
    ctx.fillText("横队预览需要全部选中设备已标定且定位正常", 24, 42);
    return;
  }
  const points = Object.values(offsets);
  const xs = points.map((point) => point.x);
  const ys = points.map((point) => point.y);
  const minX = Math.min(...xs, -1);
  const maxX = Math.max(...xs, 1);
  const minY = Math.min(...ys, -1);
  const maxY = Math.max(...ys, 1);
  const extent = Math.max(maxX - minX, maxY - minY, 1.5) * 1.35;
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  const scale = Math.min(canvas.width, canvas.height) / extent;
  const toScreen = (point) => [
    canvas.width / 2 - (point.y - centerY) * scale,
    canvas.height / 2 - (point.x - centerX) * scale,
  ];
  ctx.strokeStyle = "#354542";
  ctx.lineWidth = 1;
  for (let x = 40; x < canvas.width; x += 40) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke(); }
  for (let y = 40; y < canvas.height; y += 40) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke(); }
  numbers.forEach((number) => {
    const point = offsets[number];
    const [x, y] = toScreen(point);
    ctx.fillStyle = number === numbers[0] ? "#2fe49d" : "#ffd15c";
    ctx.beginPath();
    ctx.arc(x, y, 16, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#17221f";
    ctx.font = "bold 13px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(`#${number}`, x, y);
  });
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.fillStyle = "#b6c5bf";
  ctx.font = "12px sans-serif";
  ctx.fillText(`上方为前进方向，左侧为机身左侧；#${numbers[0]} 当前位姿为基准`, 14, canvas.height - 14);
}

async function runFormation() {
  const targets = selectedNumbers();
  if (targets.length < 2) {
    setMessage(byId("formationMessage"), "自动列队至少需要选择两台机器人。", true);
    return;
  }
  if (!byId("confirmMotion").checked || !byId("confirmFormation").checked) {
    setMessage(byId("formationMessage"), "需要同时勾选真机动作确认和列队区域确认。", true);
    return;
  }
  if (!window.confirm(`确认对 ${targets.join(", ")} 执行闭环自动列队？请确认每台已完成 map frame 标定，区域必须空旷。`)) return;
  try {
    const result = await api("/api/formation", {
      method: "POST",
      body: JSON.stringify({
        targets,
        confirm_motion: byId("confirmMotion").checked,
        confirm_formation: byId("confirmFormation").checked,
        pattern: byId("formationPattern").value,
        spacing: byId("formationSpacing").value,
        formation_speed: byId("formationSpeed").value,
        timeout: byId("formationTimeout").value,
        yaw_rate: byId("yawRate").value,
      }),
    });
    state.devices = result.devices;
    renderDevices();
    const failures = result.results.filter((item) => !item.ok);
    setMessage(byId("formationMessage"), failures.length ? failures.map((item) => `#${item.number}: ${item.error}`).join("; ") : "自动列队完成。", failures.length > 0);
  } catch (error) {
    setMessage(byId("formationMessage"), error.message, true);
  }
}

function renderVideos() {
  const container = byId("videoList");
  const entries = state.devices.filter((device) => device.connected);
  if (!entries.length) {
    container.className = "video-list empty";
    container.textContent = "连接并开启视频后显示画面。";
    return;
  }
  container.className = "video-list";
  container.innerHTML = "";
  entries.forEach((device) => {
    const video = state.videos[String(device.number)] || {};
    const card = document.createElement("article");
    card.className = "video-card";
    const title = document.createElement("div");
    title.className = "video-title";
    title.innerHTML = `<strong>#${device.number}</strong><span>${device.ip}</span><b>${video.enabled ? "视频开启" : "视频关闭"}</b>`;
    const img = document.createElement("img");
    img.alt = `Go2 #${device.number} video`;
    img.src = video.enabled ? `/api/video/frame/${device.number}?t=${Date.now()}` : "";
    const detail = document.createElement("p");
    detail.textContent = video.enabled
      ? `frames=${video.frames || 0}, age=${video.age ?? "?"}s, ${video.width || 0}x${video.height || 0}${video.error ? `, error=${video.error}` : ""}`
      : "未开启";
    card.append(title, img, detail);
    container.appendChild(card);
  });
}

async function refreshVideos() {
  try {
    const result = await api("/api/video/status");
    state.videos = result.videos;
    renderVideos();
  } catch (_error) {}
}

async function setVideo(enabled) {
  try {
    const result = await api("/api/video", {
      method: "POST",
      body: JSON.stringify({ targets: selectedNumbers(), enabled }),
    });
    state.videos = result.videos;
    renderVideos();
    setMessage(byId("videoMessage"), enabled ? "已开启选中视频。" : "已关闭选中视频。");
  } catch (error) {
    setMessage(byId("videoMessage"), error.message, true);
  }
}

function makeControl(metadata) {
  const controls = document.createElement("div");
  controls.className = "action-controls";
  const values = {};
  if (metadata.parameter === "bool") {
    const select = document.createElement("select");
    [["true", "开启"], ["false", "关闭"]].forEach(([value, text]) => {
      const option = document.createElement("option");
      option.value = value;
      option.textContent = text;
      select.appendChild(option);
    });
    controls.appendChild(select);
    values.value = () => select.value === "true";
  } else if (metadata.parameter === "int" || metadata.parameter === "float") {
    const input = document.createElement("input");
    input.type = "number";
    input.value = metadata.parameter === "int" ? "1" : "0.1";
    input.step = metadata.parameter === "int" ? "1" : "0.05";
    controls.appendChild(input);
    values.value = () => input.value;
  } else if (metadata.parameter === "xyz") {
    ["x", "y", "z"].forEach((axis) => {
      const input = document.createElement("input");
      input.type = "number";
      input.value = "0";
      input.step = "0.05";
      input.title = axis;
      controls.appendChild(input);
      values[axis] = () => input.value;
    });
  }
  const send = document.createElement("button");
  send.textContent = metadata.parameter === "query" ? "读取" : "执行";
  send.disabled = !metadata.enabled;
  send.addEventListener("click", async () => {
    const parameters = Object.fromEntries(Object.entries(values).map(([key, getter]) => [key, getter()]));
    const activatesHighRisk = metadata.risk === "high" && !(metadata.parameter === "bool" && parameters.value === false);
    if (activatesHighRisk) {
      if (!byId("confirmHighRisk").checked) {
        setMessage(byId("controlMessage"), "该动作属于高风险动作，请勾选高风险确认。", true);
        return;
      }
      if (!window.confirm(`高风险动作：${metadata.label}。确认对选中的 Go2 执行？`)) return;
    }
    if (state.controlBusy) return;
    state.controlBusy = true;
    send.disabled = true;
    try {
      if (metadata.parameter !== "query" && !(await stopJoystick())) return;
      await sendCommand("sport_action", {
      profile: byId("sportProfile").value,
      action: metadata.name,
      ...parameters,
      });
    } finally { state.controlBusy = false; send.disabled = !metadata.enabled; }
  });
  controls.appendChild(send);
  return controls;
}

function renderCatalog() {
  const profileName = byId("sportProfile").value;
  const profile = state.profiles[profileName];
  if (!profile) return;
  byId("profileNote").textContent = profile.note;
  const container = byId("sportCatalog");
  container.textContent = "";
  const grouped = {};
  Object.values(profile.actions).forEach((metadata) => {
    (grouped[metadata.category] ||= []).push(metadata);
  });
  Object.entries(grouped).forEach(([categoryName, actions]) => {
    const category = document.createElement("section");
    category.className = "category";
    const heading = document.createElement("h3");
    heading.textContent = categoryName;
    const grid = document.createElement("div");
    grid.className = "action-grid";
    actions.sort((left, right) => left.api_id - right.api_id).forEach((metadata) => {
      const card = document.createElement("article");
      card.className = `action-card ${metadata.risk}`;
      const header = document.createElement("div");
      header.className = "action-header";
      const label = document.createElement("strong");
      label.textContent = metadata.label;
      const apiId = document.createElement("span");
      apiId.className = "api-id";
      apiId.textContent = `${metadata.name} / ${metadata.api_id}`;
      header.append(label, apiId);
      if (metadata.risk === "high") {
        const warning = document.createElement("div");
        warning.className = "risk-tag";
        warning.textContent = "高风险";
        card.appendChild(warning);
      }
      const note = document.createElement("p");
      note.textContent = metadata.note || (metadata.enabled ? "" : "仅列出，不可直接执行");
      card.append(header, note, makeControl(metadata));
      grid.appendChild(card);
    });
    category.append(heading, grid);
    container.appendChild(category);
  });
}

async function loadCatalog() {
  const result = await api("/api/sport/catalog");
  state.profiles = result.profiles;
  const profileSelect = byId("sportProfile");
  Object.entries(result.profiles).forEach(([key, profile]) => {
    const option = document.createElement("option");
    option.value = key;
    option.textContent = profile.label;
    profileSelect.appendChild(option);
  });
  profileSelect.addEventListener("change", renderCatalog);
  renderCatalog();
}

function velocityFromSticks() {
  const speed = Number(byId("speed").value);
  const yawRate = Number(byId("yawRate").value);
  return {
    x: -sticks.left.y * speed,
    y: -sticks.left.x * speed,
    z: -sticks.right.x * yawRate,
  };
}

function showVelocity(velocity) {
  byId("vxValue").textContent = velocity.x.toFixed(2);
  byId("vyValue").textContent = velocity.y.toFixed(2);
  byId("wzValue").textContent = velocity.z.toFixed(2);
}

function updateJoystickModeNote() {
  const mode = byId("joystickMode").value;
  byId("joystickModeNote").textContent = mode === "wireless"
    ? "当前使用无线遥控器输入：机载自由避障会参与过滤。释放摇杆或窗口失焦立即停止。"
    : "当前使用 Sport Move：直接发送速度命令，闭环控制器也使用此通道。只在区域安全时使用。";
  byId("joystickModeNote").style.color = mode === "wireless" ? "" : "#ffb4a9";
}

async function publishJoystick() {
  if (state.controlBusy || state.joystickInFlight || (!sticks.left.active && !sticks.right.active)) return;
  const velocity = velocityFromSticks();
  showVelocity(velocity);
  state.joystickInFlight = true;
  try {
    state.joystickPromise = sendCommand("joystick", {...velocity, targets: state.joystickTargets}, false, true);
    await state.joystickPromise;
  } finally {
    state.joystickInFlight = false;
  }
}

function centerKnob(name) {
  const stick = byId(`${name}Stick`);
  stick.querySelector(".knob").style.transform = "translate(0px, 0px)";
  sticks[name].x = 0;
  sticks[name].y = 0;
}

let joystickStopPromise = null;
function stopJoystick() {
  const targets = [...state.joystickTargets];
  sticks.left.active = false;
  sticks.right.active = false;
  centerKnob("left");
  centerKnob("right");
  showVelocity({x:0, y:0, z:0});
  if (stickTimer) { clearInterval(stickTimer); stickTimer = null; }
  if (joystickStopPromise) return joystickStopPromise;
  if (!targets.length) return Promise.resolve(true);
  joystickStopPromise = (async () => {
    // Stop now, then stop again after any outstanding joystick request has drained.
    const first = sendCommand("stop", {targets}, false, true);
    const pending = state.joystickPromise;
    await first;
    if (pending) await pending;
    const ok = await sendCommand("stop", {targets}, false, true);
    if (ok) state.joystickTargets = [];
    return ok;
  })().finally(() => { joystickStopPromise = null; });
  return joystickStopPromise;
}

function configureStick(name, lockVertical = false) {
  const stick = byId(`${name}Stick`);
  const knob = stick.querySelector(".knob");
  const update = (event) => {
    const bounds = stick.getBoundingClientRect();
    const radius = bounds.width / 2 - knob.offsetWidth / 2 - 4;
    let x = (event.clientX - (bounds.left + bounds.width / 2)) / radius;
    let y = lockVertical ? 0 : (event.clientY - (bounds.top + bounds.height / 2)) / radius;
    const magnitude = Math.hypot(x, y);
    if (magnitude > 1) {
      x /= magnitude;
      y /= magnitude;
    }
    sticks[name].x = x;
    sticks[name].y = y;
    knob.style.transform = `translate(${x * radius}px, ${y * radius}px)`;
    publishJoystick();
  };
  stick.addEventListener("pointerdown", (event) => {
    if (state.controlBusy || joystickStopPromise) return;
    if (!byId("confirmMotion").checked) {
      setMessage(byId("controlMessage"), "使用摇杆前必须开启真机动作安全确认。", true);
      return;
    }
    if (!sticks.left.active && !sticks.right.active) state.joystickTargets = selectedNumbers();
    stick.setPointerCapture(event.pointerId);
    sticks[name].active = true;
    update(event);
    if (!stickTimer) stickTimer = setInterval(publishJoystick, 120);
  });
  stick.addEventListener("pointermove", (event) => {
    if (sticks[name].active) update(event);
  });
  const release = () => {
    if (!sticks[name].active) return;
    sticks[name].active = false;
    centerKnob(name);
    if (!sticks.left.active && !sticks.right.active) stopJoystick();
    else publishJoystick();
  };
  stick.addEventListener("pointerup", release);
  stick.addEventListener("pointercancel", release);
}

byId("scanButton").addEventListener("click", scan);
byId('startupAutoConnect').addEventListener('change', async () => {
  try {
    await api('/api/settings', {method:'POST', body:JSON.stringify({auto_connect_enabled:byId('startupAutoConnect').checked})});
    setMessage(byId('persistenceMessage'), '启动连接偏好已保存。设备和密钥自动保存；旧标定在开启点云后验证恢复。连接异常请查看设备列表。');
  } catch (error) { setMessage(byId('persistenceMessage'), error.message, true); }
});
byId("addButton").addEventListener("click", addManual);
byId("manualIp").addEventListener("keydown", (event) => { if (event.key === "Enter") addManual(); });
byId("selectAllButton").addEventListener("click", () => { state.devices.forEach((device) => state.selected.add(device.number)); renderDevices(); drawFormationPreview(); });
byId("selectConnectedButton").addEventListener("click", () => {
  state.selected = new Set(state.devices.filter((device) => device.connected).map((device) => device.number));
  renderDevices();
  drawFormationPreview();
});
byId("selectNoneButton").addEventListener("click", () => { state.selected.clear(); renderDevices(); drawFormationPreview(); });
byId("connectButton").addEventListener("click", () => connectionAction("/api/connect"));
byId("disconnectButton").addEventListener("click", () => connectionAction("/api/disconnect"));
byId("refreshTelemetryButton").addEventListener("click", () => refreshTelemetry(false));
byId("pointCloudOnButton").addEventListener("click", () => setPointCloud(true));
byId("pointCloudOffButton").addEventListener("click", () => setPointCloud(false));
byId("alignButton").addEventListener("click", alignFrame);
byId("clearAlignButton").addEventListener("click", clearAlignment);
byId("alignmentMode").addEventListener("change", updateAlignmentMode);
updateAlignmentMode();
byId("autoAlignButton").addEventListener("click", autoAlign);
byId("applyAutoAlignButton").addEventListener("click", applyAutoAlign);
byId("cancelAutoAlignButton").addEventListener("click", () => {
  ++alignmentRequest;
  alignmentPreview = null;
  alignmentView = null;
  byId("autoAlignButton").disabled = false;
  renderAlignmentResults();
  drawSharedMap();
  setMessage(byId("frameMessage"), "已退出预览，现有坐标未改变；后台匹配任务会自行结束。");
});
byId("adjustLeft").addEventListener("click", () => adjustPreview(0,.1));
byId("adjustRight").addEventListener("click", () => adjustPreview(0,-.1));
byId("adjustForward").addEventListener("click", () => adjustPreview(.1,0));
byId("adjustBack").addEventListener("click", () => adjustPreview(-.1,0));
byId("adjustCCW").addEventListener("click", () => adjustPreview(0,0,Math.PI/90));
byId("adjustCW").addEventListener("click", () => adjustPreview(0,0,-Math.PI/90));
byId("adjustRobot").addEventListener("change", drawSharedMap);
byId("acceptAdjustment").addEventListener("click", () => {
  const r = alignmentPreview?.results.find((item) => item.number === Number(byId("adjustRobot").value));
  if (!r?.transform) return;
  r.manualConfirmed = true;
  renderAlignmentResults();
});
byId("sharedMapCanvas").addEventListener("pointerdown", (event) => {
  if (!alignmentPreview?.done || !alignmentView || !byId("adjustRobot").value) return;
  alignmentDrag = {x:event.clientX,y:event.clientY};
  event.currentTarget.setPointerCapture(event.pointerId);
});
byId("sharedMapCanvas").addEventListener("pointermove", (event) => {
  if (!alignmentDrag || !alignmentView) return;
  const canvas = event.currentTarget, rect = canvas.getBoundingClientRect();
  const dx = -(event.clientY-alignmentDrag.y)*canvas.height/rect.height/alignmentView.scale;
  const dy = -(event.clientX-alignmentDrag.x)*canvas.width/rect.width/alignmentView.scale;
  alignmentDrag = {x:event.clientX,y:event.clientY};
  adjustPreview(dx,dy);
});
["pointerup","pointercancel","lostpointercapture"].forEach((name) => byId("sharedMapCanvas").addEventListener(name, () => {alignmentDrag=null;}));
byId("videoOnButton").addEventListener("click", () => setVideo(true));
byId("videoOffButton").addEventListener("click", () => setVideo(false));
byId("stopButton").addEventListener("click", () => { stopJoystick(); sendCommand("stop", {}, true); });
byId("joystickMode").addEventListener("change", () => {
  stopJoystick();
  updateJoystickModeNote();
});
byId("formationButton").addEventListener("click", async () => { if (await stopJoystick()) runFormation(); });
byId("motionCapture").addEventListener("click", captureMotion);
byId("motionStart").addEventListener("click", startMotion);
byId("motionStop").addEventListener("click", async () => {
  try { renderMotion((await api("/api/motion/stop", {method:"POST"})).motion); }
  catch (error) { setMessage(byId("motionMessage"), error.message, true); }
});
byId("motionImport").addEventListener("change", async (event) => {
  try {
    const file = event.target.files[0];
    if (!file) return;
    if (file.size > 1000000) throw new Error("编排文件不能超过 1 MB");
    byId("motionProgram").value = JSON.stringify(JSON.parse(await file.text()), null, 2);
  } catch (error) { setMessage(byId("motionMessage"), error.message, true); }
});
byId("motionExport").addEventListener("click", () => {
  try {
    const text = JSON.stringify(JSON.parse(byId("motionProgram").value), null, 2);
    const url = URL.createObjectURL(new Blob([text], {type:"application/json"}));
    const a = document.createElement("a"); a.href=url; a.download="closed-loop-program.json"; a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  } catch (error) { setMessage(byId("motionMessage"), error.message, true); }
});
[byId("formationPattern"), byId("formationSpacing"), byId("formationSpeed"), byId("formationTimeout")].forEach((element) => {
  element.addEventListener("input", drawFormationPreview);
  element.addEventListener("change", drawFormationPreview);
});
document.querySelectorAll("[data-command]").forEach((button) => {
  button.addEventListener("click", async () => {
    if (button.dataset.command === 'stop') { stopJoystick(); await sendCommand('stop'); return; }
    if (state.controlBusy) return;
    state.controlBusy = true;
    try { if (await stopJoystick()) await sendCommand(button.dataset.command); }
    finally { state.controlBusy = false; }
  });
});
configureStick("left");
configureStick("right", true);
updateJoystickModeNote();
window.addEventListener("blur", stopJoystick);
document.addEventListener("visibilitychange", () => { if (document.hidden) stopJoystick(); });

refresh().catch((error) => setMessage(byId("scanMessage"), error.message, true));
loadSavedSettings().catch((error) => setMessage(byId('persistenceMessage'), error.message, true));
refreshTelemetry().catch(() => {});
refreshSharedMap().catch(() => {});
refreshVideos().catch(() => {});
refreshMotion().catch(() => {});
loadCatalog().catch((error) => setMessage(byId("controlMessage"), error.message, true));
drawFormationPreview();
drawSharedMap();
function pollWithoutOverlap(task, interval) {
  let busy = false;
  setInterval(async () => {
    if (busy) return;
    busy = true;
    try { await task(); } catch (_error) { /* The next poll can recover. */ }
    finally { busy = false; }
  }, interval);
}
pollWithoutOverlap(refresh, 1000);
pollWithoutOverlap(refreshTelemetry, 1000);
pollWithoutOverlap(refreshSharedMap, 1200);
pollWithoutOverlap(refreshVideos, 1000);
pollWithoutOverlap(refreshMotion, 500);

async function homeOperation(operation) {
  if (state.controlBusy) return;
  state.controlBusy = true;
  try {
    if (!(await stopJoystick())) throw new Error('停止未确认');
    const standing = operation === 'return' ? window.confirm('确认表演已结束、全部站稳，且回位路线和间距安全？') : false;
    if (operation === 'return' && !standing) return;
    const result = await api('/api/motion/home', {method:'POST', body:JSON.stringify({
      operation, targets:selectedNumbers(), confirm_standing:standing,
      confirm_motion:byId('confirmMotion').checked, confirm_formation:byId('confirmFormation').checked})});
    if (result.motion) renderMotion(result.motion);
    setMessage(byId('motionMessage'), operation === 'capture' ? '起点已记录；重连或重新标定后需重新记录。' : '低速回位已启动，请观察机器。');
  } catch (error) { setMessage(byId('motionMessage'), error.message, true); }
  finally { state.controlBusy = false; }
}
byId('homeCapture').addEventListener('click', () => homeOperation('capture'));
byId('homeReturn').addEventListener('click', () => homeOperation('return'));
