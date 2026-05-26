const state = { devices: [], selected: new Set(), profiles: {}, joystickInFlight: false };
const sticks = {
  left: { x: 0, y: 0, active: false },
  right: { x: 0, y: 0, active: false },
};
let stickTimer = null;

function byId(id) { return document.getElementById(id); }

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const result = await response.json();
  if (!response.ok || !result.ok) throw new Error(result.error || "请求失败");
  return result;
}

function setMessage(element, text, failed = false) {
  element.textContent = text;
  element.style.color = failed ? "#b63326" : "";
}

function selectedNumbers() {
  return Array.from(state.selected).sort((a, b) => a - b);
}

function renderDevices() {
  const container = byId("deviceList");
  if (!state.devices.length) {
    container.className = "device-list empty";
    container.textContent = "尚未发现设备";
    byId("connectedCount").textContent = "0";
    return;
  }
  container.className = "device-list";
  container.innerHTML = "";
  state.devices.forEach((device) => {
    const row = document.createElement("label");
    row.className = `device ${device.connected ? "connected" : ""}`;
    const status = device.busy ? "处理中" : device.connected ? "已连接" : "未连接";
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
    checkbox.checked = state.selected.has(device.number);
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
    const statusPill = document.createElement("span");
    statusPill.className = "status";
    statusPill.textContent = status;
    row.append(checkbox, number, information, statusPill);
    container.appendChild(row);
  });
  container.querySelectorAll("input[type=checkbox]").forEach((checkbox) => {
    checkbox.addEventListener("change", () => {
      const number = Number(checkbox.dataset.number);
      checkbox.checked ? state.selected.add(number) : state.selected.delete(number);
    });
  });
  byId("connectedCount").textContent = state.devices.filter((item) => item.connected).length;
}

async function refresh() {
  const result = await api("/api/devices");
  state.devices = result.devices;
  renderDevices();
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
      body: JSON.stringify({ ip: byId("manualIp").value }),
    });
    state.devices = result.devices;
    state.selected.add(result.device.number);
    byId("manualIp").value = "";
    renderDevices();
    setMessage(byId("scanMessage"), `已登记 Go2 #${result.device.number}。`);
  } catch (error) {
    setMessage(byId("scanMessage"), error.message, true);
  }
}

async function connectionAction(path) {
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
        ...extra,
      }),
    });
    state.devices = result.devices;
    renderDevices();
    const failures = result.results.filter((item) => !item.ok);
    if (!quiet || failures.length) {
      setMessage(byId("controlMessage"), failures.length ? failures.map((item) => `#${item.number}: ${item.error}`).join("; ") : `命令 ${command} 已发送到 ${targets.join(", ")}。`, failures.length > 0);
    }
  } catch (error) {
    setMessage(byId("controlMessage"), error.message, true);
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
  send.addEventListener("click", () => {
    const parameters = Object.fromEntries(Object.entries(values).map(([key, getter]) => [key, getter()]));
    const activatesHighRisk = metadata.risk === "high" && !(metadata.parameter === "bool" && parameters.value === false);
    if (activatesHighRisk) {
      if (!byId("confirmHighRisk").checked) {
        setMessage(byId("controlMessage"), "该动作属于高风险动作，请勾选高风险确认。", true);
        return;
      }
      if (!window.confirm(`高风险动作：${metadata.label}。确认对选中的 Go2 执行？`)) return;
    }
    sendCommand("sport_action", {
      profile: byId("sportProfile").value,
      action: metadata.name,
      ...parameters,
    });
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

async function publishJoystick() {
  if (state.joystickInFlight || (!sticks.left.active && !sticks.right.active)) return;
  const velocity = velocityFromSticks();
  showVelocity(velocity);
  state.joystickInFlight = true;
  try {
    await sendCommand("joystick", velocity, false, true);
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

function stopJoystick() {
  const wasActive = sticks.left.active || sticks.right.active;
  sticks.left.active = false;
  sticks.right.active = false;
  centerKnob("left");
  centerKnob("right");
  showVelocity({ x: 0, y: 0, z: 0 });
  if (stickTimer) {
    clearInterval(stickTimer);
    stickTimer = null;
  }
  if (wasActive) sendCommand("stop", {}, false, true);
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
    if (!byId("confirmMotion").checked) {
      setMessage(byId("controlMessage"), "使用摇杆前必须开启真机动作安全确认。", true);
      return;
    }
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
byId("addButton").addEventListener("click", addManual);
byId("manualIp").addEventListener("keydown", (event) => { if (event.key === "Enter") addManual(); });
byId("selectAllButton").addEventListener("click", () => { state.devices.forEach((device) => state.selected.add(device.number)); renderDevices(); });
byId("selectNoneButton").addEventListener("click", () => { state.selected.clear(); renderDevices(); });
byId("connectButton").addEventListener("click", () => connectionAction("/api/connect"));
byId("disconnectButton").addEventListener("click", () => connectionAction("/api/disconnect"));
byId("stopButton").addEventListener("click", () => { stopJoystick(); sendCommand("stop", {}, true); });
document.querySelectorAll("[data-command]").forEach((button) => {
  button.addEventListener("click", () => { stopJoystick(); sendCommand(button.dataset.command); });
});
configureStick("left");
configureStick("right", true);
window.addEventListener("blur", stopJoystick);
document.addEventListener("visibilitychange", () => { if (document.hidden) stopJoystick(); });

refresh().catch((error) => setMessage(byId("scanMessage"), error.message, true));
loadCatalog().catch((error) => setMessage(byId("controlMessage"), error.message, true));
setInterval(() => refresh().catch(() => {}), 2000);
