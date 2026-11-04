/* 天轶 2.5 消防巡检后台 —— 第一阶段前端逻辑（无框架，离线可用） */
"use strict";

const state = {
  limit: 10,
  offset: 0,
  total: 0,
  events: { limit: 10, offset: 0, total: 0 },
  manual: { limit: 10, offset: 0, total: 0 },
  robotOnline: false,
  currentRound: null,
  statusLoading: false,
  pollTimer: null,
};

const STATUS_CN = {
  running: "进行中",
  paused: "已暂停",
  completed: "已完成",
  stopped: "已停止",
  aborted: "已中止",
};

const OUTCOME_CN = {
  normal: "正常",
  abnormal: "异常",
  uncertain: "不确定",
};

const POI_STATUS_CN = {
  0: "正常",
  1: "异常",
  2: "不确定",
  3: "超时",
  4: "无图像",
  5: "无模型",
  6: "错误",
  SKIPPED: "已跳过",
};

/* 事件文案：字段为网页侧建议契约，待机器人端确认（见 README） */
const EVENT_TYPE_CN = {
  alarm_detected: "防空警报",
  vision_abnormal: "视觉异常",
  voice_broadcast: "语音播报",
  comm_failure: "通信异常",
  comm_restored: "通信恢复",
};
function eventTypeLabel(item) {
  const kind = item?.details?.kind;

  if (kind === "smoking") return "吸烟行为";
  if (kind === "air_raid") return "防空警报";

  return EVENT_TYPE_CN[item.event_type] || item.event_type;
}
const SEVERITY_CN = {
  critical: "严重",
  warning: "警告",
  info: "提示",
};

const EVENT_STATUS_CN = {
  open: "待处理",
  acknowledged: "已确认",
  resolved: "已解决",
};

const $ = (id) => document.getElementById(id);

function esc(value) {
  return String(value ?? "").replace(/[&<>"']/g, (ch) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#39;",
  }[ch]));
}

function fmtTime(iso) {
  if (!iso) return "—";
  const d = new Date(iso);
  if (Number.isNaN(d.getTime())) return esc(iso);
  const pad = (n) => String(n).padStart(2, "0");
  return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} ` +
         `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}

function showError(message) {
  $("global-error-text").textContent = message;
  $("global-error").hidden = false;
}

function clearError() {
  $("global-error").hidden = true;
}

function sourceBadge(source, isDemo) {
  if (isDemo || source === "demo") return '<span class="pill pill-demo">演示数据</span>';
  if (source === "simulated") return '<span class="pill pill-warn">模拟数据</span>';
  return '<span class="pill pill-real">真实</span>';
}

function statusPill(status) {
  const tone = status === "running" ? "info" : status === "paused" ? "warn" : "muted";
  return `<span class="pill pill-${tone}">${esc(STATUS_CN[status] || status)}</span>`;
}

function outcomePill(outcome) {
  if (!outcome) return "—";
  const tone = outcome === "normal" ? "ok" : outcome === "abnormal" ? "err" : "warn";
  return `<span class="pill pill-${tone}">${esc(OUTCOME_CN[outcome] || outcome)}</span>`;
}

function poiStatusPill(result) {
  const name = result.skipped ? "已跳过" : (result.status_name || String(result.status_code));
  const cn = POI_STATUS_CN[name] || name;
  let tone = "muted";
  if (!result.skipped && result.status_code === 0) tone = "ok";
  else if (!result.skipped && result.status_code === 1) tone = "err";
  else if (!result.skipped && (result.status_code === 2 || result.status_code === 3)) tone = "warn";
  else if (!result.skipped && result.status_code >= 4) tone = "err";
  const sim = result.simulated ? ' <span class="pill pill-demo">模拟</span>' : "";
  return `<span class="pill pill-${tone}">${esc(cn)}</span>${sim}`;
}

function detectionsCell(detections) {
  if (!detections || !detections.length) return "—";
  return '<div class="det-list">' + detections.map((det) => {
    const cls = esc(det.cls || "?");
    const conf = typeof det.conf === "number" ? ` ${Math.round(det.conf * 100)}%` : "";
    return `<span class="det-chip">${cls}${conf}</span>`;
  }).join("") + "</div>";
}

function currentPoi(round) {
  if (!round || !round.ros_state) return "";
  const tail = String(round.ros_state).split(":").pop();
  return /^POI_\d+$/.test(tail) ? tail : "";
}

async function fetchJson(url, options) {
  const resp = await fetch(url, options);
  if (resp.status === 401) {
    // 会话过期 / 未登录：回登录页
    window.location.href = "/login";
    return { ok: false, status: 401, body: null };
  }
  let body = null;
  try {
    body = await resp.json();
  } catch (err) {
    body = null;
  }
  return { ok: resp.ok, status: resp.status, body };
}

/* ---------------- 机器人 / 当前状态 ---------------- */

async function loadStatus() {
  if (state.statusLoading) return;
  state.statusLoading = true;
  try {
    const { ok, body, status } = await fetchJson("/api/status");
    if (!ok || !body) throw new Error(`状态接口异常（HTTP ${status}）`);
    clearError();
    state.robotOnline = Boolean(body.robot && body.robot.online);
    state.currentRound = body.current_round || null;
    renderRobot(body.robot || {});
    renderStatus(body.current_round || null, body.robot || {});
    updateControls();
    $("status-updated").textContent = `更新于 ${fmtTime(body.server_time)}`;
    await loadRounds();
    await loadEvents();
    await loadManuals();
  } catch (err) {
    $("status-loading").hidden = true;
    showError(`无法获取后端状态：${err.message}。请确认服务已启动（127.0.0.1:8000）。`);
    renderRobot({ online: false, detail: "后端不可达" });
  } finally {
    state.statusLoading = false;
  }
}

function renderRobot(robot) {
  const badge = $("robot-badge");
  const online = Boolean(robot.online);
  badge.className = `badge ${online ? "badge-online" : "badge-offline"}`;
  badge.textContent = online
    ? `机器人在线${robot.state_text ? ` · ${robot.state_text}` : ""}`
    : `机器人离线${robot.adapter ? ` · 适配层 ${robot.adapter}` : ""}`;
  const note = $("control-note");
  if (online) {
    note.textContent = "机器人在线：控制请求会经 ROS 适配层下发到 /fire_inspection/control。";
  } else {
    note.textContent = `机器人未连接（${robot.detail || "ROS 适配层未启用"}）；控制按钮已禁用，不会发送任何指令。`;
  }
}

function renderStatus(round, robot) {
  $("status-loading").hidden = true;
  const content = $("status-content");
  const empty = $("status-empty");
  if (!round) {
    content.hidden = true;
    empty.hidden = false;
    $("poi-count").textContent = "";
    renderPoiResults([]);
    updateDemoBanner(Boolean(robot && robot.online === false && false));
    return;
  }
  empty.hidden = true;
  content.hidden = false;
  $("st-status").innerHTML = statusPill(round.status);
  $("st-round").textContent = round.round_id;
  $("st-source").innerHTML = sourceBadge(round.source, round.is_demo);
  $("st-operator").textContent = round.operator_id || "—";
  $("st-started").textContent = fmtTime(round.started_at);
  $("st-recorded").textContent = `${round.recorded_count} 点` +
    (round.abnormal_count ? `（异常 ${round.abnormal_count}）` : "");

  const rosState = $("st-ros-state");
  const poi = currentPoi(round);
  rosState.textContent = [
    round.ros_state ? `ROS 状态：${round.ros_state}` : "",
    poi ? `当前点位：${poi}` : "",
  ].filter(Boolean).join("　|　");
  renderProgress(round);

  const offlineNote = $("st-offline-note");
  if (!robot.online) {
    offlineNote.hidden = false;
    offlineNote.textContent = `${robot.detail || "ROS 适配层未启用"}。当前轮次数据来自本地数据库，不代表机器人实时回传。`;
  } else {
    offlineNote.hidden = true;
  }

  renderPoiResults(round.results || []);
  state.statusDemo = Boolean(round.is_demo);
  updateDemoBanner();
}

function renderPoiResults(results) {
  const body = $("poi-body");
  const empty = $("poi-empty");
  body.innerHTML = "";
  if (!results.length) {
    empty.hidden = false;
    $("poi-count").textContent = "";
    return;
  }
  empty.hidden = true;
  $("poi-count").textContent = `共 ${results.length} 条`;
  results.forEach((item, index) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${index + 1}</td>
      <td class="mono">${esc(item.poi)}</td>
      <td>${poiStatusPill(item)}</td>
      <td>${esc(item.detail || "—")}</td>
      <td>${detectionsCell(item.detections)}</td>
      <td class="muted small">${fmtTime(item.checked_at)}</td>`;
    body.appendChild(tr);
  });
}

function renderProgress(round) {
  const wrap = $("progress-wrap");
  const total = Number(round.total_pois || 0);
  const processed = Number(round.processed_count || 0);
  if (!total) {
    // 机器人端尚未提供计划点位总数时，不臆造百分比
    wrap.hidden = true;
    return;
  }
  const done = Math.min(processed, total);
  const percent = Math.round((done / total) * 100);
  wrap.hidden = false;
  $("progress-text").textContent = `${done} / ${total} 点 · ${percent}%`;
  $("progress-bar").style.width = `${percent}%`;
}

function updateControls() {
  const online = state.robotOnline;
  const round = state.currentRound;
  const status = round ? round.status : null;
  const active = status === "running" || status === "paused";
  const enabled = {
    start: online && !active,
    pause: online && status === "running",
    resume: online && status === "paused",
    skip: online && active,
    stop: online && active,
  };
  document.querySelectorAll("[data-cmd]").forEach((btn) => {
    btn.disabled = !enabled[btn.dataset.cmd];
  });
}

/* ---------------- 历史记录 ---------------- */

function currentFilters() {
  return {
    status: $("filter-status").value,
    outcome: $("filter-outcome").value,
    source: $("filter-source").value,
  };
}

async function loadRounds() {
  const params = new URLSearchParams();
  const filters = currentFilters();
  Object.entries(filters).forEach(([key, value]) => {
    if (value) params.set(key, value);
  });
  params.set("limit", String(state.limit));
  params.set("offset", String(state.offset));

  const { ok, body, status } = await fetchJson(`/api/rounds?${params.toString()}`);
  if (!ok || !body) {
    throw new Error(`历史记录接口异常（HTTP ${status}）`);
  }
  state.total = body.total;
  renderRounds(body.items);
  renderPager();
}

function renderRounds(items) {
  const body = $("rounds-body");
  const empty = $("rounds-empty");
  body.innerHTML = "";
  if (!items.length) {
    empty.hidden = false;
    return;
  }
  empty.hidden = true;
  items.forEach((round) => {
    const tr = document.createElement("tr");
    tr.className = "clickable";
    tr.innerHTML = `
      <td class="mono">${esc(round.round_id)}</td>
      <td class="muted small">${fmtTime(round.started_at)}</td>
      <td>${statusPill(round.status)}</td>
      <td>${outcomePill(round.outcome)}</td>
      <td>${round.recorded_count} 点${round.abnormal_count ? ` <span class="pill pill-err">异常 ${round.abnormal_count}</span>` : ""}</td>
      <td>${esc(round.operator_id || "—")}</td>
      <td>${sourceBadge(round.source, round.is_demo)}</td>`;
    tr.addEventListener("click", () => openDetail(round.round_id));
    body.appendChild(tr);
  });
  updateDemoBanner();
}

function renderPager() {
  const page = Math.floor(state.offset / state.limit) + 1;
  const pages = Math.max(1, Math.ceil(state.total / state.limit));
  $("page-info").textContent = `第 ${page} / ${pages} 页 · 共 ${state.total} 条`;
  $("prev-page").disabled = state.offset <= 0;
  $("next-page").disabled = state.offset + state.limit >= state.total;
}

function updateDemoBanner(extra) {
  const rows = $("rounds-body").querySelectorAll("tr");
  let hasDemo = Boolean(extra);
  rows.forEach((tr) => {
    if (tr.textContent.includes("演示数据")) hasDemo = true;
  });
  $("demo-banner").hidden = !hasDemo;
}

/* ---------------- 详情 ---------------- */

async function openDetail(roundId) {
  const modal = $("detail-modal");
  $("modal-body").innerHTML = '<p class="placeholder">加载中…</p>';
  modal.hidden = false;
  try {
    const { ok, body, status } = await fetchJson(`/api/rounds/${encodeURIComponent(roundId)}`);
    if (status === 404) {
      $("modal-body").innerHTML = '<p class="placeholder">巡检记录不存在。</p>';
      return;
    }
    if (!ok || !body) throw new Error(`详情接口异常（HTTP ${status}）`);
    renderDetail(body);
  } catch (err) {
    $("modal-body").innerHTML = `<p class="placeholder">加载详情失败：${esc(err.message)}</p>`;
  }
}

function renderDetail(round) {
  const results = round.results || [];
  const rows = results.map((item, index) => `
    <tr>
      <td>${index + 1}</td>
      <td class="mono">${esc(item.poi)}</td>
      <td>${poiStatusPill(item)}</td>
      <td>${esc(item.detail || "—")}</td>
      <td>${detectionsCell(item.detections)}</td>
      <td class="muted small">${fmtTime(item.checked_at)}</td>
    </tr>`).join("");

  $("modal-body").innerHTML = `
    <div class="status-grid">
      <div class="stat"><span class="stat-label">轮次编号</span><strong class="mono">${esc(round.round_id)}</strong></div>
      <div class="stat"><span class="stat-label">状态</span><strong>${statusPill(round.status)}</strong></div>
      <div class="stat"><span class="stat-label">结果</span><strong>${outcomePill(round.outcome)}</strong></div>
      <div class="stat"><span class="stat-label">数据来源</span><strong>${sourceBadge(round.source, round.is_demo)}</strong></div>
      <div class="stat"><span class="stat-label">开始时间</span><strong>${fmtTime(round.started_at)}</strong></div>
      <div class="stat"><span class="stat-label">结束时间</span><strong>${fmtTime(round.finished_at)}</strong></div>
      <div class="stat"><span class="stat-label">操作人</span><strong>${esc(round.operator_id || "—")}</strong></div>
      <div class="stat"><span class="stat-label">已记录点位</span><strong>${round.recorded_count} 点${round.abnormal_count ? ` · 异常 ${round.abnormal_count}` : ""}</strong></div>
    </div>
    ${round.summary ? `<p class="notice">结论摘要：${esc(round.summary)}</p>` : ""}
    ${round.error_message ? `<p class="notice">异常说明：${esc(round.error_message)}</p>` : ""}
    <h3>巡检点结果</h3>
    <div class="table-wrap">
      <table class="data-table">
        <thead><tr><th>#</th><th>点位</th><th>结果</th><th>说明</th><th>检测数据</th><th>时间</th></tr></thead>
        <tbody>${rows || '<tr><td colspan="6" class="muted">本轮没有点位结果</td></tr>'}</tbody>
      </table>
    </div>`;
}

function closeDetail() {
  $("detail-modal").hidden = true;
}

function closeEventModal() {
  $("event-modal").hidden = true;
}

/* ---------------- 控制 ---------------- */

async function sendControl(cmd) {
  if (!state.robotOnline) {
    showFeedback(false, "机器人离线，未发送任何指令。控制按钮仅在 ROS 适配层启用后可用。");
    return;
  }
  const payload = {
    cmd,
    arg: $("arg-input").value.trim(),
    operator_id: $("operator-input").value.trim(),
  };
  showFeedback(null, "正在下发指令…");
  try {
    const { ok, body, status } = await fetchJson("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const message = body && body.message ? body.message : `HTTP ${status}`;
    showFeedback(Boolean(ok && body && body.success), message);
    if (!ok && status === 503) {
      showFeedback(false, `${message}（指令未下发）`);
    }
    await loadStatus();
  } catch (err) {
    showFeedback(false, `控制请求失败：${err.message}`);
  }
}

function showFeedback(ok, message) {
  const el = $("control-feedback");
  el.hidden = false;
  el.className = "feedback " + (ok === null ? "" : ok ? "feedback-ok" : "feedback-err");
  el.textContent = message;
}

/* ---------------- 异常事件 ---------------- */

function severityPill(severity) {
  const tone = severity === "critical" ? "err" : severity === "warning" ? "warn" : "info";
  return `<span class="pill pill-${tone}">${esc(SEVERITY_CN[severity] || severity)}</span>`;
}

function eventStatusPill(status) {
  const tone = status === "resolved" ? "ok" : status === "acknowledged" ? "info" : "warn";
  return `<span class="pill pill-${tone}">${esc(EVENT_STATUS_CN[status] || status)}</span>`;
}

function eventRelated(item) {
  const parts = [];
  if (item.round_id) parts.push(item.round_id);
  if (item.poi) parts.push(item.poi);
  return parts.length ? `<span class="mono small">${esc(parts.join(" / "))}</span>` : "—";
}

function currentEventFilters() {
  return {
    event_type: $("filter-event-type").value,
    severity: $("filter-event-severity").value,
    status: $("filter-event-status").value,
  };
}

async function loadEvents() {
  const params = new URLSearchParams();
  Object.entries(currentEventFilters()).forEach(([key, value]) => {
    if (value) params.set(key, value);
  });
  params.set("limit", String(state.events.limit));
  params.set("offset", String(state.events.offset));

  const errorEl = $("events-error");
  try {
    const { ok, body, status } = await fetchJson(`/api/events?${params.toString()}`);
    if (!ok || !body) throw new Error(`事件接口异常（HTTP ${status}）`);
    errorEl.hidden = true;
    state.events.total = body.total;
    renderEvents(body.items);
    renderEventsPager();
  } catch (err) {
    $("events-body").innerHTML = "";
    $("events-empty").hidden = true;
    errorEl.hidden = false;
    errorEl.textContent = `无法获取异常事件：${err.message}。后端不可用时事件列表为空，请在服务恢复后刷新。`;
  }
}

function renderEvents(items) {
  const body = $("events-body");
  const empty = $("events-empty");
  body.innerHTML = "";
  if (!items.length) {
    empty.hidden = false;
    return;
  }
  empty.hidden = true;
  items.forEach((item) => {
    const tr = document.createElement("tr");
    tr.className = "clickable";
    tr.innerHTML = `
      <td class="muted small">${fmtTime(item.timestamp)}</td>
      <td>${esc(eventTypeLabel(item))}${item.simulated ? ' <span class="pill pill-demo">模拟</span>' : ""}</td>
      <td>${severityPill(item.severity)}</td>
      <td class="mono small">${esc(item.source)}</td>
      <td>${esc(item.message)}</td>
      <td>${eventStatusPill(item.status)}</td>
      <td>${eventRelated(item)}</td>`;
    tr.addEventListener("click", () => openEventDetail(item.event_id));
    body.appendChild(tr);
  });
}

function renderEventsPager() {
  const page = Math.floor(state.events.offset / state.events.limit) + 1;
  const pages = Math.max(1, Math.ceil(state.events.total / state.events.limit));
  $("events-page-info").textContent = `第 ${page} / ${pages} 页 · 共 ${state.events.total} 条`;
  $("events-prev").disabled = state.events.offset <= 0;
  $("events-next").disabled = state.events.offset + state.events.limit >= state.events.total;
}

async function openEventDetail(eventId) {
  const modal = $("event-modal");
  $("event-modal-body").innerHTML = '<p class="placeholder">加载中…</p>';
  modal.hidden = false;
  try {
    const { ok, body, status } = await fetchJson(`/api/events/${encodeURIComponent(eventId)}`);
    if (status === 404) {
      $("event-modal-body").innerHTML = '<p class="placeholder">事件不存在。</p>';
      return;
    }
    if (!ok || !body) throw new Error(`事件详情接口异常（HTTP ${status}）`);
    renderEventDetail(body);
  } catch (err) {
    $("event-modal-body").innerHTML = `<p class="placeholder">加载事件详情失败：${esc(err.message)}</p>`;
  }
}

function renderEventDetail(item) {
  const details = item.details
    ? `<pre class="details-json">${esc(JSON.stringify(item.details, null, 2))}</pre>`
    : '<p class="placeholder">没有附加字段。</p>';
  const canAck = item.status === "open";
  $("event-modal-body").innerHTML = `
    <div class="status-grid">
      <div class="stat"><span class="stat-label">事件编号</span><strong class="mono">${esc(item.event_id)}</strong></div>
      <div class="stat"><span class="stat-label">类型</span><strong>${esc(eventTypeLabel(item))}${item.simulated ? ' <span class="pill pill-demo">模拟</span>' : ""}</strong></div>
      <div class="stat"><span class="stat-label">发生时间</span><strong>${fmtTime(item.timestamp)}</strong></div>
      <div class="stat"><span class="stat-label">来源</span><strong class="mono">${esc(item.source)}</strong></div>
      <div class="stat"><span class="stat-label">级别</span><strong>${severityPill(item.severity)}</strong></div>
      <div class="stat"><span class="stat-label">处理状态</span><strong>${eventStatusPill(item.status)}</strong></div>
      <div class="stat"><span class="stat-label">关联轮次 / 点位</span><strong class="mono">${esc(item.round_id || "—")} / ${esc(item.poi || "—")}</strong></div>
      <div class="stat"><span class="stat-label">处理人 / 时间</span><strong>${esc(item.handled_by || "—")} / ${fmtTime(item.handled_at)}</strong></div>
    </div>
    <p class="notice">${esc(item.message)}</p>
    <h3>事件数据（details）</h3>
    ${details}
    <div class="modal-actions">
      ${canAck ? '<button id="event-ack-btn" class="btn btn-primary" type="button">标记已处理</button>' : ""}
    </div>`;
  if (canAck) {
    $("event-ack-btn").addEventListener("click", () => ackEvent(item.event_id));
  }
}

async function ackEvent(eventId) {
  const operator = $("operator-input").value.trim();
  try {
    const { ok, body, status } = await fetchJson(
      `/api/events/${encodeURIComponent(eventId)}/ack`,
      {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ operator_id: operator, status: "acknowledged" }),
      }
    );
    if (!ok || !body) throw new Error(`HTTP ${status}`);
    renderEventDetail(body);
    await loadEvents();
  } catch (err) {
    $("event-modal-body").insertAdjacentHTML(
      "beforeend",
      `<p class="feedback feedback-err">标记处理失败：${esc(err.message)}</p>`
    );
  }
}

/* ---------------- 手动点名检查记录 ---------------- */

function manualResultPill(item) {
  // 没有正式结果时绝不显示"正常"：明确标注"无结果"
  if (item.status_code === null || item.status_code === undefined) {
    const sim = item.simulated ? ' <span class="pill pill-demo">模拟</span>' : "";
    return `<span class="pill pill-muted">无结果</span>${sim}`;
  }
  return poiStatusPill(item);
}

async function loadManuals() {
  const params = new URLSearchParams();
  const filter = $("filter-manual-status").value;
  if (filter) params.set("status", filter);
  params.set("limit", String(state.manual.limit));
  params.set("offset", String(state.manual.offset));

  const errorEl = $("manual-error");
  try {
    const { ok, body, status } = await fetchJson(`/api/manual-inspections?${params.toString()}`);
    if (!ok || !body) throw new Error(`手动检查接口异常（HTTP ${status}）`);
    errorEl.hidden = true;
    state.manual.total = body.total;
    renderManuals(body.items);
    renderManualPager();
  } catch (err) {
    $("manual-body").innerHTML = "";
    $("manual-empty").hidden = true;
    errorEl.hidden = false;
    errorEl.textContent = `无法获取手动检查记录：${err.message}。后端不可用时列表为空，请在服务恢复后刷新。`;
  }
}

function renderManuals(items) {
  const body = $("manual-body");
  const empty = $("manual-empty");
  body.innerHTML = "";
  if (!items.length) {
    empty.hidden = false;
    return;
  }
  empty.hidden = true;
  items.forEach((item) => {
    const tr = document.createElement("tr");
    tr.className = "clickable";
    tr.innerHTML = `
      <td class="mono">${esc(item.poi)}</td>
      <td>${manualResultPill(item)}</td>
      <td>${statusPill(item.status)}</td>
      <td>${esc(item.operator_id || "—")}</td>
      <td class="muted small">${fmtTime(item.checked_at || item.started_at)}</td>
      <td class="muted small">${esc(item.error_message || item.detail || "—")}</td>`;
    tr.addEventListener("click", () => openManualDetail(item.manual_id));
    body.appendChild(tr);
  });
}

function renderManualPager() {
  const page = Math.floor(state.manual.offset / state.manual.limit) + 1;
  const pages = Math.max(1, Math.ceil(state.manual.total / state.manual.limit));
  $("manual-page-info").textContent = `第 ${page} / ${pages} 页 · 共 ${state.manual.total} 条`;
  $("manual-prev").disabled = state.manual.offset <= 0;
  $("manual-next").disabled = state.manual.offset + state.manual.limit >= state.manual.total;
}

async function openManualDetail(manualId) {
  const modal = $("detail-modal");
  $("modal-body").innerHTML = '<p class="placeholder">加载中…</p>';
  modal.hidden = false;
  try {
    const { ok, body, status } = await fetchJson(`/api/manual-inspections/${encodeURIComponent(manualId)}`);
    if (status === 404) {
      $("modal-body").innerHTML = '<p class="placeholder">手动检查记录不存在。</p>';
      return;
    }
    if (!ok || !body) throw new Error(`详情接口异常（HTTP ${status}）`);
    renderManualDetail(body);
  } catch (err) {
    $("modal-body").innerHTML = `<p class="placeholder">加载详情失败：${esc(err.message)}</p>`;
  }
}

function renderManualDetail(item) {
  $("modal-body").innerHTML = `
    <div class="status-grid">
      <div class="stat"><span class="stat-label">手动检查编号</span><strong class="mono">${esc(item.manual_id)}</strong></div>
      <div class="stat"><span class="stat-label">点位</span><strong class="mono">${esc(item.poi)}</strong></div>
      <div class="stat"><span class="stat-label">执行状态</span><strong>${statusPill(item.status)}</strong></div>
      <div class="stat"><span class="stat-label">检查结果</span><strong>${manualResultPill(item)}</strong></div>
      <div class="stat"><span class="stat-label">操作人</span><strong>${esc(item.operator_id || "—")}</strong></div>
      <div class="stat"><span class="stat-label">开始时间</span><strong>${fmtTime(item.started_at)}</strong></div>
      <div class="stat"><span class="stat-label">检查时间</span><strong>${fmtTime(item.checked_at)}</strong></div>
      <div class="stat"><span class="stat-label">结束时间</span><strong>${fmtTime(item.finished_at)}</strong></div>
    </div>
    ${item.ros_state ? `<p class="notice">ROS 状态：<span class="mono">${esc(item.ros_state)}</span></p>` : ""}
    ${item.error_message ? `<p class="notice">异常说明：${esc(item.error_message)}</p>` : ""}
    ${item.detail ? `<p class="notice">检查说明：${esc(item.detail)}</p>` : ""}
    <h3>检测数据</h3>
    ${item.detections && item.detections.length
      ? `<div class="table-wrap"><table class="data-table"><thead><tr><th>类别</th><th>置信度</th></tr></thead><tbody>${
          item.detections.map((det) => `<tr><td>${esc(det.cls || "?")}</td><td>${
            typeof det.conf === "number" ? `${Math.round(det.conf * 100)}%` : "—"
          }</td></tr>`).join("")
        }</tbody></table></div>`
      : '<p class="placeholder">没有检测数据。</p>'}`;
}

/* ---------------- 开发模拟联调（默认关闭） ---------------- */

async function loadDevPanel() {
  const panel = $("dev-panel");
  try {
    const { ok, body } = await fetchJson("/api/dev/scenarios");
    if (!ok || !body || !body.items) {
      panel.hidden = true;
      return;
    }
    panel.hidden = false;
    renderDevButtons(body.items);
  } catch (err) {
    panel.hidden = true;
  }
}

function renderDevButtons(items) {
  const wrap = $("dev-buttons");
  wrap.innerHTML = "";
  items.forEach((item) => {
    const btn = document.createElement("button");
    btn.className = "btn";
    btn.type = "button";
    btn.textContent = item.title;
    btn.title = item.description;
    btn.addEventListener("click", () => runDevScenario(item.key, item.title));
    wrap.appendChild(btn);
  });
}

async function runDevScenario(key, title) {
  const fb = $("dev-feedback");
  fb.hidden = false;
  fb.className = "feedback";
  fb.textContent = `正在执行模拟场景「${title}」…`;
  try {
    const { ok, body, status } = await fetchJson(
      `/api/dev/scenarios/${encodeURIComponent(key)}`,
      { method: "POST" }
    );
    if (!ok || !body) {
      throw new Error(body && body.detail ? body.detail : `HTTP ${status}`);
    }
    fb.className = "feedback feedback-ok";
    fb.textContent = `已执行模拟场景「${body.title || title}」（模拟数据，非真实机器人结果）。`;
    await loadStatus();
    await loadEvents();
  } catch (err) {
    fb.className = "feedback feedback-err";
    fb.textContent = `模拟场景执行失败：${err.message}`;
  }
}

/* ---------------- 事件绑定 ---------------- */

function bindEvents() {
  $("refresh-btn").addEventListener("click", () => loadStatus());
  $("retry-btn").addEventListener("click", () => loadStatus());
  $("prev-page").addEventListener("click", () => {
    state.offset = Math.max(0, state.offset - state.limit);
    loadRounds().catch((err) => showError(err.message));
  });
  $("next-page").addEventListener("click", () => {
    state.offset += state.limit;
    loadRounds().catch((err) => showError(err.message));
  });
  ["filter-status", "filter-outcome", "filter-source"].forEach((id) => {
    $(id).addEventListener("change", () => {
      state.offset = 0;
      loadRounds()
        .then(() => updateDemoBanner(false))
        .catch((err) => showError(err.message));
    });
  });
  document.querySelectorAll("[data-cmd]").forEach((btn) => {
    btn.addEventListener("click", () => sendControl(btn.dataset.cmd));
  });
  $("modal-close").addEventListener("click", closeDetail);
  $("detail-modal").addEventListener("click", (event) => {
    if (event.target === $("detail-modal")) closeDetail();
  });

  ["filter-event-type", "filter-event-severity", "filter-event-status"].forEach((id) => {
    $(id).addEventListener("change", () => {
      state.events.offset = 0;
      loadEvents();
    });
  });
  $("events-prev").addEventListener("click", () => {
    state.events.offset = Math.max(0, state.events.offset - state.events.limit);
    loadEvents();
  });
  $("events-next").addEventListener("click", () => {
    state.events.offset += state.events.limit;
    loadEvents();
  });
  $("event-modal-close").addEventListener("click", closeEventModal);
  $("event-modal").addEventListener("click", (event) => {
    if (event.target === $("event-modal")) closeEventModal();
  });

  const logoutBtn = $("logout-btn");
  if (logoutBtn) {
    logoutBtn.addEventListener("click", async () => {
      try {
        await fetch("/logout", { method: "POST", credentials: "same-origin" });
      } catch (err) {
        /* 网络失败也要回登录页，由服务端会话状态兜底 */
      }
      window.location.href = "/login";
    });
  }

  $("filter-manual-status").addEventListener("change", () => {
    state.manual.offset = 0;
    loadManuals();
  });
  $("manual-prev").addEventListener("click", () => {
    state.manual.offset = Math.max(0, state.manual.offset - state.manual.limit);
    loadManuals();
  });
  $("manual-next").addEventListener("click", () => {
    state.manual.offset += state.manual.limit;
    loadManuals();
  });

  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      closeDetail();
      closeEventModal();
    }
  });
}

function startPolling() {
  if (state.pollTimer) clearInterval(state.pollTimer);
  state.pollTimer = setInterval(() => loadStatus(), 5000);
}

document.addEventListener("DOMContentLoaded", () => {
  bindEvents();
  loadStatus();
  loadDevPanel();
  startPolling();
});
