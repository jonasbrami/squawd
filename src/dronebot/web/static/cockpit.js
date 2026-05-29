const $ = (id) => document.getElementById(id);
const wsURL = (path) => `ws://${location.host}${path}`;

// --- chat ---
const log = $("log");
function addLine(who, text) {
  const div = document.createElement("div");
  div.className = who;
  div.textContent = (who === "you" ? "you> " : who === "drone" ? "drone> " : "") + text;
  log.appendChild(div);
  log.scrollTop = log.scrollHeight;
}
let chat = new WebSocket(wsURL("/chat"));
chat.onmessage = (e) => { if (e.data.trim()) addLine("drone", e.data); };
chat.onclose = () => addLine("sys", "chat disconnected");
$("chat-form").addEventListener("submit", (e) => {
  e.preventDefault();
  const v = $("msg").value.trim();
  if (!v) return;
  addLine("you", v);
  chat.send(v);
  $("msg").value = "";
});
$("abort").addEventListener("click", () => { addLine("sys", "ABORT sent"); chat.send("abort"); });

// --- telemetry + map ---
const conn = $("conn");
let last = null;
let tele = new WebSocket(wsURL("/telemetry"));
tele.onmessage = (e) => {
  const t = JSON.parse(e.data);
  last = t;
  conn.textContent = t.connected ? "online" : "offline";
  conn.classList.toggle("online", !!t.connected);
  $("t-mode").textContent = t.flight_mode ?? "—";
  $("t-armed").textContent = t.armed ? "yes" : "no";
  $("t-air").textContent = t.in_air ? "yes" : "no";
  $("t-alt").textContent = t.rel_alt == null ? "—" : t.rel_alt.toFixed(1) + " m";
  $("t-batt").textContent = t.battery == null ? "—" : Math.round(t.battery * 100) + "%";
  $("obstacle").textContent = "surroundings: " + (t.surroundings ?? "—");
  drawMap(t);
};

const cv = $("map"), ctx = cv.getContext("2d");
function drawMap(t) {
  const w = cv.width, h = cv.height, cx = w / 2, cy = h / 2;
  ctx.clearRect(0, 0, w, h);
  // geofence ring (scaled so the fence radius ~ 0.42 * canvas)
  const R = (t.geofence_radius_m || 100);
  const scale = (Math.min(w, h) * 0.42) / R; // px per meter
  ctx.strokeStyle = "#1f2a37";
  ctx.beginPath(); ctx.arc(cx, cy, R * scale, 0, 2 * Math.PI); ctx.stroke();
  // home
  ctx.fillStyle = "#7da2c7"; ctx.fillRect(cx - 3, cy - 3, 6, 6);
  if (!t.position || !t.home) return;
  // north/east offset of drone from home, in meters (small-angle)
  const dN = (t.position.lat - t.home.lat) * 111320;
  const dE = (t.position.lon - t.home.lon) * 111320 * Math.cos(t.home.lat * Math.PI / 180);
  const px = cx + dE * scale, py = cy - dN * scale; // north = up
  ctx.fillStyle = "#3ddc97";
  ctx.beginPath(); ctx.arc(px, py, 4, 0, 2 * Math.PI); ctx.fill();
}
