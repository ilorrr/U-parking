/* U‑Parking Console (Vanilla JS SPA)
   - Hash-router (#/route)
   - Local demo auth (localStorage). Replace auth calls with Django endpoints.
*/

// -------- Settings / config --------
const DEFAULT_API_BASE = "http://127.0.0.1:8000";
const LS = {
  session: "up_session",
  settings: "up_settings",
  alerts: "up_alerts_demo",
  lot: "up_lot_demo",
  res: "up_res_demo",
};

function loadSettings(){
  try { return JSON.parse(localStorage.getItem(LS.settings) || "{}"); }
  catch { return {}; }
}
function saveSettings(s){ localStorage.setItem(LS.settings, JSON.stringify(s)); }
function apiBase(){
  const s = loadSettings();
  return (s.apiBase || DEFAULT_API_BASE).replace(/\/$/, "");
}

function getSession(){
  try { return JSON.parse(localStorage.getItem(LS.session) || "null"); }
  catch { return null; }
}
function setSession(session){
  localStorage.setItem(LS.session, JSON.stringify(session));
}
function clearSession(){
  localStorage.removeItem(LS.session);
}

// -------- Utilities --------
const $ = (sel, root=document) => root.querySelector(sel);
const $$ = (sel, root=document) => [...root.querySelectorAll(sel)];
function nowISO(){ return new Date().toISOString(); }

function setText(sel, txt){ const el=$(sel); if(el) el.textContent = txt; }
function show(el){ el.classList.remove("hidden"); }
function hide(el){ el.classList.add("hidden"); }

function cloneTemplate(id){
  const t = document.getElementById(id);
  return document.importNode(t.content, true);
}

function requireAuth(){
  const s = getSession();
  return !!(s && s.token);
}

function setActiveLink(route){
  $$(".sidebar-link").forEach(a => {
    const href = a.getAttribute("href") || "";
    a.classList.toggle("active", href === "#/" + route);
  });
}

// -------- API helper --------
// Replace these endpoints with your Django REST (or DRF + SimpleJWT).
async function apiFetch(path, opts={}){
  const token = getSession()?.token;
  const headers = new Headers(opts.headers || {});
  headers.set("Content-Type", "application/json");
  if(token) headers.set("Authorization", `Bearer ${token}`);
  const res = await fetch(apiBase() + path, { ...opts, headers });
  if(!res.ok){
    const text = await res.text().catch(()=> "");
    throw new Error(`HTTP ${res.status}: ${text || res.statusText}`);
  }
  const ct = res.headers.get("content-type") || "";
  return ct.includes("application/json") ? res.json() : res.text();
}

// -------- Demo data (works without backend) --------
function seedDemoData(){
  if(!localStorage.getItem(LS.alerts)){
    const demo = [
      { time: nowISO(), level: "info", message: "Console started (demo mode)." },
      { time: new Date(Date.now()- 5*60*1000).toISOString(), level: "warn", message: "Rover heartbeat delayed (simulated)." },
      { time: new Date(Date.now()- 24*60*60*1000).toISOString(), level: "info", message: "Lot occupancy updated." },
    ];
    localStorage.setItem(LS.alerts, JSON.stringify(demo));
  }
  if(!localStorage.getItem(LS.lot)){
    // 50 spots: A01..A25, B01..B25
    const spots = [];
    const statusCycle = ["free","occupied","free","reserved","free"];
    let k=0;
    for(const row of ["A","B"]){
      for(let i=1;i<=25;i++){
        const label = row + String(i).padStart(2,"0");
        spots.push({ id: label, label, status: statusCycle[k++ % statusCycle.length] });
      }
    }
    localStorage.setItem(LS.lot, JSON.stringify(spots));
  }
  if(!localStorage.getItem(LS.res)){
    const demoRes = [
      { id: 101, user: "student@tamiu.edu", spot: "A03", start: new Date(Date.now()-30*60*1000).toISOString(), end: new Date(Date.now()+60*60*1000).toISOString(), status: "active" },
      { id: 102, user: "visitor@gmail.com", spot: "B12", start: new Date(Date.now()-2*60*60*1000).toISOString(), end: new Date(Date.now()-60*60*1000).toISOString(), status: "completed" },
    ];
    localStorage.setItem(LS.res, JSON.stringify(demoRes));
  }
}

function getDemoAlerts(){ return JSON.parse(localStorage.getItem(LS.alerts) || "[]"); }
function getDemoLot(){ return JSON.parse(localStorage.getItem(LS.lot) || "[]"); }
function setDemoLot(spots){ localStorage.setItem(LS.lot, JSON.stringify(spots)); }
function getDemoRes(){ return JSON.parse(localStorage.getItem(LS.res) || "[]"); }
function setDemoRes(rows){ localStorage.setItem(LS.res, JSON.stringify(rows)); }

// -------- Router --------
const routes = {
  "": () => go("dashboard"),
  "login": renderLogin,
  "register": renderRegister,
  "dashboard": renderDashboard,
  "lot": renderLot,
  "reservations": renderReservations,
  "rover": renderRover,
  "alerts": renderAlerts,
  "settings": renderSettings,
};

function currentRoute(){
  const hash = (location.hash || "#/dashboard").replace(/^#\//, "");
  return hash.split("?")[0];
}

function go(route){
  location.hash = "#/" + route;
}

function render(){
  const route = currentRoute();
  setActiveLink(route);

  // Auth gating
  const publicRoutes = new Set(["login","register"]);
  if(!publicRoutes.has(route) && !requireAuth()){
    location.hash = "#/login";
    return;
  }

  const fn = routes[route] || (() => notFound(route));
  fn();
}

function notFound(route){
  const app = document.getElementById("app");
  app.innerHTML = "";
  const div = document.createElement("div");
  div.className = "card";
  div.innerHTML = `<h2>Not found</h2><p class="helper">Route <code>${route}</code> does not exist.</p>`;
  app.appendChild(div);
}

// -------- Shell controls --------
function syncTopbar(){
  const s = getSession();
  const loggedIn = !!(s && s.token);
  const logoutBtn = $("#logoutBtn");
  const settingsBtn = $("#settingsBtn");
  const sessionPill = $("#sessionPill");
  if(loggedIn){
    show(logoutBtn); show(settingsBtn); show(sessionPill);
    sessionPill.textContent = "Online";
    sessionPill.style.borderColor = "rgba(34,197,94,.35)";
  } else {
    hide(logoutBtn); hide(settingsBtn); hide(sessionPill);
  }
}

function setupSidebarToggle(){
  const menuToggle = $("#menuToggle");
  const sidebar = $("#sidebar");
  menuToggle?.addEventListener("click", () => {
    const open = sidebar.classList.toggle("open");
    menuToggle.setAttribute("aria-expanded", open ? "true" : "false");
  });
  // close when clicking a link (mobile)
  $$(".sidebar-link").forEach(a => {
    a.addEventListener("click", () => sidebar.classList.remove("open"));
  });
}

$("#logoutBtn")?.addEventListener("click", () => {
  clearSession();
  syncTopbar();
  go("login");
});

$("#settingsBtn")?.addEventListener("click", () => go("settings"));

// -------- Page renderers --------
function mount(templateId){
  const app = document.getElementById("app");
  app.innerHTML = "";
  app.appendChild(cloneTemplate(templateId));
  syncTopbar();
}

function renderLogin(){
  mount("login-template");
  const msg = $("#loginMsg");

  $("#loginForm")?.addEventListener("submit", async (e) => {
    e.preventDefault();
    msg.textContent = "";
    msg.className = "alert hidden";

    const email = $("#loginEmail").value.trim();
    const password = $("#loginPassword").value;

    // basic validation
    $("#loginEmailErr").textContent = email ? "" : "Email required.";
    $("#loginPasswordErr").textContent = password ? "" : "Password required.";
    if(!email || !password) return;

    try{
      // --- OPTION A: call your Django endpoint ---
      // const data = await apiFetch("/api/auth/login", { method:"POST", body: JSON.stringify({ email, password }) });
      // setSession({ token: data.access, user: { email, name: data.name || email } });

      // --- OPTION B: demo mode (no backend) ---
      setSession({ token: "demo-token", user: { email, name: email.split("@")[0] } });

      syncTopbar();
      go("dashboard");
    }catch(err){
      msg.className = "alert error";
      msg.textContent = err?.message || "Login failed.";
    }
  });
}

function renderRegister(){
  mount("register-template");
  const msg = $("#regMsg");

  $("#regForm")?.addEventListener("submit", async (e) => {
    e.preventDefault();
    msg.textContent = "";
    msg.className = "alert hidden";

    const name = $("#regName").value.trim();
    const email = $("#regEmail").value.trim();
    const password = $("#regPassword").value;

    $("#regNameErr").textContent = name ? "" : "Name required.";
    $("#regEmailErr").textContent = email ? "" : "Email required.";
    $("#regPasswordErr").textContent = password.length >= 8 ? "" : "Password must be 8+ chars.";
    if(!name || !email || password.length < 8) return;

    try{
      // --- Django endpoint (optional) ---
      // await apiFetch("/api/auth/register", { method:"POST", body: JSON.stringify({ name, email, password }) });

      // demo: auto-login
      setSession({ token: "demo-token", user: { email, name } });
      syncTopbar();
      go("dashboard");
    }catch(err){
      msg.className = "alert error";
      msg.textContent = err?.message || "Register failed.";
    }
  });
}

async function renderDashboard(){
  mount("dashboard-template");
  const user = getSession()?.user;
  const usernameEl = document.querySelector("[data-username]");
  if(usernameEl) usernameEl.textContent = user?.name || user?.email || "";

  // Demo summary from localStorage
  const spots = getDemoLot();
  const free = spots.filter(s => s.status === "free").length;
  const occ = spots.filter(s => s.status === "occupied").length;

  $("#statFree").textContent = String(free);
  $("#statOcc").textContent = String(occ);
  $("#lastUpdated").textContent = "Last update: " + new Date().toLocaleString();

  // system/rover summary (demo)
  $("#sysStatus").textContent = "OK";
  $("#sysMeta").textContent = "API base: " + apiBase();
  $("#roverStatus").textContent = "Connected";
  $("#roverMeta").textContent = "Heartbeat < 5s";

  const alerts = getDemoAlerts().slice().sort((a,b)=> (a.time<b.time?1:-1)).slice(0,6);
  const list = $("#alertList");
  list.innerHTML = "";
  for(const a of alerts){
    const li = document.createElement("li");
    li.innerHTML = `<span class="badge">${a.level.toUpperCase()}</span> <span>${escapeHtml(a.message)}</span> <span class="helper">(${new Date(a.time).toLocaleString()})</span>`;
    li.style.marginBottom = "10px";
    list.appendChild(li);
  }
}

async function renderLot(){
  mount("lot-template");
  const grid = $("#lotGrid");
  const msg = $("#lotMsg");
  const filter = $("#lotFilter");

  function draw(spots){
    grid.innerHTML = "";
    const f = filter.value;
    const shown = (f==="all") ? spots : spots.filter(s => s.status === f);
    for(const s of shown){
      const el = document.createElement("div");
      el.className = `spot ${s.status}`;
      el.innerHTML = `<strong>${escapeHtml(s.label)}</strong><small>${escapeHtml(s.status)}</small>`;
      el.addEventListener("click", () => {
        // toggle demo status on click
        const next = s.status === "free" ? "occupied" : (s.status === "occupied" ? "reserved" : "free");
        const all = getDemoLot();
        const idx = all.findIndex(x => x.id === s.id);
        if(idx >= 0){ all[idx].status = next; setDemoLot(all); }
        draw(getDemoLot());
      });
      grid.appendChild(el);
    }
  }

  async function refresh(){
    msg.textContent = "";
    msg.className = "alert hidden";
    try{
      // --- OPTION A: Django ---
      // const spots = await apiFetch("/api/lot/spots");
      // draw(spots);

      // --- demo ---
      draw(getDemoLot());
    }catch(err){
      msg.className = "alert error";
      msg.textContent = err?.message || "Failed to load lot.";
      draw(getDemoLot());
    }
  }

  $("#refreshLot").addEventListener("click", refresh);
  filter.addEventListener("change", () => draw(getDemoLot()));
  refresh();
}

async function renderReservations(){
  mount("reservations-template");

  const tbody = $("#resBody");
  const msg = $("#resMsg");
  const modal = $("#modalBackdrop");
  const formMsg = $("#resFormMsg");

  function openModal(){ modal.classList.remove("hidden"); formMsg.className = "alert hidden"; formMsg.textContent=""; }
  function closeModal(){ modal.classList.add("hidden"); }
  $("#newResBtn").addEventListener("click", openModal);
  $("#closeModal").addEventListener("click", closeModal);
  $("#cancelModal").addEventListener("click", closeModal);
  modal.addEventListener("click", (e)=> { if(e.target === modal) closeModal(); });

  function draw(rows){
    tbody.innerHTML = "";
    for(const r of rows){
      const tr = document.createElement("tr");
      tr.innerHTML = `
        <td>${escapeHtml(String(r.id))}</td>
        <td>${escapeHtml(r.user)}</td>
        <td>${escapeHtml(r.spot)}</td>
        <td>${formatDT(r.start)}</td>
        <td>${formatDT(r.end)}</td>
        <td><span class="badge">${escapeHtml(r.status)}</span></td>
        <td><button class="btn btn-outline" data-del="${escapeHtml(String(r.id))}">Delete</button></td>
      `;
      tbody.appendChild(tr);
    }
    $$("button[data-del]", tbody).forEach(btn => {
      btn.addEventListener("click", async () => {
        const id = btn.getAttribute("data-del");
        try{
          // await apiFetch(`/api/reservations/${id}`, { method:"DELETE" });
          const rows = getDemoRes().filter(x => String(x.id) !== String(id));
          setDemoRes(rows);
          draw(rows);
        }catch(err){
          msg.className = "alert error";
          msg.textContent = err?.message || "Delete failed.";
        }
      });
    });
  }

  async function refresh(){
    msg.textContent = "";
    msg.className = "alert hidden";
    try{
      // const rows = await apiFetch("/api/reservations");
      // draw(rows);
      draw(getDemoRes());
    }catch(err){
      msg.className = "alert error";
      msg.textContent = err?.message || "Failed to load reservations.";
      draw(getDemoRes());
    }
  }

  $("#resForm").addEventListener("submit", async (e) => {
    e.preventDefault();
    formMsg.textContent = "";
    formMsg.className = "alert hidden";

    const user = $("#resUser").value.trim();
    const spot = $("#resSpot").value.trim();
    const start = $("#resStart").value;
    const end = $("#resEnd").value;

    if(!user || !spot || !start || !end){
      formMsg.className = "alert error";
      formMsg.textContent = "All fields required.";
      return;
    }

    try{
      // await apiFetch("/api/reservations", { method:"POST", body: JSON.stringify({ user, spot, start, end }) });
      const rows = getDemoRes();
      const id = Math.max(0, ...rows.map(r => Number(r.id)||0)) + 1;
      rows.unshift({ id, user, spot, start: new Date(start).toISOString(), end: new Date(end).toISOString(), status: "active" });
      setDemoRes(rows);
      draw(rows);
      closeModal();
    }catch(err){
      formMsg.className = "alert error";
      formMsg.textContent = err?.message || "Create failed.";
    }
  });

  refresh();
}

async function renderRover(){
  mount("rover-template");
  const msg = $("#roverMsg");

  async function refresh(){
    msg.textContent = "";
    msg.className = "alert hidden";
    try{
      // const data = await apiFetch("/api/rover/status");
      // demo
      const data = {
        battery_percent: 78,
        location: { x: 12.4, y: 3.1, frame: "lot" },
        heartbeat_iso: nowISO(),
        mode: "patrol",
        last_task: "scan_row_A",
      };

      $("#rvBattery").textContent = data.battery_percent + "%";
      $("#rvLocation").textContent = `(${data.location.x}, ${data.location.y})`;
      $("#rvHeartbeat").textContent = new Date(data.heartbeat_iso).toLocaleString();
      $("#rvTelemetry").textContent = JSON.stringify(data, null, 2);
    }catch(err){
      msg.className = "alert error";
      msg.textContent = err?.message || "Failed to load rover status.";
    }
  }

  $("#refreshRover").addEventListener("click", refresh);
  refresh();
}

async function renderAlerts(){
  mount("alerts-template");
  const tbody = $("#alertsBody");
  const msg = $("#alertsMsg");
  const q = $("#alertSearch");
  const level = $("#alertLevel");
  const sort = $("#alertSort");

  function draw(rows){
    tbody.innerHTML = "";
    for(const a of rows){
      const tr = document.createElement("tr");
      tr.innerHTML = `
        <td>${formatDT(a.time)}</td>
        <td><span class="badge">${escapeHtml(a.level)}</span></td>
        <td>${escapeHtml(a.message)}</td>
      `;
      tbody.appendChild(tr);
    }
  }

  function filtered(rows){
    const text = (q.value || "").toLowerCase().trim();
    const lv = level.value;
    let out = rows.slice();
    if(lv !== "all") out = out.filter(a => a.level === lv);
    if(text) out = out.filter(a => (a.message||"").toLowerCase().includes(text));
    out.sort((a,b)=> sort.value === "oldest" ? (a.time>b.time?1:-1) : (a.time<b.time?1:-1));
    return out;
  }

  async function refresh(){
    msg.textContent = "";
    msg.className = "alert hidden";
    try{
      // const rows = await apiFetch("/api/alerts");
      const rows = getDemoAlerts();
      draw(filtered(rows));
      q.oninput = () => draw(filtered(rows));
      level.onchange = () => draw(filtered(rows));
      sort.onchange = () => draw(filtered(rows));
    }catch(err){
      msg.className = "alert error";
      msg.textContent = err?.message || "Failed to load alerts.";
    }
  }

  $("#refreshAlerts").addEventListener("click", refresh);
  refresh();
}

async function renderSettings(){
  mount("settings-template");
  const msg = $("#settingsMsg");
  const input = $("#apiBase");
  const s = loadSettings();
  input.value = s.apiBase || DEFAULT_API_BASE;

  $("#saveSettings").addEventListener("click", () => {
    const val = input.value.trim() || DEFAULT_API_BASE;
    saveSettings({ ...s, apiBase: val });
    msg.className = "alert success";
    msg.textContent = "Saved.";
    setTimeout(()=> { msg.className = "alert hidden"; msg.textContent=""; }, 1200);
  });

  $("#clearData").addEventListener("click", () => {
    localStorage.removeItem(LS.alerts);
    localStorage.removeItem(LS.lot);
    localStorage.removeItem(LS.res);
    msg.className = "alert";
    msg.textContent = "Cleared demo data. Refresh pages to re-seed.";
  });
}

// -------- helpers --------
function escapeHtml(str){
  return String(str).replace(/[&<>"']/g, s => ({
    "&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"
  }[s]));
}
function formatDT(iso){
  try{ return new Date(iso).toLocaleString(); }
  catch{ return String(iso); }
}

// -------- Boot --------
seedDemoData();
setupSidebarToggle();
syncTopbar();
window.addEventListener("hashchange", render);
window.addEventListener("load", render);
