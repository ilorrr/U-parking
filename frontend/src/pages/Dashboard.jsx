import React from "react";
import { apiFetch, apiBase } from "../api/apiFetch";

function toUiStatus(spaceValue) {
  if (spaceValue === "Vacant") return "free";
  if (spaceValue === "Occupied") return "occupied";
  return "unknown";
}

export default function Dashboard() {
  const [apiStatus, setApiStatus] = React.useState("CHECKING…");
  const [apiMsg, setApiMsg] = React.useState("");
  const [spaces, setSpaces] = React.useState([]);
  const [error, setError] = React.useState("");

  // Health + spaces
  React.useEffect(() => {
    let alive = true;

    (async () => {
      try {
        const health = await apiFetch("/");
        if (!alive) return;
        setApiStatus(String(health.status || "ok").toUpperCase());
        setApiMsg(String(health.message || ""));
      } catch (e) {
        if (!alive) return;
        setApiStatus("OFFLINE");
        setApiMsg("");
        setError(e?.message || "Cannot reach backend");
      }

      try {
        const data = await apiFetch("/api/spaces/");
        if (!alive) return;
        setSpaces(Array.isArray(data) ? data : []);
      } catch (e) {
        if (!alive) return;
        setError(e?.message || "Failed to load spaces");
      }
    })();

    return () => {
      alive = false;
    };
  }, []);

  const total = spaces.length;
  const occupied = spaces.filter((s) => toUiStatus(s.space) === "occupied").length;
  const vacant = spaces.filter((s) => toUiStatus(s.space) === "free").length;
  const occupancyPct = total ? Math.round((occupied / total) * 100) : 0;

  // Placeholder events (replace later with /api/events or websocket)
  const events = [
    { time: "Just now", level: "INFO", msg: "Dashboard loaded." },
    { time: "1m ago", level: "INFO", msg: "Connected to API." },
    { time: "5m ago", level: "WARN", msg: "Simulation feed stable." },
  ];

  return (
    <div className="dash">
      {/* Header */}
      <div className="dash-header">
        <div>
          <div className="dash-title">U-Parking Dashboard</div>
          <div className="helper">
            API: {apiBase()} • Status: <span className={`pill ${apiStatus === "OK" ? "pill-ok" : "pill-bad"}`}>{apiStatus}</span>
          </div>
          {apiMsg ? <div className="helper">{apiMsg}</div> : null}
          {error ? <div className="alert error" style={{ marginTop: 12 }}>{error}</div> : null}
        </div>

        <div className="dash-actions">
          <button
            className="btn btn-outline"
            onClick={async () => {
              setError("");
              try {
                const data = await apiFetch("/api/spaces/");
                setSpaces(Array.isArray(data) ? data : []);
              } catch (e) {
                setError(e?.message || "Refresh failed");
              }
            }}
          >
            Refresh
          </button>
        </div>
      </div>

      {/* Grid */}
      <div className="dash-grid">
        {/* Live Video Feed */}
        <div className="card panel panel-video">
          <div className="panel-head">
            <h2>Live Video Feed</h2>
            <span className="helper">Camera 1</span>
          </div>

          <div className="video-frame">
            {/* Replace this placeholder with <video> or <img> stream */}
            <div className="video-placeholder">
              <div className="video-badge">LIVE</div>
              <div className="helper">Video stream placeholder</div>
            </div>
          </div>
        </div>

        {/* Parking Lot Map */}
        <div className="card panel panel-map">
          <div className="panel-head">
            <h2>Parking Lot Map</h2>
            <span className="helper">Overview</span>
          </div>

          <div className="map-frame">
            {/* Placeholder map. Later: draw a real map, or a canvas/SVG */}
            <div className="map-placeholder">
              <div className="helper">Map placeholder</div>
              <div className="mini-grid">
                {spaces.slice(0, 12).map((s) => (
                  <div
                    key={String(s.id)}
                    className={`mini-spot ${toUiStatus(s.space)}`}
                    title={`Space ${s.id}: ${s.space}`}
                  />
                ))}
              </div>
              {spaces.length > 12 ? (
                <div className="helper" style={{ marginTop: 10 }}>
                  Showing 12 / {spaces.length} spaces
                </div>
              ) : null}
            </div>
          </div>
        </div>

        {/* Key Metrics */}
        <div className="card panel panel-metrics">
          <div className="panel-head">
            <h2>Key Metrics</h2>
            <span className="helper">Current</span>
          </div>

          <div className="metrics">
            <div className="metric">
              <div className="metric-label">Total Spaces</div>
              <div className="metric-value">{total}</div>
            </div>

            <div className="metric">
              <div className="metric-label">Occupied</div>
              <div className="metric-value">{occupied}</div>
              <div className="metric-sub pill pill-bad">+{occupancyPct}%</div>
            </div>

            <div className="metric">
              <div className="metric-label">Vacant</div>
              <div className="metric-value">{vacant}</div>
              <div className="metric-sub pill pill-ok">{100 - occupancyPct}% free</div>
            </div>
          </div>
        </div>

        {/* Event Logs */}
        <div className="card panel panel-logs">
          <div className="panel-head">
            <h2>Event Logs</h2>
            <span className="helper">Latest</span>
          </div>

          <div className="logs">
            {events.map((e, idx) => (
              <div className="log-row" key={idx}>
                <div className={`log-level ${e.level.toLowerCase()}`}>{e.level}</div>
                <div className="log-msg">{e.msg}</div>
                <div className="log-time helper">{e.time}</div>
              </div>
            ))}
          </div>

          <div style={{ marginTop: 10 }}>
            <button className="btn btn-outline" disabled>
              View all (hook to API later)
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}