import React from "react";

import { apiFetch } from "../api/apiFetch";

function toUiStatus(spaceValue) {
  if (spaceValue === "Vacant") return "free";
  if (spaceValue === "Occupied") return "occupied";
  return "unknown";
}

export default function Lot() {
  const [spaces, setSpaces] = React.useState([]);
  const [error, setError] = React.useState("");
  const [loading, setLoading] = React.useState(true);

  const load = React.useCallback(async () => {
    setLoading(true);
    setError("");
    try {<i></i>
      const data = await apiFetch("/api/spaces/");
      setSpaces(Array.isArray(data) ? data : []);
    } catch (e) {
      setError(e?.message || "Failed to load spaces");
    } finally {
      setLoading(false);
    }
  }, []);

  React.useEffect(() => { load(); }, [load]);

  return (
    // Parking lot title for the split screen view
    <div className="card">
      <div className="lot-header">
  <h2 className="lot-title">Parking Lot Overview</h2>

  <button
    className="btn btn-outline lot-refresh"
    onClick={load}
    disabled={loading}
  >
    {loading ? "Loading…" : "Refresh"}
  </button></div>

      {error ? (
        <div className="alert error" style={{ marginTop: 12 }}>{error}</div>
      ) : null}

      <div className="lot-split" style={{ marginTop: 12 }}>
        {/* LEFT: Spaces grid */}
        <div className="lot-panel">
          <div className="panel-head">
            <h2 style={{ margin: 0, fontSize: 16 }}>Availability</h2>
            <span className="helper">{spaces.length} total</span>
          </div>

          <div className="lot-grid">
            {spaces.map((s) => (
              <div key={String(s.id)} className={`spot ${toUiStatus(s.space)}`}>
                <strong>Space {s.id}</strong>
                <small>{s.space}</small>
              </div>
            ))}
          </div>
        </div>

        {/* RIGHT: Map */}
        <div className="lot-panel">
          <div className="panel-head">
            <h2 style={{ margin: 0, fontSize: 16 }}>Map</h2></div>

          <div className="map-frame">
            <div className="helper">Map preview (same data)</div>

            {/* Simple “map” using the same spaces list */}
            <div className="mini-grid" style={{ marginTop: 12 }}>
              {spaces.map((s) => (
                <div
                  key={`m-${s.id}`}
                  className={`mini-spot ${toUiStatus(s.space)}`}
                  title={`Space ${s.id}: ${s.space}`}
                />
              ))}
            </div>

            <div className="helper" style={{ marginTop: 12 }}>
              Red or Green for spot availability
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}