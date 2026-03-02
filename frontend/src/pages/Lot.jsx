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
    
  <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', width: '100%' }}>
    
    {/* Title - centered to full page */}
    <h2 className="lot-title" style={{ width: '100%', textAlign: 'center', marginBottom: 8 }}>
      Parking Lot Overview
    </h2>

    {/* Refresh + error - left aligned */}
    <div style={{ width: '100%', textAlign: 'left', marginBottom: 8, marginLeft: '180px' }}>
      <button className="btn btn-outline" onClick={load} disabled={loading}>
        {loading ? "Loading…" : "Refresh"}
      </button>
      {error ? <div className="alert error" style={{ marginTop: 8 }}>{error}</div> : null}
    </div>

    {/* Availability */}
    <div style={{ textAlign: 'center', marginBottom: 8 }}>
      <h2 style={{ margin: 0, fontSize: 16, display: 'inline' }}>Availability</h2>
      <div></div>
      <span className="helper" style={{ marginLeft: 8 }}>{spaces.length} <b>Total</b></span>
    </div>

    {/* Map */}
    <div style={{ textAlign: 'center', width: '100%', maxWidth: 600 }}>
      <h2 style={{ margin: '0 0 8px 0', fontSize: 16 }}>Map</h2>
      <div className="map-frame" style={{ width: '100%' }}>
        <div className="mini-grid" style={{ marginTop: 12 }}>
          {spaces.map((s) => (
            <div key={`m-${s.id}`} className={`mini-spot ${toUiStatus(s.space)}`} title={`Space ${s.id}: ${s.space}`} />
          ))}
        </div>
        <div className="helper" style={{ marginTop: 12 }}>
          Model map will be placed here at a later date
        </div>
      </div>
    </div>

  </div>

    </div>
  );
}