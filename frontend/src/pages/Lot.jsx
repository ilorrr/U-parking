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
    try {
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
    <div className="card">
      <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between", gap: 12 }}>
        <h2 style={{ margin: 0 }}>Parking Spaces</h2>
        <button className="btn btn-outline" onClick={load} disabled={loading}>
          {loading ? "Loading…" : "Refresh"}
        </button>
      </div>

      {error ? <div className="alert error" style={{ marginTop: 12 }}>{error}</div> : null}

      <div className="lot-grid" style={{ marginTop: 12 }}>
        {spaces.map((s) => {
          const status = toUiStatus(s.space);
          return (
            <div key={String(s.id)} className={`spot ${status}`}>
              <strong>Space {s.id}</strong>
              <small>{s.space}</small>
            </div>
          );
        })}
      </div>
    </div>
  );
}