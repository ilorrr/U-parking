import React from "react";
import { apiFetch, apiBase } from "../api/apiFetch";




// Backend -> UI status mapping
function toUiStatus(s) {
 if (s?.is_occupied === true) return "occupied";
 if (s?.is_occupied === false) return "free";
 return "unknown";
}




export default function Dashboard() {
 const [spaces, setSpaces] = React.useState([]);
 const [metrics, setMetrics] = React.useState({ total: 0, occupied: 0, free: 0, occupancy_percent: 0 });
 const [error, setError] = React.useState("");
 const [apiStatus, setApiStatus] = React.useState("…");
 const [apiMsg, setApiMsg] = React.useState("");




 const refreshAll = React.useCallback(async () => {
 setError("");
 setApiStatus("…");
 setApiMsg("");




 let ok = true;




 // Metrics
 try {
   const m = await apiFetch("/api/metrics/");
   setMetrics({
     total: Number(m.total || 0),
     occupied: Number(m.occupied || 0),
     free: Number(m.free || 0),
     occupancy_percent: Number(m.occupancy_percent || 0),
     last_telemetry: m.last_telemetry || null,
   });
 } catch (e) {
   ok = false;
   setMetrics({ total: 0, occupied: 0, free: 0, occupancy_percent: 0 });
   const msg = e?.message || "Failed to load metrics";
   setError((prev) => prev || msg);
   setApiMsg((prev) => prev || msg);
 }




 // Spaces
 try {
   const data = await apiFetch("/api/spaces/");
   setSpaces(Array.isArray(data) ? data : []);
 } catch (e) {
   ok = false;
   setSpaces([]);
   const msg = e?.message || "Failed to load spaces";
   setError((prev) => prev || msg);
   setApiMsg((prev) => prev || msg);
 }




 setApiStatus(ok ? "OK" : "BAD");
}, []);




 React.useEffect(() => {
   let alive = true;
   (async () => {
     if (!alive) return;
     await refreshAll();
   })();
   return () => {
     alive = false;
   };
 }, [refreshAll]);
// Prefer backend metrics; fallback to computing from spaces if backend returns 0s
const computedTotal = spaces.length;
const computedOccupied = spaces.filter((s) => toUiStatus(s) === "occupied").length;
const computedFree = spaces.filter((s) => toUiStatus(s) === "free").length;




const total = metrics.total > 0 ? metrics.total : computedTotal;
const occupied = metrics.occupied > 0 ? metrics.occupied : computedOccupied;
const free = metrics.free > 0 ? metrics.free : computedFree;
const occupancyPct = total ? Math.round((occupied / total) * 100) : 0;




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
       
        {/*                                          //THIS IS FOR ALL THE THINGS ABOVE SHOWCASING THE STATUS ON THE DASHBOARD TAB
         <div className="helper">
           API: {apiBase()} • Status:{" "}
           <span className={`pill ${apiStatus === "OK" ? "pill-ok" : "pill-bad"}`}>
             {apiStatus}
           </span>
         </div>
         {apiMsg ? <div className="helper">{apiMsg}</div> : null}
        */}
         {/*
         {metrics?.last_telemetry ? (
           <div className="helper">Last telemetry: {String(metrics.last_telemetry)}</div>
         ) : null}
         {error ? <div className="alert error" style={{ marginTop: 12 }}>{error}</div> : null}
         */}
       </div>




       <div className="dash-actions">
       
       </div>
     </div>




     {/* Grid */}
     <div className="dash-grid">
       {/* Live Video Feed */}
       <div className="card panel panel-video">
         <div className="panel-head">
           <h2>Live Drone Feed</h2>
         </div>




         <div className="video-frame">
           <div className="video-placeholder">
             <div className="helper">Video stream placeholder</div>
           </div>
         </div>
       </div>




       {/* Parking Lot Map */}
       <div className="card panel panel-map">
         <div className="panel-head">
           <h2>Parking Lot Map</h2>
           <span className="helper">{total ? `${occupied}/${total} occupied` : ""}</span>
         </div>




         <div className="map-frame">
           <div className="map-placeholder">
             <div className="helper">Map placeholder</div>




             <div className="mini-grid">
             {spaces.slice(0, 12).map((s) => (
 <div
   key={String(s.id)}
   className={`mini-spot ${toUiStatus(s)}`}
   title={`S${s.section} ${s.label}: ${s.is_occupied ? "Occupied" : "Vacant"}`}
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


      
              
      
       {/* Event Logs */}
      
       <div className="card panel panel-metrics">
         <div className="panel-head">
           <h2>Event Logs</h2>
           <span className="helper">Current</span>
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
      




       {/* Key Metrics */}
       <div className="card panel panel-logs">
         <div className="panel-head">
           <h2>Key Metrics</h2>
           <span className="helper">Latest</span>
         </div>




        
        
        


         <div className="metrics">
           <div className="metric">
             <div className="metric-label">Total Spaces</div>
             <div className="metric-value">{total}</div>
           </div>




           <div className="metric">
             <div className="metric-label">Occupied</div>
             <div className="metric-value">{occupied}</div>
             <div className="metric-sub pill pill-bad">{occupancyPct}%</div>
           </div>




           <div className="metric">
             <div className="metric-label">Free</div>
             <div className="metric-value">{free}</div>
             <div className="metric-sub pill pill-ok">{100 - occupancyPct}% free</div>
           </div>


          
         </div>
        
       </div>
      
     </div>
    
   </div>
 );
}
