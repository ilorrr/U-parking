import React, { useEffect, useState, useCallback } from "react";
import { View, Text, ScrollView } from "react-native";
import { globalStyles as styles } from "../styles/globalStyles";
import { apiFetch } from "../api/apiFetch";

function toUiStatus(s) {
  if (s?.is_occupied === true) return "occupied";
  if (s?.is_occupied === false) return "free";
  return "unknown";
}

export default function DashboardScreen() {

  const [spaces, setSpaces] = useState([]);
  const [metrics, setMetrics] = useState({
    total: 0,
    occupied: 0,
    free: 0,
    occupancy_percent: 0
  });

  const [error, setError] = useState("");

  const refreshAll = useCallback(async () => {

    try {
      const m = await apiFetch("/api/metrics/");
      setMetrics(m);
    } catch (e) {
      setError("Metrics failed");
    }

    try {
      const s = await apiFetch("/api/spaces/");
      setSpaces(s);
    } catch (e) {
      setError("Spaces failed");
    }

  }, []);

  useEffect(() => {
    refreshAll();
    const interval = setInterval(refreshAll, 2000);
    return () => clearInterval(interval);
  }, [refreshAll]);

  // fallback metrics if backend returns 0
  const computedTotal = spaces.length;
  const computedOccupied = spaces.filter(s => toUiStatus(s) === "occupied").length;
  const computedFree = spaces.filter(s => toUiStatus(s) === "free").length;

  const total = metrics.total || computedTotal;
  const occupied = metrics.occupied || computedOccupied;
  const free = metrics.free || computedFree;
  const occupancyPct = total ? Math.round((occupied / total) * 100) : 0;

  const events = [
    { time: "Just now", level: "INFO", msg: "Dashboard loaded" },
    { time: "1m ago", level: "INFO", msg: "Connected to API" },
    { time: "5m ago", level: "WARN", msg: "Simulation stable" }
  ];

  return (
    <ScrollView style={styles.container}>

      {/* VIDEO PANEL */}
      <View style={styles.card}>
        <Text style={styles.title}>Live Drone Feed</Text>

        <View style={styles.videoBox}>
          <Text style={styles.helper}>Video Stream Placeholder</Text>
        </View>
      </View>

      {/* PARKING MAP */}
      <View style={styles.card}>
        <Text style={styles.title}>
          Parking Lot Map ({occupied}/{total})
        </Text>

        <View style={styles.grid}>
          {spaces.slice(0, 12).map((s) => {
            const status = toUiStatus(s);

            return (
              <View
                key={s.id}
                style={[
                  styles.spot,
                  status === "occupied" && styles.occupied,
                  status === "free" && styles.free
                ]}
              >
                <Text style={styles.spotLabel}>{s.label}</Text>
              </View>
            );
          })}
        </View>

        {spaces.length > 12 && (
          <Text style={styles.helper}>
            Showing 12 / {spaces.length}
          </Text>
        )}
      </View>

      {/* EVENT LOGS */}
      <View style={styles.card}>
        <Text style={styles.title}>Event Logs</Text>

        {events.map((e, i) => (
          <View key={i} style={{ marginBottom: 8 }}>
            <Text>
              <Text style={{ fontWeight: "bold" }}>{e.level}</Text> — {e.msg}
            </Text>
            <Text style={styles.helper}>{e.time}</Text>
          </View>
        ))}
      </View>

      {/* METRICS */}
      <View style={styles.card}>
        <Text style={styles.title}>Key Metrics</Text>

        <View style={styles.metricRow}>
          <View style={styles.metric}>
            <Text style={styles.metricLabel}>Total</Text>
            <Text style={styles.metricValue}>{total}</Text>
          </View>

          <View style={styles.metric}>
            <Text style={styles.metricLabel}>Occupied</Text>
            <Text style={styles.metricValue}>{occupied}</Text>
            <Text style={styles.metricSub}>{occupancyPct}%</Text>
          </View>

          <View style={styles.metric}>
            <Text style={styles.metricLabel}>Free</Text>
            <Text style={styles.metricValue}>{free}</Text>
            <Text style={styles.metricSub}>{100 - occupancyPct}%</Text>
          </View>
        </View>
      </View>

      {error && <Text style={styles.error}>{error}</Text>}

    </ScrollView>
  );
}