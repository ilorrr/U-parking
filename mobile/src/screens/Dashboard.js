import react, { useEffect, useState, useCallback } from "react";
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
  const [metrics, setMetrics] = useState({});

  const refreshAll = useCallback(async () => {
    try {
      const m = await apiFetch("/api/metrics/");
      setMetrics(m);
      const s = await apiFetch("/api/spaces/");
      setSpaces(s);
    } catch (e) {
      console.log(e);
    }
  }, []);

  useEffect(() => {
    refreshAll();
    const interval = setInterval(refreshAll, 2000);
    return () => clearInterval(interval);
  }, [refreshAll]);

  return (
    <ScrollView style={styles.container}>
      <View style={styles.card}>
        <Text style={styles.title}>Parking Lot Map</Text>

        <View style={styles.grid}>
          {spaces.map((s) => {
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
      </View>

      <View style={styles.card}>
        <Text style={styles.title}>Metrics</Text>
        <View style={styles.metricRow}>
          <View style={styles.metric}>
            <Text style={styles.metricLabel}>Total</Text>
            <Text style={styles.metricValue}>{metrics.total}</Text>
          </View>

          <View style={styles.metric}>
            <Text style={styles.metricLabel}>Occupied</Text>
            <Text style={styles.metricValue}>{metrics.occupied}</Text>
          </View>

          <View style={styles.metric}>
            <Text style={styles.metricLabel}>Free</Text>
            <Text style={styles.metricValue}>{metrics.free}</Text>
          </View>
        </View>
      </View>
    </ScrollView>
  );
}