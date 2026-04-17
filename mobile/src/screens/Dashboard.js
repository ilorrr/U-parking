import React from "react";
import {
  View,
  Text,
  ScrollView,
  RefreshControl,
} from "react-native";
import { globalStyles as styles } from "../styles/globalStyles";

const API_BASE = "http://128.0.0.1:8000"; // change to your backend IP

function toUiStatus(s) {
  if (s?.is_occupied === true) return "occupied";
  if (s?.is_occupied === false) return "free";
  return "unknown";
}

export default function DashboardScreen() {
  const [spaces, setSpaces] = React.useState([]);
  const [metrics, setMetrics] = React.useState({
    total: 0,
    occupied: 0,
    free: 0,
    occupancy_percent: 0,
  });
  const [refreshing, setRefreshing] = React.useState(false);

  const fetchData = async () => {
    try {
      const m = await fetch(`${API_BASE}/api/metrics/`);
      const metricsData = await m.json();

      const s = await fetch(`${API_BASE}/api/spaces/`);
      const spacesData = await s.json();

      setMetrics(metricsData);
      setSpaces(spacesData);
    } catch (err) {
      console.log("API error:", err);
    }
  };

  React.useEffect(() => {
    fetchData();

    const interval = setInterval(fetchData, 5000); // auto refresh
    return () => clearInterval(interval);
  }, []);

  const onRefresh = async () => {
    setRefreshing(true);
    await fetchData();
    setRefreshing(false);
  };

  return (
    <ScrollView
      style={styles.container}
      refreshControl={
        <RefreshControl refreshing={refreshing} onRefresh={onRefresh} />
      }
    >
      {/* METRICS */}
      <View style={styles.metricsContainer}>
        <View style={styles.metricCard}>
          <Text style={styles.metricTitle}>Total Spaces</Text>
          <Text style={styles.metricValue}>{metrics.total}</Text>
        </View>

        <View style={styles.metricCard}>
          <Text style={styles.metricTitle}>Occupied</Text>
          <Text style={styles.metricValue}>{metrics.occupied}</Text>
        </View>

        <View style={styles.metricCard}>
          <Text style={styles.metricTitle}>Free</Text>
          <Text style={styles.metricValue}>{metrics.free}</Text>
        </View>
      </View>

      {/* PARKING GRID PREVIEW */}
      <View style={styles.card}>
        <Text style={styles.title}>Parking Lot Status</Text>

        <View style={styles.grid}>
          {spaces.slice(0, 20).map((s) => {
            const status = toUiStatus(s);

            return (
              <View
                key={s.id}
                style={[
                  styles.spot,
                  status === "occupied" && styles.spotOccupied,
                  status === "free" && styles.spotFree,
                ]}
              />
            );
          })}
        </View>

        <Text style={styles.helper}>
          Showing {Math.min(spaces.length, 20)} of {spaces.length} spaces
        </Text>
      </View>

      {/* VIDEO PLACEHOLDER */}
      <View style={styles.card}>
        <Text style={styles.title}>Drone Feed</Text>

        <View style={styles.videoPlaceholder}>
          <Text style={styles.helper}>Live video coming soon</Text>
        </View>
      </View>
    </ScrollView>
  );
}