import { StyleSheet } from "react-native";

export const globalStyles = StyleSheet.create({


  /* Homescreen styles*/

  hero: {
  marginBottom: 20
},

heroTitle: {
  fontSize: 34,
  fontWeight: "800",
  marginBottom: 8
},

heroText: {
  fontSize: 16,
  color: "#475569"
},

cardNavContainer: {
  flexDirection: "row",
  justifyContent: "space-between",
  marginBottom: 20
},

navCard: {
  backgroundColor: "white",
  borderRadius: 14,
  padding: 18,
  width: "48%",
  elevation: 3
},

navIcon: {
  fontSize: 28,
  marginBottom: 6
},

navTitle: {
  fontSize: 16,
  fontWeight: "700"
},

navText: {
  fontSize: 13,
  color: "#64748b",
  marginTop: 4
},

aboutCard: {
  marginTop: 12
},

aboutIcon: {
  fontSize: 22
},

aboutTitle: {
  fontWeight: "700",
  marginTop: 4
},

aboutText: {
  color: "#64748b"
},

  /* ===== Dashboard screen ===== */

  container: {
    flex: 1,
    backgroundColor: "#f4f6f8",
    paddingHorizontal: 16,
    paddingTop: 40
  },

  /* ===== CARDS / PANELS ===== */

  card: {
    backgroundColor: "#ffffff",
    borderRadius: 14,
    padding: 16,
    marginBottom: 16,
    shadowColor: "#000",
    shadowOpacity: 0.08,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 4 },
    elevation: 3
  },

  title: {
    fontSize: 18,
    fontWeight: "700",
    marginBottom: 12,
    color: "#1e293b"
  },

  helper: {
    fontSize: 13,
    color: "#64748b"
  },

  error: {
    textAlign: "center",
    color: "#ef4444",
    marginTop: 10
  },

  /* ===== VIDEO PANEL ===== */

  videoBox: {
    height: 190,
    backgroundColor: "#e5e7eb",
    borderRadius: 12,
    justifyContent: "center",
    alignItems: "center"
  },

  /* ===== PARKING GRID ===== */

  grid: {
    flexDirection: "row",
    flexWrap: "wrap",
    justifyContent: "flex-start"
  },

  spot: {
    width: 64,
    height: 64,
    borderRadius: 10,
    margin: 6,
    justifyContent: "center",
    alignItems: "center",
    backgroundColor: "#cbd5e1"
  },

  occupied: {
    backgroundColor: "#ef4444"
  },

  free: {
    backgroundColor: "#22c55e"
  },

  unknown: {
    backgroundColor: "#94a3b8"
  },

  spotLabel: {
    color: "#ffffff",
    fontWeight: "700",
    fontSize: 15
  },

  /* ===== EVENT LOGS ===== */

  logRow: {
    marginBottom: 10
  },

  logLevel: {
    fontWeight: "700",
    color: "#0f172a"
  },

  logMsg: {
    fontSize: 14,
    color: "#334155"
  },

  logTime: {
    fontSize: 12,
    color: "#94a3b8"
  },

  /* ===== METRICS ===== */

  metricRow: {
    flexDirection: "row",
    justifyContent: "space-between",
    marginTop: 6
  },

  metric: {
    alignItems: "center",
    flex: 1
  },

  metricLabel: {
    fontSize: 13,
    color: "#64748b",
    marginBottom: 4
  },

  metricValue: {
    fontSize: 26,
    fontWeight: "800",
    color: "#0f172a"
  },

  metricSub: {
    fontSize: 12,
    color: "#94a3b8",
    marginTop: 2
  }

});