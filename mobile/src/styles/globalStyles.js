import { StyleSheet, Platform } from "react-native";




// ─── Design Tokens ───────────────────────────────────────────────────────────
const C = {
 // Brand
 accent:       "#430506",   // matches web app
 accentLight:  "#FEE2E2",
 accentDark:   "#2D0304",




 // Status
 occupied:     "#EF4444",
 occupiedBg:   "#FEF2F2",
 free:         "#10B981",
 freeBg:       "#ECFDF5",
 unknown:      "#94A3B8",
 unknownBg:    "#F1F5F9",




 // Neutrals
 bg:           "#F0F4F8",   // cool off-white
 surface:      "#FFFFFF",
 border:       "#E2E8F0",
 borderStrong: "#CBD5E1",




 // Text
 textPrimary:  "#0F172A",
 textSecondary:"#475569",
 textMuted:    "#94A3B8",




 // Shadows
 shadowColor:  "#0F172A",
};




const RADIUS = {
 sm:  8,
 md:  14,
 lg:  20,
 xl:  28,
};




const SHADOW = {
 sm: Platform.select({
   ios:     { shadowColor: C.shadowColor, shadowOpacity: 0.06, shadowRadius: 6,  shadowOffset: { width: 0, height: 2 } },
   android: { elevation: 2 },
 }),
 md: Platform.select({
   ios:     { shadowColor: C.shadowColor, shadowOpacity: 0.10, shadowRadius: 12, shadowOffset: { width: 0, height: 4 } },
   android: { elevation: 4 },
 }),
 lg: Platform.select({
   ios:     { shadowColor: C.shadowColor, shadowOpacity: 0.14, shadowRadius: 20, shadowOffset: { width: 0, height: 8 } },
   android: { elevation: 8 },
 }),
};




// ─── Styles ──────────────────────────────────────────────────────────────────
export const globalStyles = StyleSheet.create({




 // ── Layout ──────────────────────────────────────────────────────────────────




 container: {
   flex: 1,
   backgroundColor: C.bg,
   paddingHorizontal: 18,
   paddingTop: 48,
 },




 // ── Cards ───────────────────────────────────────────────────────────────────




 card: {
   backgroundColor: C.surface,
   borderRadius: RADIUS.lg,
   padding: 20,
   marginBottom: 16,
   borderWidth: 1,
   borderColor: C.border,
   ...SHADOW.md,
 },




 title: {
   fontSize: 17,
   fontWeight: "700",
   color: C.textPrimary,
   marginBottom: 14,
   letterSpacing: -0.3,
 },




 helper: {
   fontSize: 13,
   color: C.textMuted,
   lineHeight: 18,
 },




 error: {
   textAlign: "center",
   color: C.occupied,
   marginTop: 10,
   fontSize: 13,
 },




 // ── Home Hero ────────────────────────────────────────────────────────────────




 hero: {
   backgroundColor: C.accent,
   borderRadius: RADIUS.xl,
   padding: 28,
   marginBottom: 20,
   ...SHADOW.lg,
 },




 heroTitle: {
   fontSize: 36,
   fontWeight: "800",
   color: "#FFFFFF",
   letterSpacing: -1,
   marginBottom: 10,
 },




 heroText: {
   fontSize: 15,
   color: "rgba(255,255,255,0.82)",
   lineHeight: 22,
 },




 // ── Nav Cards (Home) ─────────────────────────────────────────────────────────




 cardNavContainer: {
   flexDirection: "row",
   justifyContent: "space-between",
   marginBottom: 20,
   gap: 12,
 },




 navCard: {
   backgroundColor: C.surface,
   borderRadius: RADIUS.lg,
   padding: 20,
   flex: 1,
   borderWidth: 1,
   borderColor: C.border,
   ...SHADOW.md,
 },




 navIcon: {
   fontSize: 30,
   marginBottom: 10,
 },




 navTitle: {
   fontSize: 15,
   fontWeight: "700",
   color: C.textPrimary,
   marginBottom: 6,
 },




 navText: {
   fontSize: 13,
   color: C.textSecondary,
   lineHeight: 18,
 },




 // ── About Cards (Home) ───────────────────────────────────────────────────────




 aboutCard: {
   backgroundColor: C.bg,
   borderRadius: RADIUS.md,
   padding: 16,
   marginTop: 10,
   borderWidth: 1,
   borderColor: C.border,
 },




 aboutIcon: {
   fontSize: 24,
   marginBottom: 6,
 },




 aboutTitle: {
   fontWeight: "700",
   fontSize: 14,
   color: C.textPrimary,
   marginBottom: 4,
 },




 aboutText: {
   fontSize: 13,
   color: C.textSecondary,
   lineHeight: 19,
 },




 // ── Metrics Row ──────────────────────────────────────────────────────────────




 metricsContainer: {
   flexDirection: "row",
   gap: 10,
   marginBottom: 16,
 },




 metricCard: {
   flex: 1,
   backgroundColor: C.surface,
   borderRadius: RADIUS.md,
   padding: 16,
   alignItems: "center",
   borderWidth: 1,
   borderColor: C.border,
   ...SHADOW.sm,
 },




 metricTitle: {
   fontSize: 11,
   fontWeight: "600",
   color: C.textMuted,
   textTransform: "uppercase",
   letterSpacing: 0.6,
   marginBottom: 6,
 },




 metricValue: {
   fontSize: 28,
   fontWeight: "800",
   color: C.textPrimary,
   letterSpacing: -0.5,
 },




 metricLabel: {
   fontSize: 11,
   color: C.textMuted,
   marginBottom: 4,
   textTransform: "uppercase",
   letterSpacing: 0.5,
 },




 metricSub: {
   fontSize: 12,
   color: C.textMuted,
   marginTop: 2,
 },




 metricRow: {
   flexDirection: "row",
   justifyContent: "space-between",
   marginTop: 6,
 },




 metric: {
   alignItems: "center",
   flex: 1,
 },




 // ── Parking Grid ─────────────────────────────────────────────────────────────




 grid: {
   flexDirection: "row",
   flexWrap: "wrap",
   gap: 8,
   marginBottom: 12,
 },




 spot: {
   width: 56,
   height: 56,
   borderRadius: RADIUS.sm,
   justifyContent: "center",
   alignItems: "center",
   backgroundColor: C.unknownBg,
   borderWidth: 1.5,
   borderColor: C.border,
 },




 spotOccupied: {
   backgroundColor: C.occupiedBg,
   borderColor: C.occupied,
 },




 spotFree: {
   backgroundColor: C.freeBg,
   borderColor: C.free,
 },




 spotLabel: {
   color: C.textPrimary,
   fontWeight: "700",
   fontSize: 13,
 },




 occupied: {
   backgroundColor: C.occupiedBg,
   borderColor: C.occupied,
 },




 free: {
   backgroundColor: C.freeBg,
   borderColor: C.free,
 },




 unknown: {
   backgroundColor: C.unknownBg,
   borderColor: C.border,
 },




 // ── Video Panel ──────────────────────────────────────────────────────────────




 videoBox: {
   height: 200,
   backgroundColor: C.bg,
   borderRadius: RADIUS.md,
   justifyContent: "center",
   alignItems: "center",
   borderWidth: 1.5,
   borderColor: C.border,
   borderStyle: "dashed",
 },




 videoPlaceholder: {
   height: 190,
   backgroundColor: "#0F172A",
   borderRadius: RADIUS.md,
   justifyContent: "center",
   alignItems: "center",
   overflow: "hidden",
 },




 // ── Event Logs ───────────────────────────────────────────────────────────────




 logRow: {
   marginBottom: 12,
   paddingBottom: 12,
   borderBottomWidth: 1,
   borderBottomColor: C.border,
 },




 logLevel: {
   fontWeight: "700",
   fontSize: 11,
   color: C.accent,
   textTransform: "uppercase",
   letterSpacing: 0.8,
   marginBottom: 2,
 },




 logMsg: {
   fontSize: 14,
   color: C.textPrimary,
   lineHeight: 20,
 },




 logTime: {
   fontSize: 12,
   color: C.textMuted,
   marginTop: 2,
 },




 // ── Lot Overview ─────────────────────────────────────────────────────────────




 lotTitle: {
   fontSize: 22,
   fontWeight: "800",
   color: C.textPrimary,
   letterSpacing: -0.5,
   marginBottom: 4,
 },




 lotSubtitle: {
   fontSize: 14,
   color: C.textSecondary,
   marginBottom: 16,
 },




 lotGrid: {
   flexDirection: "row",
   flexWrap: "wrap",
   gap: 8,
 },




 lotSpot: {
   width: 52,
   height: 52,
   borderRadius: RADIUS.sm,
   justifyContent: "center",
   alignItems: "center",
   borderWidth: 1.5,
 },




 lotSpotLabel: {
   fontSize: 11,
   fontWeight: "700",
   color: C.textPrimary,
 },




 // ── Legend ───────────────────────────────────────────────────────────────────




 legend: {
   flexDirection: "row",
   gap: 16,
   marginTop: 14,
   paddingTop: 14,
   borderTopWidth: 1,
   borderTopColor: C.border,
 },




 legendItem: {
   flexDirection: "row",
   alignItems: "center",
   gap: 6,
 },




 legendDot: {
   width: 10,
   height: 10,
   borderRadius: 5,
 },




 legendLabel: {
   fontSize: 12,
   color: C.textSecondary,
 },




 // ── Buttons ──────────────────────────────────────────────────────────────────




 btn: {
   paddingVertical: 10,
   paddingHorizontal: 18,
   borderRadius: RADIUS.sm,
   borderWidth: 1.5,
   borderColor: C.border,
   backgroundColor: C.surface,
   alignItems: "center",
 },




 btnText: {
   fontSize: 14,
   fontWeight: "600",
   color: C.textPrimary,
 },




 btnPrimary: {
   backgroundColor: C.accent,
   borderColor: C.accent,
 },




 btnPrimaryText: {
   color: "#FFFFFF",
   fontSize: 14,
   fontWeight: "600",
 },




});




// Export tokens for use in component-level styles
export { C, RADIUS, SHADOW };
