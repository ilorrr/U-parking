import React from "react";
import { View, Text, ScrollView, Pressable } from "react-native";
import { globalStyles as styles } from "../styles/globalStyles";

export default function HomeScreen({ navigation }) {
  return (
    <ScrollView style={styles.container}>

      {/* HERO */}
      <View style={styles.hero}>
        <Text style={styles.heroTitle}>U-Parking</Text>
        <Text style={styles.heroText}>
          Real-time parking management powered by drone telemetry.
          Monitor occupancy, track availability, and optimize your lot — all from one place.
        </Text>
      </View>

      {/* NAV CARDS */}
      <View style={styles.cardNavContainer}>

        <Pressable
          style={styles.navCard}
          onPress={() => navigation.navigate("Dashboard")}
        >
          <Text style={styles.navIcon}>📊</Text>
          <Text style={styles.navTitle}>Dashboard</Text>
          <Text style={styles.navText}>
            Live drone feed, parking map, event logs, and key metrics.
          </Text>
        </Pressable>

        <Pressable
          style={styles.navCard}
          onPress={() => navigation.navigate("Lot")}
        >
          <Text style={styles.navIcon}>🅿️</Text>
          <Text style={styles.navTitle}>Lot Overview</Text>
          <Text style={styles.navText}>
            Full parking lot map with real-time occupancy status.
          </Text>
        </Pressable>

      </View>

      {/* ABOUT */}
      <View style={styles.card}>
        <Text style={styles.title}>About This Project</Text>

        <View style={styles.aboutCard}>
          <Text style={styles.aboutIcon}>🎯</Text>
          <Text style={styles.aboutTitle}>Our Goal</Text>
          <Text style={styles.aboutText}>
            Make parking smarter using drone computer vision to detect open spaces.
          </Text>
        </View>

        <View style={styles.aboutCard}>
          <Text style={styles.aboutIcon}>🚁</Text>
          <Text style={styles.aboutTitle}>How It Works</Text>
          <Text style={styles.aboutText}>
            A drone captures live footage and our AI detects occupied and vacant spaces.
          </Text>
        </View>

        <View style={styles.aboutCard}>
          <Text style={styles.aboutIcon}>📡</Text>
          <Text style={styles.aboutTitle}>Live Data</Text>
          <Text style={styles.aboutText}>
            Occupancy updates stream into the dashboard in real time.
          </Text>
        </View>

        <View style={styles.aboutCard}>
          <Text style={styles.aboutIcon}>🔭</Text>
          <Text style={styles.aboutTitle}>Future Plans</Text>
          <Text style={styles.aboutText}>
            Multi-lot expansion, predictive modeling, and navigation guidance.
          </Text>
        </View>

      </View>

    </ScrollView>
  );
}