import React from "react";
import { Link } from "react-router-dom";

export default function Home() {
  return (
    <div className="home-page">

      {/* Hero */}
      <div className="home-hero">
        <h1>U-Parking</h1>
        <p>Real-time parking management powered by drone telemetry. Monitor occupancy, track availability, and optimize your lot — all from one place.</p>
      </div>

      {/* Quick-nav cards */}
      <div className="home-cards">

        <Link to="/dashboard" className="home-card">
          <div className="home-card-icon">📊</div>
          <h3>Dashboard</h3>
          <p>Live drone feed, parking map, event logs, and key metrics at a glance.</p>
        </Link>

        <Link to="/lot" className="home-card">
          <div className="home-card-icon">🅿️</div>
          <h3>Lot Overview</h3>
          <p>Full parking lot map with real-time occupancy status for every space.</p>
        </Link>

      </div>

      {/* About / Goals Section */}
      <div className="home-about">

        <div className="home-about-header">
          <h2>About This Project</h2>
          <p>Why we built U-Parking and what we're working toward.</p>
        </div>

        <div className="home-about-grid">

          <div className="home-about-card">
            <div className="home-about-icon">🎯</div>
            <h3>Our Goal</h3>
            <p>To make parking smarter and more efficient by using drone-based computer vision to monitor lot occupancy in real time — eliminating the frustration of searching for a space.</p>
          </div>

          <div className="home-about-card">
            <div className="home-about-icon">🚁</div>
            <h3>How It Works</h3>
            <p>A drone flies over the parking lot and captures live footage. Our system processes the feed using a machine learning model to detect and classify each space as occupied or vacant.</p>
          </div>

          <div className="home-about-card">
            <div className="home-about-icon">📡</div>
            <h3>Live Data</h3>
            <p>Occupancy data is sent to this dashboard in real time, giving operators and drivers an up-to-date view of available spaces without any manual input.</p>
          </div>

          <div className="home-about-card">
            <div className="home-about-icon">🔭</div>
            <h3>Future Plans</h3>
            <p>We plan to expand to multiple lots, add predictive occupancy modeling, and integrate navigation tools to guide drivers directly to open spaces.</p>
          </div>

        </div>
      </div>

    </div>
  );
}
