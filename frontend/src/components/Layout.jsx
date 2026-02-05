//for resuable userinterface react
import React from "react";
import { NavLink } from "react-router-dom";

export default function Layout({ children }) {
  return (
    <div className="layout">
      <aside className="sidebar" id="sidebar">
        <h1 className="sidebar-title">U-Parking</h1>
        <nav className="sidebar-nav">
          <NavLink to="/" end className={({ isActive }) => `sidebar-link ${isActive ? "active" : ""}`}>
            Dashboard
          </NavLink>
          <NavLink to="/lot" className={({ isActive }) => `sidebar-link ${isActive ? "active" : ""}`}>
            Lot
          </NavLink>
        </nav>
      </aside>

      <main className="main">
        <header className="topbar">
          <div className="topbar-title">Console</div>
        </header>
        <section className="content">{children}</section>
      </main>
    </div>
  );
}