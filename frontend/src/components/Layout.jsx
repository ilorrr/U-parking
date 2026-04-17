import { NavLink } from "react-router-dom";
import { useState, useEffect } from "react";

export default function Layout({ children }) {
  const [drawerOpen, setDrawerOpen] = useState(false);

  useEffect(() => {
    const handleKey = (e) => { if (e.key === "Escape") setDrawerOpen(false); };
    window.addEventListener("keydown", handleKey);
    return () => window.removeEventListener("keydown", handleKey);
  }, []);

  useEffect(() => {
    document.body.style.overflow = drawerOpen ? "hidden" : "";
    return () => { document.body.style.overflow = ""; };
  }, [drawerOpen]);

  const navLinks = [
    { to: "/",          end: true,  icon: "🏠", label: "Home"      },
    { to: "/dashboard", end: false, icon: "📊", label: "Dashboard" },
    /*{ to: "/lot",       end: false, icon: "🅿️", label: "Lot"       },*/
  ];

  return (
    <div className="app-shell">

      <header className="app-header">
        <button
          className="hamburger"
          onClick={() => setDrawerOpen(true)}
          aria-label="Open menu"
        >
          <span /><span /><span />
        </button>
        <h1 className="app-title">U-Parking</h1>
      </header>

      <div className="layout">

        {/* Desktop hover-sidebar */}
        <aside className="sidebar desktop-sidebar">
          <nav className="sidebar-nav">
            {navLinks.map(({ to, end, icon, label }) => (
              <NavLink
                key={to}
                to={to}
                end={end}
                className={({ isActive }) => `sidebar-link ${isActive ? "active" : ""}`}
              >
                <span className="sidebar-icon">{icon}</span>
                <span className="sidebar-label">{label}</span>
              </NavLink>
            ))}
          </nav>
        </aside>

        {/* Mobile overlay */}
        {drawerOpen && (
          <div className="drawer-overlay" onClick={() => setDrawerOpen(false)} />
        )}

        {/* Mobile drawer */}
        <aside className={`mobile-drawer ${drawerOpen ? "open" : ""}`}>
          <div className="drawer-header">
            <span className="drawer-title">U-Parking</span>
            <button className="drawer-close" onClick={() => setDrawerOpen(false)} aria-label="Close menu">✕</button>
          </div>
          <nav className="drawer-nav">
            {navLinks.map(({ to, end, icon, label }) => (
              <NavLink
                key={to}
                to={to}
                end={end}
                onClick={() => setDrawerOpen(false)}
                className={({ isActive }) => `drawer-link ${isActive ? "active" : ""}`}
              >
                <span className="drawer-icon">{icon}</span>
                <span>{label}</span>
              </NavLink>
            ))}
          </nav>
        </aside>

        <main className="main">
          <section className="content">{children}</section>
        </main>

      </div>
    </div>
  );
}
