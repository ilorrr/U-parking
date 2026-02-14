import { NavLink } from "react-router-dom";

export default function Layout({ children }) {
  return (
    <div className="app-shell">
      {/* FULL-WIDTH HEADER */}
      <header className="app-header">
        <h1 className="app-title">U-Parking</h1>
      </header>

      {/* BODY */}
      <div className="layout">
        <aside className="sidebar">
          <nav className="sidebar-nav">
            <NavLink
              to="/"
              end
              className={({ isActive }) =>
                `sidebar-link ${isActive ? "active" : ""}`
              }
            >
              Dashboard
            </NavLink>

            <NavLink
              to="/lot"
              className={({ isActive }) =>
                `sidebar-link ${isActive ? "active" : ""}`
              }
            >
              Lot
            </NavLink>
          </nav>
        </aside>

        <main className="main">
          <section className="content">{children}</section>
        </main>
      </div>
    </div>
  );
}