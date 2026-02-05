import { Routes, Route } from "react-router-dom";
import Layout from "./components/Layout";
import Dashboard from "./pages/Dashboard";
import Lot from "./pages/Lot";
import React from "react";

export default function App() {
  return (
    <Layout>
      <Routes>
        <Route path="/" element={<Dashboard />} />
        <Route path="/lot" element={<Lot />} />
        <Route
          path="*"
          element={
            <div className="card">
              <h2>Not found</h2>
            </div>
          }
        />
      </Routes>
    </Layout>
  );
}