// src/api.js

// Use environment variable from frontend .env (REACT_APP_BACKEND_URL)
const BACKEND_URL =
  process.env.REACT_APP_BACKEND_URL || "http://127.0.0.1:8000";

// Full health check URL
export const API_URL = `${BACKEND_URL}/api/health/`;

// Fetch function
export async function fetchHealth() {
  try {
    const res = await fetch(API_URL);
    if (!res.ok) throw new Error(`HTTP error status: ${res.status}`);
    return await res.json();
  } catch (err) {
    console.error("Error fetching health:", err);
    return null;
  }
}