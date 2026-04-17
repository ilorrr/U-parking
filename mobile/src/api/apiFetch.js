const DEFAULT_API_BASE = "192.168.1.161"; 
// NOT 127.0.0.1 for mobile!

export function apiBase() {
  return DEFAULT_API_BASE.replace(/\/$/, "");
}

export async function apiFetch(path, opts = {}) {
  const url = apiBase() + path;

  const res = await fetch(url, {
    ...opts,
    headers: {
      "Content-Type": "application/json",
      ...(opts.headers || {}),
    },
  });

  if (!res.ok) {
    const text = await res.text().catch(() => "");
    throw new Error(`HTTP ${res.status}: ${text || res.statusText}`);
  }

  return res.json();
}