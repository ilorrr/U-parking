import React, { useEffect, useState } from "react";
import { fetchHealth } from "./api/client.js";

function App() {
  const [health, setHealth] = useState(null);

  useEffect(() => {
    fetchHealth().then(setHealth);
  }, []);

  return (
    <div style={{ padding: "2rem" }}>
      <h1>U-Parking Web</h1>
      {health ? (
        <pre>{JSON.stringify(health, null, 2)}</pre>
      ) : (
        <p>Loading backend test...</p>
      )}
    </div>
  );
}

export default App;