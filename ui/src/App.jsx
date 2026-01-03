import { useEffect, useState } from "react";
import Layout from "./components/Layout";

const SNAPSHOT_URL = "http://127.0.0.1:8000/snapshot";
const REFRESH_MS = 500; // matches daemon period

export default function App() {
  const [snapshot, setSnapshot] = useState(null);
  const [error, setError] = useState(null);

  useEffect(() => {
    let alive = true;
    let inFlight = false;

    async function tick() {
      if (inFlight) return;
      inFlight = true;
      try {
        const res = await fetch(SNAPSHOT_URL);
        if (!res.ok) throw new Error(res.statusText);
        const data = await res.json();
        if (alive) setSnapshot(data);
      } catch (e) {
        if (alive) setError(e.toString());
      } finally {
        inFlight = false;
      }
    }

    tick();
    const id = setInterval(tick, REFRESH_MS);

    return () => {
      alive = false;
      clearInterval(id);
    };
  }, []);

  if (error) {
    return (
      <Layout>
        <pre>Error: {error}</pre>
      </Layout>
    );
  }

  if (!snapshot) {
    return (
      <Layout>
        <pre>Loading snapshot...</pre>
      </Layout>
    );
  }

  const { ok, ts, health, graph } = snapshot;

  return (
    <Layout>
      <h2>System Health</h2>

      <div style={{ marginBottom: 12 }}>
        Status:{" "}
        <strong style={{ color: ok ? "#22c55e" : "#ef4444" }}>
          {ok ? "GOOD" : "ERROR"}
        </strong>
        <span style={{ marginLeft: 12, opacity: 0.7 }}>
          ts={ts?.toFixed(3)}
        </span>
      </div>

      {health?.nodes?.map((n) => (
        <div
          key={n.name}
          style={{
            background: "#161a22",
            border: "1px solid #222",
            borderRadius: 8,
            padding: 12,
            marginBottom: 12,
          }}
        >
          <strong>{n.name}</strong>

          <div>Publishes: {n.publishes?.join(", ") || "—"}</div>
          <div>Subscribes: {n.subscribes?.join(", ") || "—"}</div>

          {n.warnings?.length > 0 && (
            <div style={{ color: "#f59e0b", marginTop: 6 }}>
              Warnings:
              <ul>
                {n.warnings.map((w, i) => (
                  <li key={i}>{w}</li>
                ))}
              </ul>
            </div>
          )}
        </div>
      ))}

      <h2>ROS Graph Snapshot</h2>

      <div style={{ display: "flex", gap: 16 }}>
        <div style={{ flex: 1 }}>
          <strong>Nodes ({graph?.nodes?.length || 0})</strong>
          <ul>
            {graph?.nodes?.map((n) => (
              <li key={n.name}>{n.name}</li>
            ))}
          </ul>
        </div>

        <div style={{ flex: 1 }}>
          <strong>Topics ({graph?.topics?.length || 0})</strong>
          <ul>
            {graph?.topics?.map((t) => (
              <li key={t.name}>
                {t.name}
                <div style={{ fontSize: 12, opacity: 0.7 }}>
                  {t.types?.join(", ")}
                </div>
              </li>
            ))}
          </ul>
        </div>
      </div>
    </Layout>
  );
}
