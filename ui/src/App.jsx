import { useEffect, useState } from "react";
import Layout from "./components/Layout";
import GraphPanel from "./components/GraphPanel";

function Card({ title, children }) {
  return (
    <div
      style={{
        background: "rgba(255,255,255,0.04)",
        border: "1px solid rgba(255,255,255,0.12)",
        borderRadius: 14,
        padding: 14,
        boxShadow: "0 10px 30px rgba(0,0,0,0.25)",
      }}
    >
      <div style={{ fontSize: 14, fontWeight: 650, opacity: 0.9, marginBottom: 10 }}>
        {title}
      </div>
      {children}
    </div>
  );
}

export default function App() {
  const [snap, setSnap] = useState(null);
  const [err, setErr] = useState(null);

  useEffect(() => {
    const load = async () => {
      try {
        const r = await fetch("http://127.0.0.1:8000/snapshot");
        const j = await r.json();
        setSnap(j);
        setErr(null);
      } catch (e) {
        setErr(String(e));
      }
    };
    load();
    const t = setInterval(load, 1000);
    return () => clearInterval(t);
  }, []);

  const ok = snap?.ok;
  const graph = snap?.graph;
  const healthNodes = snap?.health?.nodes || [];

  const nodeNames = (graph?.nodes || []).map((n) => n.name).sort();
  const topicNames = (graph?.topics || []).map((t) => t.name).sort();

  return (
    <Layout title="ROS Supervisor">
      <div style={{ display: "flex", flexDirection: "column", gap: 14 }}>
        <div style={{ fontSize: 20, fontWeight: 700 }}>System Health</div>

        <div style={{ display: "flex", gap: 12, alignItems: "baseline" }}>
          <div style={{ opacity: 0.85 }}>
            Status:{" "}
            <span
              style={{
                fontWeight: 700,
                fontSize: 16,
                color: ok ? "#4ade80" : "#f87171",
              }}
            >
              {snap ? (ok ? "GOOD" : "ERROR") : "LOADING"}
            </span>
          </div>
          <div style={{ opacity: 0.6, fontSize: 13 }}>
            {snap ? `ts=${snap.ts}` : ""}
          </div>
        </div>

        {err && (
          <div style={{ opacity: 0.75, fontSize: 13 }}>
            Fetch error: {err}
          </div>
        )}
        {snap?.error && (
          <div style={{ opacity: 0.9, fontSize: 13, color: "#f87171" }}>
            {snap.error}
          </div>
        )}

        {/* Node health tiles */}
        <div
          style={{
            display: "grid",
            gridTemplateColumns: "repeat(auto-fit, minmax(260px, 1fr))",
            gap: 14,
          }}
        >
          {healthNodes.map((n) => (
            <Card key={n.name} title={n.name}>
              <div style={{ fontSize: 14, opacity: 0.95 }}>
                <div style={{ marginBottom: 4 }}>
                  Publishes: {(n.publishes || []).join(", ") || "—"}
                </div>
                <div>
                  Subscribes: {(n.subscribes || []).join(", ") || "—"}
                </div>

                {n.warnings?.length > 0 && (
                  <div
                    style={{
                      marginTop: 8,
                      fontSize: 13,
                      color: "#fbbf24",
                      opacity: 0.95,
                    }}
                  >
                    {n.warnings.join(" | ")}
                  </div>
                )}
              </div>
            </Card>
          ))}
        </div>

        {/* Below health: Nodes + Topics + QoS + TF */}
        <div
          style={{
            display: "grid",
            gridTemplateColumns: "repeat(auto-fit, minmax(240px, 1fr))",
            gap: 14,
          }}
        >
          <Card title={`Nodes (${nodeNames.length})`}>
            <ul style={{ margin: 0, paddingLeft: 18, fontSize: 14 }}>
              {nodeNames.map((n) => (
                <li key={n}>{n}</li>
              ))}
            </ul>
          </Card>

          <Card title={`Topics (${topicNames.length})`}>
            <ul style={{ margin: 0, paddingLeft: 18, fontSize: 14 }}>
              {topicNames.map((t) => (
                <li key={t}>{t}</li>
              ))}
            </ul>
          </Card>

          <Card title="QoS">
            <div style={{ fontSize: 14, opacity: 0.85 }}>
              Not wired into snapshot yet.
              <div style={{ marginTop: 8, opacity: 0.7, fontSize: 13 }}>
                Next: show mismatched pub/sub QoS per topic.
              </div>
            </div>
          </Card>

          <Card title="TF">
            <div style={{ fontSize: 14, opacity: 0.85 }}>
              Not wired into snapshot yet.
              <div style={{ marginTop: 8, opacity: 0.7, fontSize: 13 }}>
                Next: show missing transforms + disconnected trees.
              </div>
            </div>
          </Card>
        </div>

        {/* Graph full-width */}
        <Card title="ROS Graph">
          <GraphPanel graph={graph} />
        </Card>
      </div>
    </Layout>
  );
}
