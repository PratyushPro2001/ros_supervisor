import { useEffect, useMemo, useState } from "react";
import Layout from "./components/Layout";

function StatCard({ label, value, color }) {
  return (
    <div style={{
      background: "#161a22",
      border: "1px solid #222",
      borderRadius: 12,
      padding: 16,
      minWidth: 160
    }}>
      <div style={{ opacity: 0.7, fontSize: 12, letterSpacing: 0.6 }}>{label}</div>
      <div style={{ fontSize: 28, fontWeight: 800, marginTop: 6, color: color || "#e5e7eb" }}>
        {value}
      </div>
    </div>
  );
}

function StatusBadge({ status }) {
  const s = String(status || "UNKNOWN").toUpperCase();
  const bg = s === "GOOD" ? "#123d2a" : s === "WARN" ? "#3b2f08" : s === "ERROR" ? "#3b0a0a" : "#222";
  const fg = s === "GOOD" ? "#34d399" : s === "WARN" ? "#fbbf24" : s === "ERROR" ? "#f87171" : "#e5e7eb";
  return (
    <span style={{
      marginLeft: 10,
      padding: "2px 10px",
      borderRadius: 999,
      background: bg,
      color: fg,
      fontWeight: 700,
      fontSize: 12,
      letterSpacing: 0.5
    }}>
      {s}
    </span>
  );
}

export default function App() {
  const [snap, setSnap] = useState(null);
  const [err, setErr] = useState(null);

  useEffect(() => {
    let alive = true;

    async function tick() {
      try {
        const r = await fetch("http://127.0.0.1:8000/snapshot");
        const j = await r.json();
        if (!alive) return;
        setSnap(j);
        setErr(j.ok ? null : (j.error || "snapshot not ok"));
      } catch (e) {
        if (!alive) return;
        setErr(String(e));
      }
    }

    tick();
    const id = setInterval(tick, 500); // MVP: 500ms
    return () => { alive = false; clearInterval(id); };
  }, []);

  const health = snap?.health;
  const graph = snap?.graph;

  const summary = useMemo(() => {
    const nodes = (health?.nodes || []);
    const total = nodes.length;

    let good = 0, warn = 0, error = 0;
    for (const n of nodes) {
      const w = (n.warnings || []);
      const e = (n.errors || []);
      if (e.length > 0) error++;
      else if (w.length > 0) warn++;
      else good++;
    }
    return { total, good, warn, error };
  }, [health]);

  return (
    <Layout>
      <div style={{ display: "flex", gap: 14, flexWrap: "wrap", marginBottom: 18 }}>
        <StatCard label="NODES" value={summary.total} />
        <StatCard label="GOOD" value={summary.good} color="#34d399" />
        <StatCard label="WARN" value={summary.warn} color="#fbbf24" />
        <StatCard label="ERROR" value={summary.error} color="#f87171" />
      </div>

      <h2 style={{ margin: "18px 0 12px" }}>System Health</h2>

      {err && (
        <div style={{
          background: "#2a1212",
          border: "1px solid #442",
          borderRadius: 12,
          padding: 12,
          marginBottom: 14
        }}>
          <div style={{ fontWeight: 800, color: "#fca5a5" }}>Backend status</div>
          <div style={{ opacity: 0.9, marginTop: 6 }}>{err}</div>
        </div>
      )}

      {!health ? (
        <div style={{ opacity: 0.8 }}>Loading snapshot...</div>
      ) : (
        <div>
          {(health.nodes || []).map((n) => {
            const hasErr = (n.errors || []).length > 0;
            const hasWarn = (n.warnings || []).length > 0;
            const status = hasErr ? "ERROR" : hasWarn ? "WARN" : "GOOD";

            return (
              <div
                key={n.name}
                style={{
                  background: "#161a22",
                  border: "1px solid #222",
                  borderRadius: 14,
                  padding: 16,
                  marginBottom: 12,
                }}
              >
                <div style={{ display: "flex", alignItems: "center" }}>
                  <div style={{ fontWeight: 800 }}>{n.name}</div>
                  <StatusBadge status={status} />
                  <div style={{ marginLeft: "auto", opacity: 0.6, fontSize: 12 }}>
                    {snap?.ts ? new Date(snap.ts * 1000).toLocaleTimeString() : ""}
                  </div>
                </div>

                <div style={{ marginTop: 10, opacity: 0.92 }}>
                  <div>Publishes: {(n.publishes && n.publishes.join(", ")) || "—"}</div>
                  <div>Subscribes: {(n.subscribes && n.subscribes.join(", ")) || "—"}</div>
                </div>

                {(n.warnings && n.warnings.length > 0) && (
                  <div style={{ marginTop: 10, color: "#fbbf24" }}>
                    <div style={{ fontWeight: 800 }}>Warnings</div>
                    <ul style={{ margin: "6px 0 0 18px" }}>
                      {n.warnings.map((w, i) => <li key={i}>{w}</li>)}
                    </ul>
                  </div>
                )}

                {(n.errors && n.errors.length > 0) && (
                  <div style={{ marginTop: 10, color: "#f87171" }}>
                    <div style={{ fontWeight: 800 }}>Errors</div>
                    <ul style={{ margin: "6px 0 0 18px" }}>
                      {n.errors.map((e, i) => <li key={i}>{e}</li>)}
                    </ul>
                  </div>
                )}
              </div>
            );
          })}
        </div>
      )}

      <h2 style={{ margin: "24px 0 12px" }}>ROS Graph Snapshot</h2>

      {!graph ? (
        <div style={{ opacity: 0.8 }}>Loading graph...</div>
      ) : (
        <div style={{ display: "flex", gap: 14, flexWrap: "wrap" }}>
          <div style={{
            background: "#161a22",
            border: "1px solid #222",
            borderRadius: 14,
            padding: 16,
            minWidth: 320
          }}>
            <div style={{ opacity: 0.7, fontSize: 12 }}>Nodes ({(graph.nodes || []).length})</div>
            <ul style={{ margin: "10px 0 0 18px" }}>
              {(graph.nodes || []).map((x, i) => <li key={i}>{x}</li>)}
            </ul>
          </div>

          <div style={{
            background: "#161a22",
            border: "1px solid #222",
            borderRadius: 14,
            padding: 16,
            minWidth: 360
          }}>
            <div style={{ opacity: 0.7, fontSize: 12 }}>Topics ({(graph.topics || []).length})</div>
            <div style={{ marginTop: 10 }}>
              {(graph.topics || []).map((t, i) => (
                <div key={i} style={{ marginBottom: 10 }}>
                  <div style={{ fontWeight: 800 }}>{t.name}</div>
                  <div style={{ opacity: 0.8, fontSize: 12 }}>
                    {(t.types || []).join(", ")}
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}
    </Layout>
  );
}
