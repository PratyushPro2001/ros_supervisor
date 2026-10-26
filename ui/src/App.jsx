import { useEffect, useRef, useState } from "react";
import Layout from "./components/Layout";
import GraphPanel from "./components/GraphPanel";

// Backend address. Default unchanged; override with VITE_API_URL (e.g. when the API runs in Docker on :8100).
const API_BASE = import.meta.env.VITE_API_URL || "http://127.0.0.1:8000";

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

function TFTree({ tf }) {
  const edge = {};
  tf.edges.forEach((e) => { edge[`${e.parent}>${e.child}`] = e; });
  const rows = [];
  const walk = (f, depth, via) => {
    const stale = via && !via.static && (via.age_sec === null || via.age_sec > 3);
    rows.push(
      <div key={`${depth}-${f}`} style={{ paddingLeft: depth * 16, fontSize: 13 }}>
        {f}{" "}
        {via && (
          <span style={{ fontSize: 12, color: via.static ? "#9ca3af" : stale ? "#f87171" : "#4ade80" }}>
            {via.static ? "static" : stale ? "STALLED" : `${via.hz} Hz`}
          </span>
        )}
      </div>
    );
    (tf.children_map[f] || []).forEach((c) => walk(c, depth + 1, edge[`${f}>${c}`]));
  };
  tf.roots.forEach((r) => walk(r, 0, null));
  return <div>{rows}</div>;
}

export default function App() {
  const [snap, setSnap] = useState(null);
  const [err, setErr] = useState(null);
  const [now, setNow] = useState(Date.now());
  const lastTs = useRef({ ts: null, at: Date.now() });

  useEffect(() => {
    const load = async () => {
      try {
        const r = await fetch(`${API_BASE}/snapshot`);
        const j = await r.json();
        if (j.ts !== lastTs.current.ts) lastTs.current = { ts: j.ts, at: Date.now() };
        setSnap(j);
        setErr(null);
      } catch (e) {
        setErr(String(e));
      }
    };
    load();
    const t = setInterval(load, 1000);
    const c = setInterval(() => setNow(Date.now()), 500);
    return () => { clearInterval(t); clearInterval(c); };
  }, []);

  const ok = snap?.ok;
  const graph = snap?.graph;
  const healthNodes = snap?.health?.nodes || [];

  const staleSec = (now - lastTs.current.at) / 1000;
  let label = snap ? snap.status || (ok ? "GOOD" : "ERROR") : "LOADING";
  if (err) label = "DISCONNECTED";
  else if (snap && staleSec > 3) label = "STALE";
  const COLORS = { GOOD: "#4ade80", WARN: "#fbbf24", ERROR: "#f87171", STALE: "#f87171", DISCONNECTED: "#f87171", LOADING: "#9ca3af" };
  const issues = snap?.issues || [];
  const rates = {};
  (snap?.health?.topic_rates || []).forEach((r) => { rates[r.topic] = r; });

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
                color: COLORS[label] || "#f87171",
              }}
            >
              {label}
            </span>
          </div>
          <div style={{ opacity: 0.6, fontSize: 13 }}>
            {snap ? `updated ${Math.max(0, staleSec).toFixed(1)}s ago` : ""}
          </div>
        </div>

        {issues.length > 0 && (
          <ul style={{ margin: 0, paddingLeft: 18, fontSize: 13, color: "#fbbf24" }}>
            {issues.map((i) => <li key={i}>{i}</li>)}
          </ul>
        )}

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
              {topicNames.map((t) => {
                const r = rates[t];
                const hz = r ? r.hz : undefined;
                const col = hz === undefined ? undefined : hz === null ? "#9ca3af" : hz < 0.2 ? "#f87171" : "#4ade80";
                return (
                  <li key={t}>
                    {t}{" "}
                    {hz !== undefined && (
                      <span style={{ color: col, fontSize: 12 }}>
                        {hz === null ? "…" : `${hz} Hz`}
                      </span>
                    )}
                  </li>
                );
              })}
            </ul>
          </Card>

          <Card title="QoS">
            <div style={{ fontSize: 14 }}>
              {!snap?.qos ? "…" : snap.qos.ok ? (
                <span style={{ color: "#4ade80" }}>
                  OK: {snap.qos.pairs_checked} publisher/subscriber pairs on {snap.qos.topics_checked} topics are compatible.
                </span>
              ) : (
                (snap.qos.mismatches || []).map((m, i) => (
                  <div key={i} style={{ marginBottom: 8, color: "#fbbf24" }}>
                    <b>{m.topic}</b>: {m.publisher} to {m.subscriber}
                    <div style={{ fontSize: 13 }}>{(m.issues || []).join("; ")}</div>
                    <div style={{ fontSize: 12, opacity: 0.8 }}>Fix: {(m.suggestions || []).join(" ")}</div>
                  </div>
                ))
              )}
            </div>
          </Card>

          <Card title="TF">
            <div style={{ fontSize: 14 }}>
              {!snap?.tf ? "…" : snap.tf.edges.length === 0 ? (
                <span style={{ opacity: 0.8 }}>No transforms seen on /tf or /tf_static.</span>
              ) : (
                <>
                  <div style={{ opacity: 0.8, marginBottom: 6 }}>
                    {snap.tf.frames.length} frames, {snap.tf.edges.length} transforms
                  </div>
                  {(snap.tf.validation?.errors || []).map((e) => (
                    <div key={e} style={{ color: "#f87171", fontSize: 13 }}>{e}</div>
                  ))}
                  <TFTree tf={snap.tf} />
                </>
              )}
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
