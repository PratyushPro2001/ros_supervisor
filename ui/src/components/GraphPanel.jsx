import { useEffect, useState } from "react";

const API_BASE = "http://127.0.0.1:8000";

function Card({ title, children }) {
  return (
    <div
      style={{
        background: "#161a22",
        borderRadius: 12,
        padding: 16,
        boxShadow: "0 0 0 1px rgba(255,255,255,0.03)",
      }}
    >
      <div style={{ fontSize: 14, opacity: 0.8, marginBottom: 10 }}>{title}</div>
      {children}
    </div>
  );
}

function TopicTile({ name, types }) {
  return (
    <div
      style={{
        background: "rgba(255,255,255,0.02)",
        borderRadius: 10,
        padding: 12,
        boxShadow: "0 0 0 1px rgba(255,255,255,0.04)",
        minWidth: 0,
      }}
    >
      <div
        title={name}
        style={{
          fontWeight: 750,
          whiteSpace: "nowrap",
          overflow: "hidden",
          textOverflow: "ellipsis",
          marginBottom: 6,
        }}
      >
        {name}
      </div>
      <div
        title={(types && types.length) ? types.join(", ") : "—"}
        style={{
          opacity: 0.75,
          fontSize: 13,
          lineHeight: 1.25,
          wordBreak: "break-word",
        }}
      >
        {(types && types.length) ? types.join(", ") : "—"}
      </div>
    </div>
  );
}

export default function GraphPanel() {
  const [data, setData] = useState(null);
  const [err, setErr] = useState(null);

  useEffect(() => {
    fetch(`${API_BASE}/graph`)
      .then((r) => {
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        return r.json();
      })
      .then(setData)
      .catch((e) => setErr(String(e)));
  }, []);

  if (err) return <Card title="ROS Graph Snapshot"><pre>Error: {err}</pre></Card>;
  if (!data) return <Card title="ROS Graph Snapshot"><pre>Loading…</pre></Card>;

  const nodes = data.nodes ?? [];
  const topics = data.topics ?? data.topic_names_and_types ?? [];

  // topics may be [{name, types}] already or raw tuples depending on backend evolution
  const normalizedTopics = topics.map((t) => {
    if (Array.isArray(t) && t.length >= 2) return { name: t[0], types: t[1] };
    return { name: t.name, types: t.types ?? [] };
  });

  return (
    <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 16 }}>
      <Card title={`Nodes (${nodes.length})`}>
        <div style={{ maxHeight: 320, overflow: "auto" }}>
          {nodes.length === 0 ? (
            <div style={{ opacity: 0.7 }}>—</div>
          ) : (
            <ul style={{ margin: 0, paddingLeft: 18 }}>
              {nodes.map((n) => (
                <li key={n} style={{ marginBottom: 6 }}>
                  {n}
                </li>
              ))}
            </ul>
          )}
        </div>
      </Card>

      <Card title={`Topics (${normalizedTopics.length})`}>
        <div style={{ maxHeight: 320, overflow: "auto" }}>
          {normalizedTopics.length === 0 ? (
            <div style={{ opacity: 0.7 }}>—</div>
          ) : (
            <div
              style={{
                display: "grid",
                gridTemplateColumns: "repeat(auto-fit, minmax(220px, 1fr))",
                gap: 10,
                alignItems: "start",
              }}
            >
              {normalizedTopics.map((t) => (
                <TopicTile key={t.name} name={t.name} types={t.types} />
              ))}
            </div>
          )}
        </div>
      </Card>
    </div>
  );
}
