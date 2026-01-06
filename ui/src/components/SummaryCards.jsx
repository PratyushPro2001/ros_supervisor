export default function SummaryCards({ nodes }) {
  const total = nodes.length;
  const good = nodes.filter(n => !n.warnings?.length && !n.errors?.length).length;
  const warn = nodes.filter(n => n.warnings?.length).length;
  const error = nodes.filter(n => n.errors?.length).length;

  const Card = ({ label, value, color }) => (
    <div
      style={{
        flex: 1,
        background: "#161a22",
        borderRadius: 10,
        padding: 16,
        boxShadow: "0 0 0 1px rgba(255,255,255,0.03)",
      }}
    >
      <div style={{ fontSize: 14, opacity: 0.7 }}>{label}</div>
      <div style={{ fontSize: 28, fontWeight: 700, color }}>{value}</div>
    </div>
  );

  return (
    <div style={{ display: "flex", gap: 16, marginBottom: 24 }}>
      <Card label="NODES" value={total} color="#e5e7eb" />
      <Card label="GOOD" value={good} color="#22c55e" />
      <Card label="WARN" value={warn} color="#f59e0b" />
      <Card label="ERROR" value={error} color="#ef4444" />
    </div>
  );
}
