export default function StatusBadge({ status }) {
  const map = {
    ok: { label: "GOOD", color: "#22c55e" },
    warn: { label: "WARN", color: "#f59e0b" },
    error: { label: "ERROR", color: "#ef4444" },
  };

  const cfg = map[status] || map.ok;

  return (
    <span
      style={{
        padding: "2px 10px",
        borderRadius: 6,
        fontSize: 12,
        fontWeight: 700,
        backgroundColor: cfg.color,
        color: "#000",
        letterSpacing: 0.5,
      }}
    >
      {cfg.label}
    </span>
  );
}
