export default function Layout({ children }) {
  return (
    <div style={{
      minHeight: "100vh",
      backgroundColor: "#0f1115",
      color: "#e5e7eb",
      fontFamily: '"Ubuntu Mono","JetBrains Mono","Fira Code",ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,"Liberation Mono","Courier New",monospace'
    }}>
      <header style={{
        padding: "16px 24px",
        borderBottom: "1px solid #1f2430",
        fontWeight: 800,
        fontSize: 18
      }}>
        ROS Supervisor
      </header>

      <main style={{ padding: 24 }}>
        {children}
      </main>
    </div>
  );
}
