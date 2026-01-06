import React from "react";

export default class ErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error) {
    return { hasError: true, error };
  }

  componentDidCatch(error, info) {
    // Useful in devtools
    console.error("UI crashed:", error, info);
  }

  render() {
    if (this.state.hasError) {
      return (
        <div
          style={{
            background: "#161a22",
            borderRadius: 12,
            padding: 16,
            boxShadow: "0 0 0 1px rgba(255,255,255,0.03)",
          }}
        >
          <div style={{ fontSize: 14, opacity: 0.8, marginBottom: 10 }}>
            UI Error
          </div>
          <pre style={{ whiteSpace: "pre-wrap", margin: 0 }}>
            {String(this.state.error)}
          </pre>
        </div>
      );
    }
    return this.props.children;
  }
}
