import { useEffect, useMemo, useRef, useState } from "react";

function uniqSorted(arr) {
  return Array.from(new Set(arr)).sort();
}

export default function GraphPanel({ graph }) {
  const wrapRef = useRef(null);
  const [wrapW, setWrapW] = useState(0);

  useEffect(() => {
    if (!wrapRef.current) return;
    const el = wrapRef.current;

    const ro = new ResizeObserver((entries) => {
      for (const e of entries) setWrapW(e.contentRect.width || 0);
    });
    ro.observe(el);
    setWrapW(el.getBoundingClientRect().width || 0);

    return () => ro.disconnect();
  }, []);

  const edges = graph?.edges || [];

  const { pubs, subs, topics } = useMemo(() => {
    const pubs0 = [];
    const subs0 = [];
    const topics0 = [];
    for (const e of edges) {
      pubs0.push(e.from);
      subs0.push(e.to);
      topics0.push(e.topic);
    }
    return {
      pubs: uniqSorted(pubs0),
      subs: uniqSorted(subs0),
      topics: uniqSorted(topics0),
    };
  }, [edges]);

  if (!graph) return <div style={{ opacity: 0.8 }}>Loading graph…</div>;
  if (!edges.length) {
    return <div style={{ opacity: 0.8 }}>No edges yet (start some nodes).</div>;
  }

  // --- Base layout (unscaled, in "SVG units")
  const pad = 24;
  const colGap = 110;
  const rowGap = 18;

  const nodeW = 240;
  const nodeH = 44;

  const topicW = 320;
  const topicH = 40;

  const leftX = pad;
  const midX = leftX + nodeW + colGap;
  const rightX = midX + topicW + colGap;

  const rows = Math.max(pubs.length, topics.length, subs.length);
  const layoutH = pad * 2 + rows * nodeH + (rows - 1) * rowGap;
  const layoutW = rightX + nodeW + pad;

  // Scale-to-fit (no scrolling)
  const availableW = Math.max(0, (wrapW || 0) - 2); // small safety
  const scale = availableW > 0 ? Math.min(1, availableW / layoutW) : 1;
  const svgHpx = Math.max(180, Math.ceil(layoutH * scale));

  const yForRow = (i) => pad + i * (nodeH + rowGap);

  const rect = (x, y, w, h, fill, stroke) => (
    <rect
      x={x}
      y={y}
      width={w}
      height={h}
      rx={12}
      ry={12}
      fill={fill}
      stroke={stroke}
      strokeWidth={1.2}
    />
  );

  const label = (x, y, text, size = 14) => (
    <text
      x={x}
      y={y}
      fill="#ffffff"
      fontSize={size}
      fontFamily="ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace"
    >
      {text}
    </text>
  );

  const arrow = (x1, y1, x2, y2) => (
    <path
      d={`M ${x1} ${y1} C ${x1 + 60} ${y1}, ${x2 - 60} ${y2}, ${x2} ${y2}`}
      fill="none"
      stroke="rgba(255,255,255,0.45)"
      strokeWidth={2}
      markerEnd="url(#arrow)"
    />
  );

  return (
    <div ref={wrapRef} style={{ width: "100%" }}>
      <svg
        width="100%"
        height={svgHpx}
        viewBox={`0 0 ${layoutW} ${layoutH}`}
        preserveAspectRatio="xMidYMid meet"
        style={{
          display: "block",
          borderRadius: 12,
          background: "rgba(255,255,255,0.03)",
          border: "1px solid rgba(255,255,255,0.10)",
        }}
      >
        <defs>
          <marker
            id="arrow"
            viewBox="0 0 10 10"
            refX="9"
            refY="5"
            markerWidth="7"
            markerHeight="7"
            orient="auto-start-reverse"
          >
            <path d="M 0 0 L 10 5 L 0 10 z" fill="rgba(255,255,255,0.7)" />
          </marker>
        </defs>

        {/* We scale the whole drawing group */}
        <g transform={`scale(${scale})`}>
          {/* columns labels (optional) */}
          {label(leftX, pad - 6, "Nodes (publishers)", 12)}
          {label(midX, pad - 6, "Topics", 12)}
          {label(rightX, pad - 6, "Nodes (subscribers)", 12)}

          {/* left nodes */}
          {pubs.map((name, i) => {
            const y = yForRow(i);
            return (
              <g key={`p-${name}`}>
                {rect(leftX, y, nodeW, nodeH, "rgba(74,163,255,0.28)", "rgba(74,163,255,0.55)")}
                {label(leftX + 14, y + 28, name, 14)}
              </g>
            );
          })}

          {/* topics */}
          {topics.map((t, i) => {
            const y = yForRow(i);
            return (
              <g key={`t-${t}`}>
                {rect(midX, y + 2, topicW, topicH, "rgba(255,255,255,0.18)", "rgba(255,255,255,0.35)")}
                {label(midX + 14, y + 28, t, 14)}
              </g>
            );
          })}

          {/* right nodes */}
          {subs.map((name, i) => {
            const y = yForRow(i);
            return (
              <g key={`s-${name}`}>
                {rect(rightX, y, nodeW, nodeH, "rgba(251,191,36,0.22)", "rgba(251,191,36,0.45)")}
                {label(rightX + 14, y + 28, name, 14)}
              </g>
            );
          })}

          {/* arrows: pub -> topic */}
          {edges.map((e, idx) => {
            const pIdx = pubs.indexOf(e.from);
            const tIdx = topics.indexOf(e.topic);
            if (pIdx < 0 || tIdx < 0) return null;

            const y1 = yForRow(pIdx) + nodeH / 2;
            const y2 = yForRow(tIdx) + nodeH / 2;

            const x1 = leftX + nodeW;
            const x2 = midX;

            return <g key={`pt-${idx}`}>{arrow(x1, y1, x2, y2)}</g>;
          })}

          {/* arrows: topic -> sub */}
          {edges.map((e, idx) => {
            const tIdx = topics.indexOf(e.topic);
            const sIdx = subs.indexOf(e.to);
            if (tIdx < 0 || sIdx < 0) return null;

            const y1 = yForRow(tIdx) + nodeH / 2;
            const y2 = yForRow(sIdx) + nodeH / 2;

            const x1 = midX + topicW;
            const x2 = rightX;

            return <g key={`ts-${idx}`}>{arrow(x1, y1, x2, y2)}</g>;
          })}
        </g>
      </svg>
    </div>
  );
}
