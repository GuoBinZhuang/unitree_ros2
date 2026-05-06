import { PanelExtensionContext, MessageEvent } from "@foxglove/extension";
import { useEffect, useLayoutEffect, useState, useCallback } from "react";
import { createRoot } from "react-dom/client";

// ── G1 29-DoF 关节定义 ──────────────────────────────────────────
const JOINTS = [
  // 左腿
  { id: 0,  name: "L髋俯仰",  group: "left_leg",  x: 72,  y: 268 },
  { id: 1,  name: "L髋侧摆",  group: "left_leg",  x: 58,  y: 255 },
  { id: 2,  name: "L髋偏航",  group: "left_leg",  x: 65,  y: 280 },
  { id: 3,  name: "L膝",      group: "left_leg",  x: 65,  y: 345 },
  { id: 4,  name: "L踝俯仰",  group: "left_leg",  x: 65,  y: 415 },
  { id: 5,  name: "L踝侧摆",  group: "left_leg",  x: 58,  y: 428 },
  // 右腿
  { id: 6,  name: "R髋俯仰",  group: "right_leg", x: 128, y: 268 },
  { id: 7,  name: "R髋侧摆",  group: "right_leg", x: 142, y: 255 },
  { id: 8,  name: "R髋偏航",  group: "right_leg", x: 135, y: 280 },
  { id: 9,  name: "R膝",      group: "right_leg", x: 135, y: 345 },
  { id: 10, name: "R踝俯仰",  group: "right_leg", x: 135, y: 415 },
  { id: 11, name: "R踝侧摆",  group: "right_leg", x: 142, y: 428 },
  // 腰部
  { id: 12, name: "腰偏航",   group: "waist",     x: 100, y: 210 },
  { id: 13, name: "腰侧摆",   group: "waist",     x: 88,  y: 222 },
  { id: 14, name: "腰俯仰",   group: "waist",     x: 112, y: 222 },
  // 左臂
  { id: 15, name: "L肩俯仰",  group: "left_arm",  x: 48,  y: 148 },
  { id: 16, name: "L肩侧摆",  group: "left_arm",  x: 35,  y: 160 },
  { id: 17, name: "L肩偏航",  group: "left_arm",  x: 38,  y: 175 },
  { id: 18, name: "L肘",      group: "left_arm",  x: 32,  y: 220 },
  { id: 19, name: "L腕滚转",  group: "left_arm",  x: 28,  y: 265 },
  { id: 20, name: "L腕俯仰",  group: "left_arm",  x: 22,  y: 280 },
  // 右臂
  { id: 21, name: "R肩俯仰",  group: "right_arm", x: 152, y: 148 },
  { id: 22, name: "R肩侧摆",  group: "right_arm", x: 165, y: 160 },
  { id: 23, name: "R肩偏航",  group: "right_arm", x: 162, y: 175 },
  { id: 24, name: "R肘",      group: "right_arm", x: 168, y: 220 },
  { id: 25, name: "R腕滚转",  group: "right_arm", x: 172, y: 265 },
  { id: 26, name: "R腕俯仰",  group: "right_arm", x: 178, y: 280 },
  // 灵巧手
  { id: 27, name: "L手",      group: "left_hand", x: 18,  y: 310 },
  { id: 28, name: "R手",      group: "right_hand",x: 182, y: 310 },
];

// ── 颜色映射：0→蓝 50%→绿 80%→黄 100%→红 ──────────────────────
function tauToColor(ratio: number): string {
  const r = Math.max(0, Math.min(1, ratio));
  if (r < 0.5) {
    const t = r / 0.5;
    const R = Math.round(0   + t * 0);
    const G = Math.round(120 + t * 135);
    const B = Math.round(255 - t * 155);
    return `rgb(${R},${G},${B})`;
  } else if (r < 0.8) {
    const t = (r - 0.5) / 0.3;
    const R = Math.round(0   + t * 255);
    const G = Math.round(255 - t * 35);
    const B = Math.round(100 - t * 100);
    return `rgb(${R},${G},${B})`;
  } else {
    const t = (r - 0.8) / 0.2;
    const R = 255;
    const G = Math.round(220 - t * 220);
    const B = 0;
    return `rgb(${R},${G},${B})`;
  }
}

// ── 默认阈值（Nm） ──────────────────────────────────────────────
const DEFAULT_TAU_MAX = 40;

// ── 分组样式设置 ────────────────────────────────────────────────
const GROUPS = [
  { key: "left_leg", label: "左腿 (Left Leg)" },
  { key: "right_leg", label: "右腿 (Right Leg)" },
  { key: "waist", label: "腰部 (Waist)" },
  { key: "left_arm", label: "左臂 (Left Arm)" },
  { key: "right_arm", label: "右臂 (Right Arm)" },
  { key: "left_hand", label: "左手 (Left Hand)" },
  { key: "right_hand", label: "右手 (Right Hand)" },
];

// ── 主组件 ──────────────────────────────────────────────────────
type JointState = {
  tau: number;
  q: number;
  dq: number;
  temp: number;
};

function G1HeatmapPanel({ context }: { context: PanelExtensionContext }) {
  const [jointStates, setJointStates] = useState<JointState[]>(
    Array(29).fill({ tau: 0, q: 0, dq: 0, temp: 0 })
  );
  const [tauMax, setTauMax] = useState(DEFAULT_TAU_MAX);
  const [topic, setTopic] = useState("/lowstate");
  const [hoveredId, setHoveredId] = useState<number | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [mode, setMode] = useState<"tau" | "temp">("tau");

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);

      const msgs = renderState.currentFrame ?? [];
      for (const msg of msgs as readonly MessageEvent<any>[]) {
        if (msg.topic === topic) {
          const motorState = (msg.message as any)?.motor_state ?? [];
          const next: JointState[] = Array(29).fill(null).map((_, i) => {
            const m = motorState[i] ?? {};
            return {
              tau:  Math.abs(m.tau_est ?? 0),
              q:    m.q   ?? 0,
              dq:   m.dq  ?? 0,
              temp: m.temperature ?? 0,
            };
          });
          setJointStates(next);
        }
      }
    };

    context.watch("currentFrame");
    context.subscribe([{ topic }]);
  }, [context, topic]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone, jointStates]);

  const getValue = useCallback(
    (j: JointState) => (mode === "tau" ? j.tau : j.temp),
    [mode]
  );
  const maxVal = mode === "tau" ? tauMax : 80; // 温度上限 80°C

  const alarmCount = jointStates.filter((j) => getValue(j) / maxVal >= 1.0).length;

  return (
    <div style={{
      display: "flex", flexDirection: "column", height: "100%",
      background: "#0f111a", color: "#e2e8f0", fontFamily: "system-ui, -apple-system, sans-serif", fontSize: 13,
      userSelect: "none", overflow: "hidden"
    }}>
      {/* ── 顶部工具栏 ── */}
      <div style={{
        display: "flex", alignItems: "center", gap: 16,
        padding: "10px 16px", background: "#1a1d27", borderBottom: "1px solid #2d313f",
        flexWrap: "wrap", boxShadow: "0 4px 6px -1px rgba(0, 0, 0, 0.1)"
      }}>
        <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
          <div style={{ width: 8, height: 16, background: "#3b82f6", borderRadius: 4 }}></div>
          <span style={{ fontWeight: "600", fontSize: 15, color: "#f8fafc", letterSpacing: "0.5px" }}>G1 热力监控 (Heatmap)</span>
        </div>

        {/* 模式切换 */}
        <div style={{ display: "flex", background: "#0f111a", borderRadius: 6, padding: 2, border: "1px solid #2d313f" }}>
          {(["tau", "temp"] as const).map((m) => (
            <button key={m} onClick={() => setMode(m)} style={{
              padding: "4px 12px", borderRadius: 4, cursor: "pointer",
              background: mode === m ? "#3b82f6" : "transparent",
              color: mode === m ? "#ffffff" : "#94a3b8",
              border: "none", outline: "none",
              fontWeight: mode === m ? "600" : "400",
              transition: "all 0.2s ease"
            }}>
              {m === "tau" ? "力矩 (Tau)" : "温度 (Temp)"}
            </button>
          ))}
        </div>

        {/* 阈值设置 */}
        <div style={{ display: "flex", alignItems: "center", gap: 6, background: "#1e212b", padding: "4px 10px", borderRadius: 6, border: "1px solid #2d313f" }}>
          <span style={{ color: "#94a3b8", fontSize: 12 }}>
            Max {mode === "tau" ? "(Nm)" : "(°C)"}:
          </span>
          <input
            type="number" min={1} max={200}
            value={mode === "tau" ? tauMax : maxVal}
            onChange={(e) => { if (mode === "tau") setTauMax(Number(e.target.value)); }}
            style={{
              width: 48, background: "transparent", color: "#f8fafc",
              border: "none", outline: "none", fontSize: 13, fontWeight: "500", textAlign: "right"
            }}
          />
        </div>

        {/* 话题 */}
        <div style={{ display: "flex", alignItems: "center", gap: 6, background: "#1e212b", padding: "4px 10px", borderRadius: 6, border: "1px solid #2d313f" }}>
          <span style={{ color: "#94a3b8", fontSize: 12 }}>Topic:</span>
          <input
            value={topic}
            onChange={(e) => setTopic(e.target.value)}
            style={{
              width: 140, background: "transparent", color: "#3b82f6",
              border: "none", outline: "none", fontSize: 13, fontFamily: "monospace"
            }}
          />
        </div>

        {/* 报警提示 */}
        {alarmCount > 0 && (
          <div style={{
            marginLeft: "auto", padding: "4px 12px", borderRadius: 20,
            background: "rgba(239, 68, 68, 0.15)", color: "#ef4444", fontWeight: "600",
            border: "1px solid rgba(239, 68, 68, 0.5)",
            animation: "pulse 1.5s infinite ease-in-out",
            display: "flex", alignItems: "center", gap: 6
          }}>
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"></path>
              <line x1="12" y1="9" x2="12" y2="13"></line>
              <line x1="12" y1="17" x2="12.01" y2="17"></line>
            </svg>
            {alarmCount} 处告警 (Warnings)
          </div>
        )}
      </div>

      {/* ── 主体区域 ── */}
      <div style={{ display: "flex", flex: 1, overflow: "hidden", position: "relative" }}>

        {/* 左：SVG 机器人轮廓 */}
        <div style={{ flex: "0 0 240px", position: "relative", background: "radial-gradient(circle at 50% 50%, #1e212b 0%, #0f111a 100%)", borderRight: "1px solid #2d313f", display: "flex", justifyContent: "center" }}>
          {/* 发光网格背景效应 */}
          <div style={{ position: "absolute", inset: 0, opacity: 0.05, backgroundImage: "linear-gradient(#3b82f6 1px, transparent 1px), linear-gradient(90deg, #3b82f6 1px, transparent 1px)", backgroundSize: "20px 20px" }}></div>
          <svg viewBox="0 0 200 480" width="100%" height="100%"
            style={{ display: "block", paddingTop: "20px", zIndex: 1 }}>
            {/* 身体轮廓（科技感增强） */}
            <g stroke="#334155" fill="none" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
              {/* 头 */}
              <rect x="78" y="30" width="44" height="52" rx="12" fill="#1e293b" />
              <path d="M 85 55 L 115 55" stroke="#475569" strokeWidth="6" />
              {/* 颈 */}
              <line x1="100" y1="82" x2="100" y2="100" strokeWidth="4" />
              {/* 躯干 */}
              <path d="M 68 110 L 132 110 L 128 220 L 72 220 Z" fill="#1e293b" />
              <line x1="100" y1="110" x2="100" y2="220" stroke="#0f111a" strokeWidth="2" />
              {/* 左臂 */}
              <path d="M 68 120 L 40 145 L 32 220 L 20 295" strokeWidth="4" />
              {/* 右臂 */}
              <path d="M 132 120 L 160 145 L 168 220 L 180 295" strokeWidth="4" />
              {/* 左腿 */}
              <path d="M 82 230 L 65 350 L 65 435" strokeWidth="5" />
              {/* 右腿 */}
              <path d="M 118 230 L 135 350 L 135 435" strokeWidth="5" />
              {/* 左脚 */}
              <path d="M 65 435 L 40 445 L 50 455 L 65 455" fill="#1e293b" />
              {/* 右脚 */}
              <path d="M 135 435 L 160 445 L 150 455 L 135 455" fill="#1e293b" />
            </g>

            {/* 关节连线 Glow */}
            <g>
              {JOINTS.map((j) => {
                const val   = getValue(jointStates[j.id] ?? { tau: 0, q: 0, dq: 0, temp: 0 });
                const ratio = val / maxVal;
                const color = tauToColor(ratio);
                const alarm = ratio >= 1.0;
                
                if (!alarm) return null;
                return (
                  <circle key={`glow-${j.id}`} cx={j.x} cy={j.y} r={14}
                    fill={color} opacity="0.2" style={{ filter: "blur(4px)" }} />
                );
              })}
            </g>

            {/* 关节圆圈 */}
            {JOINTS.map((j) => {
              const val   = getValue(jointStates[j.id] ?? { tau: 0, q: 0, dq: 0, temp: 0 });
              const ratio = val / maxVal;
              const color = tauToColor(ratio);
              const alarm = ratio >= 1.0;
              const hovered = hoveredId === j.id;
              
              const radius = hovered ? 9 : (alarm ? 8 : 6);
              
              return (
                <g key={j.id}
                  onMouseEnter={() => setHoveredId(j.id)}
                  onMouseLeave={() => setHoveredId(null)}
                  style={{ cursor: "pointer", transition: "all 0.2s" }}>
                  {alarm && (
                    <circle cx={j.x} cy={j.y} r={12}
                      fill="none" stroke="#ef4444" strokeWidth="1.5"
                      style={{ animation: "pulse 1s infinite" }} />
                  )}
                  <circle
                    cx={j.x} cy={j.y}
                    r={radius}
                    fill={color}
                    stroke={hovered ? "#ffffff" : "#0f111a"}
                    strokeWidth={hovered ? 2 : (alarm ? 0 : 2)}
                    style={{ transition: "r 0.2s, fill 0.3s" }}
                  />
                  {(hovered || alarm) && (
                    <text x={j.x} y={j.y - 14} textAnchor="middle" fontWeight="bold"
                      fill={alarm ? "#ef4444" : "#ffffff"} fontSize="10" 
                      style={{ textShadow: "0 1px 3px rgba(0,0,0,0.8), 0 0 2px #000" }}>
                      {val.toFixed(1)}
                    </text>
                  )}
                </g>
              );
            })}
          </svg>
        </div>

        {/* 右：数值列表（按部位分组） */}
        <div style={{
          flex: 1, overflowY: "auto", padding: "20px", display: "flex", flexDirection: "column", gap: 16
        }}>
          {/* 色阶图例 */}
          <div style={{ display: "flex", alignItems: "center", gap: 12, padding: "12px 16px", background: "#1e212b", borderRadius: 8, border: "1px solid #2d313f" }}>
            <span style={{ color: "#94a3b8", fontSize: 12, fontWeight: 500 }}>Safe</span>
            <div style={{
              flex: 1, height: 6, borderRadius: 3,
              background: "linear-gradient(to right, rgb(0,120,255), rgb(0,255,100), rgb(255,220,0), rgb(255,0,0))",
            }} />
            <span style={{ color: "#ef4444", fontSize: 12, fontWeight: 600 }}>
              Danger ({mode === "tau" ? `${tauMax}Nm` : `${maxVal}°C`})
            </span>
          </div>

          {/* 分组网络布局 */}
          <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(280px, 1fr))", gap: 16 }}>
            {GROUPS.map(group => {
              const groupJoints = JOINTS.filter(j => j.group === group.key);
              if (groupJoints.length === 0) return null;
              
              // 检查该组是否有报警
              const hasAlarm = groupJoints.some(j => {
                const val = getValue(jointStates[j.id] ?? { tau: 0, q: 0, dq: 0, temp: 0 });
                return val / maxVal >= 1.0;
              });
              
              return (
                <div key={group.key} style={{
                  background: "#161821", borderRadius: 8, border: hasAlarm ? "1px solid rgba(239,68,68,0.4)" : "1px solid #2d313f",
                  overflow: "hidden", boxShadow: "0 2px 4px rgba(0,0,0,0.1)"
                }}>
                  <div style={{ 
                    padding: "8px 12px", background: hasAlarm ? "rgba(239,68,68,0.1)" : "#1e212b", 
                    borderBottom: "1px solid #2d313f", fontSize: 12, fontWeight: "600", color: hasAlarm ? "#fca5a5" : "#cbd5e1",
                    display: "flex", justifyContent: "space-between"
                  }}>
                    <span>{group.label}</span>
                    {hasAlarm && <span style={{ color: "#ef4444" }}>⚠</span>}
                  </div>
                  <div style={{ padding: "8px" }}>
                    {groupJoints.map(j => {
                      const val = getValue(jointStates[j.id] ?? { tau: 0, q: 0, dq: 0, temp: 0 });
                      const ratio = Math.max(0, Math.min(val / maxVal, 1));
                      const color = tauToColor(ratio);
                      const alarm = val / maxVal >= 1.0;
                      const isHovered = hoveredId === j.id;
                      
                      return (
                        <div key={j.id}
                          onMouseEnter={() => setHoveredId(j.id)}
                          onMouseLeave={() => setHoveredId(null)}
                          style={{
                            display: "flex", alignItems: "center", gap: 10,
                            padding: "6px 8px", borderRadius: 4,
                            background: alarm ? "rgba(239,68,68,0.1)" : (isHovered ? "#212533" : "transparent"),
                            transition: "background 0.2s"
                          }}>
                          <span style={{ fontSize: 10, color: "#64748b", width: 18, textAlign: "right" }}>{j.id}</span>
                          <span style={{ width: 68, fontSize: 12, color: alarm ? "#fca5a5" : "#e2e8f0", whiteSpace: "nowrap", overflow: "hidden", textOverflow: "ellipsis" }}>
                            {j.name}
                          </span>
                          
                          <div style={{ flex: 1, position: "relative", height: 6, background: "#0f111a", borderRadius: 3, border: "1px solid #2d313f", overflow: "hidden" }}>
                            <div style={{
                              position: "absolute", top: 0, left: 0, bottom: 0,
                              width: `${ratio * 100}%`, background: color,
                              transition: "width 0.3s ease, background 0.3s ease",
                              boxShadow: alarm ? "0 0 4px #ef4444" : "none"
                            }} />
                          </div>
                          
                          <span style={{ 
                            width: 44, textAlign: "right", fontSize: 12, fontFamily: "monospace",
                            color: alarm ? "#ef4444" : (ratio > 0.8 ? "#fbbf24" : "#93c5fd"),
                            fontWeight: alarm ? "bold" : (isHovered ? "bold" : "normal")
                          }}>
                            {val.toFixed(1)}
                          </span>
                        </div>
                      );
                    })}
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      </div>

      {/* 动画定义 */}
      <style>{`
        @keyframes pulse { 
          0% { opacity: 1; transform: scale(1); } 
          50% { opacity: 0.7; transform: scale(1.02); } 
          100% { opacity: 1; transform: scale(1); } 
        }
        @keyframes blink { 
          50% { opacity: 0; } 
        }
      `}</style>
    </div>
  );
}

// ── 注册入口 ────────────────────────────────────────────────────
export function initG1HeatmapPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<G1HeatmapPanel context={context} />);
  return () => root.unmount();
}