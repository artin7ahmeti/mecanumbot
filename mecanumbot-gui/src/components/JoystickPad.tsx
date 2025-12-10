import { useEffect, useMemo, useRef, useState } from "react"

type JoystickValue = {
  linearX: number
  linearY: number
}

type Mode = "xy" | "x" | "y"

type Props = {
  maxLinear?: number
  maxAngular?: number
  disabled?: boolean
  onChange?: (v: JoystickValue) => void
  onEnd?: () => void
  showReadout?: boolean

  mode?: Mode
  showYaw?: boolean
  size?: number
  title?: string
  description?: string
}

export default function JoystickPad({
  maxLinear = 0.4,
  maxAngular = 1.0,
  disabled = false,
  onChange,
  onEnd,
  showReadout = true,

  mode = "xy",
  showYaw = true,
  size = 220,
  title = "Virtual Joystick",
  description = "Drag to move. Use Yaw slider for rotation.",
}: Props) {
  const areaRef = useRef<HTMLDivElement | null>(null)

  // ✅ IMPORTANT: store callbacks in refs to avoid re-render loops
  const onChangeRef = useRef<Props["onChange"]>(onChange)
  const onEndRef = useRef<Props["onEnd"]>(onEnd)

  useEffect(() => {
    onChangeRef.current = onChange
  }, [onChange])

  useEffect(() => {
    onEndRef.current = onEnd
  }, [onEnd])

  const [dragging, setDragging] = useState(false)
  const [knob, setKnob] = useState({ x: 0, y: 0 }) // normalized -1..1
  const [yawNorm, setYawNorm] = useState(0)

  const value = useMemo<JoystickValue>(() => {
    const linearX = mode === "y" ? 0 : -knob.y * maxLinear
    const linearY = mode === "x" ? 0 : knob.x * maxLinear
    const angularZ = showYaw ? yawNorm * maxAngular : 0
    return { linearX, linearY, angularZ }
  }, [knob.x, knob.y, yawNorm, maxLinear, maxAngular, mode, showYaw])

  // ✅ CRITICAL: depend only on value
  useEffect(() => {
    onChangeRef.current?.(value)
  }, [value])

  function updateFromPointer(clientX: number, clientY: number) {
    const el = areaRef.current
    if (!el) return

    const rect = el.getBoundingClientRect()
    const cx = rect.left + rect.width / 2
    const cy = rect.top + rect.height / 2

    const dx = clientX - cx
    const dy = clientY - cy

    const radius = Math.min(rect.width, rect.height) / 2
    let nx = Math.max(-1, Math.min(1, dx / radius))
    let ny = Math.max(-1, Math.min(1, dy / radius))

    const mag = Math.sqrt(nx * nx + ny * ny)
    const scale = mag > 1 ? 1 / mag : 1

    nx *= scale
    ny *= scale

    // Lock motion depending on mode
    if (mode === "x") nx = 0
    if (mode === "y") ny = 0

    setKnob({ x: nx, y: ny })
  }

  function endDrag() {
    setDragging(false)
    setKnob({ x: 0, y: 0 })
    onEndRef.current?.()
  }

  function handlePointerDown(e: React.PointerEvent) {
    if (disabled) return
    ;(e.currentTarget as HTMLElement).setPointerCapture(e.pointerId)
    setDragging(true)
    updateFromPointer(e.clientX, e.clientY)
  }

  function handlePointerMove(e: React.PointerEvent) {
    if (disabled || !dragging) return
    updateFromPointer(e.clientX, e.clientY)
  }

  function handlePointerUp(e: React.PointerEvent) {
    if (disabled) return
    try {
      ;(e.currentTarget as HTMLElement).releasePointerCapture(e.pointerId)
    } catch {
      // ignore
    }
    endDrag()
  }

  function handlePointerCancel(e: React.PointerEvent) {
    if (disabled) return
    try {
      ;(e.currentTarget as HTMLElement).releasePointerCapture(e.pointerId)
    } catch {
      // ignore
    }
    endDrag()
  }

  const card: React.CSSProperties = {
    background: "rgba(255,255,255,0.06)",
    border: "1px solid rgba(255,255,255,0.10)",
    borderRadius: 14,
    padding: 16,
    backdropFilter: "blur(8px)",
  }

  const areaStyle: React.CSSProperties = {
    width: size,
    height: size,
    borderRadius: "50%",
    margin: "0 auto",
    position: "relative",
    background: "rgba(255,255,255,0.04)",
    border: "1px solid rgba(255,255,255,0.08)",
    touchAction: "none",
    opacity: disabled ? 0.5 : 1,
  }

  const knobStyle: React.CSSProperties = {
    width: Math.max(46, size * 0.25),
    height: Math.max(46, size * 0.25),
    borderRadius: "50%",
    position: "absolute",
    left: "50%",
    top: "50%",
    transform: `translate(calc(-50% + ${knob.x * (size * 0.32)}px), calc(-50% + ${
      knob.y * (size * 0.32)
    }px))`,
    background: dragging
      ? "rgba(124, 155, 255, 0.35)"
      : "rgba(255,255,255,0.08)",
    border: "1px solid rgba(255,255,255,0.12)",
    transition: dragging ? "none" : "transform 120ms ease",
  }

  return (
    <div style={card}>
      <h3 style={{ margin: 0, fontSize: 14 }}>{title}</h3>
      <p style={{ marginTop: 6, fontSize: 11, opacity: 0.7 }}>
        {description}
      </p>

      <div
        ref={areaRef}
        style={areaStyle}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerCancel}
        onPointerLeave={() => dragging && endDrag()}
      >
        <div style={knobStyle} />
      </div>

      {showYaw && (
        <div style={{ marginTop: 14 }}>
          <div
            style={{
              display: "flex",
              justifyContent: "space-between",
              fontSize: 11,
              opacity: 0.75,
            }}
          >
            <span>Yaw</span>
            <span>{(yawNorm * maxAngular).toFixed(2)}</span>
          </div>
          <input
            style={{ width: "100%" }}
            type="range"
            min={-100}
            max={100}
            value={yawNorm * 100}
            onChange={(e) => setYawNorm(Number(e.target.value) / 100)}
            disabled={disabled}
          />
        </div>
      )}

      {showReadout && (
        <div
          style={{
            marginTop: 12,
            fontSize: 11,
            opacity: 0.75,
            display: "grid",
            gap: 4,
            textAlign: "center",
          }}
        >
          <div>linearX: {value.linearX.toFixed(2)}</div>
          <div>linearY: {value.linearY.toFixed(2)}</div>
        </div>
      )}
    </div>
  )
}
