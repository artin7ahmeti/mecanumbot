import { useCallback, useEffect, useMemo, useRef, useState } from "react"
import JoystickPad from "../components/JoystickPad"
import { useRobot } from "../hook/useRobot"

const SEND_HZ = 20
const DEAD = 0.01

export default function Control() {
  const { publish } = useRobot()

  const [fb, setFb] = useState({ linearX: 0 })
  const [lr, setLr] = useState({ linearY: 0 })

  const handleFbChange = useCallback((v: { linearX: number }) => {
    setFb((prev) => (prev.linearX === v.linearX ? prev : { linearX: v.linearX }))
  }, [])

  const handleLrChange = useCallback((v: { linearY: number }) => {
    setLr((prev) => (prev.linearY === v.linearY ? prev : { linearY: v.linearY }))
  }, [])

  const handleFbEnd = useCallback(() => setFb({ linearX: 0 }), [])
  const handleLrEnd = useCallback(() => setLr({ linearY: 0 }), [])

  const last = useMemo(() => {
    return {
      linearX: fb.linearX,
      linearY: lr.linearY,
      angularZ: 0,
    }
  }, [fb.linearX, lr.linearY])

  // Keep latest command in a ref (avoid interval resets)
  const lastRef = useRef(last)

  useEffect(() => {
    lastRef.current = last
  }, [last])

  // Stable publish loop
  useEffect(() => {
    const id = window.setInterval(() => {
      const cmd = lastRef.current

      const nearZero =
        Math.abs(cmd.linearX) < DEAD &&
        Math.abs(cmd.linearY) < DEAD &&
        Math.abs(cmd.angularZ) < DEAD

      publish(
        nearZero
          ? { linearX: 0, linearY: 0, angularZ: 0 }
          : cmd
      )
    }, 1000 / SEND_HZ)

    return () => {
      window.clearInterval(id)
      // Safety: stop robot if leaving page/route
      publish({ linearX: 0, linearY: 0})
    }
  }, [publish])

  return (
    <div style={{ display: "grid", gap: 16 }}>
      <h1 style={{ margin: 0 }}>Control</h1>

      <div
        style={{
          display: "grid",
          gap: 16,
          gridTemplateColumns: "repeat(auto-fit, minmax(280px, 1fr))",
          alignItems: "start",
        }}
      >
        {/* Left stick: Forward / Backward */}
        <JoystickPad
          mode="x"
          showYaw={false}
          title="Left Stick (Forward / Back)"
          description="Mimics Xbox left stick vertical axis."
          onChange={(v) => handleFbChange({ linearX: v.linearX })}
          onEnd={handleFbEnd}
          size={200}
          showReadout
        />

        {/* Right stick: Left / Right strafe */}
        <JoystickPad
          mode="y"
          showYaw={false}
          title="Right Stick (Left / Right)"
          description="Mimics Xbox right stick horizontal strafe."
          onChange={(v) => handleLrChange({ linearY: v.linearY })}
          onEnd={handleLrEnd}
          size={200}
          showReadout
        />
      </div>

      {/* STOP = immediate zero */}
      <button
        onClick={() => {
          setFb({ linearX: 0 })
          setLr({ linearY: 0 })
          publish({ linearX: 0, linearY: 0 })
        }}
        style={{
          justifySelf: "start",
          padding: "10px 14px",
          borderRadius: 10,
          fontSize: 12,
          border: "1px solid rgba(255,80,80,0.45)",
          background: "rgba(255,80,80,0.16)",
          color: "#fff",
          cursor: "pointer",
        }}
      >
        STOP
      </button>

      <div style={{ fontSize: 12, opacity: 0.75 }}>
        Last command:{" "}
        {`x=${last.linearX.toFixed(2)}, y=${last.linearY.toFixed(
          2
        )}, yaw=${last.angularZ.toFixed(2)}`}
      </div>
    </div>
  )
}
