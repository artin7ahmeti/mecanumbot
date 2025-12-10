// hook/useRobot.ts
import { useCallback, useEffect, useMemo, useRef, useState } from "react"
import type { RobotState } from "../types/robot"
import type { Twist } from "../services/robotClient"
import { getClient } from "../services/clientFactory"
import { mockState } from "../services/mockState"

export function useRobot() {
  const client = useMemo(() => getClient(), [])
  const [state, setState] = useState<RobotState>(mockState)
  const [status, setStatus] = useState(client.status)

  // Keep a stable status update tick without setState sync in effect body
  const statusTimer = useRef<number | null>(null)

  useEffect(() => {
    client.connect()

    const off = client.onState(setState)

    if (!statusTimer.current) {
      statusTimer.current = window.setInterval(() => {
        setStatus(client.status)
      }, 250)
    }

    return () => {
      off?.()
      if (statusTimer.current) {
        window.clearInterval(statusTimer.current)
        statusTimer.current = null
      }

      // ✅ IMPORTANT:
      // Do NOT disconnect here.
      // Tab switching should not tear down the shared WS connection.
      // The app-level unmount is where you'd close it if needed.
    }
  }, [client])

  const publish = useCallback((cmd: Twist) => {
    client.publishCmd(cmd)
  }, [client])

  const startRecording = useCallback((name?: string) => {
    return client.startRecording(name)
  }, [client])

  const stopRecording = useCallback((id: string) => {
    return client.stopRecording(id)
  }, [client])

  const listRecordings = useCallback(() => {
    return client.listRecordings()
  }, [client])

  return {
    state,
    status,
    publish,
    startRecording,
    stopRecording,
    listRecordings,
  }
}
