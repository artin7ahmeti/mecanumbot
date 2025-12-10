import { useCallback, useEffect, useMemo, useState } from "react"
import RecordControls from "../components/RecordControls"
import SessionList, { type Session } from "../components/SessionList"
import { useRobot } from "../hook/useRobot"

const LS_ACTIVE_ID = "mb_active_recording_id"

function readActiveId(): string | null {
  const v = localStorage.getItem(LS_ACTIVE_ID)
  const t = v?.trim()
  return t ? t : null
}

function writeActiveId(id: string) {
  localStorage.setItem(LS_ACTIVE_ID, id)
}

function clearActiveId() {
  localStorage.removeItem(LS_ACTIVE_ID)
}

export default function Recording() {
  const { startRecording, stopRecording, listRecordings } = useRobot()

  const [activeId, setActiveId] = useState<string | null>(() => readActiveId())
  const [sessions, setSessions] = useState<Session[]>([])

  const isRecording = useMemo(() => Boolean(activeId), [activeId])

  const mapList = useCallback((list: Array<{ id: string | number; name?: string; startedAt?: string; endedAt?: string; sizeBytes?: number }>): Session[] => {
    return list.map((r) => ({
      id: String(r.id),
      name: String(r.name ?? r.id),
      startedAt: String(r.startedAt ?? new Date().toISOString()),
      endedAt: r.endedAt ? String(r.endedAt) : undefined,
      sizeBytes: typeof r.sizeBytes === "number" ? r.sizeBytes : undefined,
    }))
  }, [])

  const buildSessions = useCallback(
    (list: Array<{ id: string | number; name?: string; startedAt?: string; endedAt?: string; sizeBytes?: number }>, id: string | null) => {
      const mapped = mapList(list)

      // If we have an active id but it isn't listed yet,
      // keep UI consistent with a placeholder.
      if (id && !mapped.some((s) => s.id === id)) {
        mapped.unshift({
          id,
          name: id,
          startedAt: new Date().toISOString(),
        })
      }

      return mapped
    },
    [mapList]
  )

  // ✅ A refresh helper for event handlers (focus, after stop)
  const refreshSessions = useCallback(
    async (preferredActiveId?: string | null) => {
      try {
        const list = await listRecordings()
        const id = preferredActiveId ?? activeId ?? readActiveId()
        setSessions(buildSessions(list, id))
      } catch {
        // ok in mock/early states
      }
    },
    [listRecordings, buildSessions, activeId]
  )

  // ✅ Rehydrate from ROS on mount + whenever activeId changes
  // FIX: avoid calling a state-setting helper directly in effect body
  useEffect(() => {
    let mounted = true

    ;(async () => {
      try {
        const list = await listRecordings()
        if (!mounted) return

        const id = activeId ?? readActiveId()
        const next = buildSessions(list, id)
        setSessions(next)
      } catch {
        // ok
      }
    })()

    return () => {
      mounted = false
    }
  }, [listRecordings, activeId, buildSessions])

  // ✅ Refresh when user returns to window/tab
  useEffect(() => {
    const onFocus = () => {
      refreshSessions(readActiveId())
    }
    window.addEventListener("focus", onFocus)
    return () => window.removeEventListener("focus", onFocus)
  }, [refreshSessions])

  const start = useCallback(
    async (name?: string) => {
      try {
        const id = await startRecording(name)

        writeActiveId(id)
        setActiveId(id)

        // Optimistic insert/update on top
        setSessions((prev) => {
          const clean = prev.filter((s) => s.id !== id)
          const session: Session = {
            id,
            name: name?.trim() || id,
            startedAt: new Date().toISOString(),
          }
          return [session, ...clean]
        })
      } catch (err: unknown) {
        const msg = String((err as Error)?.message ?? err).toLowerCase()

        // ✅ Node contract: "Recording already active"
        if (msg.includes("recording already active")) {
          const existing = readActiveId()

          if (existing) {
            setActiveId(existing)
            refreshSessions(existing)
            return
          }

          // Recover UI state if localStorage was cleared
          await refreshSessions(null)

          // Last resort placeholder
          const fallback = "active_session"
          writeActiveId(fallback)
          setActiveId(fallback)
          setSessions((prev) => [
            {
              id: fallback,
              name: "Active session",
              startedAt: new Date().toISOString(),
            },
            ...prev,
          ])
          return
        }

        throw err
      }
    },
    [startRecording, refreshSessions]
  )

  const stop = useCallback(async () => {
    const id = activeId ?? readActiveId()
    if (!id) return

    await stopRecording(id)

    clearActiveId()
    setActiveId(null)

    // Optimistic UI end stamp
    setSessions((prev) =>
      prev.map((s) =>
        s.id === id && !s.endedAt
          ? { ...s, endedAt: new Date().toISOString() }
          : s
      )
    )

    // Refresh sizes/metadata from disk if list service provides it
    refreshSessions(null)
  }, [activeId, stopRecording, refreshSessions])

  return (
    <div style={{ display: "grid", gap: 16, backdropFilter: "blur(20px)" }}>
      <h1 style={{ margin: 0 }}>Recording</h1>

      <RecordControls
        isRecording={isRecording}
        onStart={start}
        onStop={stop}
      />

      <SessionList sessions={sessions} />
    </div>
  )
}
