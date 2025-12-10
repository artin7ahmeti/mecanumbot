// services/rosbridgeClient.ts
import * as ROSLIB from "roslib"
import type { RobotClient, Twist } from "./robotClient"
import type { RobotState } from "../types/robot"

type Config = {
  url: string // ws://<robot-ip>:9090
  namespace?: string // default "/mecanumbot" (can also be "" for root topics)
  cmdVelTopic?: string // override command topic (default "/cmd_vel")
}

type BatteryStateMsg = {
  voltage?: number
  percentage?: number
}

type OdometryMsg = {
  pose?: {
    pose?: {
      position?: {
        x?: number
        y?: number
      }
    }
  }
}

type TwistMsg = {
  linear: { x: number; y: number }
  angular: { x: number; y: number }
}

const AGE_TICK_MS = 200

export function createRosbridgeClient(cfg: Config): RobotClient {

  const nsRaw = (cfg.namespace ?? "/mecanumbot").trim()
  const ns = nsRaw.replace(/\/$/, "")

  const topicName = (base: string) => {
    // Use namespace for state topics
    const cleanBase = base.replace(/^\//, "")
    if (!ns || ns === "/") return `/${cleanBase}`
    const cleanNs = ns.startsWith("/") ? ns : `/${ns}`
    return `${cleanNs}/${cleanBase}`
  }

  const normalizeAbs = (name: string) => {
    const n = name.trim()
    return n.startsWith("/") ? n : `/${n}`
  }

  const cmdVelName = normalizeAbs(cfg.cmdVelTopic ?? "/cmd_vel")

  let ros: ROSLIB.Ros | null = null
  const listeners = new Set<(s: RobotState) => void>()

  let batteryTopic: ROSLIB.Topic<BatteryStateMsg> | null = null
  let odomTopic: ROSLIB.Topic<OdometryMsg> | null = null
  let cmdVelTopic: ROSLIB.Topic<TwistMsg> | null = null

  // For connection health age calculation
  let lastSeenAt = 0
  let ageTimer: number | null = null

  // cached latest parts to compose RobotState
  let latest: Partial<RobotState> = {
    connected: false,
    lastSeenMs: Number.POSITIVE_INFINITY,
    battery: { percent: 0, voltage: 0 },
    odom: { x: 0, y: 0, yaw: 0 },
    velocity: { linearX: 0, linearY: 0 },
  }

  const client: RobotClient = {
    status: "disconnected",

    connect() {
      if (client.status === "connected" || client.status === "connecting") return
      client.status = "connecting"

      ros = new ROSLIB.Ros({ url: cfg.url })

      ros.on("connection", () => {
        client.status = "connected"
        latest = { ...latest, connected: true }
        emit()

        initPublishers()
        attachSubscriptions()
        startAgeTimer()
      })

      ros.on("close", () => {
        client.status = "disconnected"
        latest = {
          ...latest,
          connected: false,
          lastSeenMs: Number.POSITIVE_INFINITY,
        }
        stopAgeTimer()
        cleanupTopics()
        emit()
      })

      ros.on("error", () => {
        client.status = "error"
        latest = {
          ...latest,
          connected: false,
          lastSeenMs: Number.POSITIVE_INFINITY,
        }
        stopAgeTimer()
        cleanupTopics()
        emit()
      })
    },

    disconnect() {
      ros?.close()
      ros = null

      client.status = "disconnected"
      latest = {
        ...latest,
        connected: false,
        lastSeenMs: Number.POSITIVE_INFINITY,
      }

      stopAgeTimer()
      cleanupTopics()
      emit()
    },

    onState(cb) {
      listeners.add(cb)
      cb(latest as RobotState)
      return () => listeners.delete(cb)
    },

    publishCmd(cmd: Twist) {
      if (!ros || client.status !== "connected") return
      if (!cmdVelTopic) initPublishers()
      if (!cmdVelTopic) return

      const msg: TwistMsg = {
        linear: { x: cmd.linearX, y: cmd.linearY },
        angular: { x: 0, y: 0 },
      }

      cmdVelTopic.publish(msg)

      latest = {
        ...latest,
        velocity: {
          linearX: cmd.linearX,
          linearY: cmd.linearY,
        },
      }
      emit()
    },

    async startRecording(name?: string) {
      void name
      return crypto.randomUUID()
    },

    async stopRecording(sessionId: string) {
      void sessionId
    },

    async listRecordings() {
      return []
    },
  }

  function emit() {
    const state = latest as RobotState
    listeners.forEach((cb) => cb(state))
  }

  function markSeen() {
    lastSeenAt = performance.now()
    latest = { ...latest, lastSeenMs: 0 }
  }

  function startAgeTimer() {
    if (ageTimer) return
    ageTimer = window.setInterval(() => {
      const age =
        lastSeenAt > 0
          ? Math.max(0, performance.now() - lastSeenAt)
          : Number.POSITIVE_INFINITY
      latest = { ...latest, lastSeenMs: age }
      emit()
    }, AGE_TICK_MS)
  }

  function stopAgeTimer() {
    if (ageTimer) window.clearInterval(ageTimer)
    ageTimer = null
    lastSeenAt = 0
  }

  function initPublishers() {
    if (!ros || cmdVelTopic) return

    cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: cmdVelName,
      messageType: "geometry_msgs/Twist",
    })
  }

  function attachSubscriptions() {
    if (!ros) return

    batteryTopic = new ROSLIB.Topic({
      ros,
      name: topicName("battery_state"),
      messageType: "sensor_msgs/BatteryState",
    })

    odomTopic = new ROSLIB.Topic({
      ros,
      name: topicName("odom"),
      messageType: "nav_msgs/Odometry",
    })

    batteryTopic.subscribe((msg: BatteryStateMsg) => {
      markSeen()

      const rawPct = msg.percentage ?? 0
      const pct = rawPct <= 1 ? rawPct * 100 : rawPct

      latest = {
        ...latest,
        connected: true,
        battery: {
          percent: Math.max(0, Math.min(100, pct)),
          voltage: msg.voltage ?? 0,
        },
      }
      emit()
    })

    odomTopic.subscribe((msg: OdometryMsg) => {
      markSeen()

      const p = msg.pose?.pose?.position
      latest = {
        ...latest,
        odom: {
          x: p?.x ?? 0,
          y: p?.y ?? 0,
          yaw: 0,
        },
      }
      emit()
    })
  }

  function cleanupTopics() {
    try {
      batteryTopic?.unsubscribe()
    } catch {
      // ignore unsubscribe errors
    }
    try {
      odomTopic?.unsubscribe()
    } catch {
      // ignore unsubscribe errors
    }

    batteryTopic = null
    odomTopic = null
    cmdVelTopic = null
  }

  return client
}
