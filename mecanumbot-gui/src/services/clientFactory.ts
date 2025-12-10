// services/clientFactory.ts
import type { RobotClient } from "./robotClient"
import { createMockClient } from "./mockClient"
import { createRosbridgeClient } from "./rosbridgeClient"

function createNewClient(): RobotClient {
  const rawMode = (localStorage.getItem("mb_mode") ?? "ros").trim()
  const mode = rawMode === "ros" ? "ros" : "mock"

  const url = (localStorage.getItem("mb_ws_url") ?? "ws://192.168.1.240:9090").trim()
  const ns = (localStorage.getItem("mb_ns") ?? "/mecanumbot").trim()

  if (mode === "ros") {
    return createRosbridgeClient({ url, namespace: ns })
  }

  return createMockClient()
}

// ✅ Singleton instance shared across the app
let singleton: RobotClient | null = null

export function getClient(): RobotClient {
  if (!singleton) singleton = createNewClient()
  return singleton
}

// ✅ Keep old API working
export function createClient(): RobotClient {
  return getClient()
}

// Optional utility if you ever need to rebuild the client
export function resetClient() {
  singleton = null
}
