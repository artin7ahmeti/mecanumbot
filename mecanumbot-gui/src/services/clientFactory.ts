// services/clientFactory.ts
import type { RobotClient } from "./robotClient"
import { createMockClient } from "./mockClient"
import { createRosbridgeClient } from "./rosbridgeClient"

export function createClient(): RobotClient {
  const rawMode = (localStorage.getItem("mb_mode") ?? "ros").trim()
  const mode = rawMode === "ros" ? "ros" : "mock"

  const url = (localStorage.getItem("mb_ws_url") ?? "ws://192.168.1.240:9090").trim()
  const ns = (localStorage.getItem("mb_ns") ?? "/mecanumbot").trim()

  const cmdVelTopic = (localStorage.getItem("mb_cmd_vel") ?? "/cmd_vel").trim()

  if (mode === "ros") {
    return createRosbridgeClient({
      url,
      namespace: ns,
      cmdVelTopic,
    })
  }

  
  return createMockClient()
}
