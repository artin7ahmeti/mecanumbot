export type BatteryState = {
  percent: number
  voltage: number
}

export type OdomState = {
  x: number
  y: number
  yaw: number
}

export type VelocityState = {
  linearX: number
  linearY: number
}

export type RobotState = {
  connected: boolean
  lastSeenMs: number
  battery: BatteryState
  odom: OdomState
  velocity: VelocityState
}
