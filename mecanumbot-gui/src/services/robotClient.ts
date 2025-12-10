import type { RobotState } from "../types/robot"

export type Twist = {
  linearX: number
  linearY: number
}

export type ClientStatus =
  | "disconnected"
  | "connecting"
  | "connected"
  | "error"

export interface RobotClient {
  status: ClientStatus
  connect(): void
  disconnect(): void
  onState(cb: (state: RobotState) => void): () => void
  publishCmd(cmd: Twist): void

  startRecording(name?: string): Promise<string>
  stopRecording(sessionId: string): Promise<void>
  listRecordings(): Promise<
    {
      id: string
      name: string
      startedAt: string
      endedAt?: string
      sizeBytes?: number
    }[]
  >
}
