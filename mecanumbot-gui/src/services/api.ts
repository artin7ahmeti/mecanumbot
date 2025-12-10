// services/api.ts
export async function sendCmdVel(payload: {
  linearX: number
  linearY: number
}) {
  return fetch("/api/cmd_vel", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  })
}
