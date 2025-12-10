# Mecanumbot Observer
---

## Requirements

- Build an easy to use GUI to control and observe the Mecanumbot and record measurement   
- Easy reconfiguration possibility for the controller  
- Monitor battery state  
- Use a controller to command the robot  
- Pack the project into a Docker container
- Use a web browser to create the GUI   
- Be usable on Windows and Linux as well  
- Inner code is a ROS2 package mainly using Python  
- Have the interface and connection possibility to connect the external camera system  
- Include a save function and figure out how to store the measurements  
- Clear documentation  

---


## Tech Stack (GUI)

### Frontend (React)

**Pages**
- `Dashboard`
  - Battery card (percent + voltage)
  - Odom card
  - Connection badge
- `Control`
  - Two joystick pads matching intended Xbox stick usage:
    - **Left stick** (virtual) → forward/back (`linearX`)
    - **Right stick** (virtual) → left/right strafe (`linearY`)
  - Stable publish loop (10 Hz)
  - STOP button
- `Recording`
  - Mock sessions list + start/stop controls
- `Camera`
  - External camera stream URL input + preview

### Data/Control Abstraction

We implemented a clean contract so the UI doesn’t care where data comes from.

- `RobotClient` interface
- `MockClient` for UI development
- `RosbridgeClient` (WebSocket via `roslib`) for lab use
- `useRobot()` hook that exposes:
  - `state`
  - `status`
  - `connected`
  - `publish()`


## Project Structure (GUI)

```
mecanumbot-gui/
  src/
    pages/
      Dashboard.tsx
      Control.tsx
      Recording.tsx
      Camera.tsx
    components/
      BatteryCard.tsx
      OdomCard.tsx
      ConnectionBadge.tsx
      JoystickPad.tsx
      RecordControls.tsx
      SessionList.tsx
      background/FloatingLines.tsx
                 FloatingLines.css
    services/
      robotClient.ts
      mockClient.ts
      rosbridgeClient.ts
      clientFactory.ts
      mockState.ts
    hook/
      useRobot.ts
    types/
      robot.ts
```


## Running the GUI Locally
After cloning the repository:
```bash
cd mecanumbot-gui
npm install
npm run dev
```

## Routing

We use canonical page routes:

- `/dashboard`
- `/control`
- `/recording`
- `/camera`

And redirect rules:

- `/` → `/dashboard`
- `*` → `/dashboard`

---

## Mode Handling

We keep mode switching minimal for clean UI.

### Default
- **Mock mode** is default.

### Switch to ROS Mode (Lab)

In browser DevTools Console:

```js
localStorage.setItem("mb_mode", "ros")
localStorage.setItem("mb_ws_url", "ws://<ROBOT-IP>:9090")
localStorage.setItem("mb_ns", "/mecanumbot")
location.reload()
```

### Return to Mock Mode

```js
localStorage.setItem("mb_mode", "mock")
location.reload()
```
---
## Notes

- The GUI is **mock-first** for testing and UI design purposes.
- Real-data mode is enabled via `localStorage` to keep the interface minimal.
