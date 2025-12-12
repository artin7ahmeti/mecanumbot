# Mecanumbot Observer
---

### Web-based GUI and ROS 2 tools for observing and controlling the **Mecanumbot** robot, monitoring its state, accessing an external camera and recording measurement sessions.
---
The project consists of two main parts:

- **`mecanumbot-gui/`** – React + TypeScript web UI (runs in a browser).
- **`mecanumbot_observer/`** – ROS 2 (Humble) Python package with a recorder node that wraps `ros2 bag record` and exposes simple services.

## 1. High‑Level Architecture

```text
Browser (React + TypeScript)
  ├─ Control page (joystick → /cmd_vel)
  ├─ Dashboard (battery + odom)
  ├─ Recording page (start/stop + session list)
  └─ Camera page (MJPEG from web_video_server)
          │
          │  WebSocket (rosbridge, ws://<robot-ip>:9090)
          ▼
ROS 2 robot (mecanumbot stack)
  ├─ mecanumbot core nodes (IO, sensors, odom, etc.)
  ├─ mecanumbot_observer_recorder
  │    ├─ /observer/start_recording (std_srvs/Trigger)
  │    ├─ /observer/stop_recording  (std_srvs/Trigger)
  │    └─ /observer/list_recordings (std_srvs/Trigger)
  ├─ rosbridge_websocket (rosbridge_server)
  └─ web_video_server  →  HTTP MJPEG stream (/camera/image_color)
```

## 1.1. Project Structure (GUI)
### Tech Stack: React & TypeScript

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
  App.css
  App.tsx
  index.css
  main.tsx
```
## 1.2. Routing

We use canonical page routes:

- `/dashboard`
- `/control`
- `/recording`
- `/camera`
## 2. ROS 2 Side

### 2.1. Dependencies

On the **robot / ROS machine**:

- ROS 2 Humble
- Mecanumbot stack (any topic can be provided, to list a few):
  - `/mecanumbot/battery_state` (`sensor_msgs/BatteryState`)
  - `/mecanumbot/odom` (`nav_msgs/Odometry`)
  - `/mecanumbot/imu` (`sensor_msgs/Imu`)
  - `/cmd_vel` (`geometry_msgs/Twist`)
- Packages:
  - `rosbridge_server`
  - `web_video_server`
  - `rosbag2`
  - External camera node (`basler_camera` in our case)

### 2.2. Build the `mecanumbot_observer` package

Place the package in a ROS workspace, then:

```bash
cd ~/dev_ws
colcon build --packages-select mecanumbot_observer
source install/setup.bash
```

### 2.3. Start the core robot stack

Bringup the robot:

```bash
# Start the mecanumbot
ssh_mecanum
start_robot
```
### 2.5. Start rosbridge (WebSocket bridge)

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
# WebSocket endpoint: ws://<robot-ip>:9090
```

### 2.6. Start external camera + web_video_server

External camera node (Basler in our case):

```bash
ros2 run basler_camera basler_camera
```

MJPEG streaming for the GUI:

```bash
ros2 run web_video_server web_video_server
```

Typical stream URL used in the GUI:

```text
http://<host-ip>:8080/stream?topic=/camera/image_color
```

You can test it directly in a browser first.


### 2.4. Start the recorder node

```bash
ros2 run mecanumbot_observer mecanumbot_recorder
# Services:
#   /observer/start_recording  (std_srvs/Trigger)
#   /observer/stop_recording   (std_srvs/Trigger)
#   /observer/list_recordings  (std_srvs/Trigger)
```

By default the node:

- Writes sessions under: `~/mecanumbot_recordings/`
- Records topics:
  - `/mecanumbot/battery_state`
  - `/mecanumbot/odom`
  - `/mecanumbot/imu`
  - `/cmd_vel`
- Creates one folder per session: `session_<timestamp>` or `session_<NNN>` containing rosbag2 data.

## 3. Web GUI (`mecanumbot-gui`)

### 3.1. Requirements

On the **GUI / development machine**:

- Node.js >= 18
- npm / pnpm / yarn (examples use `npm`)

### 3.2. Install & run the GUI

```bash
cd mecanumbot-gui

# Install dependencies
npm install

# Development server (accessible from LAN)
npm run dev
```

Open in a browser (if from the same machine):

```text
http://localhost:5173
```

From another machine on the same network:

```text
http://<host-ip>:5173
```


## 5. GUI Features

### 5.1. Dashboard

- Shows connection status (via `rosbridge_client`).
- Displays:
  - Battery percentage and voltage (from `/mecanumbot/battery_state`).
  - Odometry pose (x, y, yaw=0 for now) from `/mecanumbot/odom`.

### 5.2. Control Page

- Two virtual joysticks mapped to a `Twist` command:
  - Left stick → `linear.x` (forward/backward).
  - Right stick → `linear.y` (left/right strafe).
- Commands published at a fixed rate (**10 Hz**) to `/cmd_vel` through rosbridge.
- A **STOP** button immediately sends zero velocity.

Implementation detail:

- The front‑end uses a `RobotClient` abstraction with a `publishCmd()` method.
- In ROS mode this calls `roslib.Topic(...).publish()` on `/cmd_vel`.

### 5.3. Recording Page

- Start/stop measurement sessions using the recorder node:
  - Start → calls `/observer/start_recording` (`std_srvs/Trigger`).
  - Stop  → calls `/observer/stop_recording`.
- Sessions list is fetched from `/observer/list_recordings` and shows:
  - Session ID / folder name.
  - Start time and end time.
  - Approximate size in bytes (once recording has finished).

Recordings end up as standard rosbag2 folders under `~/mecanumbot_recordings/` and can be replayed with:

```bash
ros2 bag info ~/mecanumbot_recordings/session_001
ros2 bag play ~/mecanumbot_recordings/session_001
```

### 5.4. Camera Page

- Input for **external camera stream URL**.
- Shows the MJPEG stream via a simple `<img>` tag.
- Typical URL, when `web_video_server` and `basler_camera` are running:

```text
http://<host-ip>:8080/stream?topic=/camera/image_color
```

## 6. Quick Command Cheat‑Sheet

### For the robot

#### Terminal 1
```bash
# SSH into the robot
ssh_mecanum

# Start mecanumbot
start_robot
```

#### Terminal 2
```bash
ssh_mecanum

# Start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

### In the workspace 
#### Terminal 3
```bash
cd ~/dev_ws

source_ros
source_ws

# Start external camera
ros2 run basler_camera basler_camera
```

#### Terminal 4
```bash
cd ~/dev_ws

source_ros
source_ws

# Start web_video_server
ros2 run web_video_server web_video_server
```

#### Terminal 5
```bash
cd ~/dev_ws

source_ros
source_ws

# Start recorder node
ros2 run mecanumbot_observer mecanumbot_recorder
```

### Run the GUI
#### Terminal 6
```bash
cd mecanumbot-gui
npm install
npm run dev
```
Then open ```http://localhost:5173``` (or ```http://<host-ip>:5173``` if on another machine over the same network) in a browser.

## 7. Future Improvements

- Implement file download integration in the GUI (e.g. zipped bag download).
- Dockerize:
  - A container for the GUI.
  - A container for the observer/recorder node, rosbridge, and web_video_server.
- Add additional metrics to the dashboard (IMU status, diagnostics, etc.).
- A launch file to make it easier to run everything we need at once with a command.
- Optional: keyboard controller input mapped to the same `cmd_vel` pipeline.


