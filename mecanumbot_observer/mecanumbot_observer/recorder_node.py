import os
import signal
import subprocess
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

DEFAULT_TOPICS = [
    "/mecanumbot/battery_state",
    "/mecanumbot/odom",
    "/mecanumbot/imu",
    "/cmd_vel",
]


class MecanumbotRecorder(Node):
    def __init__(self):
        super().__init__("mecanumbot_observer_recorder")

        # Parameters
        self.declare_parameter(
            "output_dir",
            os.path.expanduser("~/mecanumbot_recordings"),
        )
        self.declare_parameter("topics", DEFAULT_TOPICS)

        self.output_dir = self.get_parameter("output_dir").value
        self.topics = list(self.get_parameter("topics").value)
        os.makedirs(self.output_dir, exist_ok=True)

        # State
        self.proc = None
        self.current_path = ""

        # Initialize counter from existing session_XXX dirs (3-digit suffix only)
        self.session_counter = self._init_session_counter()

        # Services: start, stop, list
        self.start_srv = self.create_service(
            Trigger,
            "/observer/start_recording",
            self.handle_start,
        )
        self.stop_srv = self.create_service(
            Trigger,
            "/observer/stop_recording",
            self.handle_stop,
        )
        self.list_srv = self.create_service(
            Trigger,
            "/observer/list_recordings",
            self.handle_list,
        )

        self.get_logger().info(
            f"Recorder ready. Output dir: {self.output_dir} "
            f"(starting from session {self.session_counter:03d})"
        )

    # ---------- helpers ----------

    def _init_session_counter(self) -> int:
        """
        Look in output_dir for existing session_<NNN> folders (3-digit index)
        and return the next number (max+1).

        Older timestamp-style folders like session_1765375454897 are ignored.
        """
        max_idx = 0
        for name in os.listdir(self.output_dir):
            full = os.path.join(self.output_dir, name)
            if not os.path.isdir(full):
                continue
            if not name.startswith("session_"):
                continue

            tail = name[len("session_"):]  # part after "session_"
            # Only count 3-digit purely numeric tails, e.g. "001", "007"
            if not (len(tail) == 3 and tail.isdigit()):
                continue

            idx = int(tail)
            if idx > max_idx:
                max_idx = idx

        return max_idx + 1 if max_idx > 0 else 1

    def _allocate_new_session_dir(self) -> str:
        """
        Choose a new unique session path: session_001, session_002, ...
        We DO NOT create the directory here; ros2 bag will create it.
        """
        while True:
            session_name = f"session_{self.session_counter:03d}"
            self.session_counter += 1

            path = os.path.join(self.output_dir, session_name)
            if not os.path.exists(path):
                return path
            else:
                # Somehow this name exists already; try the next one
                self.get_logger().warn(
                    f"Session dir {path} already exists, trying next index..."
                )

    def _get_dir_size_bytes(self, path: str) -> int:
        total = 0
        for root, _, files in os.walk(path):
            for f in files:
                fp = os.path.join(root, f)
                try:
                    total += os.path.getsize(fp)
                except Exception:
                    pass
        return total

    # ---------- service handlers ----------

    def handle_start(self, req, res):
        if self.proc:
            res.success = False
            res.message = "Recording already active"
            return res

        # Pick a fresh session path; ros2 bag will create the directory
        self.current_path = self._allocate_new_session_dir()

        cmd = ["ros2", "bag", "record", "-o", self.current_path, *self.topics]
        self.get_logger().info(f"Starting rosbag: {' '.join(cmd)}")

        try:
            self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        except Exception as e:
            self.proc = None
            res.success = False
            res.message = f"Failed to start: {e}"
            return res

        res.success = True
        # Return just the session id (e.g. "session_001") for the web app
        res.message = os.path.basename(self.current_path)
        return res

    def handle_stop(self, req, res):
        if not self.proc:
            res.success = False
            res.message = "No active recording"
            return res

        self.get_logger().info("Stopping rosbag...")
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            except Exception:
                pass

        path = self.current_path
        self.proc = None
        self.current_path = ""

        res.success = True
        res.message = f"Recording stopped: {path}"
        return res

    def handle_list(self, req, res):
        """
        List all session_* folders as JSON for the web GUI.

        Response.message = JSON string:
        [
          {
            "id": "session_001",
            "name": "session_001",
            "startedAt": "2025-12-10T15:30:00",
            "endedAt": null,
            "sizeBytes": 12345
          },
          ...
        ]
        """
        sessions = []

        try:
            if not os.path.isdir(self.output_dir):
                os.makedirs(self.output_dir, exist_ok=True)

            # Collect session_* folders
            names = []
            for name in os.listdir(self.output_dir):
                full = os.path.join(self.output_dir, name)
                if os.path.isdir(full) and name.startswith("session_"):
                    names.append(name)

            # Sort newest first by numeric suffix if possible
            def key_fn(n):
                tail = n[len("session_"):]
                try:
                    return int(tail)
                except ValueError:
                    return 0

            names.sort(key=key_fn, reverse=True)

            for name in names:
                full = os.path.join(self.output_dir, name)

                # Approx startedAt from folder mtime
                try:
                    started_ts = os.path.getmtime(full)
                    started_at = datetime.fromtimestamp(started_ts).isoformat()
                except Exception:
                    started_at = datetime.now().isoformat()

                size_bytes = self._get_dir_size_bytes(full)

                sessions.append({
                    "id": name,
                    "name": name,
                    "startedAt": started_at,
                    "endedAt": None,  # not tracking exact stop time yet
                    "sizeBytes": size_bytes if size_bytes > 0 else None,
                })

            # Ensure active session is present even if just started
            if self.proc and self.current_path:
                active_id = os.path.basename(self.current_path)
                if not any(s["id"] == active_id for s in sessions):
                    sessions.insert(0, {
                        "id": active_id,
                        "name": active_id,
                        "startedAt": datetime.now().isoformat(),
                        "endedAt": None,
                        "sizeBytes": None,
                    })

            res.success = True
            res.message = json.dumps(sessions)
            return res

        except Exception as e:
            res.success = False
            res.message = f"List failed: {e}"
            return res


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
