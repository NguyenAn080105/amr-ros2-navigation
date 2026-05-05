#!/usr/bin/env python3
"""
navigator.py
==========================================
Redesigned state machine with 4 explicit control commands:

    go <id>   -> Navigate to a checkpoint
    stop      -> Stop immediately (can be resumed)
    continue  -> Resume a stopped journey
    reset     -> Cancel journey, wait 30s, then auto-return Home

Main Flow:
  IDLE ──go──► COMPUTING_PATH ──► PRE_ROTATING ──► NAVIGATING
                                                        │
                                            succeeded ──┘─► IDLE
                                            stop     ──────► STOPPED
                                                               │
                                                    continue ──┤─► COMPUTING_PATH
                                                    reset    ──┘─► WAITING_RESET
                                                                        │
                                                            go ─────────┤─► COMPUTING_PATH
                                                            30s timeout ──► RETURNING_HOME
Topics:
  Sub: /robot/command  (std_msgs/String) — "go:1" | "stop" | "continue" | "reset"
  Pub: /robot/state    (std_msgs/String)
  Pub: /robot/current_checkpoint (std_msgs/Int32)
  Pub: /robot/status_message     (std_msgs/String)
  Pub: /cmd_vel        (geometry_msgs/Twist) — Used only during PRE_ROTATING
"""

import math
import os
import time
from enum import Enum

import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros
import yaml

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from std_msgs.msg import Int32, String


# ============================================================
#  TUNING PARAMETERS
# ============================================================

# Pre-rotate configuration
PRE_ROTATE_THRESHOLD = 0.40     # [rad] (~23°) Trigger threshold for in-place rotation
PRE_ROTATE_STOP_THR  = 0.05     # [rad] (~3°) Alignment tolerance to stop rotation
PRE_ROTATE_LOOKAHEAD = 0.50     # [m] Lookahead distance on path for initial heading computation
PRE_ROTATE_KP        = 1.5      # P-gain for in-place rotation controller
PRE_ROTATE_MAX_W     = 0.80     # [rad/s] Maximum angular velocity
PRE_ROTATE_MIN_W     = 0.15     # [rad/s] Minimum angular velocity (to overcome static friction)
PRE_ROTATE_TIMEOUT   = 12.0     # [s] Maximum time allowed for pre-rotation

# Home retry (when RETURNING_HOME is aborted)
HOME_RETRY_MAX       = 3        # Maximum retry attempts
HOME_RETRY_DELAY_S   = 5.0      # [s] Delay between retry attempts

# Action server timeouts
NAV2_SERVER_TIMEOUT_S = 30.0    # [s] Max wait time for Nav2 action servers

# Reset wait timeout (before auto-returning home)
RESET_WAIT_TIMEOUT_S  = 30      # [s] Countdown timer (1s tick resolution)

class State(Enum):
    IDLE           = "IDLE"
    COMPUTING_PATH = "COMPUTING_PATH"
    PRE_ROTATING   = "PRE_ROTATING"
    NAVIGATING     = "NAVIGATING"
    STOPPED        = "STOPPED" 
    WAITING_RESET  = "WAITING_RESET" 
    RETURNING_HOME = "RETURNING_HOME"

# States where the robot is actively moving/processing -> 'go' command is blocked
_BUSY_STATES = {
    State.COMPUTING_PATH,
    State.PRE_ROTATING,
    State.NAVIGATING,
    State.RETURNING_HOME,
}

# States where the 'go' command is valid
_GO_VALID_STATES = {State.IDLE, State.WAITING_RESET}

# States where the 'stop' command is valid
_STOP_VALID_STATES = {State.COMPUTING_PATH, State.PRE_ROTATING, State.NAVIGATING}

class CheckpointNavigator(Node):
    def __init__(self):
        super().__init__("checkpoint_navigator")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("checkpoint_file",    "")
        self.declare_parameter("home_checkpoint_id", 0)

        self.home_id = self.get_parameter("home_checkpoint_id").value

        self.checkpoints = self._load_checkpoints()
        if not self.checkpoints:
            self.get_logger().error("No checkpoints loaded. Navigator exiting.")
            return

        # ── State variables ──────────────────────────────────────────────────
        self.state        = State.IDLE
        self.current_cp   = -1
        self.target_cp    = None
        self.goal_handle  = None

        # Pre-rotate
        self._target_yaw       = 0.0
        self._pre_rotate_start = None
        self._pending_cp_id    = None

        # Stop/continue/reset flags
        self._stop_requested   = False 
        self._intentional_cancel = Fals
        self._saved_target_cp  = None

        # Returning home
        self._is_returning_home = False
        self._home_retry_count  = 0

        # WAITING_RESET timer state
        self._reset_elapsed    = 0
        self._reset_timer      = None   # timer handle

        # ── TF ──────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Action clients ───────────────────────────────────────────────────
        self._nav     = ActionClient(self, NavigateToPose,    "navigate_to_pose")
        self._planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

        self.get_logger().info(
            f"Waiting for Nav2 action servers (timeout={NAV2_SERVER_TIMEOUT_S:.0f}s)...")
        nav_ok     = self._nav.wait_for_server(timeout_sec=NAV2_SERVER_TIMEOUT_S)
        planner_ok = self._planner.wait_for_server(timeout_sec=NAV2_SERVER_TIMEOUT_S)

        if not nav_ok:
            raise RuntimeError(
                f"navigate_to_pose not available after {NAV2_SERVER_TIMEOUT_S:.0f}s. "
                "Is Nav2 running?")
        if not planner_ok:
            raise RuntimeError(
                f"compute_path_to_pose not available after {NAV2_SERVER_TIMEOUT_S:.0f}s. "
                "Is planner_server running?")

        self.get_logger().info("Nav2 action servers ready ✓")

        # ── Publishers ───────────────────────────────────────────────────────
        self._state_pub  = self.create_publisher(String, "/robot/state",              10)
        self._cp_pub     = self.create_publisher(Int32,  "/robot/current_checkpoint", 10)
        self._status_pub = self.create_publisher(String, "/robot/status_message",     10)
        self._cmdvel_pub = self.create_publisher(Twist,  "/cmd_vel",                  10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String, "/robot/command", self._on_command, 10)

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(0.1, self._state_machine_tick)  # 10 Hz — pre-rotate loop
        self.create_timer(1.0, self._publish_state)       # 1 Hz  — state broadcast

        self.get_logger().info(
            f"CheckpointNavigator v2 ready │ "
            f"{len(self.checkpoints)} checkpoints │ "
            f"home_id={self.home_id} │ "
            f"pre_rotate_thr={math.degrees(PRE_ROTATE_THRESHOLD):.0f}° │ "
            f"reset_wait={RESET_WAIT_TIMEOUT_S}s"
        )

    # ════════════════════════════════════════════════════════
    #  CHECKPOINT LOADING
    # ════════════════════════════════════════════════════════

    def _load_checkpoints(self) -> dict:
        path = self.get_parameter("checkpoint_file").value
        if not path or not os.path.exists(path):
            try:
                pkg  = get_package_share_directory("mobile_robot")
                path = os.path.join(pkg, "config", "checkpoints_v2.yaml")
            except Exception:
                self.get_logger().error("Package 'mobile_robot' not found.")
                return {}

        if not os.path.exists(path):
            self.get_logger().error(f"Checkpoint file not found: {path}")
            return {}

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        result = {}
        for cp in data.get("checkpoints", []):
            pose = PoseStamped()
            pose.header.frame_id    = data.get("frame_id", "map")
            pose.pose.position.x    = float(cp["position"]["x"])
            pose.pose.position.y    = float(cp["position"]["y"])
            pose.pose.position.z    = float(cp["position"].get("z", 0.0))
            pose.pose.orientation.x = float(cp["orientation"].get("x", 0.0))
            pose.pose.orientation.y = float(cp["orientation"].get("y", 0.0))
            pose.pose.orientation.z = float(cp["orientation"].get("z", 0.0))
            pose.pose.orientation.w = float(cp["orientation"].get("w", 1.0))
            result[cp["id"]] = {
                "id":   cp["id"],
                "name": cp.get("display_name", cp["name"]),
                "pose": pose,
            }
        self.get_logger().info(
            f"Loaded {len(result)} checkpoints from {path}")
        return result

    # ════════════════════════════════════════════════════════
    #  COMMAND DISPATCHER
    # ════════════════════════════════════════════════════════

    def _on_command(self, msg: String):
        """
        Receives control commands from /robot/command.
        Format: "go:<id>" | "stop" | "continue" | "reset"
        """
        raw = msg.data.strip().lower()
        self.get_logger().info(
            f"[CMD] Received: '{raw}' │ Current state: {self.state.name}")

        if raw.startswith("go:"):
            self._cmd_go(raw)
        elif raw == "stop":
            self._cmd_stop()
        elif raw == "continue":
            self._cmd_continue()
        elif raw == "reset":
            self._cmd_reset()
        else:
            self.get_logger().warn(
                f"[CMD] Unknown command: '{raw}'. "
                "Valid: 'go:<id>', 'stop', 'continue', 'reset'")

    # ════════════════════════════════════════════════════════
    #  COMMAND: go
    # ════════════════════════════════════════════════════════

    def _cmd_go(self, raw: str):
        """
        go:<id> — Navigate to checkpoint id.
        Valid in: IDLE, WAITING_RESET
        """
        # Parse ID
        try:
            cp_id = int(raw.split(":")[1])
        except (IndexError, ValueError):
            self.get_logger().error(
                f"[GO] Invalid format '{raw}'. Expected 'go:<int>'")
            return

        # Validate checkpoint
        if cp_id not in self.checkpoints:
            self.get_logger().error(
                f"[GO] Checkpoint {cp_id} not found. "
                f"Valid: {sorted(self.checkpoints.keys())}")
            return

        # Check state
        if self.state in _BUSY_STATES:
            self.get_logger().warn(
                f"[GO] Rejected: robot busy in {self.state.name} → "
                f"navigating to [{self.target_cp}]. Use 'stop' first.")
            self._pub_status(
                f"Busy ({self.state.name}). Use 'stop' first.")
            return

        if self.state == State.STOPPED:
            self.get_logger().warn(
                "[GO] Rejected: robot is STOPPED. Use 'continue' or 'reset'.")
            self._pub_status("Robot STOPPED. Use 'continue' or 'reset'.")
            return

        if self.state not in _GO_VALID_STATES:
            self.get_logger().warn(
                f"[GO] Rejected: not valid in state {self.state.name}.")
            return

        # WAITING_RESET → cancel timer 30s before excecute go command
        if self.state == State.WAITING_RESET:
            self._cancel_reset_timer()
            self.get_logger().info(
                f"[GO] Received during WAITING_RESET → canceling reset timer.")

        # If already at the target checkpoint and in IDLE state
        if self.state == State.IDLE and self.current_cp == cp_id:
            self.get_logger().info(
                f"[GO] Already at checkpoint [{cp_id}] "
                f"'{self.checkpoints[cp_id]['name']}'.")
            return

        self.get_logger().info(
            f"[GO] ✓ Navigating to [{cp_id}] "
            f"'{self.checkpoints[cp_id]['name']}'")

        # Reset returning-home flags when a new go command is issued
        self._is_returning_home = False
        self._home_retry_count  = 0
        self._stop_requested    = False

        self._request_navigation(cp_id)

    # ════════════════════════════════════════════════════════
    #  COMMAND: stop
    # ════════════════════════════════════════════════════════

    def _cmd_stop(self):
        """
        stop — Stop immediately, save the target for continuation.
        Valid in: COMPUTING_PATH, PRE_ROTATING, NAVIGATING
        """
        if self.state not in _STOP_VALID_STATES:
            self.get_logger().warn(
                f"[STOP] Rejected: not valid in state {self.state.name}. "
                f"Valid states: {[s.name for s in _STOP_VALID_STATES]}")
            return

        # Save the checkpoint being targeted
        self._saved_target_cp = self.target_cp
        saved_name = self.checkpoints.get(self._saved_target_cp, {}).get("name", "?")

        self.get_logger().info(
            f"[STOP] ✓ Stopping. Saving target=[{self._saved_target_cp}] "
            f"'{saved_name}'")

        # Set flag BEFORE cancelling — _on_result checks this flag
        self._intentional_cancel = True

        # Block _send_nav_goal if in COMPUTING_PATH/PRE_ROTATING
        self._stop_requested = True

        # Halt motion instantly
        self._stop_robot()

        # Cancel Nav2 goal nếu đang NAVIGATING
        if self.state == State.NAVIGATING and self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
            self.goal_handle = None
            self.get_logger().info(
                "[STOP] Nav2 goal cancel sent.")

        self.state = State.STOPPED
        self._pub_status(
            f"STOPPED. Target [{self._saved_target_cp}] '{saved_name}' saved. "
            "Use 'continue' or 'reset'.")

    def _on_cancel_done(self, future):
        """Callback invoked after Nav2 confirms goal cancellation."""
        try:
            result = future.result()
            self.get_logger().info(
                f"[STOP] Nav2 cancel confirmed. Goals canceled: "
                f"{len(result.goals_canceling)}")
        except Exception as e:
            self.get_logger().warn(f"[STOP] Cancel callback error: {e}")

    # ════════════════════════════════════════════════════════
    #  COMMAND: continue
    # ════════════════════════════════════════════════════════

    def _cmd_continue(self):
        """
        continue — Continue the interrupted journey.
        Valid in: STOPPED
        Re-compute path (because the robot may have been manually moved) → PRE_ROTATING.
        """
        if self.state != State.STOPPED:
            self.get_logger().warn(
                f"[CONTINUE] Rejected: only valid in STOPPED, "
                f"current={self.state.name}")
            return

        if self._saved_target_cp is None:
            self.get_logger().error(
                "[CONTINUE] No saved target checkpoint. Use 'go:<id>' instead.")
            return

        cp_id = self._saved_target_cp
        saved_name = self.checkpoints.get(cp_id, {}).get("name", "?")

        self.get_logger().info(
            f"[CONTINUE] Re-planning to [{cp_id}] '{saved_name}' "
            "(full re-compute including pre-rotate)")

        # Reset flags
        self._stop_requested    = False
        self._intentional_cancel = False
        self._saved_target_cp   = None

        self._pub_status(
            f"Continuing to [{cp_id}] '{saved_name}'...")
        self._request_navigation(cp_id)

    # ════════════════════════════════════════════════════════
    #  COMMAND: reset
    # ════════════════════════════════════════════════════════

    def _cmd_reset(self):
        """
        reset — Cancel the current journey and enter WAITING_RESET.
        Valid in: STOPPED
        After 30s, automatically return to Home; if 'go:<id>' is received within 30s → navigate to that id.
        """
        if self.state != State.STOPPED:
            self.get_logger().warn(
                f"[RESET] Rejected: only valid in STOPPED, "
                f"current={self.state.name}")
            return

        self.get_logger().info(
            f"[RESET] ✓ Entering WAITING_RESET. "
            f"Will go home [{self.home_id}] in {RESET_WAIT_TIMEOUT_S}s "
            "unless 'go:<id>' is received.")

        # Xóa saved target
        self._saved_target_cp   = None
        self._stop_requested    = False
        self._intentional_cancel = False

        self.state = State.WAITING_RESET
        self._pub_status(
            f"WAITING_RESET: send 'go:<id>' within {RESET_WAIT_TIMEOUT_S}s, "
            f"or robot will return home [{self.home_id}].")

        # Khởi động one-shot timer 30s
        self._start_reset_timer()

    def _start_reset_timer(self):
        """Khởi động bộ đếm ngược RESET_WAIT_TIMEOUT_S giây (pattern 1s tick)."""
        self._reset_elapsed = 0
        self._reset_timer   = self.create_timer(1.0, self._reset_tick)
        self.get_logger().info(
            f"[RESET_TIMER] Started. Countdown: {RESET_WAIT_TIMEOUT_S}s")

    def _reset_tick(self):
        """Tick 1s — đếm ngược. Khi hết giờ → về Home."""
        # Nếu state đã thay đổi (nhận lệnh go), timer tự hủy
        if self.state != State.WAITING_RESET:
            self._reset_timer.cancel()
            self._reset_timer = None
            self.get_logger().info("[RESET_TIMER] Canceled (state changed).")
            return

        self._reset_elapsed += 1
        remaining = RESET_WAIT_TIMEOUT_S - self._reset_elapsed

        if remaining % 10 == 0 or remaining <= 5:
            self.get_logger().info(
                f"[RESET_TIMER] {remaining}s remaining before going home [{self.home_id}].")

        if self._reset_elapsed >= RESET_WAIT_TIMEOUT_S:
            self._reset_timer.cancel()
            self._reset_timer = None
            self.get_logger().info(
                f"[RESET_TIMER] Timeout! Returning home [{self.home_id}].")
            self._pub_status(
                f"Reset timeout. Returning home [{self.home_id}]...")
            self._is_returning_home = True
            self._home_retry_count  = 0
            self._request_navigation(self.home_id)

    def _cancel_reset_timer(self):
        """Hủy timer WAITING_RESET nếu đang chạy."""
        if self._reset_timer is not None:
            self._reset_timer.cancel()
            self._reset_timer = None
            self.get_logger().info(
                "[RESET_TIMER] Canceled (new go command received).")

    # ════════════════════════════════════════════════════════
    #  STATE MACHINE TICK (10 Hz)
    # ════════════════════════════════════════════════════════

    def _state_machine_tick(self):
        """Main 10 Hz loop — only handle PRE_ROTATING tick."""
        if self.state == State.PRE_ROTATING:
            self._pre_rotate_tick()

    # ════════════════════════════════════════════════════════
    #  NAVIGATION PIPELINE
    # ════════════════════════════════════════════════════════

    def _request_navigation(self, cp_id: int):
        """
        Entry point for all navigation requests.
        Step 1: Call ComputePathToPose to get the path → extract the initial heading.
        """
        self._pending_cp_id = cp_id
        self.target_cp      = cp_id
        self.state          = State.COMPUTING_PATH

        cp_name = self.checkpoints[cp_id]["name"]
        self.get_logger().info(
            f"[COMPUTING_PATH] → [{cp_id}] '{cp_name}' │ "
            f"is_returning_home={self._is_returning_home}")
        self._pub_status(f"Computing path to [{cp_id}] '{cp_name}'...")

        goal_pose              = self.checkpoints[cp_id]["pose"]
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        compute_goal            = ComputePathToPose.Goal()
        compute_goal.pose       = goal_pose
        compute_goal.planner_id = ""  # dùng default planner (NavFn/A*)

        future = self._planner.send_goal_async(compute_goal)
        future.add_done_callback(self._on_path_goal_response)

    # ─── Step 1: Planner accepted/rejected ────────────────────────────────

    def _on_path_goal_response(self, future):
        """Planner response to goal acceptance/rejection."""
        # Guard: stop has been called while waiting for planner
        if self._stop_requested:
            self.get_logger().info(
                "[COMPUTING_PATH] stop_requested → discarding path goal response. "
                "State already STOPPED.")
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                "[COMPUTING_PATH] Planner REJECTED goal. "
                "Falling back to direct NavigateToPose (no pre-rotate).")
            self._send_nav_goal(self._pending_cp_id)
            return

        goal_handle.get_result_async().add_done_callback(self._on_path_result)

    # ─── Step 2: Planner returns path ──────────────────────────────────────

    def _on_path_result(self, future):
        """Analyze path, decide if pre-rotate is needed."""
        # Guard: stop during planner computation
        if self._stop_requested:
            self.get_logger().info(
                "[COMPUTING_PATH] stop_requested → discarding path result. "
                "Will NOT send NavigateToPose.")
            return

        result = future.result()

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"[COMPUTING_PATH] Planner failed (status={result.status}). "
                "Falling back to direct nav.")
            self._send_nav_goal(self._pending_cp_id)
            return

        path: Path = result.result.path

        if len(path.poses) < 2:
            self.get_logger().warn(
                "[COMPUTING_PATH] Path < 2 poses (goal very close?). "
                "Skipping pre-rotate.")
            self._send_nav_goal(self._pending_cp_id)
            return

        # Extract robot's current pose to compute heading error
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn(
                "[COMPUTING_PATH] TF unavailable → Skipping pre-rotate.")
            self._send_nav_goal(self._pending_cp_id)
            return

        rx, ry, ryaw = robot_pose
        initial_heading = self._extract_initial_heading(path, rx, ry)
        heading_error   = self._normalize_angle(initial_heading - ryaw)

        self.get_logger().info(
            f"[COMPUTING_PATH] Path heading: {math.degrees(initial_heading):+.1f}° │ "
            f"Robot yaw: {math.degrees(ryaw):+.1f}° │ "
            f"Error: {math.degrees(heading_error):+.1f}° "
            f"(threshold: ±{math.degrees(PRE_ROTATE_THRESHOLD):.1f}°)")

        if abs(heading_error) < PRE_ROTATE_THRESHOLD:
            self.get_logger().info(
                "[COMPUTING_PATH] Heading error within threshold → Skipping pre-rotate.")
            self._send_nav_goal(self._pending_cp_id)
            return

        # Initiate pre-rotate
        self._target_yaw       = initial_heading
        self._pre_rotate_start = time.monotonic()
        self.state             = State.PRE_ROTATING

        self.get_logger().info(
            f"[PRE_ROTATING] Rotating {math.degrees(heading_error):+.1f}° "
            f"to align with path tangent...")
        self._pub_status(
            f"Pre-rotating {math.degrees(heading_error):+.0f}° "
            f"→ [{self._pending_cp_id}] '{self.checkpoints[self._pending_cp_id]['name']}'")

    # ─── Pre-rotate control loop ──────────────────────────────────────────

    def _pre_rotate_tick(self):
        """Run at 10 Hz when state == PRE_ROTATING."""
        # Guard: stop during pre-rotating
        if self._stop_requested:
            self._stop_robot()
            self.get_logger().info(
                "[PRE_ROTATING] stop_requested → aborting pre-rotate. "
                "State already STOPPED.")
            return

        elapsed = time.monotonic() - self._pre_rotate_start
        if elapsed > PRE_ROTATE_TIMEOUT:
            self.get_logger().warn(
                f"[PRE_ROTATING] Timeout ({PRE_ROTATE_TIMEOUT:.0f}s). "
                "Proceeding to navigate anyway.")
            self._stop_robot()
            self._send_nav_goal(self._pending_cp_id)
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return

        _, _, ryaw    = robot_pose
        heading_error = self._normalize_angle(self._target_yaw - ryaw)

        if abs(heading_error) < PRE_ROTATE_STOP_THR:
            self.get_logger().info(
                f"[PRE_ROTATING] Converged │ "
                f"residual={math.degrees(heading_error):+.2f}° │ "
                f"elapsed={elapsed:.1f}s")
            self._stop_robot()
            self._send_nav_goal(self._pending_cp_id)
            return

        # P controller
        raw_w = PRE_ROTATE_KP * heading_error
        sign  = 1.0 if raw_w >= 0.0 else -1.0
        w     = sign * max(PRE_ROTATE_MIN_W, min(PRE_ROTATE_MAX_W, abs(raw_w)))

        twist = Twist()
        twist.angular.z = w
        self._cmdvel_pub.publish(twist)

        # Debug log every 1s
        if int(elapsed / 0.1) % 10 == 0:
            self.get_logger().debug(
                f"[PRE_ROTATING] err={math.degrees(heading_error):+.1f}° │ "
                f"w={w:+.2f} rad/s │ elapsed={elapsed:.1f}s")

    # ─── Send NavigateToPose goal ─────────────────────────────────────────

    def _send_nav_goal(self, cp_id: int):
        """
        Sends the NavigateToPose goal to Nav2.
        This is the end of the pipeline — guarded by `stop_requested` before sending.
        """
        # Guard: Stop has been requested -> do not send goal
        if self._stop_requested:
            self.get_logger().warn(
                f"[NAVIGATING] stop_requested → blocked NavigateToPose to [{cp_id}].")
            return

        self.target_cp = cp_id
        self.state     = State.NAVIGATING

        pose              = self.checkpoints[cp_id]["pose"]
        pose.header.stamp = self.get_clock().now().to_msg()

        goal      = NavigateToPose.Goal()
        goal.pose = pose

        cp_name = self.checkpoints[cp_id]["name"]
        self.get_logger().info(
            f"[NAVIGATING] ► [{cp_id}] '{cp_name}' │ "
            f"x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f} │ "
            f"is_returning_home={self._is_returning_home}")
        self._pub_status(f"Navigating → [{cp_id}] '{cp_name}'")

        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        """Callback invoked when Nav2 accepts or rejects the goal."""
        handle = future.result()

        # Guard: Stop requested between sending the goal and this callback
        if self._stop_requested:
            self.get_logger().warn(
                "[NAVIGATING] stop_requested after goal sent → canceling accepted goal.")
            if handle.accepted:
                handle.cancel_goal_async()
            return

        self.goal_handle = handle
        if not self.goal_handle.accepted:
            self.get_logger().error(
                "[NAVIGATING] Nav2 REJECTED NavigateToPose goal!")
            self.state = State.IDLE
            self._pub_status("Goal rejected by Nav2. Robot IDLE.")
            return

        self.get_logger().info(
            f"[NAVIGATING] Nav2 accepted goal to [{self.target_cp}].")
        self.goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        """Callback invoked when Nav2 finishes the goal (succeeded/canceled/aborted)."""
        status = future.result().status
        cp_id  = self.target_cp
        cp_name = self.checkpoints.get(cp_id, {}).get("name", "?")

        self.get_logger().info(
            f"[RESULT] Goal to [{cp_id}] '{cp_name}' finished │ "
            f"status={status} │ "
            f"is_returning_home={self._is_returning_home} │ "
            f"intentional_cancel={self._intentional_cancel}")

        # ── SUCCEEDED ────────────────────────────────────────────────────
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.current_cp        = cp_id
            self._home_retry_count = 0

            if self._is_returning_home:
                self._is_returning_home = False
                self.state = State.IDLE
                self.get_logger().info(
                    f"[RESULT] ✓ Arrived at HOME [{cp_id}] '{cp_name}'. "
                    "State: IDLE.")
                self._pub_status(
                    f"At Home [{cp_id}] '{cp_name}'. IDLE.")
            else:
                self.state = State.IDLE
                self.get_logger().info(
                    f"[RESULT] ✓ Arrived at [{cp_id}] '{cp_name}'. "
                    "State: IDLE.")
                self._pub_status(
                    f"Arrived at [{cp_id}] '{cp_name}'. "
                    "Send 'go:<id>' for next destination.")

        # ── CANCELED ─────────────────────────────────────────────────────
        elif status == GoalStatus.STATUS_CANCELED:
            if self._intentional_cancel:
                # Intentional cancel from the 'stop' command → state remains STOPPED
                self._intentional_cancel = False
                self.get_logger().info(
                    "[RESULT] Intentional cancel confirmed. State remains STOPPED.")
            else:
                # Unexpected cancel (Nav2 canceled internally)
                self.get_logger().warn(
                    "[RESULT] Unexpected cancel from Nav2. State → IDLE.")
                self._is_returning_home = False
                self._home_retry_count  = 0
                self.current_cp         = -1
                self.state              = State.IDLE
                self._pub_status("Navigation canceled unexpectedly. IDLE.")

        # ── ABORTED ──────────────────────────────────────────────────────
        elif status == GoalStatus.STATUS_ABORTED:
            if self._is_returning_home and self._home_retry_count < HOME_RETRY_MAX:
                self._home_retry_count += 1
                self.get_logger().warn(
                    f"[RESULT] RETURNING_HOME aborted │ "
                    f"Retry {self._home_retry_count}/{HOME_RETRY_MAX} "
                    f"in {HOME_RETRY_DELAY_S:.0f}s...")
                self._pub_status(
                    f"Home path blocked. Retry {self._home_retry_count}/{HOME_RETRY_MAX} "
                    f"in {HOME_RETRY_DELAY_S:.0f}s.")
                self.create_timer(HOME_RETRY_DELAY_S, self._retry_home_once)
            else:
                if self._is_returning_home:
                    self.get_logger().error(
                        f"[RESULT] CRITICAL: Cannot reach home after "
                        f"{HOME_RETRY_MAX} retries. Manual intervention needed!")
                    self._pub_status(
                        f"CRITICAL: Cannot reach home [{self.home_id}]. "
                        "Please manually move robot.")
                else:
                    self.get_logger().error(
                        f"[RESULT] Navigation to [{cp_id}] '{cp_name}' ABORTED. "
                        "Path blocked or planner failed.")
                    self._pub_status(
                        f"Aborted. Could not reach [{cp_id}] '{cp_name}'. IDLE.")

                self._is_returning_home = False
                self._home_retry_count  = 0
                self.current_cp         = -1
                self.state              = State.IDLE

    def _retry_home_once(self):
        """Timer callback for home retry."""
        if not self._is_returning_home or self.state == State.STOPPED:
            return
        self.get_logger().info(
            f"[HOME_RETRY] Attempt {self._home_retry_count}/{HOME_RETRY_MAX} → "
            f"home [{self.home_id}]")
        self._request_navigation(self.home_id)

    def _extract_initial_heading(self, path: Path, rx: float, ry: float) -> float:
        """Returns the heading (in radians, map frame) from the robot to the lookahead point on the path."""
        poses = path.poses

        if len(poses) == 1:
            px = poses[0].pose.position.x
            py = poses[0].pose.position.y
            return math.atan2(py - ry, px - rx)

        for pose_stamped in poses:
            px   = pose_stamped.pose.position.x
            py   = pose_stamped.pose.position.y
            dist = math.hypot(px - rx, py - ry)
            if dist >= PRE_ROTATE_LOOKAHEAD:
                return math.atan2(py - ry, px - rx)

        # All points are closer than the lookahead distance → use the final point
        last = poses[-1].pose.position
        return math.atan2(last.y - ry, last.x - rx)

    def _get_robot_pose(self):
        """Returns (x, y, yaw_rad) of base_footprint in the map frame. Returns None on error."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_footprint",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF error: {e}", throttle_duration_sec=2.0)
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return x, y, math.atan2(siny, cosy)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap to (−π, π]."""
        while angle >  math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _stop_robot(self):
        """Sends a Twist(0) command to stop the robot immediately."""
        self._cmdvel_pub.publish(Twist())

    #  STATE PUBLISHERS
    def _publish_state(self):
        """1 Hz — broadcasts the current state and checkpoint."""
        m = String(); m.data = self.state.value;  self._state_pub.publish(m)
        m = Int32();  m.data = self.current_cp;   self._cp_pub.publish(m)

    def _pub_status(self, message: str):
        """Publishes a status message and logs it to the terminal."""
        self.get_logger().info(f"[STATUS] {message}")
        m = String(); m.data = message
        self._status_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = CheckpointNavigator()
    except RuntimeError as e:
        import logging
        logging.getLogger("navigator_v2").error(
            f"CheckpointNavigator failed to start: {e}")
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
