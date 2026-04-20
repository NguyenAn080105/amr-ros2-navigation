#!/usr/bin/env python3
"""
session_logger.py
═══════════════════════════════════════════════════════════════════════════════
Ghi structured session log ra file mỗi lần chạy nav_launch.py.

Output: ~/robot_logs/session_YYYY-MM-DD_HH-MM-SS.log

Format log:
  [HH:MM:SS.mmm] [LEVEL] [source] message

Features:
  - Subscribe /rosout_agg để capture log từ tất cả node
  - Deduplicate: lỗi lặp giống nhau → chỉ ghi 1 lần + counter (xN)
  - Summary khi shutdown: tổng warn/error của từng node
  - Ghi heartbeat mỗi 60s để biết hệ thống còn sống
  - File header ghi metadata: floor, timestamp, ROS version

Usage:
  Node này được launch tự động từ nav_launch.py.
  Không cần chạy tay.

Log dir: ~/robot_logs/ (tạo tự động nếu chưa có)
═══════════════════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import os
import sys
import time
import datetime
import signal
from collections import defaultdict
from typing import Tuple, Dict


# Level mapping từ rcl_interfaces/Log severity
LEVEL_MAP = {
    10: 'DEBUG',
    20: 'INFO',
    30: 'WARN',
    40: 'ERROR',
    50: 'FATAL',
}

# Các node muốn suppress INFO (chỉ ghi WARN+)
# Vì các node này spam INFO rất nhiều trong quá trình hoạt động bình thường
INFO_SUPPRESSED_NODES = {
    'robot_state_publisher',
    'joint_state_publisher',
    'bno055',
    'sllidar_node',
    'scan_to_scan_filter_chain',
    'lifecycle_manager_localization',
    'lifecycle_manager_navigation',
    # costmap nodes
    'local_costmap',
    'global_costmap',
}

# Các message pattern coi là "expected" → chỉ ghi lần đầu tiên
EXPECTED_PATTERNS = [
    'Invalid frame ID "laser_frame"',          # transient khi startup
    'Could not get transform, irgnoring',       # transient khi startup
    'Failed to meet update rate',               # EKF perf warning
    'Publisher already registered',             # rosout artifact
    'Lookup would require extrapolation',       # race condition khi set initial pose
]


class SessionLogger(Node):

    def __init__(self):
        super().__init__('session_logger')

        log_dir = os.path.expanduser('~/mbrobot_ws/src/mobile_robot/logs')
        os.makedirs(log_dir, exist_ok=True)

        # Tên file theo timestamp lúc khởi độngs
        ts = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self._log_path = os.path.join(log_dir, f'session_{ts}.log')

        # Symlink "latest.log" → log mới nhất để dễ xem
        latest_link = os.path.join(log_dir, 'latest.log')
        if os.path.islink(latest_link):
            os.remove(latest_link)
        os.symlink(self._log_path, latest_link)

        self._log_file = open(self._log_path, 'w', buffering=1)  # line-buffered

        # Tracking dedup
        # key: (node_name, message_normalized) → count
        self._msg_count: Dict[tuple, int] = defaultdict(int)
        # key: pattern_string → đã ghi lần đầu chưa
        self._expected_logged: dict[str, bool] = {}

        # Summary counters per node
        self._warn_count:  Dict[str, int] = defaultdict(int)
        self._error_count: Dict[str, int] = defaultdict(int)

        self._start_time = time.time()

        # Write header
        self._write_header(ts)

        # Subscribe /rosout_agg — aggregated log từ tất cả node
        self.create_subscription(Log, '/rosout_agg', self._on_log, 100)

        # Heartbeat timer mỗi 60s
        self.create_timer(60.0, self._heartbeat)

        # Cleanup on shutdown
        self.get_logger().info(f'Session logger started → {self._log_path}')
        self.get_logger().info(f'Quick view: tail -f ~/robot_logs/latest.log')

    def _write_header(self, ts: str):
        self._log_file.write('═' * 72 + '\n')
        self._log_file.write(f'  mobile_robot — Session Log\n')
        self._log_file.write(f'  Started : {ts}\n')
        self._log_file.write(f'  Host    : {os.uname().nodename}\n')
        self._log_file.write(f'  PID     : {os.getpid()}\n')
        self._log_file.write('═' * 72 + '\n\n')

    def _elapsed(self) -> str:
        e = time.time() - self._start_time
        m, s = divmod(int(e), 60)
        return f'+{m:02d}:{s:02d}'

    def _wall_time(self) -> str:
        return datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]

    def _normalize(self, msg: str) -> str:
        """Normalize message để dedup: loại bỏ số liệu thay đổi theo thời gian."""
        import re
        # Loại bỏ timestamp dạng 1776675570.xxx
        msg = re.sub(r'\d{10}\.\d+', 'T', msg)
        # Loại bỏ số thập phân
        msg = re.sub(r'\d+\.\d+', 'N', msg)
        # Loại bỏ số nguyên lớn (PID, memory addr)
        msg = re.sub(r'\b\d{4,}\b', 'N', msg)
        return msg.strip()

    def _is_expected(self, msg: str) -> Tuple[bool, str]:
        """Kiểm tra message có phải expected pattern không."""
        for pattern in EXPECTED_PATTERNS:
            if pattern in msg:
                return True, pattern
        return False, ''

    def _on_log(self, log_msg: Log):
        level_int = log_msg.level
        level_str = LEVEL_MAP.get(level_int, 'UNKN')
        node      = log_msg.name
        msg       = log_msg.msg.strip()

        # Luôn bỏ qua DEBUG
        if level_int <= 10:
            return

        # Với INFO, suppress các node "noisy" bình thường
        if level_int == 20 and node in INFO_SUPPRESSED_NODES:
            return

        # Kiểm tra expected (transient startup) patterns
        is_exp, pattern = self._is_expected(msg)
        if is_exp:
            if not self._expected_logged.get(pattern, False):
                # Lần đầu gặp → ghi 1 lần với note
                self._expected_logged[pattern] = True
                line = (f'[{self._wall_time()}] {self._elapsed()} '
                        f'[{level_str}] [{node}] {msg}  '
                        f'← expected startup transient, suppressing duplicates\n')
                self._log_file.write(line)
            # Lần sau → bỏ qua hoàn toàn (không count)
            return

        # Dedup thông thường: lỗi giống nhau lặp → ghi xN
        key = (node, self._normalize(msg))
        self._msg_count[key] += 1
        count = self._msg_count[key]

        # Lần đầu → ghi bình thường
        if count == 1:
            line = (f'[{self._wall_time()}] {self._elapsed()} '
                    f'[{level_str}] [{node}] {msg}\n')
            self._log_file.write(line)
        # Lần 5, 10, 50 → ghi summary "repeated xN"
        elif count in (5, 10, 25, 50, 100) or (count % 100 == 0):
            line = (f'[{self._wall_time()}] {self._elapsed()} '
                    f'[{level_str}] [{node}] ↑ repeated x{count}: {msg[:60]}...\n')
            self._log_file.write(line)

        # Cập nhật summary counter
        if level_int == 30:
            self._warn_count[node] += 1
        elif level_int >= 40:
            self._error_count[node] += 1

    def _heartbeat(self):
        elapsed = time.time() - self._start_time
        m, s = divmod(int(elapsed), 60)
        self._log_file.write(
            f'\n[{self._wall_time()}] +{m:02d}:{s:02d} '
            f'[HEARTBEAT] System alive — '
            f'warns={sum(self._warn_count.values())} '
            f'errors={sum(self._error_count.values())}\n\n'
        )

    def write_summary(self):
        """Ghi summary khi shutdown."""
        elapsed = time.time() - self._start_time
        m, s = divmod(int(elapsed), 60)

        self._log_file.write('\n' + '═' * 72 + '\n')
        self._log_file.write(f'  SESSION SUMMARY\n')
        self._log_file.write(f'  Duration : {m:02d}m {s:02d}s\n')
        self._log_file.write(f'  Ended    : {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
        self._log_file.write('─' * 72 + '\n')

        # Nodes có warn/error
        all_nodes = set(self._warn_count.keys()) | set(self._error_count.keys())
        if all_nodes:
            self._log_file.write('  Node                              WARN  ERROR\n')
            self._log_file.write('  ' + '-' * 50 + '\n')
            for node in sorted(all_nodes):
                w = self._warn_count.get(node, 0)
                e = self._error_count.get(node, 0)
                marker = ' ← !' if e > 0 else ''
                self._log_file.write(f'  {node:<34} {w:4d}  {e:5d}{marker}\n')
        else:
            self._log_file.write('  No warnings or errors recorded. ✓\n')

        self._log_file.write('═' * 72 + '\n')
        self._log_file.close()
        print(f'\n[session_logger] Log saved → {self._log_path}')


def main(args=None):
    rclpy.init(args=args)
    node = SessionLogger()

    def _on_shutdown(sig, frame):
        node.write_summary()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _on_shutdown)
    signal.signal(signal.SIGTERM, _on_shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()