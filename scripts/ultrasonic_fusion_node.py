#!/usr/bin/env python3
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data

WHEEL_RADIUS = 0.08255

SENSOR_HEIGHT = {
    'us_top_left':    0.48 + WHEEL_RADIUS,
    'us_top_right':   0.48 + WHEEL_RADIUS,
    'us_mid_1_left':  0.36 + WHEEL_RADIUS,
    'us_mid_1_right': 0.36 + WHEEL_RADIUS,
    'us_mid_2_left':  0.24 + WHEEL_RADIUS,
    'us_mid_2_right': 0.24 + WHEEL_RADIUS,
    'us_bot_left':    0.12 + WHEEL_RADIUS,
    'us_bot_right':   0.12 + WHEEL_RADIUS,
}

# ── Floor echo filter ────────────────────────────────────────────────────────
# us_bot pitch = 0° (nằm ngang) → beam tâm song song sàn.
# Beam cone FOV = 0.26 rad → bán góc BEAM_HALF_ANGLE = 0.13 rad ≈ 7.45°
# Edge dưới của beam chạm sàn tại: h / tan(BEAM_HALF_ANGLE)
#   us_bot: h = 0.20255 m → floor_baseline = 0.20255 / tan(7.45°) ≈ 1.55 m
#
# Cập nhật SENSOR_PITCH nếu sau này hardware được điều chỉnh góc mount.
# Dương = ngẩng lên, âm = cúi xuống (rad, khớp với URDF rpy).
SENSOR_PITCH = {
    'us_bot_left':    0.0,
    'us_bot_right':   0.0,
    'us_mid_2_left':  0.0,
    'us_mid_2_right': 0.0,
    # top/mid_1 đủ cao → floor echo nằm ngoài tầm đo → không cần filter
}

BEAM_HALF_ANGLE = 0.13    # rad — FOV/2 của HC-SR04 (FOV ≈ 0.26 rad = 15°)
FLOOR_TOLERANCE = 0.15    # m  — dải ± quanh baseline bị lọc
                           #      tăng nếu vẫn còn phantom, giảm nếu miss vật cản thật


def _compute_floor_baseline(sensor_name: str) -> float:
    """
    Khoảng cách dọc theo trục beam mà edge dưới của cone chạm sàn.

    effective_angle = pitch - BEAM_HALF_ANGLE
      > 0 → edge dưới vẫn ngẩng lên → không bao giờ chạm sàn → inf
      = 0 → edge dưới nằm ngang      → chạm sàn ở ∞              → inf
      < 0 → edge dưới cúi xuống      → chạm sàn tại h/tan(|eff|)
    """
    h     = SENSOR_HEIGHT.get(sensor_name)
    pitch = SENSOR_PITCH.get(sensor_name)
    if h is None or pitch is None:
        return float('inf')
    effective = pitch - BEAM_HALF_ANGLE
    if effective >= 0:
        return float('inf')
    return h / math.tan(abs(effective))


# Pre-compute một lần khi load module
FLOOR_BASELINE = {n: _compute_floor_baseline(n) for n in SENSOR_PITCH}


class UltrasonicFusionNode(Node):

    COSTMAP_RANGE_MAX    = 2.0    # m
    HARD_STOP_THRESHOLD  = 0.20   # m — kích hoạt hard stop
    HARD_STOP_HYSTERESIS = 0.30   # m — giải phóng hard stop
    SENSOR_X_OFFSET      = 0.35   # m — offset sensor từ base_footprint
    FILTER_WINDOW        = 5

    def __init__(self):
        super().__init__('ultrasonic_fusion_node')

        self.sensor_names = [
            'us_top_left',    'us_top_right',
            'us_mid_1_left',  'us_mid_1_right',
            'us_mid_2_left',  'us_mid_2_right',
            'us_bot_left',    'us_bot_right',
        ]

        self.buffers       = {n: deque(maxlen=self.FILTER_WINDOW) for n in self.sensor_names}
        self.latest        = {n: float('inf') for n in self.sensor_names}
        self.sensor_angles = {n: math.radians(0) for n in self.sensor_names}

        self._hard_stop_active = False

        for name in self.sensor_names:
            self.create_subscription(
                Range, f'/ultrasonic/{name}',
                lambda msg, n=name: self._range_cb(msg, n), 10)

        self.create_subscription(Twist, '/cmd_vel_nav', self._cmdvel_cb, 10)

        self.scan_pub   = self.create_publisher(LaserScan, '/ultrasonic_scan', qos_profile_sensor_data)
        self.safety_pub = self.create_publisher(Bool,      '/safety_stop',     10)
        self.cmdvel_pub = self.create_publisher(Twist,     '/cmd_vel',         10)

        self.create_timer(0.1, self._publish_scan)

        # Log floor baseline để verify sau khi deploy
        for name, baseline in FLOOR_BASELINE.items():
            if not math.isinf(baseline):
                self.get_logger().info(
                    f'[FloorFilter] {name}: h={SENSOR_HEIGHT[name]:.3f}m | '
                    f'pitch={math.degrees(SENSOR_PITCH[name]):.1f}deg | '
                    f'baseline={baseline:.3f}m ± {FLOOR_TOLERANCE}m')

        self.get_logger().info('=== Ultrasonic Fusion Node ===')
        self.get_logger().info(
            '(Costmap): /ultrasonic_scan → Nav2 local_costmap auto replanning')
        self.get_logger().info(
            f'(Hard Stop): ON < {self.HARD_STOP_THRESHOLD}m | '
            f'OFF > {self.HARD_STOP_HYSTERESIS}m')
        self.get_logger().info('ultrasonic_fusion_node')

    # ── Sensor callback ───────────────────────────────────────────────────────
    def _range_cb(self, msg: Range, name: str):
        r = msg.range

        # Floor echo filter: nếu reading nằm trong vùng [baseline-tol, baseline+tol]
        # → beam đang phản xạ từ sàn, không phải vật cản → bỏ qua hoàn toàn
        baseline = FLOOR_BASELINE.get(name, float('inf'))
        if not math.isinf(baseline) and abs(r - baseline) <= FLOOR_TOLERANCE:
            return

        if msg.min_range <= r <= msg.max_range:
            self.buffers[name].append(r)
        elif r >= msg.max_range:
            self.buffers[name].append(float('inf'))
        # < min_range → nhiễu cực gần, bỏ qua

        if self.buffers[name]:
            finite_vals = [v for v in self.buffers[name] if not math.isinf(v)]
            self.latest[name] = float(np.median(finite_vals)) if finite_vals else float('inf')

    # ── Publish scan → costmap ───────────────────────────────────────
    def _publish_scan(self):
        num_rays        = 360
        ranges          = [float('inf')] * num_rays
        spread          = 8
        angle_min_scan  = -math.pi
        angle_increment = 2 * math.pi / num_rays

        for name, angle_rad in self.sensor_angles.items():
            raw = self.latest[name]
            if math.isinf(raw):
                continue  # ray inf → costmap tự clearing

            dist = raw + self.SENSOR_X_OFFSET
            center_idx = int(round(
                (angle_rad - angle_min_scan) / angle_increment
            )) % num_rays

            for offset in range(-spread, spread + 1):
                idx = (center_idx + offset) % num_rays
                cos_val = math.cos(math.radians(offset * (15.0 / spread)))
                if cos_val > 0:
                    ranges[idx] = min(ranges[idx], dist / cos_val)

        scan                 = LaserScan()
        scan.header.stamp    = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_footprint'
        scan.angle_min       = -math.pi
        scan.angle_max       =  math.pi
        scan.angle_increment = angle_increment
        scan.time_increment  = 0.0
        scan.scan_time       = 0.1
        scan.range_min       = 0.01
        scan.range_max       = self.COSTMAP_RANGE_MAX
        scan.ranges          = ranges

        self.scan_pub.publish(scan)
        self._update_hard_stop()

    # ── Hard stop với hysteresis ─────────────────────────────────────
    def _update_hard_stop(self):
        """
        Kích hoạt: BẤT KỲ sensor < HARD_STOP_THRESHOLD
        Giải phóng: TẤT CẢ sensor > HARD_STOP_HYSTERESIS

        KHÔNG cancel Nav2 goal → Nav2 tự kích hoạt recovery (spin/backup/wait)
        → tự replanning khi vật cản qua đi. Đây là hành vi ĐÚNG với Nav2.
        """
        if not self._hard_stop_active:
            for name in self.sensor_names:
                val = self.latest[name]
                if not math.isinf(val) and val < self.HARD_STOP_THRESHOLD:
                    self._hard_stop_active = True
                    self.get_logger().warn(
                        f'[HARD STOP ON]  {name}={val:.3f}m | Nav2 is still running, wait for recovery.',
                        throttle_duration_sec=1.0)
                    break
        else:
            all_clear = all(
                math.isinf(self.latest[n]) or self.latest[n] > self.HARD_STOP_HYSTERESIS
                for n in self.sensor_names
            )
            if all_clear:
                self._hard_stop_active = False
                self.get_logger().info('[HARD STOP OFF] Obstacle cleared. Nav2 continue.')

        self.safety_pub.publish(Bool(data=self._hard_stop_active))

    # ── cmd_vel gate ──────────────────────────────────────────────────────────
    def _cmdvel_cb(self, msg: Twist):
        """
        Hard stop active  → Twist(0,0)   [dừng cứng, Nav2 vẫn sống]
        Hard stop inactive → forward msg  [Nav2 điều khiển bình thường]
        """
        if self._hard_stop_active:
            self.cmdvel_pub.publish(Twist())
        else:
            self.cmdvel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()