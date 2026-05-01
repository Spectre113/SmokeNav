from __future__ import annotations

import math
import random

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class ScanSmokeFilter(Node):
    def __init__(self) -> None:
        super().__init__("scan_smoke_filter")

        self.declare_parameter("density", 0.0)  # 0..1
        self.declare_parameter("seed", 42)
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_smoked")

        self._rng = random.Random(int(self.get_parameter("seed").value))

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self._pub = self.create_publisher(LaserScan, output_topic, 10)
        self._sub = self.create_subscription(LaserScan, input_topic, self._on_scan, 10)

    def _on_scan(self, msg: LaserScan) -> None:
        density = float(self.get_parameter("density").value)
        density = _clamp(density, 0.0, 1.0)

        # Tuned for "noticeable but not catastrophic" degradation.
        max_range_scale = 1.0 - 0.6 * density
        noise_std = 0.02 + 0.10 * density
        dropout_prob = 0.02 + 0.25 * density

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time

        out.range_min = msg.range_min
        out.range_max = msg.range_max * max_range_scale

        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities) if msg.intensities else []

        for i, r in enumerate(out.ranges):
            # Treat invalids consistently
            if r is None or math.isnan(r):
                continue

            # Simulate random dropouts (returns "no hit")
            if self._rng.random() < dropout_prob:
                out.ranges[i] = float("inf")
                if out.intensities:
                    out.intensities[i] = 0.0
                continue

            # If it's a finite reading, perturb it and clamp to new max range.
            if math.isfinite(r):
                r_noisy = r + self._rng.gauss(0.0, noise_std)
                out.ranges[i] = _clamp(r_noisy, out.range_min, out.range_max)

        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanSmokeFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

