from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


def _dist_xy(a: Odometry, b: Odometry) -> float:
    dx = a.pose.pose.position.x - b.pose.pose.position.x
    dy = a.pose.pose.position.y - b.pose.pose.position.y
    return math.hypot(dx, dy)


class MetricsLogger(Node):
    def __init__(self) -> None:
        super().__init__("metrics_logger")

        self.declare_parameter("output_csv", "sim_metrics.csv")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("human_pose_topic", "/human_pose")

        output_csv = str(self.get_parameter("output_csv").value)
        self._csv_path = Path(output_csv).expanduser().resolve()
        self._csv_path.parent.mkdir(parents=True, exist_ok=True)

        self._prev_odom: Optional[Odometry] = None
        self._path_len_m = 0.0
        self._cmd_count = 0
        self._last_human_pose: Optional[PoseStamped] = None

        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 20
        )
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value), self._on_cmd, 20
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("human_pose_topic").value),
            self._on_human_pose,
            10,
        )

        self._timer = self.create_timer(1.0, self._flush_row)

        self._csv_file = self._csv_path.open("w", newline="")
        self._writer = csv.DictWriter(
            self._csv_file,
            fieldnames=[
                "t_sec",
                "path_len_m",
                "cmd_vel_msgs",
                "human_pose_received",
                "human_x",
                "human_y",
            ],
        )
        self._writer.writeheader()

    def _on_odom(self, msg: Odometry) -> None:
        if self._prev_odom is not None:
            self._path_len_m += _dist_xy(msg, self._prev_odom)
        self._prev_odom = msg

    def _on_cmd(self, _msg: Twist) -> None:
        self._cmd_count += 1

    def _on_human_pose(self, msg: PoseStamped) -> None:
        self._last_human_pose = msg

    def _flush_row(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        row = {
            "t_sec": f"{now:.3f}",
            "path_len_m": f"{self._path_len_m:.3f}",
            "cmd_vel_msgs": str(self._cmd_count),
            "human_pose_received": "1" if self._last_human_pose is not None else "0",
            "human_x": "",
            "human_y": "",
        }
        if self._last_human_pose is not None:
            row["human_x"] = f"{self._last_human_pose.pose.position.x:.3f}"
            row["human_y"] = f"{self._last_human_pose.pose.position.y:.3f}"

        self._writer.writerow(row)
        self._csv_file.flush()

    def destroy_node(self):
        try:
            self._csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

