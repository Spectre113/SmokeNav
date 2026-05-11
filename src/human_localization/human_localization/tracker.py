import math

import numpy as np


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class KalmanTrack:
    """2D constant-velocity Kalman track for a detected human.

    State vector: [x, y, vx, vy]^T in the tracking frame.
    Measurement:  [x, y]^T from /humans PoseArray.
    """

    def __init__(
        self,
        track_id,
        x,
        y,
        stamp,
        process_noise=1.0,
        measurement_noise=0.25,
        initial_position_variance=1.0,
        initial_velocity_variance=4.0,
        min_hits=3,
        max_misses=5,
    ):
        self.id = track_id
        self.x = np.array([[x], [y], [0.0], [0.0]], dtype=float)
        self.P = np.diag(
            [
                initial_position_variance,
                initial_position_variance,
                initial_velocity_variance,
                initial_velocity_variance,
            ]
        ).astype(float)

        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.min_hits = int(min_hits)
        self.max_misses = int(max_misses)

        self.hits = 1
        self.misses = 0
        self.confirmed = False
        self.confidence = 0.3
        self.last_stamp = stamp
        self.last_update_time = stamp_to_seconds(stamp)

    @property
    def px(self) -> float:
        return float(self.x[0, 0])

    @property
    def py(self) -> float:
        return float(self.x[1, 0])

    @property
    def vx(self) -> float:
        return float(self.x[2, 0])

    @property
    def vy(self) -> float:
        return float(self.x[3, 0])

    # Backwards-compatible aliases used by the node.
    @property
    def x_pos(self) -> float:
        return self.px

    @property
    def y_pos(self) -> float:
        return self.py

    def _motion_matrices(self, dt):
        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        # White-noise acceleration model.
        q = self.process_noise
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        Q = q * np.array(
            [
                [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
                [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
                [dt3 / 2.0, 0.0, dt2, 0.0],
                [0.0, dt3 / 2.0, 0.0, dt2],
            ],
            dtype=float,
        )
        return F, Q

    def predict_to(self, stamp):
        now = stamp_to_seconds(stamp)
        dt = max(0.0, now - self.last_update_time)
        if dt <= 0.0:
            self.last_stamp = stamp
            return

        F, Q = self._motion_matrices(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_update_time = now
        self.last_stamp = stamp

    def gating_distance(self, z_xy) -> float:
        """Mahalanobis distance for assignment gating."""
        z = np.array([[z_xy[0]], [z_xy[1]]], dtype=float)
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=float)
        R = np.eye(2, dtype=float) * self.measurement_noise
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        try:
            return float(math.sqrt((y.T @ np.linalg.inv(S) @ y)[0, 0]))
        except np.linalg.LinAlgError:
            return float('inf')

    def update(self, x, y, stamp):
        self.predict_to(stamp)

        z = np.array([[x], [y]], dtype=float)
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=float)
        R = np.eye(2, dtype=float) * self.measurement_noise

        innovation = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        I = np.eye(4, dtype=float)
        # Joseph form is more numerically stable than P=(I-KH)P.
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T

        self.hits += 1
        self.misses = 0
        self.confidence = min(1.0, self.confidence + 0.2)
        if self.hits >= self.min_hits:
            self.confirmed = True

    def mark_missed(self):
        self.misses += 1
        self.confidence = max(0.0, self.confidence - 0.15)
        return self.misses <= self.max_misses


class Tracker:
    def __init__(
        self,
        match_distance=1.0,
        min_hits=3,
        max_misses=5,
        process_noise=1.0,
        measurement_noise=0.25,
        initial_position_variance=1.0,
        initial_velocity_variance=4.0,
    ):
        self.tracks = []
        self.next_id = 0
        self.match_distance = float(match_distance)
        self.min_hits = int(min_hits)
        self.max_misses = int(max_misses)
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.initial_position_variance = float(initial_position_variance)
        self.initial_velocity_variance = float(initial_velocity_variance)

    def update(self, detections, stamp):
        # 1. Predict all tracks to the current timestamp.
        for track in self.tracks:
            track.predict_to(stamp)

        # 2. Greedy nearest-neighbor association using Mahalanobis gating.
        used_detections = set()
        for track in self.tracks:
            best_idx = -1
            best_dist = float('inf')
            for i, detection in enumerate(detections):
                if i in used_detections:
                    continue
                dist = track.gating_distance(detection)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i

            if best_idx != -1 and best_dist < self.match_distance:
                x, y = detections[best_idx]
                track.update(x, y, stamp)
                used_detections.add(best_idx)
            elif not track.mark_missed():
                track.to_delete = True

        self.tracks = [t for t in self.tracks if not hasattr(t, 'to_delete')]

        # 3. Start new tracks for unmatched detections.
        for i, (x, y) in enumerate(detections):
            if i in used_detections:
                continue
            self.tracks.append(
                KalmanTrack(
                    self.next_id,
                    x,
                    y,
                    stamp,
                    process_noise=self.process_noise,
                    measurement_noise=self.measurement_noise,
                    initial_position_variance=self.initial_position_variance,
                    initial_velocity_variance=self.initial_velocity_variance,
                    min_hits=self.min_hits,
                    max_misses=self.max_misses,
                )
            )
            self.next_id += 1

        return self.tracks
