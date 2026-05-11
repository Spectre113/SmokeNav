import math


class Track:
    def __init__(self, track_id, x, y, stamp, measurement_confidence=0.3):
        self.id = track_id
        self.x = x
        self.y = y

        self.vx = 0.0
        self.vy = 0.0

        self.hits = 1
        self.misses = 0
        self.confirmed = False

        self.confidence = max(0.05, min(1.0, measurement_confidence))
        self.last_stamp = stamp

    def update(self, x, y, stamp, measurement_confidence=0.3):
        dt = (stamp.sec - self.last_stamp.sec) + \
             (stamp.nanosec - self.last_stamp.nanosec) / 1e9

        if dt > 0:
            self.vx = (x - self.x) / dt
            self.vy = (y - self.y) / dt

        self.x = x
        self.y = y

        self.hits += 1
        self.misses = 0
        measurement_confidence = max(0.05, min(1.0, measurement_confidence))
        self.confidence = min(
            1.0,
            0.45 * self.confidence + 0.55 * measurement_confidence + 0.12
        )

        if self.hits >= 3:
            self.confirmed = True

        self.last_stamp = stamp

    def predict_miss(self):
        self.misses += 1
        self.confidence = max(0.0, self.confidence - 0.15)
        return self.misses <= 5


class Tracker:
    def __init__(self, match_distance=1.0):
        self.tracks = []
        self.next_id = 0
        self.match_distance = match_distance

    def update(self, detections, stamp):
        used = set()

        for track in self.tracks:
            best_idx = -1
            best_dist = float('inf')

            for i, det in enumerate(detections):
                if i in used:
                    continue
                x, y = det[0], det[1]

                dist = math.hypot(track.x - x, track.y - y)

                if dist < best_dist:
                    best_dist = dist
                    best_idx = i

            if best_idx != -1 and best_dist < self.match_distance:
                det = detections[best_idx]
                x, y = det[0], det[1]
                measurement_confidence = det[2] if len(det) > 2 else 0.3
                track.update(x, y, stamp, measurement_confidence)
                used.add(best_idx)
            else:
                if not track.predict_miss():
                    track.to_delete = True

        self.tracks = [t for t in self.tracks if not hasattr(t, 'to_delete')]

        for i, det in enumerate(detections):
            if i not in used:
                x, y = det[0], det[1]
                measurement_confidence = det[2] if len(det) > 2 else 0.3
                self.tracks.append(
                    Track(self.next_id, x, y, stamp, measurement_confidence)
                )
                self.next_id += 1

        return self.tracks
