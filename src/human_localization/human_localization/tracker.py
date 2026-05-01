import math


class Track:
    def __init__(self, track_id, x, y, stamp):
        self.id = track_id
        self.x = x
        self.y = y

        self.vx = 0.0
        self.vy = 0.0

        self.hits = 1
        self.misses = 0
        self.confirmed = False

        self.confidence = 0.3
        self.last_stamp = stamp

    def update(self, x, y, stamp):
        dt = (stamp.sec - self.last_stamp.sec) + \
             (stamp.nanosec - self.last_stamp.nanosec) / 1e9

        if dt > 0:
            self.vx = (x - self.x) / dt
            self.vy = (y - self.y) / dt

        self.x = x
        self.y = y

        self.hits += 1
        self.misses = 0
        self.confidence = min(1.0, self.confidence + 0.2)

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

            for i, (x, y) in enumerate(detections):
                if i in used:
                    continue

                dist = math.hypot(track.x - x, track.y - y)

                if dist < best_dist:
                    best_dist = dist
                    best_idx = i

            if best_idx != -1 and best_dist < self.match_distance:
                x, y = detections[best_idx]
                track.update(x, y, stamp)
                used.add(best_idx)
            else:
                if not track.predict_miss():
                    track.to_delete = True

        self.tracks = [t for t in self.tracks if not hasattr(t, 'to_delete')]

        for i, (x, y) in enumerate(detections):
            if i not in used:
                self.tracks.append(Track(self.next_id, x, y, stamp))
                self.next_id += 1

        return self.tracks