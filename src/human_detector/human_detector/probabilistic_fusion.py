#!/usr/bin/env python3
"""
Probabilistic Human Detection Fusion
Implements Bayesian sensor fusion for mmWave radar + thermal camera.

The fusion can now publish:
- fused radar+thermal detections with the highest confidence;
- radar-only moving detections with lower confidence when thermal is absent
  or when no thermal association exists.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class RadarDetection:
    """Single radar cluster detection."""

    x: float
    y: float
    z: float
    num_points: int = 8
    cluster_radius: float = 0.3
    radial_velocity: float = 0.0

    @property
    def range(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)


@dataclass
class ThermalDetection:
    """Single thermal blob detection."""

    norm_x: float
    norm_y: float
    area: float = 2000.0
    temp_deviation: float = 0.0
    aspect_ratio: float = 1.5


@dataclass
class FusionMatch:
    """Result of radar-only or radar+thermal association."""

    radar: RadarDetection
    confidence: float
    thermal: Optional[ThermalDetection] = None
    pixel_distance: Optional[float] = None
    source: str = 'radar_only'


@dataclass
class FusionConfig:
    """Configurable parameters for the fusion model."""

    image_width: int = 320
    image_height: int = 240
    focal_length_px: float = 277.0
    cx: float = 160.0
    cy: float = 120.0

    match_distance_px: float = 150.0

    prior_human_probability: float = 0.1
    confidence_threshold: float = 0.0
    radar_only_confidence_scale: float = 0.35

    radar_range_sigma: float = 1.5
    radar_cluster_weight: float = 0.55
    radar_motion_weight: float = 0.45
    radar_velocity_sigma: float = 0.02

    thermal_temp_sigma: float = 2.0
    thermal_size_sigma: float = 0.3

    spatial_sigma_px: float = 20.0


class ProbabilisticFusion:
    """Probabilistic sensor fusion with radar-only fallback."""

    def __init__(self, config: FusionConfig | None = None):
        self.config = config or FusionConfig()
        self._radar: List[RadarDetection] = []
        self._thermal: List[ThermalDetection] = []

    def set_radar_detections(self, detections: List[RadarDetection]):
        self._radar = detections

    def set_thermal_detections(self, detections: List[ThermalDetection]):
        self._thermal = detections

    def _radar_to_thermal_frame(self, det: RadarDetection) -> Tuple[float, float, float]:
        """Transform radar detection from radar_link to thermal_optical_frame."""
        bx = det.x + 0.1
        by = det.y
        bz = det.z + 0.18

        tx = bx - 0.2
        ty = by
        tz = bz - 0.17

        return (-ty, -tz, tx)

    def project_to_image(self, det: RadarDetection) -> Optional[Tuple[float, float]]:
        x_opt, y_opt, z_opt = self._radar_to_thermal_frame(det)
        if z_opt <= 0.5:
            return None

        u = self.config.cx + (x_opt / z_opt) * self.config.focal_length_px
        v = self.config.cy + (y_opt / z_opt) * self.config.focal_length_px

        if 0 <= u < self.config.image_width and 0 <= v < self.config.image_height:
            return (u, v)
        return None

    def _radar_likelihood(self, det: RadarDetection) -> float:
        """P(Radar | H)."""
        rng = max(det.range, 0.5)
        range_factor = math.exp(-rng / self.config.radar_range_sigma)

        if det.cluster_radius > 0:
            density = det.num_points / (det.cluster_radius ** 3 + 1e-6)
        else:
            density = float(det.num_points)
        density_norm = min(density / 100.0, 1.0)

        motion_norm = 1.0 - math.exp(
            -abs(det.radial_velocity) / max(self.config.radar_velocity_sigma, 1e-6)
        )

        base_quality = (
            self.config.radar_cluster_weight * density_norm
            + (1.0 - self.config.radar_cluster_weight) * range_factor
        )
        likelihood = (
            (1.0 - self.config.radar_motion_weight) * base_quality
            + self.config.radar_motion_weight * motion_norm
        )
        return max(likelihood, 0.01)

    def _thermal_likelihood(self, det: ThermalDetection) -> float:
        """P(Thermal | H)."""
        temp_score = math.exp(
            -0.5 * (det.temp_deviation / self.config.thermal_temp_sigma) ** 2
        )

        size_ratio = det.area / 2000.0 if det.area > 0 else 1.0
        size_score = math.exp(
            -0.5 * ((size_ratio - 1.0) / self.config.thermal_size_sigma) ** 2
        )

        return max(0.6 * temp_score + 0.4 * size_score, 0.01)

    def _spatial_agreement(self, pixel_distance: float) -> float:
        return math.exp(-0.5 * (pixel_distance / self.config.spatial_sigma_px) ** 2)

    def _posterior(self, radar: RadarDetection, thermal: ThermalDetection, pixel_distance: float) -> float:
        p_rh = self._radar_likelihood(radar)
        p_th = self._thermal_likelihood(thermal)
        p_h = self.config.prior_human_probability
        spatial = self._spatial_agreement(pixel_distance)
        return p_rh * p_th * p_h * spatial

    def _radar_only_posterior(self, radar: RadarDetection) -> float:
        p_rh = self._radar_likelihood(radar)
        p_h = self.config.prior_human_probability
        return p_rh * p_h * self.config.radar_only_confidence_scale

    def _associate(self) -> List[Tuple[int, int, float]]:
        associations = []

        for ri, rdet in enumerate(self._radar):
            r2d = self.project_to_image(rdet)
            if r2d is None:
                continue

            best_ti = -1
            best_dist = self.config.match_distance_px
            for ti, tdet in enumerate(self._thermal):
                px_dist = math.hypot(
                    r2d[0] - tdet.norm_x * self.config.image_width,
                    r2d[1] - tdet.norm_y * self.config.image_height,
                )
                if px_dist < best_dist:
                    best_dist = px_dist
                    best_ti = ti

            if best_ti >= 0:
                associations.append((ri, best_ti, best_dist))

        return associations

    def fuse(self) -> List[FusionMatch]:
        if not self._radar:
            return []

        matches: List[FusionMatch] = []
        matched_radar = set()
        matched_thermal = set()

        if self._thermal:
            for ri, ti, px_dist in self._associate():
                if ri in matched_radar or ti in matched_thermal:
                    continue

                radar = self._radar[ri]
                thermal = self._thermal[ti]
                confidence = self._posterior(radar, thermal, px_dist)
                if confidence >= self.config.confidence_threshold:
                    matches.append(
                        FusionMatch(
                            radar=radar,
                            thermal=thermal,
                            pixel_distance=px_dist,
                            confidence=confidence,
                            source='radar_thermal',
                        )
                    )
                    matched_radar.add(ri)
                    matched_thermal.add(ti)

        for ri, radar in enumerate(self._radar):
            if ri in matched_radar:
                continue
            confidence = self._radar_only_posterior(radar)
            if confidence >= self.config.confidence_threshold:
                matches.append(
                    FusionMatch(
                        radar=radar,
                        confidence=confidence,
                        source='radar_only',
                    )
                )

        matches.sort(key=lambda m: m.confidence, reverse=True)
        return matches

    def get_debug_info(self, matches: List[FusionMatch]) -> dict:
        radar_only_count = sum(1 for match in matches if match.source == 'radar_only')
        fused_count = sum(1 for match in matches if match.source == 'radar_thermal')
        return {
            'radar_count': len(self._radar),
            'thermal_count': len(self._thermal),
            'matches': len(matches),
            'fused_matches': fused_count,
            'radar_only_matches': radar_only_count,
            'confidences': [round(m.confidence, 3) for m in matches],
            'unmatched_radar': len(self._radar) - fused_count,
            'unmatched_thermal': max(0, len(self._thermal) - fused_count),
        }
