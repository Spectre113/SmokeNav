#!/usr/bin/env python3
"""
Probabilistic Human Detection Fusion
Implements Bayesian sensor fusion for mmWave radar + thermal camera:
P(H | Radar, Thermal) ∝ P(Radar | H) × P(Thermal | H) × P(H)
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class RadarDetection:
    """Single radar cluster detection."""
    x: float
    y: float
    z: float
    num_points: int = 8          # points in cluster
    cluster_radius: float = 0.3  # meters

    @property
    def range(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)


@dataclass
class ThermalDetection:
    """Single thermal blob detection."""
    norm_x: float      # normalized 0-1
    norm_y: float      # normalized 0-1
    area: float = 2000.0         # pixels²
    temp_deviation: float = 0.0  # °C from ideal body temp
    aspect_ratio: float = 1.5    # w/h


@dataclass
class FusionMatch:
    """Result of associating one radar and one thermal detection."""
    radar: RadarDetection
    thermal: ThermalDetection
    pixel_distance: float        # distance on image plane
    confidence: float            # P(H | Radar, Thermal)


@dataclass
class FusionConfig:
    """Configurable parameters for the fusion model."""
    # Camera intrinsics
    image_width: int = 320
    image_height: int = 240
    focal_length_px: float = 277.0
    cx: float = 160.0
    cy: float = 120.0

    # Association
    match_distance_px: float = 150.0      # max pixel distance to associate

    # Fusion
    prior_human_probability: float = 0.1  # P(H)
    confidence_threshold: float = 0.0     # minimum posterior to publish

    # Radar likelihood model
    radar_range_sigma: float = 1.5        # meters, reliability decay with distance
    radar_cluster_weight: float = 0.7     # balance between cluster quality vs range

    # Thermal likelihood model
    thermal_temp_sigma: float = 2.0       # °C, temp match tightness
    thermal_size_sigma: float = 0.3       # fraction, size match tightness

    # Spatial agreement
    spatial_sigma_px: float = 20.0        # pixels, how tightly projection should match


class ProbabilisticFusion:
    """
    Probabilistic sensor fusion for human detection.

    Usage:
        fusion = ProbabilisticFusion(FusionConfig())
        fusion.set_radar_detections([RadarDetection(x=2.0, y=0.1, z=0.8, ...)])
        fusion.set_thermal_detections([ThermalDetection(norm_x=0.5, norm_y=0.4, ...)])
        matches = fusion.fuse()
        # matches is list of FusionMatch with confidence >= threshold
    """

    def __init__(self, config: FusionConfig = None):
        self.config = config or FusionConfig()
        self._radar: List[RadarDetection] = []
        self._thermal: List[ThermalDetection] = []

    # ── Data input ──

    def set_radar_detections(self, detections: List[RadarDetection]):
        self._radar = detections

    def set_thermal_detections(self, detections: List[ThermalDetection]):
        self._thermal = detections

    def _radar_to_thermal_frame(self, det: RadarDetection) -> Tuple[float, float, float]:
        """
        Transform radar detection from radar_link to thermal_optical_frame.
        radar_link is at (0.1, 0, 0.18) in base_link
        thermal_link is at (0.2, 0, 0.17) in base_link
        thermal_optical_frame: z-forward, x-right, y-down
        """
        # Offset from radar to thermal in base_link coords
        dx = 0.2 - 0.1  # = 0.1 (thermal is 0.1m ahead of radar)
        dz = 0.17 - 0.18  # = -0.01
        
        # Point in base_link from radar_link
        bx = det.x + 0.1  # radar_link forward
        by = det.y         # radar_link left
        bz = det.z + 0.18  # radar_link up
        
        # Point relative to thermal_link
        tx = bx - 0.2
        ty = by
        tz = bz - 0.17
        
        # Convert to thermal_optical_frame (z-forward, x-right, y-down)
        return (-ty, -tz, tx)  # (x_opt, y_opt, z_opt)

    # ── Projection ──

    def project_to_image(self, det: RadarDetection) -> Optional[Tuple[float, float]]:
        x_opt, y_opt, z_opt = self._radar_to_thermal_frame(det)
        
        if z_opt <= 0.5:
            return None
        
        u = self.config.cx + (x_opt / z_opt) * self.config.focal_length_px
        v = self.config.cy + (y_opt / z_opt) * self.config.focal_length_px
        
        if 0 <= u < self.config.image_width and 0 <= v < self.config.image_height:
            return (u, v)
        return None

    # ── Likelihood models ──

    def _radar_likelihood(self, det: RadarDetection) -> float:
        """
        P(Radar | H) — how likely this radar signature given a human is present.

        Factors:
        - Range: radar accuracy decays with distance
        - Cluster quality: dense, multi-point clusters suggest a solid body
        """
        rng = max(det.range, 0.5)
        range_factor = math.exp(-rng / self.config.radar_range_sigma)

        if det.cluster_radius > 0:
            density = det.num_points / (det.cluster_radius ** 3 + 1e-6)
        else:
            density = 1.0
        density_norm = min(density / 100.0, 1.0)

        likelihood = (
            self.config.radar_cluster_weight * density_norm +
            (1.0 - self.config.radar_cluster_weight) * range_factor
        )
        return max(likelihood, 0.01)

    def _thermal_likelihood(self, det: ThermalDetection) -> float:
        """
        P(Thermal | H) — how likely this thermal signature given a human is present.

        Factors:
        - Temperature: how close to human body temp (34-37°C)
        - Size: expected human size in pixels at typical detection ranges
        """
        temp_score = math.exp(
            -0.5 * (det.temp_deviation / self.config.thermal_temp_sigma) ** 2
        )

        # Ideal human blob area ~2000 px² at typical range
        size_ratio = det.area / 2000.0 if det.area > 0 else 1.0
        size_score = math.exp(
            -0.5 * ((size_ratio - 1.0) / self.config.thermal_size_sigma) ** 2
        )

        return max(0.6 * temp_score + 0.4 * size_score, 0.01)

    def _spatial_agreement(self, pixel_distance: float) -> float:
        """Bonus factor for close spatial alignment of radar→thermal projection."""
        return math.exp(-0.5 * (pixel_distance / self.config.spatial_sigma_px) ** 2)

    # ── Posterior ──

    def _posterior(self, radar: RadarDetection, thermal: ThermalDetection,
                   pixel_distance: float) -> float:
        """
        P(H | Radar, Thermal) ∝ P(Radar | H) × P(Thermal | H) × P(H) × spatial_agreement
        """
        p_rh = self._radar_likelihood(radar)
        p_th = self._thermal_likelihood(thermal)
        p_h = self.config.prior_human_probability
        spatial = self._spatial_agreement(pixel_distance)

        return p_rh * p_th * p_h * spatial

    # ── Association ──

    def _associate(self) -> List[Tuple[int, int, float]]:
        """
        Greedy nearest-neighbor association.
        Returns list of (radar_idx, thermal_idx, pixel_distance).
        """
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
                    r2d[1] - tdet.norm_y * self.config.image_height
                )
                if px_dist < best_dist:
                    best_dist = px_dist
                    best_ti = ti

            if best_ti >= 0:
                associations.append((ri, best_ti, best_dist))

        return associations

    # ── Main API ──

    def fuse(self) -> List[FusionMatch]:
        """
        Run full fusion pipeline.

        Returns:
            List of FusionMatch objects with confidence >= confidence_threshold,
            sorted by confidence descending.
        """
        if not self._radar or not self._thermal:
            return []

        associations = self._associate()

        matches = []
        matched_thermal = set()

        for ri, ti, px_dist in associations:
            if ti in matched_thermal:
                continue  # one thermal blob → one radar cluster only

            radar = self._radar[ri]
            thermal = self._thermal[ti]
            confidence = self._posterior(radar, thermal, px_dist)

            if confidence >= self.config.confidence_threshold:
                matches.append(FusionMatch(
                    radar=radar,
                    thermal=thermal,
                    pixel_distance=px_dist,
                    confidence=confidence,
                ))
                matched_thermal.add(ti)

        matches.sort(key=lambda m: m.confidence, reverse=True)
        return matches

    # ── Diagnostics ──

    def get_debug_info(self, matches: List[FusionMatch]) -> dict:
        return {
            'radar_count': len(self._radar),
            'thermal_count': len(self._thermal),
            'matches': len(matches),
            'confidences': [round(m.confidence, 3) for m in matches],
            'unmatched_radar': len(self._radar) - len(matches),
            'unmatched_thermal': len(self._thermal) - len(matches),
        }