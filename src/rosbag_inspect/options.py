"""Thresholds used by the checks. All times are in seconds."""

from __future__ import annotations

from dataclasses import dataclass, field

STANDARD_GRAVITY = 9.80665


@dataclass
class Options:
    # timing
    gap_factor: float = 3.0
    """A gap is an inter-arrival time above ``gap_factor`` x the expected period ..."""
    min_gap_s: float = 0.25
    """... and above this absolute value."""
    expectations: dict[str, float] = field(default_factory=dict)
    """Expected rates in Hz by topic (``--expect /scan:10``)."""
    rate_tolerance: float = 0.2
    """Allowed relative deviation from an expected rate."""
    max_latency_s: float = 0.25
    """Warn when the 95th percentile of receive time - header.stamp exceeds this."""
    clock_skew_s: float = 0.01
    """Warn when header.stamp is later than the receive time by more than this."""
    future_s: float = 1.0
    """Error when header.stamp is later than the receive time by more than this."""
    # IMU
    gravity: float = STANDARD_GRAVITY
    gravity_tolerance: float = 0.5
    stationary_gyro: float = 0.05
    """rad/s; samples with a smaller angular rate count as stationary."""
    quaternion_tolerance: float = 0.01
    # odometry and cmd_vel
    teleport_m: float = 0.5
    odom_mismatch: float = 0.2
    """Relative disagreement between pose-derived and reported velocity."""
    odom_mismatch_abs: float = 0.05
    """... and absolute disagreement in m/s, both needed for a finding."""
    response_window_s: float = 1.0
    slow_response_s: float = 0.5
    # LaserScan
    laser_out_of_range_warn: float = 0.05
    laser_invalid_warn: float = 0.5
