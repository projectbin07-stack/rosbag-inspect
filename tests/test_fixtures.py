"""End-to-end: the simulated drive produces exactly the findings its faults should cause.

Expected values are derived from tests/drive.py by hand:

- /scan publishes every 100 ms with a receive latency of 10 + (k % 5) ms, so the
  inter-arrival times repeat 101, 101, 101, 101, 96 ms: median 101 ms, gap threshold
  3 x 101 ms = 303 ms. Scans 81..99 are missing, so the gap runs from scan 80 (received at
  8.000 + 0.010 s) to scan 100 (10.000 + 0.010 s): 2.000 s. 200 - 19 = 181 scans remain, each
  with 10 of 360 ranges = +inf -> 1810 invalid readings.
- /imu reports |a| = 1.0 whenever the yaw rate is zero. The yaw rate is 0.3 rad/s for
  command times 10..13 s delayed by 150 ms, i.e. IMU samples 1015..1314 (300 of 2000), so 1700
  samples are stationary and their median |a| is exactly 1.0.
- /imu message 500 (received at 5.000 + 0.002 s) repeats the stamp of message 499.
- /joint_states message 300 (received at 6.000 + 0.004 s) is stamped 3600 s later: 3599.996 s
  ahead of its receive time.
- /odom jumps 0.8 m between t = 11.98 s and t = 12.00 s (received 5 ms later each).
- /cmd_vel starts from rest at 2.0 s and 14.0 s (recorded 1 ms later); odometry follows the
  command 150 ms late at 50 Hz, so the first odometry sample at or above half the command is
  stamped 2.16 s / 14.16 s and received 5 ms later: 2.165 - 2.001 = 0.164 s.
- TF: base_laser -> laser is not connected to odom -> base_link -> {imu_link, camera_link}.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rosbag_inspect.analysis import analyze
from rosbag_inspect.bag import open_bag
from rosbag_inspect.findings import FAIL, Report

S = 1_000_000_000
T0 = 1_700_000_000 * S

EXPECTED = [
    ("tf_broken_chain", "", 1),
    ("imu_gravity", "/imu", 1),
    ("future_stamp", "/joint_states", 1),
    ("odom_teleport", "/odom", 1),
    ("tf_disconnected", "", 2),
    ("duplicate_stamp", "/imu", 1),
    ("gap", "/scan", 1),
    ("cmd_vel_response", "/cmd_vel", 2),
    ("laser_invalid_share", "/scan", 1810),
]


def _report(path: Path) -> Report:
    with open_bag(path) as bag:
        return analyze(bag)


@pytest.mark.parametrize("layout", ["sqlite", "mcap", "lz4", "unchunked", "nosummary"])
def test_drive_findings_identical_in_every_layout(fixtures: dict[str, Path], layout: str) -> None:
    report = _report(fixtures[layout])
    assert [(f.code, f.topic, f.count) for f in report.findings] == EXPECTED
    assert report.verdict == FAIL
    assert report.counts() == {"error": 4, "warning": 3, "info": 2}
    assert report.message_count == 1 + 181 + 2000 + 1000 * 3 + 200 + 100


def test_drive_details(fixtures: dict[str, Path]) -> None:
    report = _report(fixtures["sqlite"])
    by_code = {f.code: f for f in report.findings}

    gap = by_code["gap"]
    assert gap.data["gaps"] == [[T0 + 8_010_000_000, T0 + 10_010_000_000]]
    assert gap.data["threshold_s"] == pytest.approx(0.303)
    assert gap.items == [
        "+8.010 s -> +10.010 s: 2.000 s without messages (~19 expected messages missing)"
    ]

    gravity = by_code["imu_gravity"]
    assert gravity.data == {"median_m_s2": 1.0, "stationary_samples": 1700}
    assert "in g instead of m/s^2" in gravity.detail

    assert by_code["duplicate_stamp"].items == [
        "received at +5.002 s: stamp 1700000004.990000000 repeated"
    ]
    assert by_code["future_stamp"].items == ["received at +6.004 s: stamp is 3599.996 s ahead"]
    assert by_code["odom_teleport"].items[0].startswith("+11.985 s -> +12.005 s")
    assert by_code["cmd_vel_response"].data["latencies_s"] == pytest.approx([0.164, 0.164])
    assert by_code["tf_broken_chain"].data == {"frame": "laser", "topics": ["/scan"]}
    assert by_code["tf_disconnected"].items == [
        "root odom: base_link, camera_color_optical_frame, camera_link, imu_link, odom",
        "root base_laser: base_laser, laser",
    ]

    stats = {t.name: t for t in report.topics}
    assert stats["/imu"].rate_avg_hz == pytest.approx(100.0)
    assert stats["/scan"].period_median_s == pytest.approx(0.101)
    assert stats["/scan"].rate_min_hz == pytest.approx(0.5)
    assert stats["/scan"].latency is not None
    assert stats["/scan"].latency.min_s == pytest.approx(0.010)
    assert stats["/scan"].latency.max_s == pytest.approx(0.014)
    assert stats["/odom"].frame_ids == {"odom": 1000}
    assert stats["/camera/image_raw"].latency is not None
    assert stats["/camera/image_raw"].latency.p50_s == pytest.approx(0.030)
    assert report.tf.main_root == "odom"
    assert report.tf.unreachable == ["laser"]
    assert report.tf.missing == []
    static = {(e.parent, e.child) for e in report.tf.edges if e.static}
    assert ("base_laser", "laser") in static
    assert report.duration_s == pytest.approx(19.992)


def test_expectations_and_options(fixtures: dict[str, Path]) -> None:
    from rosbag_inspect.options import Options

    with open_bag(fixtures["mcap"]) as bag:
        report = analyze(bag, Options(expectations={"/scan": 10, "/imu": 200, "/gps": 5}))
    codes = report.codes()
    assert ("rate_mismatch", "/imu") in codes
    assert ("rate_mismatch", "/scan") not in codes  # 9.04 Hz is within 20 % of 10 Hz
    assert ("expected_topic_missing", "/gps") in codes
    scan = next(t for t in report.topics if t.name == "/scan")
    assert scan.expected_hz == 10
    assert scan.gap_threshold_s == pytest.approx(0.3)
