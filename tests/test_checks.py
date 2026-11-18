"""Unit tests of the individual checks with hand-derived expected values."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import pytest

from bagwriter import Record, write_mcap
from rosbag_inspect.analysis import analyze, is_event_topic
from rosbag_inspect.bag import open_bag
from rosbag_inspect.cdr import SkippedBytes, encode
from rosbag_inspect.findings import FAIL, PASS, WARN, TopicStats
from rosbag_inspect.msgdef import Registry
from rosbag_inspect.options import Options
from rosbag_inspect.sensors import (
    CmdSample,
    ImageCollector,
    ImuCollector,
    JointStateCollector,
    LaserCollector,
    OdomCollector,
    OdomSample,
    cmd_response_findings,
    image_findings,
    imu_findings,
    joint_state_findings,
    laser_findings,
    odom_findings,
    yaw_from_quaternion,
)
from rosbag_inspect.tfcheck import TfCollector, analyze_tf
from rosbag_inspect.timing import (
    analyze_rates,
    analyze_stamps,
    duration_text,
    histogram,
    percentile,
    timeline,
)

MS = 1_000_000
S = 1_000_000_000
T0 = 1_700_000_000 * S
OPTS = Options()


def _codes(findings: list[Any]) -> list[str]:
    return [f.code for f in findings]


# -- timing --------------------------------------------------------------------------------------


def test_rates_gaps_and_jitter_by_hand() -> None:
    # 10 Hz with one 1.0 s hole: messages at 0.0..0.9 and 1.9..2.9 s (21 messages)
    recv = [T0 + i * 100 * MS for i in range(10)] + [T0 + (19 + i) * 100 * MS for i in range(11)]
    stats = TopicStats("/scan", "x", len(recv))
    findings = analyze_rates(stats, recv, T0, recv[-1], OPTS)
    assert stats.rate_avg_hz == pytest.approx(20 / 2.9)
    assert stats.period_median_s == pytest.approx(0.1)
    assert stats.rate_min_hz == pytest.approx(1.0)
    assert stats.rate_max_hz == pytest.approx(10.0)
    assert stats.jitter_s == pytest.approx(0.0, abs=1e-9)
    assert stats.gap_threshold_s == pytest.approx(0.3)
    assert [(g.start_ns - T0, g.end_ns - T0) for g in stats.gaps] == [(900 * MS, 1900 * MS)]
    (gap,) = findings
    assert gap.items == [
        "+0.900 s -> +1.900 s: 1.000 s without messages (~9 expected messages missing)"
    ]
    assert len(stats.timeline) == 3


def test_jitter_and_expected_rate() -> None:
    # alternating 50 / 150 ms intervals: mean 100 ms, population std dev 50 ms (not > 50 %)
    recv = [T0]
    for i in range(30):
        recv.append(recv[-1] + (50 if i % 2 == 0 else 150) * MS)
    stats = TopicStats("/a", "x", len(recv))
    findings = analyze_rates(stats, recv, T0, recv[-1], OPTS)
    assert stats.jitter_s == pytest.approx(0.05)
    assert findings == []
    # 30 / 160 ms: mean 95 ms, std dev 65 ms > half the median period -> warning
    recv = [T0]
    for i in range(30):
        recv.append(recv[-1] + (30 if i % 2 == 0 else 160) * MS)
    stats = TopicStats("/a", "x", len(recv))
    assert _codes(analyze_rates(stats, recv, T0, recv[-1], OPTS)) == ["high_jitter"]
    # expectation: 10 Hz measured against 20 Hz expected
    recv = [T0 + i * 100 * MS for i in range(20)]
    stats = TopicStats("/b", "x", len(recv))
    findings = analyze_rates(stats, recv, T0, recv[-1], Options(expectations={"/b": 20.0}))
    assert _codes(findings) == ["rate_mismatch"]
    assert findings[0].data == {"rate_hz": pytest.approx(10.0), "expected_hz": 20.0}


def test_non_periodic_and_short_topics_skip_gap_checks() -> None:
    recv = [T0, T0 + S, T0 + 10 * S]
    stats = TopicStats("/rosout", "rcl_interfaces/msg/Log", 3)
    assert analyze_rates(stats, recv, T0, recv[-1], OPTS) == []
    long = [T0 + i * 10 * MS for i in range(20)] + [T0 + 10 * S]
    assert analyze_rates(TopicStats("/rosout", "", 21), long, T0, long[-1], OPTS, False) == []
    assert is_event_topic("/rosout", "rcl_interfaces/msg/Log")
    assert is_event_topic("/x/_action/feedback", "a/msg/B")
    assert is_event_topic("/s/_service_event", "example_interfaces/srv/AddTwoInts_Event")
    assert not is_event_topic("/scan", "sensor_msgs/msg/LaserScan")
    single = TopicStats("/one", "", 1)
    assert analyze_rates(single, [T0], T0, T0, OPTS) == []
    assert single.rate_avg_hz is None and single.first_ns == T0


def test_stamp_checks_by_hand() -> None:
    recv = [T0 + i * 100 * MS for i in range(8)]
    stamps = [
        recv[0] - 20 * MS,  # latency 20 ms
        recv[1] - 20 * MS,
        recv[1] - 20 * MS,  # duplicate of the previous stamp (latency 120 ms)
        0,  # zero stamp
        recv[4] + 3 * S,  # 3 s in the future
        recv[5] + 30 * MS,  # 30 ms after receive: clock skew
        recv[6] - 20 * MS,
        recv[6] - 50 * MS,  # earlier than the previous stamp
    ]
    stats = TopicStats("/x", "x", 8)
    findings = analyze_stamps(stats, recv, stamps, T0, OPTS)
    by = {f.code: f for f in findings}
    assert set(by) == {
        "zero_stamp",
        "future_stamp",
        "stamp_after_receive",
        "stamp_out_of_order",
        "duplicate_stamp",
    }
    assert by["zero_stamp"].items == ["received at +0.300 s"]
    assert by["future_stamp"].items == ["received at +0.400 s: stamp is 3.000 s ahead"]
    assert by["stamp_after_receive"].data == {"max_ahead_s": pytest.approx(0.03)}
    assert by["stamp_out_of_order"].count == 1
    # latencies (s): 0.02, 0.02, 0.12, -0.03, 0.02, 0.15 -> sorted
    assert stats.latency is not None
    assert stats.latency.samples == 6
    assert stats.latency.min_s == pytest.approx(-0.03)
    assert stats.latency.max_s == pytest.approx(0.15)
    assert stats.latency.p50_s == pytest.approx(0.02)
    assert stats.latency.mean_s == pytest.approx(0.3 / 6)


def test_high_latency_and_sim_time() -> None:
    recv = [T0 + i * 100 * MS for i in range(10)]
    stats = TopicStats("/slow", "x", 10)
    findings = analyze_stamps(stats, recv, [r - 400 * MS for r in recv], T0, OPTS)
    assert _codes(findings) == ["high_latency"]
    sim = [5 * S + i for i in range(10)]
    stats = TopicStats("/sim", "x", 10)
    findings = analyze_stamps(stats, recv, sim, T0, OPTS)
    assert _codes(findings) == ["sim_time_stamps"]
    assert stats.latency is None


def test_small_helpers() -> None:
    assert percentile([1.0, 2.0, 3.0, 4.0], 0.5) == 2.5
    assert percentile([7.0], 0.95) == 7.0
    assert histogram([1.0, 1.0]).counts == [2]
    hist = histogram([0.0, 1.0, 2.0, 3.0], bins=3)
    assert hist.counts == [1, 1, 2]
    bin_s, rates = timeline([0, S // 2, S + 1], 0, int(1.5 * S))
    assert bin_s == 1.0
    assert rates == [2.0, pytest.approx(2.0, rel=1e-6)]
    assert duration_text(2.5) == "2.500 s"
    assert duration_text(0.0125) == "12.5 ms"
    assert duration_text(2.5e-6) == "2.5 us"
    assert duration_text(3e-9) == "3 ns"


# -- TF ------------------------------------------------------------------------------------------


def test_tf_tree_problems() -> None:
    tf = TfCollector()
    tf.add_transform("map", "odom", False, "/tf")
    tf.add_transform("odom", "base_link", False, "/tf")
    tf.add_transform("base_link", "laser", True, "/tf_static")
    tf.add_transform("base_link", "laser", False, "/tf")  # static and dynamic
    tf.add_transform("map", "base_link", False, "/tf")  # second parent
    tf.add_transform("/cam_base", "cam", True, "/tf_static")  # leading slash, separate tree
    tf.reference("laser", "/scan")
    tf.reference("imu_link", "/imu")  # missing
    tf.reference("cam", "/camera")  # disconnected
    tf.reference("", "/joint_states")
    summary, findings = analyze_tf(tf)
    codes = _codes(findings)
    assert codes == [
        "tf_static_and_dynamic",
        "tf_leading_slash",
        "tf_multiple_parents",
        "tf_disconnected",
        "tf_broken_chain",
        "tf_missing_frame",
    ]
    assert findings[2].items == ["base_link: parents map, odom"]
    assert summary.main_root == "map"
    assert summary.missing == ["imu_link"]
    assert summary.unreachable == ["cam"]
    assert summary.components == [["base_link", "laser", "map", "odom"], ["cam", "cam_base"]]


def test_tf_cycle_and_absent() -> None:
    tf = TfCollector()
    tf.add_transform("a", "b", False, "/tf")
    tf.add_transform("b", "c", False, "/tf")
    tf.add_transform("c", "a", False, "/tf")
    _, findings = analyze_tf(tf)
    cycle = next(f for f in findings if f.code == "tf_cycle")
    assert cycle.detail.endswith("a -> b -> c -> a")
    empty = TfCollector()
    empty.reference("laser", "/scan")
    _, findings = analyze_tf(empty)
    assert _codes(findings) == ["tf_absent"]
    assert analyze_tf(TfCollector())[1] == []


# -- LaserScan -----------------------------------------------------------------------------------


def _scan(**overrides: Any) -> dict[str, Any]:
    scan: dict[str, Any] = {
        "angle_min": -1.0,
        "angle_max": 1.0,
        "angle_increment": 0.5,
        "range_min": 0.2,
        "range_max": 5.0,
        "scan_time": 0.1,
        "ranges": [1.0, 0.1, 6.0, math.nan, -math.inf],
        "intensities": [],
    }
    scan.update(overrides)
    return scan


def test_laser_checks_by_hand() -> None:
    col = LaserCollector()
    col.add(_scan(), T0, T0)  # (1 - -1)/0.5 + 1 = 5 beams: consistent
    col.add(_scan(ranges=[1.0] * 9, intensities=[1.0]), T0 + S, T0)  # 9 beams: mismatch
    col.add(_scan(range_min=5.0, range_max=1.0, ranges=[]), T0 + 2 * S, T0)
    findings = laser_findings("/scan", col, 5.0, OPTS)
    by = {f.code: f for f in findings}
    assert set(by) == {
        "laser_config",
        "laser_angle_mismatch",
        "laser_intensity_length",
        "laser_out_of_range",
        "laser_invalid_share",
        "laser_scan_time",
    }
    # 14 readings: 1 below range_min, 1 above range_max -> 2/14 = 14 % (warning)
    assert by["laser_out_of_range"].data == {"below_min": 1, "above_max": 1}
    assert by["laser_out_of_range"].severity == "warning"
    assert by["laser_invalid_share"].data == {"nan": 1, "pos_inf": 0, "neg_inf": 1}
    assert by["laser_invalid_share"].severity == "info"
    assert by["laser_angle_mismatch"].items == [
        "received at +1.000 s: (angle_max - angle_min) / angle_increment + 1 = 5.0 but 9 ranges"
    ]
    # scan_time 0.1 s vs a 5 Hz (0.2 s) rate
    assert by["laser_scan_time"].data == {"scan_time_s": 0.1, "period_s": 0.2}


# -- IMU -----------------------------------------------------------------------------------------


def _imu(acc: tuple[float, float, float], gyro: float = 0.0, **overrides: Any) -> dict[str, Any]:
    msg: dict[str, Any] = {
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "orientation_covariance": [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1],
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": gyro},
        "angular_velocity_covariance": [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1],
        "linear_acceleration": {"x": acc[0], "y": acc[1], "z": acc[2]},
        "linear_acceleration_covariance": [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1],
    }
    msg.update(overrides)
    return msg


@pytest.mark.parametrize(
    ("acc", "hint"),
    [
        ((0.0, 0.6, 0.8), "in g instead of m/s^2"),  # |a| = 1.0
        ((0.0, 0.0, 0.05), "Gravity seems to be removed"),
        ((0.0, 0.0, 4.9), "factor of 2"),
        ((0.0, 0.0, 7.0), "scale factor"),  # neither 1 g, 0, g/2 nor 2 g
    ],
)
def test_imu_gravity_hints(acc: tuple[float, float, float], hint: str) -> None:
    col = ImuCollector()
    for i in range(20):
        col.add(_imu(acc), T0 + i, T0, OPTS)
    (finding,) = imu_findings("/imu", col, OPTS)
    assert finding.code == "imu_gravity"
    assert hint in finding.detail
    assert finding.data["median_m_s2"] == pytest.approx(math.sqrt(sum(a * a for a in acc)))


def test_imu_good_gravity_and_other_checks() -> None:
    col = ImuCollector()
    for i in range(20):
        col.add(_imu((0.0, 0.0, 9.80665)), T0 + i, T0, OPTS)
    assert imu_findings("/imu", col, OPTS) == []

    col = ImuCollector()
    col.add(_imu((0.0, 0.0, 9.8), orientation={"x": 0, "y": 0, "z": 0, "w": 0}), T0, T0, OPTS)
    col.add(_imu((math.nan, 0.0, 9.8), orientation_covariance=[-1.0] + [0.0] * 8), T0, T0, OPTS)
    col.add(
        _imu(
            (0.0, 0.0, 9.8),
            gyro=1.0,
            angular_velocity_covariance=[-1.0] + [0.0] * 8,
            linear_acceleration_covariance=[0.0] * 9,
            orientation_covariance=[-0.5, 0, 0, 0, 0.1, 0, 0, 0, 0.1],
        ),
        T0,
        T0,
        OPTS,
    )
    codes = _codes(imu_findings("/imu", col, OPTS))
    assert codes == [
        "imu_non_finite",
        "imu_quaternion_norm",
        "imu_field_not_provided",
        "imu_no_orientation",
        "imu_zero_covariance",
        "imu_negative_variance",
        "imu_not_stationary",
    ]


# -- Odometry and cmd_vel ------------------------------------------------------------------------


def _odom_msg(x: float, y: float, yaw: float, vx: float, vy: float = 0.0, wz: float = 0.0) -> Any:
    return {
        "pose": {
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)},
            }
        },
        "twist": {
            "twist": {"linear": {"x": vx, "y": vy, "z": 0.0}, "angular": {"x": 0, "y": 0, "z": wz}}
        },
    }


def test_odometry_twist_mismatch_by_hand() -> None:
    # Heading 90 deg, moving +y in the world at 0.5 m/s, i.e. body +x at 0.5 m/s, but the
    # twist reports 0.25 m/s: mean error 0.25 m/s, ratio pose/twist = 2.00.
    col = OdomCollector()
    for i in range(50):
        t = i * 20 * MS
        col.add(_odom_msg(0.0, 0.5 * t / S, math.pi / 2, 0.25), T0 + t, T0 + t, T0, OPTS)
    (finding,) = odom_findings("/odom", col, OPTS, T0)
    assert finding.code == "odom_twist_mismatch"
    assert finding.data["mean_error_m_s"] == pytest.approx(0.25)
    assert finding.data["ratio"] == "2.00"


def test_odometry_consistent_motion_and_yaw_rate() -> None:
    col = OdomCollector()
    yaw = x = y = 0.0
    for i in range(100):
        t = i * 20 * MS
        col.add(_odom_msg(x, y, yaw, 0.4, 0.0, 0.5), T0 + t, T0 + t, T0, OPTS)
        mid = yaw + 0.5 * 0.02 / 2
        x += 0.4 * math.cos(mid) * 0.02
        y += 0.4 * math.sin(mid) * 0.02
        yaw += 0.5 * 0.02
    assert odom_findings("/odom", col, OPTS, T0) == []
    # same poses but twist.angular.z reported as 0
    bad = OdomCollector(
        samples=[OdomSample(s.recv, s.stamp, s.x, s.y, s.yaw, s.vx, s.vy, 0.0) for s in col.samples]
    )
    assert _codes(odom_findings("/odom", bad, OPTS, T0)) == ["odom_yaw_rate_mismatch"]


def test_odometry_bad_quaternion_and_teleport() -> None:
    col = OdomCollector()
    msg = _odom_msg(0.0, 0.0, 0.0, 0.0)
    msg["pose"]["pose"]["orientation"] = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
    col.add(msg, T0, T0, T0, OPTS)
    col.add(_odom_msg(2.0, 0.0, 0.0, 0.0), T0 + 20 * MS, T0 + 20 * MS, T0, OPTS)
    assert yaw_from_quaternion({"x": 0, "y": 0, "z": 1, "w": 0}) == pytest.approx(math.pi)
    findings = odom_findings("/odom", col, OPTS, T0)
    assert _codes(findings) == ["odom_quaternion_norm", "odom_teleport"]
    assert findings[1].items == [
        "+0.000 s -> +0.020 s: position jumped 2.000 m in 0.020 s while twist reports 0.000 m/s"
    ]


def test_cmd_vel_response_cases() -> None:
    cmds = [CmdSample(T0, 0.0, 0.0, 0.0), CmdSample(T0 + S, 0.5, 0.0, 0.0)]
    cmds += [CmdSample(T0 + 3 * S, 0.0, 0.0, 0.0), CmdSample(T0 + 4 * S, 0.0, 0.0, 1.0)]
    cmds += [CmdSample(T0 + 6 * S, 0.0, 0.0, 0.0), CmdSample(T0 + 7 * S, 0.0, 0.3, 0.0)]

    def odom(t: int, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> OdomSample:
        return OdomSample(t, t, 0.0, 0.0, 0.0, vx, vy, wz)

    samples = [
        odom(T0 + S + 100 * MS, 0.1),
        odom(T0 + S + 300 * MS, 0.3),  # >= 0.25: response after 300 ms
        odom(T0 + 4 * S + 200 * MS, wz=-0.8),  # rotating the wrong way
        odom(T0 + 7 * S + 500 * MS, vy=0.0),  # never responds
    ]
    findings = cmd_response_findings("/cmd_vel", cmds, "/odom", samples, OPTS, T0)
    assert _codes(findings) == ["cmd_vel_opposite", "cmd_vel_no_response", "cmd_vel_response"]
    assert findings[2].data["latencies_s"] == [pytest.approx(0.3)]
    slow = Options(slow_response_s=0.2)
    findings = cmd_response_findings("/cmd_vel", cmds[:2], "/odom", samples, slow, T0)
    assert _codes(findings) == ["cmd_vel_slow_response"]
    assert cmd_response_findings("/cmd_vel", [], "/odom", samples, OPTS, T0) == []


# -- Image and JointState ------------------------------------------------------------------------


def test_image_and_joint_state_checks() -> None:
    col = ImageCollector()
    base = {"height": 2, "width": 4, "step": 12, "encoding": "rgb8"}
    col.add({**base, "data": bytes(24)}, T0, T0)
    col.add({**base, "data": SkippedBytes(20)}, T0 + S, T0)
    col.add({**base, "step": 8, "data": bytes(16)}, T0 + 2 * S, T0)
    (finding,) = image_findings("/cam", col)
    assert finding.items == [
        "received at +1.000 s: data has 20 bytes, step*height = 24",
        "received at +2.000 s: step 8 < width 4 x 3 bytes for rgb8",
    ]
    assert image_findings("/cam", ImageCollector()) == []

    joints = JointStateCollector()
    joints.add({"name": ["a", "b"], "position": [0.0, 1.0], "velocity": [], "effort": []}, T0, T0)
    joints.add(
        {"name": ["a"], "position": [0.0, 1.0], "velocity": [math.nan], "effort": []}, T0, T0
    )
    assert _codes(joint_state_findings("/js", joints)) == [
        "joint_state_lengths",
        "joint_state_non_finite",
    ]


# -- whole-bag behaviour on small generated bags -------------------------------------------------


def _bag(tmp_path: Path, records: list[Record]) -> Path:
    path = tmp_path / "small.mcap"
    write_mcap(path, records)
    return path


def test_clean_bag_passes_and_frame_checks(tmp_path: Path) -> None:
    reg = Registry()

    def scan(t: int, frame: str) -> Record:
        msg = {
            "header": {"stamp": {"sec": (T0 + t) // S, "nanosec": (T0 + t) % S}, "frame_id": frame},
            "angle_min": -1.0,
            "angle_max": 1.0,
            "angle_increment": 1.0,
            "scan_time": 0.1,
            "range_min": 0.1,
            "range_max": 10.0,
            "ranges": [1.0, 2.0, 3.0],
        }
        return Record(
            "/scan",
            "sensor_msgs/msg/LaserScan",
            T0 + t + MS,
            encode(reg, "sensor_msgs/msg/LaserScan", msg),
        )

    clean = [scan(i * 100 * MS, "laser") for i in range(30)]
    with open_bag(_bag(tmp_path, clean)) as bag:
        report = analyze(bag)
    assert report.codes() == [("tf_absent", "")]
    assert report.verdict == PASS

    mixed = [scan(i * 100 * MS, "laser" if i % 2 else "") for i in range(30)]
    with open_bag(_bag(tmp_path, mixed)) as bag:
        report = analyze(bag)
    assert ("frame_id_changes", "/scan") in report.codes()
    assert ("empty_frame_id", "/scan") in report.codes()
    assert report.verdict == WARN

    broken = [
        *clean[:5],
        Record("/scan", "sensor_msgs/msg/LaserScan", T0 + S, b"\x00\x01\x00\x00\x01"),
    ]
    with open_bag(_bag(tmp_path, broken)) as bag:
        report = analyze(bag)
    assert ("decode_error", "/scan") in report.codes()
    assert report.verdict == FAIL
