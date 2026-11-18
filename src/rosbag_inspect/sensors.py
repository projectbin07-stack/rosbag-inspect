"""Type-specific sanity checks: LaserScan, Imu, Odometry, cmd_vel response, Image, JointState."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Any

from .cdr import Message, SkippedBytes
from .findings import ERROR, INFO, WARNING, Finding
from .options import Options
from .timing import MAX_ITEMS, median, rel

LASER = "sensor_msgs/msg/LaserScan"
IMU = "sensor_msgs/msg/Imu"
ODOM = "nav_msgs/msg/Odometry"
TWIST = "geometry_msgs/msg/Twist"
TWIST_STAMPED = "geometry_msgs/msg/TwistStamped"
IMAGE = "sensor_msgs/msg/Image"
JOINT_STATE = "sensor_msgs/msg/JointState"
TF = "tf2_msgs/msg/TFMessage"

BYTES_PER_PIXEL = {
    "mono8": 1,
    "8UC1": 1,
    "bayer_rggb8": 1,
    "bayer_bggr8": 1,
    "bayer_gbrg8": 1,
    "bayer_grbg8": 1,
    "mono16": 2,
    "16UC1": 2,
    "16SC1": 2,
    "yuv422": 2,
    "uyvy": 2,
    "yuyv": 2,
    "rgb8": 3,
    "bgr8": 3,
    "8UC3": 3,
    "rgba8": 4,
    "bgra8": 4,
    "8UC4": 4,
    "32FC1": 4,
    "rgb16": 6,
    "bgr16": 6,
    "rgba16": 8,
    "bgra16": 8,
}


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quaternion(q: dict[str, Any]) -> float:
    x, y, z, w = (float(q[k]) for k in ("x", "y", "z", "w"))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quaternion_norm(q: dict[str, Any]) -> float:
    return math.sqrt(sum(float(q[k]) ** 2 for k in ("x", "y", "z", "w")))


def _pct(part: int, whole: int) -> str:
    return f"{100 * part / whole:.1f}%" if whole else "0%"


# -- LaserScan ---------------------------------------------------------------------------------


@dataclass
class LaserCollector:
    messages: int = 0
    ranges: int = 0
    nan: int = 0
    pos_inf: int = 0
    neg_inf: int = 0
    out_of_range: int = 0
    below_min: int = 0
    above_max: int = 0
    angle_mismatch: list[str] = field(default_factory=list)
    config_errors: list[str] = field(default_factory=list)
    config_count: int = 0
    intensity_mismatch: int = 0
    scan_times: list[float] = field(default_factory=list)

    def add(self, msg: Message, recv: int, origin: int) -> None:
        self.messages += 1
        ranges: list[float] = msg["ranges"]
        rmin, rmax = msg["range_min"], msg["range_max"]
        amin, amax, inc = msg["angle_min"], msg["angle_max"], msg["angle_increment"]
        self.ranges += len(ranges)
        if not (rmin < rmax) or inc == 0 or not math.isfinite(inc):
            self.config_count += 1
            if len(self.config_errors) < MAX_ITEMS:
                self.config_errors.append(
                    f"received at {rel(recv, origin)}: range_min={rmin:g} range_max={rmax:g} "
                    f"angle_increment={inc:g}"
                )
        else:
            expected = (amax - amin) / inc + 1
            if abs(expected - len(ranges)) > 1.01:
                self.angle_mismatch.append(
                    f"received at {rel(recv, origin)}: (angle_max - angle_min) / "
                    f"angle_increment + 1 = {expected:.1f} but {len(ranges)} ranges"
                )
        for r in ranges:
            if r != r:
                self.nan += 1
            elif r == math.inf:
                self.pos_inf += 1
            elif r == -math.inf:
                self.neg_inf += 1
            elif r < rmin:
                self.below_min += 1
            elif r > rmax:
                self.above_max += 1
        intensities = msg["intensities"]
        if intensities and len(intensities) != len(ranges):
            self.intensity_mismatch += 1
        if msg["scan_time"] > 0:
            self.scan_times.append(float(msg["scan_time"]))


def laser_findings(
    topic: str, col: LaserCollector, rate_hz: float | None, opts: Options
) -> list[Finding]:
    findings: list[Finding] = []
    if col.config_count:
        findings.append(
            Finding(
                "laser_config",
                ERROR,
                topic,
                "Invalid LaserScan limits",
                f"{col.config_count} scan(s) have range_min >= range_max or a zero / "
                "non-finite angle_increment; consumers cannot interpret the ranges.",
                col.config_count,
                col.config_errors,
            )
        )
    if col.angle_mismatch:
        findings.append(
            Finding(
                "laser_angle_mismatch",
                ERROR,
                topic,
                "angle_increment does not match the number of ranges",
                f"{len(col.angle_mismatch)} scan(s) contain a different number of ranges than "
                "the angle fields imply. Costmaps and SLAM will place every beam at the wrong "
                "angle; check the driver's angle settings.",
                len(col.angle_mismatch),
                col.angle_mismatch[:MAX_ITEMS],
            )
        )
    if col.intensity_mismatch:
        findings.append(
            Finding(
                "laser_intensity_length",
                WARNING,
                topic,
                "intensities length differs from ranges",
                f"{col.intensity_mismatch} scan(s) have a non-empty intensities array whose "
                "length is not the number of ranges.",
                col.intensity_mismatch,
            )
        )
    outside = col.below_min + col.above_max
    if outside and col.ranges:
        share = outside / col.ranges
        findings.append(
            Finding(
                "laser_out_of_range",
                WARNING if share > opts.laser_out_of_range_warn else INFO,
                topic,
                "Finite ranges outside [range_min, range_max]",
                f"{outside} of {col.ranges} readings ({_pct(outside, col.ranges)}) are finite but "
                f"outside the advertised limits ({col.below_min} below range_min, "
                f"{col.above_max} above range_max). REP 117 asks drivers to report these as "
                "-inf/+inf/NaN; consumers will discard them.",
                outside,
                data={"below_min": col.below_min, "above_max": col.above_max},
            )
        )
    invalid = col.nan + col.pos_inf + col.neg_inf
    if invalid and col.ranges:
        share = invalid / col.ranges
        findings.append(
            Finding(
                "laser_invalid_share",
                WARNING if share > opts.laser_invalid_warn else INFO,
                topic,
                "Readings without a valid return",
                f"{_pct(invalid, col.ranges)} of readings are NaN/inf ({col.nan} NaN, "
                f"{col.pos_inf} +inf, {col.neg_inf} -inf)."
                + (
                    " More than half the beams return nothing: is the sensor blocked or "
                    "the range_max too small?"
                    if share > opts.laser_invalid_warn
                    else ""
                ),
                invalid,
                data={"nan": col.nan, "pos_inf": col.pos_inf, "neg_inf": col.neg_inf},
            )
        )
    if col.scan_times and rate_hz:
        scan_time = median(col.scan_times)
        period = 1 / rate_hz
        if abs(scan_time - period) > 0.2 * period:
            findings.append(
                Finding(
                    "laser_scan_time",
                    WARNING,
                    topic,
                    "scan_time does not match the publish rate",
                    f"Median scan_time is {scan_time * 1000:.1f} ms but scans arrive every "
                    f"{period * 1000:.1f} ms on average. Scans are being dropped, or the "
                    "driver's scan_time / time_increment are wrong (this skews motion "
                    "de-skewing).",
                    data={"scan_time_s": scan_time, "period_s": period},
                )
            )
    return findings


# -- IMU ---------------------------------------------------------------------------------------

_IMU_COVARIANCES = (
    ("orientation", "orientation_covariance"),
    ("angular_velocity", "angular_velocity_covariance"),
    ("linear_acceleration", "linear_acceleration_covariance"),
)


@dataclass
class ImuCollector:
    samples: list[tuple[int, float, float]] = field(default_factory=list)
    """(receive time, |linear_acceleration|, |angular_velocity|)"""
    bad_quaternion: list[str] = field(default_factory=list)
    not_provided: dict[str, int] = field(default_factory=dict)
    zero_covariance: dict[str, int] = field(default_factory=dict)
    negative_variance: dict[str, int] = field(default_factory=dict)
    non_finite: int = 0

    def add(self, msg: Message, recv: int, origin: int, opts: Options) -> None:
        a = msg["linear_acceleration"]
        g = msg["angular_velocity"]
        acc = math.sqrt(a["x"] ** 2 + a["y"] ** 2 + a["z"] ** 2)
        gyro = math.sqrt(g["x"] ** 2 + g["y"] ** 2 + g["z"] ** 2)
        if not (math.isfinite(acc) and math.isfinite(gyro)):
            self.non_finite += 1
        else:
            self.samples.append((recv, acc, gyro))
        for name, cov_name in _IMU_COVARIANCES:
            cov: list[float] = msg[cov_name]
            if cov[0] == -1:
                self.not_provided[name] = self.not_provided.get(name, 0) + 1
                continue
            if all(v == 0 for v in cov):
                self.zero_covariance[name] = self.zero_covariance.get(name, 0) + 1
            if any(cov[i] < 0 for i in (0, 4, 8)):
                self.negative_variance[name] = self.negative_variance.get(name, 0) + 1
        if msg["orientation_covariance"][0] != -1:
            norm = quaternion_norm(msg["orientation"])
            if not abs(norm - 1) <= opts.quaternion_tolerance:
                self.bad_quaternion.append(f"received at {rel(recv, origin)}: |q| = {norm:.4f}")


def imu_findings(topic: str, col: ImuCollector, opts: Options) -> list[Finding]:
    findings: list[Finding] = []
    total = len(col.samples) + col.non_finite
    if col.non_finite:
        findings.append(
            Finding(
                "imu_non_finite",
                ERROR,
                topic,
                "NaN/inf in IMU data",
                f"{col.non_finite} message(s) contain NaN or infinite rates/accelerations.",
                col.non_finite,
            )
        )
    if col.bad_quaternion:
        findings.append(
            Finding(
                "imu_quaternion_norm",
                ERROR,
                topic,
                "Orientation is not a unit quaternion",
                f"{len(col.bad_quaternion)} message(s) report an orientation whose norm differs "
                f"from 1 by more than {opts.quaternion_tolerance:g} although "
                "orientation_covariance[0] is not -1. If the IMU has no orientation estimate, "
                "set orientation_covariance[0] = -1 (sensor_msgs/Imu convention).",
                len(col.bad_quaternion),
                col.bad_quaternion[:MAX_ITEMS],
            )
        )
    for name in ("angular_velocity", "linear_acceleration"):
        count = col.not_provided.get(name, 0)
        if count:
            findings.append(
                Finding(
                    "imu_field_not_provided",
                    WARNING,
                    topic,
                    f"{name} marked as not provided",
                    f"{count} message(s) set {name}_covariance[0] = -1, which tells consumers "
                    f"to ignore {name}. An IMU driver normally provides it.",
                    count,
                    data={"field": name},
                )
            )
    if col.not_provided.get("orientation"):
        findings.append(
            Finding(
                "imu_no_orientation",
                INFO,
                topic,
                "IMU publishes no orientation",
                "orientation_covariance[0] = -1: orientation is marked as not provided "
                "(normal for a raw 6-axis IMU; use a filter such as imu_filter_madgwick).",
                col.not_provided["orientation"],
            )
        )
    if col.zero_covariance:
        fields = ", ".join(f"{k} ({v})" for k, v in sorted(col.zero_covariance.items()))
        findings.append(
            Finding(
                "imu_zero_covariance",
                INFO,
                topic,
                "All-zero covariance matrices",
                f"Covariance is all zeros for: {fields}. By the sensor_msgs/Imu convention "
                "this means 'unknown'; EKF nodes such as robot_localization need real values "
                "to weight the IMU.",
                sum(col.zero_covariance.values()),
            )
        )
    if col.negative_variance:
        fields = ", ".join(f"{k} ({v})" for k, v in sorted(col.negative_variance.items()))
        findings.append(
            Finding(
                "imu_negative_variance",
                WARNING,
                topic,
                "Negative variances",
                f"Covariance diagonals contain negative values (other than the -1 marker in "
                f"element 0) for: {fields}.",
                sum(col.negative_variance.values()),
            )
        )
    stationary = [acc for _, acc, gyro in col.samples if gyro < opts.stationary_gyro]
    if total and len(stationary) < 10:
        findings.append(
            Finding(
                "imu_not_stationary",
                INFO,
                topic,
                "Gravity check skipped",
                f"Only {len(stationary)} sample(s) with angular rate below "
                f"{opts.stationary_gyro:g} rad/s; the gravity magnitude check needs the robot "
                "standing still for a moment.",
            )
        )
    elif stationary:
        level = median(stationary)
        if abs(level - opts.gravity) > opts.gravity_tolerance:
            if 0.9 <= level <= 1.1:
                hint = (
                    " The value is close to 1: the driver probably reports acceleration in g "
                    "instead of m/s^2 (multiply by 9.80665)."
                )
            elif level < 0.5:
                hint = (
                    " Gravity seems to be removed; sensor_msgs/Imu linear_acceleration "
                    "should include it (REP 145)."
                )
            elif abs(level - opts.gravity / 2) < 0.5 or abs(level - 2 * opts.gravity) < 1:
                hint = " Off by a factor of 2: check the accelerometer full-scale setting."
            else:
                hint = " Check the accelerometer scale factor and calibration."
            findings.append(
                Finding(
                    "imu_gravity",
                    ERROR,
                    topic,
                    "Gravity magnitude is wrong",
                    f"While stationary ({len(stationary)} samples with angular rate < "
                    f"{opts.stationary_gyro:g} rad/s) the median |linear_acceleration| is "
                    f"{level:.3f} m/s^2; expected {opts.gravity:.2f} +/- "
                    f"{opts.gravity_tolerance:g}." + hint,
                    data={"median_m_s2": level, "stationary_samples": len(stationary)},
                )
            )
    return findings


# -- Odometry ----------------------------------------------------------------------------------


@dataclass
class OdomSample:
    recv: int
    stamp: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


@dataclass
class OdomCollector:
    samples: list[OdomSample] = field(default_factory=list)
    bad_quaternion: list[str] = field(default_factory=list)

    def add(self, msg: Message, recv: int, stamp: int, origin: int, opts: Options) -> None:
        pose = msg["pose"]["pose"]
        twist = msg["twist"]["twist"]
        q = pose["orientation"]
        norm = quaternion_norm(q)
        if not abs(norm - 1) <= opts.quaternion_tolerance:
            self.bad_quaternion.append(f"received at {rel(recv, origin)}: |q| = {norm:.4f}")
            yaw = 0.0 if norm == 0 or not math.isfinite(norm) else yaw_from_quaternion(q)
        else:
            yaw = yaw_from_quaternion(q)
        self.samples.append(
            OdomSample(
                recv,
                stamp,
                float(pose["position"]["x"]),
                float(pose["position"]["y"]),
                yaw,
                float(twist["linear"]["x"]),
                float(twist["linear"]["y"]),
                float(twist["angular"]["z"]),
            )
        )


def odom_findings(topic: str, col: OdomCollector, opts: Options, origin: int) -> list[Finding]:
    findings: list[Finding] = []
    if col.bad_quaternion:
        findings.append(
            Finding(
                "odom_quaternion_norm",
                ERROR,
                topic,
                "Odometry orientation is not a unit quaternion",
                f"{len(col.bad_quaternion)} message(s) have |q| != 1.",
                len(col.bad_quaternion),
                col.bad_quaternion[:MAX_ITEMS],
            )
        )
    samples = sorted(col.samples, key=lambda s: s.recv)
    use_stamp = all(s.stamp > 0 for s in samples)
    jumps: list[str] = []
    n = 0
    err_sum = speed_sum = pose_speed_sum = 0.0
    yaw_err_sum = yaw_rate_sum = 0.0
    for a, b in itertools.pairwise(samples):
        dt = ((b.stamp - a.stamp) if use_stamp else (b.recv - a.recv)) / 1e9
        if dt <= 0:
            continue
        dx, dy = b.x - a.x, b.y - a.y
        dist = math.hypot(dx, dy)
        twist_speed = max(math.hypot(a.vx, a.vy), math.hypot(b.vx, b.vy))
        if dist > opts.teleport_m and dist > 3 * twist_speed * dt + 0.1:
            jumps.append(
                f"{rel(a.recv, origin)} -> {rel(b.recv, origin)}: position jumped {dist:.3f} m "
                f"in {dt:.3f} s while twist reports {twist_speed:.3f} m/s"
            )
            continue
        if dt > 0.5:
            continue
        mid = a.yaw + wrap_angle(b.yaw - a.yaw) / 2
        c, s = math.cos(mid), math.sin(mid)
        bvx, bvy = (c * dx + s * dy) / dt, (-s * dx + c * dy) / dt
        tvx, tvy = (a.vx + b.vx) / 2, (a.vy + b.vy) / 2
        err_sum += math.hypot(bvx - tvx, bvy - tvy)
        speed_sum += math.hypot(tvx, tvy)
        pose_speed_sum += math.hypot(bvx, bvy)
        yaw_rate = wrap_angle(b.yaw - a.yaw) / dt
        twz = (a.wz + b.wz) / 2
        yaw_err_sum += abs(yaw_rate - twz)
        yaw_rate_sum += max(abs(twz), abs(yaw_rate))
        n += 1
    if jumps:
        findings.append(
            Finding(
                "odom_teleport",
                ERROR,
                topic,
                "Odometry pose jumps",
                f"{len(jumps)} jump(s) larger than {opts.teleport_m:g} m between consecutive "
                "messages that the reported velocity cannot explain (pose reset, wheel-encoder "
                "overflow, or a localisation correction published on the odometry topic).",
                len(jumps),
                jumps[:MAX_ITEMS],
            )
        )
    if n:
        mean_err, mean_speed, mean_pose = err_sum / n, speed_sum / n, pose_speed_sum / n
        if mean_err > opts.odom_mismatch_abs and mean_err > opts.odom_mismatch * max(
            mean_speed, mean_pose
        ):
            ratio = mean_pose / mean_speed if mean_speed > 1e-9 else math.inf
            ratio_text = f"{ratio:.2f}" if math.isfinite(ratio) else "inf"
            findings.append(
                Finding(
                    "odom_twist_mismatch",
                    WARNING,
                    topic,
                    "Twist disagrees with pose motion",
                    f"Velocity derived from consecutive poses differs from twist.linear by "
                    f"{mean_err:.3f} m/s on average (mean twist speed {mean_speed:.3f} m/s, "
                    f"pose-derived {mean_pose:.3f} m/s, ratio {ratio_text}). Check wheel "
                    "radius / track width parameters, and that twist is expressed in "
                    "child_frame_id as nav_msgs/Odometry requires.",
                    data={"mean_error_m_s": mean_err, "ratio": ratio_text},
                )
            )
        mean_yaw_err = yaw_err_sum / n
        if mean_yaw_err > 0.05 and mean_yaw_err > opts.odom_mismatch * (yaw_rate_sum / n):
            findings.append(
                Finding(
                    "odom_yaw_rate_mismatch",
                    WARNING,
                    topic,
                    "twist.angular.z disagrees with heading change",
                    f"Yaw rate derived from orientation differs from twist.angular.z by "
                    f"{mean_yaw_err:.3f} rad/s on average.",
                    data={"mean_error_rad_s": mean_yaw_err},
                )
            )
    return findings


# -- cmd_vel response --------------------------------------------------------------------------


@dataclass
class CmdSample:
    recv: int
    vx: float
    vy: float
    wz: float


def twist_values(msg: Message, stamped: bool) -> tuple[float, float, float]:
    twist = msg["twist"] if stamped else msg
    return (
        float(twist["linear"]["x"]),
        float(twist["linear"]["y"]),
        float(twist["angular"]["z"]),
    )


def cmd_response_findings(
    cmd_topic: str,
    cmds: list[CmdSample],
    odom_topic: str,
    odom: list[OdomSample],
    opts: Options,
    origin: int,
) -> list[Finding]:
    findings: list[Finding] = []
    cmds = sorted(cmds, key=lambda c: c.recv)
    odom = sorted(odom, key=lambda o: o.recv)
    if not cmds or not odom:
        return findings
    window = int(opts.response_window_s * 1e9)
    latencies: list[float] = []
    missing: list[str] = []
    opposite: list[str] = []
    prev_moving = False
    for c in cmds:
        lin = math.hypot(c.vx, c.vy)
        moving = lin >= 0.05 or abs(c.wz) >= 0.1
        resting = lin < 0.02 and abs(c.wz) < 0.05
        if moving and not prev_moving:
            response = _response(c, odom, window)
            label = f"command at {rel(c.recv, origin)} (vx={c.vx:g}, vy={c.vy:g}, wz={c.wz:g})"
            if response is None:
                missing.append(f"{label}: no response within {opts.response_window_s:g} s")
            elif response < 0:
                opposite.append(f"{label}: robot moved against the command")
            else:
                latencies.append(response)
        if moving:
            prev_moving = True
        elif resting:
            prev_moving = False
    rel_names = f"{cmd_topic} -> {odom_topic}"
    if opposite:
        findings.append(
            Finding(
                "cmd_vel_opposite",
                ERROR,
                cmd_topic,
                "Robot moves against the command",
                f"After {len(opposite)} command(s) the odometry velocity points the other way "
                f"({rel_names}): an inverted motor, swapped wheel order or wrong sign in the "
                "kinematics.",
                len(opposite),
                opposite[:MAX_ITEMS],
            )
        )
    if missing:
        findings.append(
            Finding(
                "cmd_vel_no_response",
                WARNING,
                cmd_topic,
                "Commands without odometry response",
                f"{len(missing)} start-from-rest command(s) were not followed by odometry "
                f"reaching half the commanded velocity within {opts.response_window_s:g} s "
                f"({rel_names}). Motor driver disabled, e-stop, a cmd_vel mux blocking the "
                "topic, or a too-short controller timeout.",
                len(missing),
                missing[:MAX_ITEMS],
            )
        )
    if latencies:
        med = median(latencies)
        findings.append(
            Finding(
                "cmd_vel_slow_response" if med > opts.slow_response_s else "cmd_vel_response",
                WARNING if med > opts.slow_response_s else INFO,
                cmd_topic,
                "Slow response to cmd_vel" if med > opts.slow_response_s else "cmd_vel response",
                f"Odometry reached half the commanded velocity {med * 1000:.0f} ms (median of "
                f"{len(latencies)} start-from-rest command(s)) after the command was recorded "
                f"({rel_names}).",
                len(latencies),
                [f"{lat * 1000:.0f} ms" for lat in latencies[:MAX_ITEMS]],
                {"median_s": med, "latencies_s": latencies},
            )
        )
    return findings


def _response(c: CmdSample, odom: list[OdomSample], window: int) -> float | None:
    """Seconds until odometry reaches half the command; negative if it moves the other way."""
    lin = math.hypot(c.vx, c.vy)
    against = False
    for o in odom:
        if o.recv < c.recv:
            continue
        if o.recv > c.recv + window:
            break
        if lin >= 0.05:
            proj = (o.vx * c.vx + o.vy * c.vy) / lin
            target = lin
        else:
            proj = o.wz * math.copysign(1, c.wz)
            target = abs(c.wz)
        if proj >= 0.5 * target:
            return (o.recv - c.recv) / 1e9
        if proj <= -0.5 * target:
            against = True
    return -1.0 if against else None


# -- Image / JointState ------------------------------------------------------------------------


@dataclass
class ImageCollector:
    problems: list[str] = field(default_factory=list)
    encodings: dict[str, int] = field(default_factory=dict)

    def add(self, msg: Message, recv: int, origin: int) -> None:
        data = msg["data"]
        length = data.length if isinstance(data, SkippedBytes) else len(data)
        height, width, step = int(msg["height"]), int(msg["width"]), int(msg["step"])
        encoding = str(msg["encoding"])
        self.encodings[encoding] = self.encodings.get(encoding, 0) + 1
        bpp = BYTES_PER_PIXEL.get(encoding)
        where = f"received at {rel(recv, origin)}"
        if length != step * height:
            self.problems.append(f"{where}: data has {length} bytes, step*height = {step * height}")
        elif bpp is not None and step < width * bpp:
            self.problems.append(
                f"{where}: step {step} < width {width} x {bpp} bytes for {encoding}"
            )


def image_findings(topic: str, col: ImageCollector) -> list[Finding]:
    if not col.problems:
        return []
    return [
        Finding(
            "image_size_mismatch",
            ERROR,
            topic,
            "Image size fields are inconsistent",
            f"{len(col.problems)} image(s) have a data length or step that does not match "
            "height, width and encoding; viewers and cv_bridge will reject them.",
            len(col.problems),
            col.problems[:MAX_ITEMS],
        )
    ]


@dataclass
class JointStateCollector:
    problems: list[str] = field(default_factory=list)
    non_finite: int = 0

    def add(self, msg: Message, recv: int, origin: int) -> None:
        n = len(msg["name"])
        for key in ("position", "velocity", "effort"):
            values = msg[key]
            if values and len(values) != n:
                self.problems.append(
                    f"received at {rel(recv, origin)}: {n} names but {len(values)} {key} values"
                )
            if any(not math.isfinite(v) for v in values):
                self.non_finite += 1


def joint_state_findings(topic: str, col: JointStateCollector) -> list[Finding]:
    findings: list[Finding] = []
    if col.problems:
        findings.append(
            Finding(
                "joint_state_lengths",
                ERROR,
                topic,
                "JointState arrays have different lengths",
                "position/velocity/effort must be empty or have one entry per name.",
                len(col.problems),
                col.problems[:MAX_ITEMS],
            )
        )
    if col.non_finite:
        findings.append(
            Finding(
                "joint_state_non_finite",
                WARNING,
                topic,
                "NaN/inf joint values",
                f"{col.non_finite} message(s) contain NaN or infinite values.",
                col.non_finite,
            )
        )
    return findings
