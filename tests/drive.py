"""A 20 s simulated drive of a mecanum robot with deliberate recording faults.

Every value is computed from integer nanosecond times so the expected findings can be derived
by hand (see ``tests/test_fixtures.py``). Faults:

1. ``/scan``: no messages between t = 8.0 s and t = 10.0 s (a 2 s LiDAR gap).
2. ``/imu``: message 500 repeats the stamp of message 499 (duplicated stamp).
3. ``/imu``: linear_acceleration is reported in g (|a| = 1.0 instead of 9.81).
4. TF: the laser is mounted as ``base_laser -> laser`` but nothing connects ``base_laser`` to
   ``base_link`` (broken chain for the frame_id of ``/scan``).
5. ``/joint_states``: message 300 is stamped 3600 s in the future.
6. ``/odom``: the pose jumps by +0.8 m in x at t = 12.0 s (teleport).
"""

from __future__ import annotations

import math
from typing import Any

from bagwriter import Record
from rosbag_inspect.cdr import encode
from rosbag_inspect.msgdef import Registry

T0 = 1_700_000_000_000_000_000
MS = 1_000_000
S = 1_000_000_000
DURATION = 20 * S
ODOM_DELAY = 150 * MS

# (start, end, vx, vy, wz) in ns and SI units; the command is zero outside these phases.
PHASES = [
    (2 * S, 10 * S, 0.3, 0.0, 0.0),
    (10 * S, 13 * S, 0.2, 0.0, 0.3),
    (14 * S, 17 * S, 0.0, 0.25, 0.0),
]
WHEELS = ["front_left", "front_right", "rear_left", "rear_right"]
WHEEL_RADIUS = 0.05


def command(t: int) -> tuple[float, float, float]:
    for start, end, vx, vy, wz in PHASES:
        if start <= t < end:
            return vx, vy, wz
    return 0.0, 0.0, 0.0


def header(stamp: int, frame: str) -> dict[str, Any]:
    return {"stamp": {"sec": stamp // S, "nanosec": stamp % S}, "frame_id": frame}


def quaternion(yaw: float) -> dict[str, float]:
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}


def integrate_pose(until: int, step: int = MS) -> list[tuple[float, float, float]]:
    """Pose (x, y, yaw) at every multiple of ``step`` from 0 to ``until`` (body velocity =
    command delayed by ODOM_DELAY), midpoint integration."""
    poses = [(0.0, 0.0, 0.0)]
    x = y = yaw = 0.0
    for i in range(until // step):
        vx, vy, wz = command(i * step - ODOM_DELAY)
        dt = step / S
        mid = yaw + wz * dt / 2
        x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
        y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
        yaw += wz * dt
        poses.append((x, y, yaw))
    return poses


def laser_ranges() -> list[float]:
    ranges = []
    for i in range(360):
        if i < 10:
            ranges.append(math.inf)  # no return behind the robot
        else:
            angle = -math.pi + i * 2 * math.pi / 360
            ranges.append(2.0 + 0.5 * math.cos(2 * angle))
    return ranges


def build_drive() -> list[Record]:
    reg = Registry()
    records: list[Record] = []

    def add(topic: str, type_name: str, recv: int, msg: dict[str, Any]) -> None:
        records.append(Record(topic, type_name, T0 + recv, encode(reg, type_name, msg)))

    # /tf_static, published once at the start
    static = [
        ("base_link", "imu_link", (0.0, 0.0, 0.1)),
        ("base_link", "camera_link", (0.15, 0.0, 0.2)),
        ("camera_link", "camera_color_optical_frame", (0.0, 0.0, 0.0)),
        ("base_laser", "laser", (0.0, 0.0, 0.05)),  # fault 4: base_link -> base_laser missing
    ]
    add(
        "/tf_static",
        "tf2_msgs/msg/TFMessage",
        0,
        {
            "transforms": [
                {
                    "header": header(T0, parent),
                    "child_frame_id": child,
                    "transform": {
                        "translation": {"x": tx, "y": ty, "z": tz},
                        "rotation": quaternion(0.0),
                    },
                }
                for parent, child, (tx, ty, tz) in static
            ]
        },
    )

    # /scan: 10 Hz, latency 10..14 ms, fault 1 (gap)
    inc = 2 * math.pi / 360
    ranges = laser_ranges()
    for k in range(200):
        t = k * 100 * MS
        if 8 * S < t < 10 * S:
            continue
        add(
            "/scan",
            "sensor_msgs/msg/LaserScan",
            t + (10 + k % 5) * MS,
            {
                "header": header(T0 + t, "laser"),
                "angle_min": -math.pi,
                "angle_max": math.pi - inc,
                "angle_increment": inc,
                "time_increment": 0.1 / 360,
                "scan_time": 0.1,
                "range_min": 0.12,
                "range_max": 12.0,
                "ranges": ranges,
                "intensities": [],
            },
        )

    poses = integrate_pose(DURATION)
    # /imu: 100 Hz, latency 2 ms, faults 2 and 3
    for k in range(2000):
        t = k * 10 * MS
        stamp_t = (k - 1) * 10 * MS if k == 500 else t
        vx, vy, wz = command(t - ODOM_DELAY)
        _, _, yaw = poses[t // MS]
        add(
            "/imu",
            "sensor_msgs/msg/Imu",
            t + 2 * MS,
            {
                "header": header(T0 + stamp_t, "imu_link"),
                "orientation": quaternion(yaw),
                "orientation_covariance": [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": wz},
                "angular_velocity_covariance": [0.0004, 0, 0, 0, 0.0004, 0, 0, 0, 0.0004],
                # fault 3: centripetal term in g as well, gravity 1.0 instead of 9.80665
                "linear_acceleration": {"x": 0.0, "y": vx * wz / 9.80665, "z": 1.0},
                "linear_acceleration_covariance": [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
            },
        )

    # /odom (50 Hz, latency 5 ms) and /tf odom -> base_link (latency 6 ms); fault 6
    for k in range(1000):
        t = k * 20 * MS
        x, y, yaw = poses[t // MS]
        if t >= 12 * S:
            x += 0.8
        vx, vy, wz = command(t - ODOM_DELAY)
        add(
            "/odom",
            "nav_msgs/msg/Odometry",
            t + 5 * MS,
            {
                "header": header(T0 + t, "odom"),
                "child_frame_id": "base_link",
                "pose": {
                    "pose": {
                        "position": {"x": x, "y": y, "z": 0.0},
                        "orientation": quaternion(yaw),
                    },
                    "covariance": [0.0] * 36,
                },
                "twist": {
                    "twist": {
                        "linear": {"x": vx, "y": vy, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": wz},
                    },
                    "covariance": [0.0] * 36,
                },
            },
        )
        add(
            "/tf",
            "tf2_msgs/msg/TFMessage",
            t + 6 * MS,
            {
                "transforms": [
                    {
                        "header": header(T0 + t, "odom"),
                        "child_frame_id": "base_link",
                        "transform": {
                            "translation": {"x": x, "y": y, "z": 0.0},
                            "rotation": quaternion(yaw),
                        },
                    }
                ]
            },
        )

    # /cmd_vel: 10 Hz Twist (no header), recorded 1 ms after the command time
    for k in range(200):
        t = k * 100 * MS
        vx, vy, wz = command(t)
        add(
            "/cmd_vel",
            "geometry_msgs/msg/Twist",
            t + 1 * MS,
            {"linear": {"x": vx, "y": vy, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": wz}},
        )

    # /joint_states: 50 Hz, latency 4 ms, fault 5 (message 300 stamped 1 h ahead)
    angles = [0.0] * 4
    for k in range(1000):
        t = k * 20 * MS
        vx, vy, wz = command(t - ODOM_DELAY)
        # mecanum inverse kinematics (lx + ly = 0.3 m)
        speeds = [
            (vx - vy - 0.3 * wz) / WHEEL_RADIUS,
            (vx + vy + 0.3 * wz) / WHEEL_RADIUS,
            (vx + vy - 0.3 * wz) / WHEEL_RADIUS,
            (vx - vy + 0.3 * wz) / WHEEL_RADIUS,
        ]
        stamp_t = t + 3600 * S if k == 300 else t
        add(
            "/joint_states",
            "sensor_msgs/msg/JointState",
            t + 4 * MS,
            {
                "header": header(T0 + stamp_t, ""),
                "name": [f"{w}_wheel_joint" for w in WHEELS],
                "position": list(angles),
                "velocity": speeds,
                "effort": [],
            },
        )
        angles = [a + s * 0.02 for a, s in zip(angles, speeds, strict=False)]

    # /camera/image_raw: 5 Hz rgb8 64x48, latency 30 ms
    pixels = bytes((i * 7) % 256 for i in range(64 * 48 * 3))
    for k in range(100):
        t = k * 200 * MS
        add(
            "/camera/image_raw",
            "sensor_msgs/msg/Image",
            t + 30 * MS,
            {
                "header": header(T0 + t, "camera_color_optical_frame"),
                "height": 48,
                "width": 64,
                "encoding": "rgb8",
                "is_bigendian": 0,
                "step": 192,
                "data": pixels,
            },
        )

    records.sort(key=lambda r: r.log_time)
    return records
