"""CDR encoding/decoding: round trips for every built-in type and hand-built byte layouts."""

from __future__ import annotations

import math
import struct
from typing import Any

import pytest

from rosbag_inspect.cdr import CdrError, SkippedBytes, decode, default_value, encapsulation, encode
from rosbag_inspect.msgdef import BUILTIN_DEFINITIONS, FieldType, Registry

REG = Registry()


def _header(sec: int = 5, nanosec: int = 7, frame: str = "base_link") -> dict[str, Any]:
    return {"stamp": {"sec": sec, "nanosec": nanosec}, "frame_id": frame}


SAMPLES: dict[str, dict[str, Any]] = {
    "sensor_msgs/msg/LaserScan": {
        "header": _header(frame="laser"),
        "angle_min": -1.5,
        "angle_max": 1.5,
        "angle_increment": 0.5,
        "time_increment": 0.25,
        "scan_time": 0.125,
        "range_min": 0.25,
        "range_max": 8.0,
        "ranges": [1.0, 2.5, math.inf, 0.5, 3.0, 4.0, 7.75],
        "intensities": [10.0, 20.0, 0.0, 1.0, 2.0, 3.0, 4.0],
    },
    "sensor_msgs/msg/Imu": {
        "header": _header(frame="imu"),
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.5, "w": 0.75},
        "orientation_covariance": [-1.0, 0, 0, 0, 0, 0, 0, 0, 0],
        "angular_velocity": {"x": 0.125, "y": -0.25, "z": 1.5},
        "angular_velocity_covariance": [0.5] * 9,
        "linear_acceleration": {"x": 0.0, "y": 0.25, "z": 9.75},
        "linear_acceleration_covariance": [float(i) for i in range(9)],
    },
    "sensor_msgs/msg/JointState": {
        "header": _header(frame=""),
        "name": ["left", "right", "z"],
        "position": [0.5, -0.5, 2.0],
        "velocity": [],
        "effort": [1.0, 2.0, 3.0],
    },
    "sensor_msgs/msg/Image": {
        "header": _header(frame="cam"),
        "height": 2,
        "width": 3,
        "encoding": "mono8",
        "is_bigendian": 0,
        "step": 3,
        "data": b"\x01\x02\x03\x04\x05\x06",
    },
    "sensor_msgs/msg/CompressedImage": {
        "header": _header(frame="cam"),
        "format": "jpeg",
        "data": b"\xff\xd8\xff",
    },
    "nav_msgs/msg/Odometry": {
        "header": _header(frame="odom"),
        "child_frame_id": "base_link",
        "pose": {
            "pose": {
                "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "covariance": [float(i) for i in range(36)],
        },
        "twist": {
            "twist": {
                "linear": {"x": 0.5, "y": 0.25, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": -0.125},
            },
            "covariance": [0.0] * 36,
        },
    },
    "geometry_msgs/msg/Twist": {
        "linear": {"x": 0.3, "y": -0.1, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.7},
    },
    "geometry_msgs/msg/TwistStamped": {
        "header": _header(),
        "twist": {"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0, "y": 0, "z": 0}},
    },
    "tf2_msgs/msg/TFMessage": {
        "transforms": [
            {
                "header": _header(frame="odom"),
                "child_frame_id": "base_link",
                "transform": {
                    "translation": {"x": 1.0, "y": 2.0, "z": 3.0},
                    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            {
                "header": _header(frame="base_link"),
                "child_frame_id": "laser",
                "transform": {
                    "translation": {"x": 0.1, "y": 0.0, "z": 0.2},
                    "rotation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0},
                },
            },
        ]
    },
    "std_msgs/msg/Header": _header(),
    "builtin_interfaces/msg/Duration": {"sec": -3, "nanosec": 999_999_999},
    "std_msgs/msg/String": {"data": "héllo"},
    "geometry_msgs/msg/PoseStamped": {
        "header": _header(),
        "pose": {
            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    },
}


def _normalize(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: _normalize(v) for k, v in value.items()}
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, list):
        return [_normalize(v) for v in value]
    return value


def test_every_builtin_type_has_a_sample_or_is_a_dependency() -> None:
    covered = set(SAMPLES)
    for name in SAMPLES:
        stack = [name]
        while stack:
            current = stack.pop()
            for fld in REG.get(current).fields:
                if not fld.type.is_primitive and fld.type.base not in covered:
                    covered.add(fld.type.base)
                    stack.append(fld.type.base)
    assert set(BUILTIN_DEFINITIONS) <= covered


@pytest.mark.parametrize("type_name", sorted(SAMPLES))
@pytest.mark.parametrize(("little", "xcdr2"), [(True, False), (False, False), (True, True)])
def test_round_trip(type_name: str, little: bool, xcdr2: bool) -> None:
    value = SAMPLES[type_name]
    data = encode(REG, type_name, value, little_endian=little, xcdr2=xcdr2)
    decoded = decode(REG, type_name, data, max_bytes=None)
    assert _normalize(decoded) == _normalize(value)


def test_float32_values_round_trip_through_single_precision() -> None:
    data = encode(REG, "sensor_msgs/msg/LaserScan", {"ranges": [0.1]})
    (value,) = decode(REG, "sensor_msgs/msg/LaserScan", data)["ranges"]
    assert value == struct.unpack("<f", struct.pack("<f", 0.1))[0]


def test_header_layout_by_hand() -> None:
    # 4-byte encapsulation (CDR_LE), int32 sec, uint32 nanosec, uint32 length incl. NUL, bytes
    data = encode(REG, "std_msgs/msg/Header", _header(1, 2, "ab"))
    assert data == bytes.fromhex("00010000") + struct.pack("<iiI", 1, 2, 3) + b"ab\0"


def test_alignment_of_float64_after_string() -> None:
    # Odometry: header (8 bytes stamp + 4 + "odom\0" = 17 -> offset 17), child_frame_id
    # length at offset 20 (3 bytes padding), "b\0" ends at 26, first float64 aligned to 32.
    value = {"header": _header(0, 0, "odom"), "child_frame_id": "b"}
    data = encode(REG, "nav_msgs/msg/Odometry", value)
    body = data[4:]
    assert body[17:20] == b"\0\0\0"
    assert struct.unpack_from("<I", body, 20)[0] == 2
    assert body[24:26] == b"b\0"
    assert body[26:32] == b"\0" * 6
    assert len(body) == 32 + 8 * 7 + 8 * 36 + 8 * 6 + 8 * 36


def test_xcdr2_aligns_eight_byte_values_to_four() -> None:
    value = {"header": _header(0, 0, "odom"), "child_frame_id": "b"}
    body = encode(REG, "nav_msgs/msg/Odometry", value, xcdr2=True)[4:]
    # "b\0" ends at 26 -> next float64 at 28 instead of 32
    assert len(body) == 28 + 8 * 7 + 8 * 36 + 8 * 6 + 8 * 36
    assert encapsulation(bytes([0, 7, 0, 0])) == ("<", 4, "CDR2_LE")


def test_empty_sequence_is_not_padded() -> None:
    reg = Registry(include_builtins=False)
    reg.add("t/msg/A", "uint8 a\nfloat64[] values\nuint8 b\n")
    data = encode(reg, "t/msg/A", {"a": 1, "values": [], "b": 2})
    # a at 0, pad to 4, count 0 at 4..8, b at 8 (no padding to 16 for the empty float64[])
    assert data[4:] == bytes([1, 0, 0, 0]) + struct.pack("<I", 0) + bytes([2])
    assert decode(reg, "t/msg/A", data) == {"a": 1, "values": [], "b": 2}


def test_big_endian_by_hand() -> None:
    reg = Registry(include_builtins=False)
    reg.add("t/msg/B", "uint8 a\nint16 b\nfloat64 c\nbool d\n")
    raw = bytes([0, 0, 0, 0]) + bytes([9, 0]) + struct.pack(">h", -2)
    raw += b"\0" * 4 + struct.pack(">d", 1.5) + bytes([1])
    assert decode(reg, "t/msg/B", raw) == {"a": 9, "b": -2, "c": 1.5, "d": True}
    assert (
        encode(reg, "t/msg/B", {"a": 9, "b": -2, "c": 1.5, "d": True}, little_endian=False) == raw
    )


def test_fixed_bounded_and_nested_arrays() -> None:
    reg = Registry()
    reg.add(
        "t/msg/C",
        "int16[3] fixed\nstring<=4 short\nstring<=2[<=2] names\n"
        "geometry_msgs/Point[2] points\nuint64[<=3] big\nchar c\nbyte[2] raw\n",
    )
    value = {
        "fixed": [1, -2, 3],
        "short": "abcd",
        "names": ["x", "yz"],
        "points": [{"x": 1.0, "y": 2.0, "z": 3.0}, {"x": 4.0, "y": 5.0, "z": 6.0}],
        "big": [2**63, 1],
        "c": 65,
        "raw": b"\x01\x02",
    }
    data = encode(reg, "t/msg/C", value)
    assert _normalize(decode(reg, "t/msg/C", data)) == _normalize(value)


def test_bounds_are_enforced_when_decoding() -> None:
    reg = Registry(include_builtins=False)
    reg.add("t/msg/D", "string s\nuint8[] v\n")
    reg.add("t/msg/D2", "string<=2 s\nuint8[<=1] v\n")
    data = encode(reg, "t/msg/D", {"s": "abc", "v": []})
    with pytest.raises(CdrError, match="exceeds bound 2"):
        decode(reg, "t/msg/D2", data)
    data = encode(reg, "t/msg/D", {"s": "", "v": [1, 2]})
    with pytest.raises(CdrError, match="exceeds bound 1"):
        decode(reg, "t/msg/D2", data)


def test_truncated_and_corrupt_payloads() -> None:
    data = encode(REG, "sensor_msgs/msg/Imu", SAMPLES["sensor_msgs/msg/Imu"])
    with pytest.raises(CdrError, match="truncated payload"):
        decode(REG, "sensor_msgs/msg/Imu", data[:-3])
    with pytest.raises(CdrError, match="shorter than the encapsulation header"):
        decode(REG, "sensor_msgs/msg/Imu", b"\0\1")
    # stamp (8 bytes), frame_id "" (length 1 + NUL), padding to 16, then a huge name count
    bogus = bytes([0, 1, 0, 0]) + b"\0" * 8 + struct.pack("<I", 1) + b"\0" * 4
    bogus += struct.pack("<I", 0xFFFFFFF0)
    with pytest.raises(CdrError, match="implausible sequence length"):
        decode(REG, "sensor_msgs/msg/JointState", bogus)


@pytest.mark.parametrize(
    ("ident", "name"), [(0x0002, "PL_CDR_BE"), (0x0009, "D_CDR2_LE"), (0x0042, "0x0042")]
)
def test_unsupported_encapsulations(ident: int, name: str) -> None:
    with pytest.raises(CdrError, match=name):
        decode(REG, "std_msgs/msg/String", bytes([ident >> 8, ident & 0xFF, 0, 0]) + b"\0" * 8)


def test_large_byte_arrays_are_skipped_not_copied() -> None:
    image = dict(SAMPLES["sensor_msgs/msg/Image"], data=bytes(10_000), height=100, step=100)
    data = encode(REG, "sensor_msgs/msg/Image", image)
    decoded = decode(REG, "sensor_msgs/msg/Image", data)
    assert decoded["data"] == SkippedBytes(10_000, True)
    # only a prefix loaded (as for SQLite image topics): length known, payload absent
    prefix = decode(REG, "sensor_msgs/msg/Image", data[:200])
    assert prefix["data"] == SkippedBytes(10_000, False)
    assert prefix["encoding"] == "mono8"


def test_wstring_is_rejected_and_unknown_types_reported() -> None:
    reg = Registry(include_builtins=False)
    reg.add("t/msg/W", "wstring w\n")
    with pytest.raises(CdrError, match="wstring"):
        decode(reg, "t/msg/W", bytes([0, 1, 0, 0]) + struct.pack("<I", 1) + b"\0\0")
    with pytest.raises(CdrError, match="wstring"):
        encode(reg, "t/msg/W", {"w": "x"})
    reg.add("t/msg/U", "other/Thing t\n")
    with pytest.raises(CdrError, match="no definition for other/msg/Thing"):
        decode(reg, "t/msg/U", bytes([0, 1, 0, 0]))


def test_defaults_and_fixed_array_length_check() -> None:
    assert default_value(REG, FieldType("float64", "fixed", 2)) == [0.0, 0.0]
    assert default_value(REG, FieldType("bool")) is False
    assert default_value(REG, FieldType("string")) == ""
    data = encode(REG, "geometry_msgs/msg/PoseWithCovariance", {})
    assert decode(REG, "geometry_msgs/msg/PoseWithCovariance", data)["covariance"] == [0.0] * 36
    with pytest.raises(CdrError, match="needs 36 elements"):
        encode(REG, "geometry_msgs/msg/PoseWithCovariance", {"covariance": [1.0]})


def test_decode_with_size_reports_consumed_bytes() -> None:
    from rosbag_inspect.cdr import decode_with_size

    data = encode(REG, "std_msgs/msg/String", {"data": "abc"})
    message, size = decode_with_size(REG, "std_msgs/msg/String", data + b"\0\0\0")
    assert message == {"data": "abc"}
    assert size == len(data) == 4 + 4 + 4
