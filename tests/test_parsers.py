"""Tests for the .msg parser, the YAML subset, metadata.yaml and the LZ4 decoder."""

from __future__ import annotations

import os
import random

import pytest

from bagwriter import lz4_compress_block, lz4_compress_frame, ros2msg_bundle
from rosbag_inspect.lz4 import LZ4Error, decompress_block, decompress_frame
from rosbag_inspect.metadata import MetadataError, parse_metadata
from rosbag_inspect.msgdef import (
    EMPTY_PLACEHOLDER,
    SEPARATOR,
    FieldType,
    MsgDefError,
    Registry,
    normalize_type,
    parse_msg,
    split_bundle,
)
from rosbag_inspect.yamlmini import YamlError, loads

# -- .msg ----------------------------------------------------------------------------------------


def test_parse_msg_fields_constants_defaults_and_comments() -> None:
    text = (
        "# a comment\n"
        "uint8 MODE_A=1   # constant\n"
        "string NAME = 'x # not a comment'\n"
        "Header header\n"
        "float64[9] cov  # fixed\n"
        "int32[] values [1, 2]\n"
        'string<=8 short "de#fault"\n'
        "string<=4[<=3] names\n"
        "geometry_msgs/Point p\n"
        "Other local\n"
        "time t\n"
    )
    d = parse_msg("pkg/Thing", text)
    assert d.name == "pkg/msg/Thing"
    assert d.constants == {"MODE_A": "1", "NAME": "'x # not a comment'"}
    assert [(f.name, str(f.type)) for f in d.fields] == [
        ("header", "std_msgs/msg/Header"),
        ("cov", "float64[9]"),
        ("values", "int32[]"),
        ("short", "string<=8"),
        ("names", "string<=4[<=3]"),
        ("p", "geometry_msgs/msg/Point"),
        ("local", "pkg/msg/Other"),
        ("t", "builtin_interfaces/msg/Time"),
    ]
    assert d.fields[4].type == FieldType("string", "sequence", 3, 4)


def test_empty_message_gets_the_rosidl_placeholder() -> None:
    d = parse_msg("test_msgs/msg/Constants", "int32 A=1\nint32 B=2\n")
    assert [(f.name, f.type.base) for f in d.fields] == [(EMPTY_PLACEHOLDER, "uint8")]


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("float64", "expected '<type> <name>'"),
        ("int32 9bad", "bad field name"),
        ("int32[0] x", "size 0"),
        ("int32[<=] x", "bounded array without bound"),
        ("int32<=3 x", "bad bounded type"),
        ("a/b/c/d x", "bad type name"),
        ("int32[x] y", "bad field type"),
    ],
)
def test_parse_msg_errors(text: str, message: str) -> None:
    with pytest.raises(MsgDefError, match=message):
        parse_msg("p/msg/T", text)


def test_normalize_type() -> None:
    assert normalize_type("std_msgs/String") == "std_msgs/msg/String"
    assert normalize_type("std_msgs/msg/String") == "std_msgs/msg/String"
    assert normalize_type("Header") == "std_msgs/msg/Header"
    with pytest.raises(MsgDefError, match="no package"):
        normalize_type("Point")


def test_bundle_round_trip_and_registry() -> None:
    bundle = ros2msg_bundle("nav_msgs/msg/Odometry")
    parts = split_bundle("nav_msgs/msg/Odometry", bundle)
    names = [name for name, _ in parts]
    assert names[0] == "nav_msgs/msg/Odometry"
    assert "MSG: std_msgs/Header" in bundle
    assert set(names[1:]) >= {"std_msgs/Header", "geometry_msgs/PoseWithCovariance"}
    reg = Registry(include_builtins=False)
    reg.add_bundle("nav_msgs/msg/Odometry", bundle)
    assert reg.missing_dependencies("nav_msgs/msg/Odometry") == []
    assert "nav_msgs/Odometry" in reg
    assert 3 not in reg
    assert "bad/x/y/z" not in reg
    assert "geometry_msgs/msg/Twist" in reg.names()


def test_bundle_errors_and_missing_dependencies() -> None:
    with pytest.raises(MsgDefError, match="IDL"):
        split_bundle("a/msg/B", f"int32 x\n{SEPARATOR}\nIDL: a/msg/C\n")
    with pytest.raises(MsgDefError, match="separator not followed"):
        split_bundle("a/msg/B", f"int32 x\n{SEPARATOR}\nint32 y\n")
    reg = Registry(include_builtins=False)
    reg.add("a/msg/B", "c/D d\nE e\n")
    assert reg.missing_dependencies("a/msg/B") == ["a/msg/E", "c/msg/D"]
    with pytest.raises(MsgDefError, match="no definition"):
        reg.get("c/msg/D")


# -- YAML ----------------------------------------------------------------------------------------


def test_yaml_subset() -> None:
    text = (
        "# leading comment\n"
        "---\n"
        "a: 1\n"
        "b: -2.5e3\n"
        "c: 'it''s'   # trailing comment\n"
        'd: "tab\\tquote\\" \\u00e9 \\x41"\n'
        "e: ~\n"
        "f: [1, 'two', \"3\"]\n"
        "g: {x: 1, y: str}\n"
        "h:\n"
        "  - plain # c\n"
        "  - k: v\n"
        "    k2: [ ]\n"
        "  -\n"
        "    nested: true\n"
        "  - - inner\n"
        "i: |\n"
        "  line1\n"
        "  line2\n"
        "j: >-\n"
        "  folded\n"
        "  text\n"
        'k: "multi\n'
        "  line\n"
        "\n"
        '  string"\n'
        "l: 0x1F\n"
        "m: .inf\n"
        "n: False\n"
        "list:\n"
        "- top-level-indent item\n"
        "o: http://example.com/a#b\n"
        "...\n"
        "ignored: after end\n"
    )
    doc = loads(text)
    assert doc == {
        "a": 1,
        "b": -2500.0,
        "c": "it's",
        "d": 'tab\tquote" \u00e9 A',
        "e": None,
        "f": [1, "two", "3"],
        "g": {"x": 1, "y": "str"},
        "h": ["plain", {"k": "v", "k2": []}, {"nested": True}, ["inner"]],
        "i": "line1\nline2\n",
        "j": "folded text",
        "k": "multi line\nstring",
        "l": 31,
        "m": float("inf"),
        "n": False,
        "list": ["top-level-indent item"],
        "o": "http://example.com/a#b",
    }
    assert loads("") is None
    assert loads("just a scalar") == "just a scalar"


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("a: &anchor 1\n", "anchors"),
        ("&x a: 1\n", "anchors"),
        ("a: 1\na: 2\n", "duplicate key"),
        ("a:\n\tb: 1\n", "tab"),
        ('a: "open\n', "unterminated"),
        ("a: [1, [2]]\n", "nested flow"),
        ("a: [1,\n  2]\n", "multi-line flow"),
        ("a: {x: 1\n", "multi-line flow"),
        ("a: {x}\n", "without ':'"),
        ('a: "\\q"\n', "unknown escape"),
        ('a: "\\u12"\n', "short"),
        ("a:\n  b: 1\n c: 2\n", "unexpected content|bad indentation"),
        ("a: |x\n  t\n", "block scalar header"),
    ],
)
def test_yaml_errors(text: str, message: str) -> None:
    with pytest.raises(YamlError, match=message):
        loads(text)


# -- metadata.yaml -------------------------------------------------------------------------------

METADATA = """\
rosbag2_bagfile_information:
  version: 9
  storage_identifier: mcap
  duration:
    nanoseconds: 151137181
  starting_time:
    nanoseconds_since_epoch: 1586406456763032325
  message_count: 7
  topics_with_message_count:
    - topic_metadata:
        name: /array_topic
        type: test_msgs/msg/Arrays
        serialization_format: cdr
        offered_qos_profiles:
          - history: keep_last
            depth: 4
        type_description_hash: RIHS01_abc
      message_count: 4
  compression_format: zstd
  compression_mode: message
  relative_file_paths:
    - cdr_test_0.mcap
  custom_data: ~
  ros_distro: jazzy
"""


def test_metadata_newer_version() -> None:
    meta = parse_metadata(METADATA)
    assert meta.version == 9
    assert meta.storage_identifier == "mcap"
    assert meta.duration_ns == 151137181
    assert meta.starting_time_ns == 1586406456763032325
    assert meta.topics[0].name == "/array_topic"
    assert meta.topics[0].message_count == 4
    assert "keep_last" in meta.topics[0].offered_qos_profiles
    assert (meta.compression_format, meta.compression_mode) == ("zstd", "message")
    assert meta.relative_file_paths == ["cdr_test_0.mcap"]
    assert meta.ros_distro == "jazzy"


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("other: 1\n", "missing key 'rosbag2_bagfile_information'"),
        ("rosbag2_bagfile_information: 3\n", "expected a mapping"),
        ("rosbag2_bagfile_information:\n  duration: 1\n", "duration: expected a mapping"),
        (
            "rosbag2_bagfile_information:\n  duration:\n    nanoseconds: x\n",
            "expected an integer",
        ),
        ("rosbag2_bagfile_information: &a\n", "metadata.yaml: line 1"),
    ],
)
def test_metadata_errors(text: str, message: str) -> None:
    with pytest.raises(MetadataError, match=message):
        parse_metadata(text)


# -- LZ4 -----------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "data",
    [
        b"",
        b"a",
        b"abcabcabcabcabcabcabcabcabcabcabcabc" * 20,
        bytes(70_000),
        os.urandom(5000),
        bytes(random.Random(1).choice(b"ab") for _ in range(100_000)),
    ],
)
def test_lz4_round_trip(data: bytes) -> None:
    frame = lz4_compress_frame(data)
    assert decompress_frame(frame, len(data)) == data
    assert decompress_frame(lz4_compress_frame(data, raw_blocks=True)) == data


def test_lz4_block_by_hand() -> None:
    # token 0x15: 1 literal 'a', match length 5+4=9 at offset 1 (overlapping run), then the
    # final literals-only sequence "bcdef"
    block = bytes([0x15]) + b"a" + bytes([1, 0]) + bytes([0x50]) + b"bcdef"
    out = bytearray()
    decompress_block(block, out)
    assert bytes(out) == b"a" * 10 + b"bcdef"
    long = b"x" * 300
    assert decompress_frame(lz4_compress_frame(long)) == long
    assert len(lz4_compress_block(long)) < 20


@pytest.mark.parametrize(
    ("frame", "message"),
    [
        (b"\x00\x00\x00\x00", "bad LZ4 frame magic"),
        (b"\x04\x22\x4d\x18\x00\x40\x00", "unsupported LZ4 frame version"),
        (b"\x04\x22\x4d\x18\x61\x40\x00", "dictionary"),
        (b"\x04\x22\x4d\x18\x60\x40\x00\x05\x00\x00\x00ab", "truncated block"),
        (b"\x04\x22\x4d\x18\x60\x40\x00\x03\x00\x00\x00\x10\x01\x00\x00\x00\x00\x00", "offset"),
        (b"\x04\x22", "truncated frame magic"),
        (b"\x04\x22\x4d\x18\x60", "truncated frame descriptor"),
        (b"\x04\x22\x4d\x18\x60\x40\x00\x01", "truncated block size"),
    ],
)
def test_lz4_errors(frame: bytes, message: str) -> None:
    with pytest.raises(LZ4Error, match=message):
        decompress_frame(frame)


def test_lz4_size_mismatch_and_skippable_frames() -> None:
    frame = lz4_compress_frame(b"hello world, hello world, hello world")
    with pytest.raises(LZ4Error, match="expected 100"):
        decompress_frame(frame, 100)
    with pytest.raises(LZ4Error, match="exceeds the declared size"):
        decompress_frame(frame, 5)
    skippable = b"\x50\x2a\x4d\x18" + (3).to_bytes(4, "little") + b"xyz"
    assert decompress_frame(skippable + frame) == b"hello world, hello world, hello world"
    for broken in (bytes([0xF0]), bytes([0x10]) + b"a" + b"\x01", bytes([0x1F, 0x41, 1, 0])):
        with pytest.raises(LZ4Error):
            decompress_block(broken, bytearray())
