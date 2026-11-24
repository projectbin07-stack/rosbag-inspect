"""Command-line interface, JSON/CSV export and the HTML report."""

from __future__ import annotations

import csv
import json
import re
import xml.etree.ElementTree as ET
from html.parser import HTMLParser
from pathlib import Path
from typing import Any

import pytest

from rosbag_inspect.analysis import analyze
from rosbag_inspect.bag import open_bag
from rosbag_inspect.cdr import SkippedBytes, decode
from rosbag_inspect.cli import EXIT_ERROR, EXIT_OK, EXIT_WARN, main
from rosbag_inspect.export import array_cell, flat_columns, flatten, number_text, to_jsonable
from rosbag_inspect.html_report import render_html
from rosbag_inspect.msgdef import Registry

VOID = {"meta", "br", "hr", "img", "input", "link", "source", "wbr", "col", "area", "base"}


def test_info_text_and_json(fixtures: dict[str, Path], capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["info", str(fixtures["sqlite"])]) == EXIT_OK
    out = capsys.readouterr().out
    assert "sqlite3 (1 file(s))" in out
    assert "schema version 3, ros_distro humble" in out
    assert re.search(r"/scan\s+181\s+9\.\d\d\s+sensor_msgs/msg/LaserScan", out)
    assert main(["info", str(fixtures["mcap"]), "--json"]) == EXIT_OK
    data = json.loads(capsys.readouterr().out)
    topics = {t["name"]: t for t in data["topics"]}
    assert topics["/imu"]["message_count"] == 2000
    assert topics["/imu"]["definition"] == "mcap schema"
    assert data["message_count"] == 5482


def test_check_exit_codes_and_json(
    fixtures: dict[str, Path], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    report = tmp_path / "report.html"
    code = main(["check", str(fixtures["lz4"]), "--report", str(report)])
    out = capsys.readouterr().out
    assert code == EXIT_ERROR
    assert "verdict: FAIL (4 error(s), 3 warning(s), 2 note(s))" in out
    assert "FAIL  /imu: Gravity magnitude is wrong [imu_gravity]" in out
    assert report.is_file()

    assert main(["check", str(fixtures["mcap"]), "--json", "--expect", "/scan:10"]) == EXIT_ERROR
    data = json.loads(capsys.readouterr().out)
    assert data["verdict"] == "fail"
    assert data["summary"] == {"error": 4, "warning": 3, "info": 2}
    scan = next(t for t in data["topics"] if t["name"] == "/scan")
    assert scan["expected_hz"] == 10


def test_warning_only_bag_and_strict(
    fixtures: dict[str, Path], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    # /scan and /cmd_vel only: the LiDAR gap is the one warning, everything else is a note.
    from bagwriter import write_mcap
    from drive import build_drive

    records = [r for r in build_drive() if r.topic in ("/scan", "/cmd_vel")]
    path = tmp_path / "warn.mcap"
    write_mcap(path, records)
    assert main(["check", str(path)]) == EXIT_OK
    assert main(["check", str(path), "--strict"]) == EXIT_WARN
    out = capsys.readouterr().out
    assert "verdict: WARN" in out
    # With a 3 s minimum gap the 2 s hole is no longer a gap; it then shows up as jitter.
    assert main(["check", str(path), "--strict", "--gap-factor", "30", "--min-gap", "3"]) == 1
    assert "[high_jitter]" in capsys.readouterr().out
    assert main(["check", str(path), "--expect", "/scan:20"]) == EXIT_ERROR
    capsys.readouterr()


def test_echo_and_export(
    fixtures: dict[str, Path], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main(["echo", str(fixtures["sqlite"]), "/camera/image_raw", "--limit", "2"]) == 0
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 2
    first = json.loads(lines[0])
    assert first["message"]["data"] == {"skipped_bytes": 9216}
    assert first["message"]["encoding"] == "rgb8"
    assert main(["echo", str(fixtures["mcap"]), "/scan", "--limit", "1"]) == 0
    scan = json.loads(capsys.readouterr().out)["message"]
    assert scan["ranges"][:10] == ["Infinity"] * 10
    assert (
        main(["echo", str(fixtures["mcap"]), "/scan", "--limit", "1", "--nonfinite", "null"]) == 0
    )
    scan_null = json.loads(capsys.readouterr().out)["message"]
    assert scan_null["ranges"][:10] == [None] * 10
    assert scan_null["ranges"][10:] == scan["ranges"][10:]
    assert scan_null["range_max"] == scan["range_max"]

    out = tmp_path / "odom.csv"
    assert main(["export", str(fixtures["mcap"]), "/odom", str(out)]) == 0
    capsys.readouterr()
    with out.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 1000
    assert rows[0]["log_time_ns"] == str(1_700_000_000_005_000_000)
    assert rows[0]["stamp_ns"] == str(1_700_000_000_000_000_000)
    assert "pose.covariance.35" in rows[0]
    assert "child_frame_id" not in rows[0]
    assert float(rows[700]["pose.pose.position.x"]) > 0.8  # after the teleport

    assert main(["export", str(fixtures["sqlite"]), "/joint_states", "-"]) == 0
    text = capsys.readouterr().out.splitlines()
    assert text[0].startswith("log_time_ns,stamp_ns,header.stamp.sec,header.stamp.nanosec,")
    assert "name.length" in text[0]


def test_export_arrays_keeps_laserscan_ranges(
    fixtures: dict[str, Path], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    out = tmp_path / "scan.csv"
    assert main(["export", str(fixtures["mcap"]), "/scan", str(out)]) == 0
    with out.open(encoding="utf-8") as f:
        plain = next(csv.DictReader(f))
    assert "ranges" not in plain
    assert main(["export", str(fixtures["mcap"]), "/scan", str(out), "--arrays"]) == 0
    assert f"column(s) to {out}" in capsys.readouterr().err
    with out.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    first = rows[0]
    # Every element is kept, in order, and reads back as the decoded float32 value.
    cells = first["ranges"].split(" ")
    ranges = [float(v) for v in cells]
    assert len(ranges) == int(first["ranges.length"])
    assert cells[:10] == ["inf"] * 10
    with open_bag(fixtures["mcap"]) as bag:
        raw = next(bag.messages(["/scan"]))
        decoded = decode(bag.registry, "sensor_msgs/msg/LaserScan", raw.data)
    assert ranges == decoded["ranges"]
    assert first["intensities.length"] == str(len(decoded["intensities"]))
    # The plain export's columns are all still there, in the same order.
    assert [c for c in first if c not in ("ranges", "intensities")] == list(plain)

    assert main(["export", str(fixtures["sqlite"]), "/joint_states", "-", "--arrays"]) == 0
    head = capsys.readouterr().out.splitlines()[0].split(",")
    assert head[head.index("position.length") + 1] == "position"
    assert "name" not in head  # strings stay out


def test_cli_errors(
    fixtures: dict[str, Path], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main(["echo", str(fixtures["mcap"]), "/nope"]) == EXIT_ERROR
    assert "topic /nope not in bag" in capsys.readouterr().err
    assert main(["export", str(fixtures["mcap"]), "/nope", "x.csv"]) == EXIT_ERROR
    assert main(["info", str(tmp_path / "missing")]) == EXIT_ERROR
    assert "no such file or directory" in capsys.readouterr().err
    with pytest.raises(SystemExit):
        main(["check", str(fixtures["mcap"]), "--expect", "scan"])
    with pytest.raises(SystemExit):
        main(["check", str(fixtures["mcap"]), "--expect", "/scan:-1"])
    with pytest.raises(SystemExit):
        main(["check", str(fixtures["mcap"]), "--gap-factor", "abc"])
    capsys.readouterr()


def test_echo_undecodable_topic(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    from bagwriter import write_mcap
    from rosbag_inspect.cdr import encode

    reg = Registry()
    data = encode(reg, "std_msgs/msg/String", {"data": "x"})
    from bagwriter import Record

    path = tmp_path / "noschema.mcap"
    write_mcap(path, [Record("/x", "std_msgs/msg/String", 1, data)], schemas=False)
    assert main(["echo", str(path), "/x"]) == EXIT_ERROR
    assert main(["export", str(path), "/x", "-"]) == EXIT_ERROR
    assert "cannot decode" in capsys.readouterr().err


def test_export_helpers() -> None:
    reg = Registry()
    assert flat_columns(reg, "geometry_msgs/msg/Twist") == [
        "linear.x",
        "linear.y",
        "linear.z",
        "angular.x",
        "angular.y",
        "angular.z",
    ]
    assert to_jsonable({"a": [float("nan"), float("-inf")], "b": SkippedBytes(3)}) == {
        "a": ["NaN", "-Infinity"],
        "b": {"skipped_bytes": 3},
    }
    assert to_jsonable(b"\x01\x02") == [1, 2]
    assert to_jsonable(bytes(100)) == {"bytes": 100}
    assert to_jsonable([float("nan"), float("inf"), 1.5], nonfinite="null") == [None, None, 1.5]
    with pytest.raises(ValueError, match="nonfinite"):
        to_jsonable(1.0, nonfinite="zero")
    assert "ranges" in flat_columns(reg, "sensor_msgs/msg/LaserScan", arrays=True)
    assert "header.frame_id" not in flat_columns(reg, "sensor_msgs/msg/LaserScan", arrays=True)
    assert array_cell([1.0, float("inf"), float("-inf"), 0.1, True]) == "1.0 inf -inf 0.1 1"
    assert array_cell(SkippedBytes(5000)) == ""
    assert array_cell(b"\x00\x07") == "0 7"
    assert number_text(float("nan")) == "nan"
    assert number_text(3) == "3"
    msg: dict[str, Any] = {"header": {"stamp": {"sec": 1, "nanosec": 2}, "frame_id": "f"}}
    msg |= {"name": ["a"], "position": [0.5, float("nan")], "velocity": [], "effort": []}
    row = flatten(reg, "sensor_msgs/msg/JointState", msg, arrays=True)
    assert row["position.length"] == 2
    assert row["position"] == "0.5 nan"
    assert row["velocity"] == ""
    assert "name" not in row


# -- HTML ----------------------------------------------------------------------------------------


class _Checker(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.stack: list[str] = []
        self.errors: list[str] = []
        self.tags: set[str] = set()

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self.tags.add(tag)
        for name, value in attrs:
            if name in ("src", "href") and value and not value.startswith("#"):
                self.errors.append(f"external reference {value}")
        if tag not in VOID:
            self.stack.append(tag)

    def handle_startendtag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self.tags.add(tag)

    def handle_endtag(self, tag: str) -> None:
        if not self.stack or self.stack[-1] != tag:
            self.errors.append(f"unexpected </{tag}> (open: {self.stack[-3:]})")
        else:
            self.stack.pop()


def test_html_report_is_well_formed(fixtures: dict[str, Path]) -> None:
    from rosbag_inspect.options import Options

    with open_bag(fixtures["sqlite"]) as bag:
        report = analyze(bag, Options(expectations={"/scan": 10}))
    page = render_html(report)
    checker = _Checker()
    checker.feed(page)
    checker.close()
    assert checker.errors == []
    assert checker.stack == []
    assert "script" not in checker.tags
    assert "http://" not in page.replace("http://www.w3.org/2000/svg", "")
    assert "https://" not in page
    svgs = re.findall(r"<svg .*?</svg>", page, flags=re.S)
    # TF tree + a rate timeline per topic with more than one message (7; /tf_static has one)
    # + latency histograms for the five topics with header stamps (/tf, /tf_static have none)
    assert len(svgs) == 1 + 7 + 5
    assert "A single message (latched or published once)" in page
    for svg in svgs:
        ET.fromstring(svg.replace("<svg ", '<svg xmlns="http://www.w3.org/2000/svg" ', 1))
    assert page.count('class="gap"') == 1
    assert 'class="expected"' in page
    assert "prefers-color-scheme: dark" in page
    assert "base_laser -&gt; laser (static" in page
    assert 'class="node bad"' in page  # the disconnected laser frame
    assert "✗ FAIL" in page


def test_html_escapes_untrusted_names(tmp_path: Path) -> None:
    from bagwriter import Record, write_mcap
    from rosbag_inspect.cdr import encode

    reg = Registry()
    msg = {"header": {"stamp": {"sec": 1, "nanosec": 0}, "frame_id": "<b>x</b>"}}
    data = encode(reg, "geometry_msgs/msg/TwistStamped", msg)
    path = tmp_path / "evil.mcap"
    write_mcap(path, [Record("/a<script>", "geometry_msgs/msg/TwistStamped", 2 * 10**9, data)])
    with open_bag(path) as bag:
        page = render_html(analyze(bag))
    assert "<script>" not in page
    assert "&lt;script&gt;" in page
    assert "&lt;b&gt;x&lt;/b&gt;" in page
    checker = _Checker()
    checker.feed(page)
    assert checker.errors == []
