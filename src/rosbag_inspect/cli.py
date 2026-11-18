"""Command-line interface: ``rosbag-inspect info | check | echo | export``."""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import TextIO

from . import __version__
from .analysis import analyze, stamp_ns
from .bag import Bag, BagError, open_bag
from .cdr import CdrError, decode
from .export import flat_columns, flatten, to_jsonable
from .findings import ERROR, INFO, WARNING, Report
from .html_report import render_html
from .msgdef import MsgDefError
from .options import Options

EXIT_OK = 0
EXIT_WARN = 1
EXIT_ERROR = 2

_MARK = {ERROR: "FAIL", WARNING: "WARN", INFO: "INFO"}


def _expectation(text: str) -> tuple[str, float]:
    topic, sep, rate = text.rpartition(":")
    if not sep or not topic:
        raise argparse.ArgumentTypeError(f"expected TOPIC:HZ, got {text!r}")
    try:
        value = float(rate)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"rate is not a number in {text!r}") from exc
    if value <= 0:
        raise argparse.ArgumentTypeError(f"rate must be positive in {text!r}")
    return topic, value


def _positive(text: str) -> float:
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}") from exc
    if value <= 0:
        raise argparse.ArgumentTypeError(f"must be positive: {text!r}")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rosbag-inspect",
        description="Health-check ROS 2 bags (.db3 / .mcap) without a ROS installation.",
    )
    parser.add_argument("--version", action="version", version=f"rosbag-inspect {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    info = sub.add_parser("info", help="list topics, types, counts and storage details")
    info.add_argument("bag", help="bag directory, .db3 or .mcap file")
    info.add_argument("--json", action="store_true", help="print JSON")

    check = sub.add_parser(
        "check",
        help="run the health checks and print a verdict",
        description="Run all checks. Exit code: 0 pass, 1 warnings (only with --strict), "
        "2 errors or unreadable bag.",
    )
    check.add_argument("bag", help="bag directory, .db3 or .mcap file")
    check.add_argument("--report", metavar="HTML", help="write a self-contained HTML report")
    check.add_argument("--json", action="store_true", help="print the report as JSON")
    check.add_argument("--strict", action="store_true", help="exit 1 when there are warnings")
    check.add_argument(
        "--expect",
        action="append",
        type=_expectation,
        default=[],
        metavar="TOPIC:HZ",
        help="expected rate of a topic, e.g. --expect /scan:10 (repeatable)",
    )
    check.add_argument(
        "--rate-tolerance",
        type=_positive,
        default=Options.rate_tolerance,
        metavar="FRACTION",
        help="allowed deviation from --expect rates (default 0.2 = 20%%)",
    )
    check.add_argument(
        "--gap-factor",
        type=_positive,
        default=Options.gap_factor,
        metavar="N",
        help="report gaps longer than N x the expected/median period (default 3)",
    )
    check.add_argument(
        "--min-gap",
        type=_positive,
        default=Options.min_gap_s,
        metavar="S",
        help="never report gaps shorter than this (default 0.25 s)",
    )
    check.add_argument(
        "--max-latency",
        type=_positive,
        default=Options.max_latency_s,
        metavar="S",
        help="warn when p95 of receive time - header.stamp exceeds this (default 0.25 s)",
    )
    check.add_argument(
        "--gravity-tolerance",
        type=_positive,
        default=Options.gravity_tolerance,
        metavar="M_S2",
        help="allowed deviation of stationary |acceleration| from 9.81 (default 0.5)",
    )
    check.add_argument(
        "--teleport",
        type=_positive,
        default=Options.teleport_m,
        metavar="M",
        help="odometry jumps above this between messages are errors (default 0.5 m)",
    )

    echo = sub.add_parser("echo", help="print decoded messages as JSON lines")
    echo.add_argument("bag", help="bag directory, .db3 or .mcap file")
    echo.add_argument("topic")
    echo.add_argument(
        "--limit", type=int, default=10, metavar="N", help="messages to print (default 10, 0 = all)"
    )

    exp = sub.add_parser("export", help="write the numeric fields of a topic to CSV")
    exp.add_argument("bag", help="bag directory, .db3 or .mcap file")
    exp.add_argument("topic")
    exp.add_argument("output", help="CSV file to write ('-' for stdout)")
    return parser


def _format_info(bag: Bag, out: TextIO) -> None:
    start, end = bag.start_ns, bag.end_ns
    duration = (end - start) / 1e9 if start is not None and end is not None else 0.0
    out.write(f"rosbag-inspect {__version__}  {bag.path}\n")
    out.write(f"  storage    {bag.storage} ({len(bag.files)} file(s))\n")
    for key, value in sorted(bag.details.items()):
        out.write(f"  {key:<10} {value}\n")
    if bag.metadata is not None:
        meta = bag.metadata
        out.write(
            f"  metadata   version {meta.version}, {meta.message_count} messages"
            + (
                f", compression {meta.compression_format}/{meta.compression_mode}"
                if meta.compression_format
                else ""
            )
            + "\n"
        )
    out.write(f"  duration   {duration:.3f} s\n")
    out.write(f"  messages   {bag.message_count}\n")
    width = max((len(t) for t in bag.topics), default=5)
    out.write(f"\n  {'topic':<{width}}  {'count':>8}  {'Hz':>8}  type\n")
    for name, topic in bag.topics.items():
        hz = "-"
        if topic.first_ns is not None and topic.last_ns is not None and topic.message_count > 1:
            span = (topic.last_ns - topic.first_ns) / 1e9
            hz = f"{(topic.message_count - 1) / span:.2f}" if span > 0 else "-"
        decoded = "" if bag.decodable(name) else "  (not decodable)"
        out.write(
            f"  {name:<{width}}  {topic.message_count:>8}  {hz:>8}  {topic.type or '?'}{decoded}\n"
        )
    for issue in bag.issues:
        out.write(f"\n  WARN  {issue}\n")


def _info_json(bag: Bag) -> dict[str, object]:
    return {
        "bag": str(bag.path),
        "storage": bag.storage,
        "files": [f.name for f in bag.files],
        "details": bag.details,
        "start_ns": bag.start_ns,
        "end_ns": bag.end_ns,
        "message_count": bag.message_count,
        "topics": [
            {
                "name": t.name,
                "type": t.type,
                "serialization_format": t.serialization_format,
                "message_count": t.message_count,
                "metadata_count": t.metadata_count,
                "decodable": bag.decodable(t.name),
                "definition": t.definition_source,
            }
            for t in bag.topics.values()
        ],
        "issues": bag.issues,
    }


def format_report(report: Report, out: TextIO) -> None:
    out.write(f"rosbag-inspect {report.tool_version}  {report.bag}\n")
    out.write(f"  storage    {report.storage}: {', '.join(report.files)}\n")
    out.write(f"  duration   {report.duration_s:.3f} s, {report.message_count} messages\n\n")
    width = max((len(t.name) for t in report.topics), default=5)
    out.write(
        f"  {'topic':<{width}}  {'count':>7}  {'avg Hz':>8}  {'min Hz':>7}  {'max Hz':>8}  "
        f"{'gaps':>4}  {'lat p95':>8}\n"
    )
    for t in report.topics:
        avg = f"{t.rate_avg_hz:.2f}" if t.rate_avg_hz is not None else "-"
        rmin = f"{t.rate_min_hz:.2f}" if t.rate_min_hz is not None else "-"
        rmax = f"{t.rate_max_hz:.2f}" if t.rate_max_hz is not None else "-"
        lat = f"{t.latency.p95_s * 1000:.1f}ms" if t.latency else "-"
        out.write(
            f"  {t.name:<{width}}  {t.count:>7}  {avg:>8}  {rmin:>7}  {rmax:>8}  "
            f"{len(t.gaps):>4}  {lat:>8}\n"
        )
    out.write("\n")
    for f in report.findings:
        where = f"{f.topic}: " if f.topic else ""
        out.write(f"  {_MARK[f.severity]}  {where}{f.title} [{f.code}]\n")
        out.write(f"        {f.detail}\n")
        for item in f.items[:5]:
            out.write(f"        - {item}\n")
        if len(f.items) > 5:
            out.write(f"        ... {len(f.items) - 5} more (see --json or --report)\n")
    counts = report.counts()
    out.write(
        f"\nverdict: {report.verdict.upper()} ({counts[ERROR]} error(s), "
        f"{counts[WARNING]} warning(s), {counts[INFO]} note(s))\n"
    )


def _check(args: argparse.Namespace, bag: Bag) -> int:
    opts = Options(
        gap_factor=args.gap_factor,
        min_gap_s=args.min_gap,
        expectations=dict(args.expect),
        rate_tolerance=args.rate_tolerance,
        max_latency_s=args.max_latency,
        gravity_tolerance=args.gravity_tolerance,
        teleport_m=args.teleport,
    )
    report = analyze(bag, opts)
    if args.report:
        Path(args.report).write_text(render_html(report), encoding="utf-8")
    if args.json:
        json.dump(report.to_dict(), sys.stdout, indent=2, allow_nan=False)
        sys.stdout.write("\n")
    else:
        format_report(report, sys.stdout)
        if args.report:
            sys.stdout.write(f"report written to {args.report}\n")
    counts = report.counts()
    if counts[ERROR]:
        return EXIT_ERROR
    if counts[WARNING] and args.strict:
        return EXIT_WARN
    return EXIT_OK


def _echo(args: argparse.Namespace, bag: Bag) -> int:
    topic = bag.topics.get(args.topic)
    if topic is None:
        raise BagError(f"topic {args.topic} not in bag (see 'rosbag-inspect info')")
    if not bag.decodable(args.topic):
        raise BagError(f"{args.topic}: no definition for {topic.type or 'its type'}; cannot decode")
    for printed, raw in enumerate(bag.messages([args.topic]), start=1):
        msg = decode(bag.registry, topic.type, raw.data)
        line = {"topic": raw.topic, "log_time_ns": raw.log_time, "message": to_jsonable(msg)}
        sys.stdout.write(json.dumps(line, allow_nan=False) + "\n")
        if args.limit and printed >= args.limit:
            break
    return EXIT_OK


def _export(args: argparse.Namespace, bag: Bag) -> int:
    topic = bag.topics.get(args.topic)
    if topic is None:
        raise BagError(f"topic {args.topic} not in bag (see 'rosbag-inspect info')")
    if not bag.decodable(args.topic):
        raise BagError(f"{args.topic}: no definition for {topic.type or 'its type'}; cannot decode")
    columns = flat_columns(bag.registry, topic.type)
    header_stamp = "header.stamp.sec" in columns
    fieldnames = ["log_time_ns", *(["stamp_ns"] if header_stamp else []), *columns]
    handle: TextIO
    if args.output == "-":
        handle = sys.stdout
    else:
        handle = open(args.output, "w", newline="", encoding="utf-8")  # noqa: SIM115
    rows = 0
    try:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for raw in bag.messages([args.topic]):
            msg = decode(bag.registry, topic.type, raw.data)
            row = {"log_time_ns": raw.log_time, **flatten(bag.registry, topic.type, msg)}
            if header_stamp:
                row["stamp_ns"] = stamp_ns(msg["header"])
            writer.writerow(row)
            rows += 1
    finally:
        if handle is not sys.stdout:
            handle.close()
    if args.output != "-":
        sys.stderr.write(f"wrote {rows} row(s), {len(fieldnames)} column(s) to {args.output}\n")
    return EXIT_OK


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        with open_bag(args.bag) as bag:
            if args.command == "info":
                if args.json:
                    json.dump(_info_json(bag), sys.stdout, indent=2)
                    sys.stdout.write("\n")
                else:
                    _format_info(bag, sys.stdout)
                return EXIT_OK
            if args.command == "check":
                return _check(args, bag)
            if args.command == "echo":
                return _echo(args, bag)
            return _export(args, bag)
    except BrokenPipeError:
        # Output piped into e.g. `head`: stop quietly, as other command-line tools do.
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, sys.stdout.fileno())
        return EXIT_OK
    except (BagError, CdrError, MsgDefError, OSError) as exc:
        sys.stderr.write(f"rosbag-inspect: error: {exc}\n")
        return EXIT_ERROR


if __name__ == "__main__":
    raise SystemExit(main())
