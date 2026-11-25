"""Run every check over a bag and collect a :class:`~rosbag_inspect.findings.Report`."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

from . import __version__
from .bag import Bag
from .cdr import CdrError, Message, decode
from .findings import ERROR, INFO, WARNING, Finding, Report, TopicStats, sort_findings
from .msgdef import MsgDefError
from .options import Options
from .sensors import (
    IMAGE,
    IMU,
    JOINT_STATE,
    LASER,
    ODOM,
    TF,
    TWIST,
    TWIST_STAMPED,
    CmdSample,
    ImageCollector,
    ImuCollector,
    JointStateCollector,
    LaserCollector,
    OdomCollector,
    cmd_response_findings,
    image_findings,
    imu_findings,
    joint_state_findings,
    laser_findings,
    odom_findings,
    twist_values,
)
from .tfcheck import TfCollector, analyze_tf
from .timing import analyze_rates, analyze_stamps

EMPTY_FRAME_TYPES = (LASER, IMU, IMAGE, ODOM, "sensor_msgs/msg/PointCloud2")


def stamp_ns(header: Message) -> int:
    stamp = header["stamp"]
    return int(stamp["sec"]) * 1_000_000_000 + int(stamp["nanosec"])


EVENT_TYPES = (
    "rcl_interfaces/msg/Log",
    "rcl_interfaces/msg/ParameterEvent",
    "rosbag2_interfaces/msg/WriteSplitEvent",
    "action_msgs/msg/GoalStatusArray",
)


def is_event_topic(topic: str, type_name: str) -> bool:
    """Topics published on events rather than periodically (no gap or jitter checks)."""
    return (
        type_name in EVENT_TYPES
        or type_name.endswith("_Event")
        or "/_action/" in topic
        or topic.startswith("/events/")
    )


def is_static_tf(topic: str) -> bool:
    return topic.rstrip("/").endswith("tf_static")


@dataclass
class _TopicState:
    recv: list[int] = field(default_factory=list)
    stamp_recv: list[int] = field(default_factory=list)
    stamps: list[int] = field(default_factory=list)
    frames: dict[str, int] = field(default_factory=dict)
    decode_errors: int = 0
    first_error: str = ""
    has_header: bool = False


def _has_header(bag: Bag, type_name: str) -> bool:
    try:
        definition = bag.registry.get(type_name)
    except MsgDefError:
        return False
    return bool(definition.fields) and (
        definition.fields[0].name == "header"
        and definition.fields[0].type.base == "std_msgs/msg/Header"
        and definition.fields[0].type.array == ""
    )


def analyze(bag: Bag, opts: Options | None = None) -> Report:
    opts = opts or Options()
    findings: list[Finding] = []
    for issue in bag.issues:
        incomplete = "interrupted" in issue or "not closed" in issue or "missing" in issue
        findings.append(
            Finding(
                "recording_incomplete" if incomplete else "storage_issue",
                WARNING,
                "",
                "Recording was not finished cleanly" if incomplete else "Storage problem",
                issue,
            )
        )

    for note in bag.notes:
        findings.append(Finding("schema_not_parsed", INFO, "", "Schema not parsed", note))

    states = {name: _TopicState() for name in bag.topics}
    decodable = {name for name in bag.topics if bag.decodable(name)}
    for name in decodable:
        states[name].has_header = _has_header(bag, bag.topics[name].type)
    lasers: dict[str, LaserCollector] = {}
    imus: dict[str, ImuCollector] = {}
    odoms: dict[str, OdomCollector] = {}
    images: dict[str, ImageCollector] = {}
    joints: dict[str, JointStateCollector] = {}
    cmds: dict[str, list[CmdSample]] = {}
    tf = TfCollector()

    # Times in findings are shown relative to the first message the storage reports.
    origin = bag.start_ns or 0
    for raw in bag.messages():
        state = states[raw.topic]
        recv = raw.log_time
        state.recv.append(recv)
        topic = raw.topic
        if topic not in decodable:
            continue
        type_name = bag.topics[topic].type
        try:
            msg = decode(bag.registry, type_name, raw.data)
        except (CdrError, MsgDefError) as exc:
            state.decode_errors += 1
            if not state.first_error:
                state.first_error = str(exc)
            continue
        stamp = 0
        if state.has_header:
            header = msg["header"]
            stamp = stamp_ns(header)
            state.stamp_recv.append(recv)
            state.stamps.append(stamp)
            frame = str(header["frame_id"])
            state.frames[frame] = state.frames.get(frame, 0) + 1
            if type_name != TF:
                tf.reference(frame, topic)
        if type_name == LASER:
            lasers.setdefault(topic, LaserCollector()).add(msg, recv, origin)
        elif type_name == IMU:
            imus.setdefault(topic, ImuCollector()).add(msg, recv, origin, opts)
        elif type_name == ODOM:
            odoms.setdefault(topic, OdomCollector()).add(msg, recv, stamp, origin, opts)
            tf.reference(str(msg["child_frame_id"]), topic)
        elif type_name == IMAGE:
            images.setdefault(topic, ImageCollector()).add(msg, recv, origin)
        elif type_name == JOINT_STATE:
            joints.setdefault(topic, JointStateCollector()).add(msg, recv, origin)
        elif type_name in (TWIST, TWIST_STAMPED):
            vx, vy, wz = twist_values(msg, type_name == TWIST_STAMPED)
            cmds.setdefault(topic, []).append(CmdSample(recv, vx, vy, wz))
        elif type_name == TF:
            static = is_static_tf(topic)
            for t in msg["transforms"]:
                parent, child = str(t["header"]["frame_id"]), str(t["child_frame_id"])
                tf.add_transform(parent, child, static, topic)

    end = max((max(s.recv) for s in states.values() if s.recv), default=origin)
    topic_stats: list[TopicStats] = []
    for name, info in bag.topics.items():
        state = states[name]
        stats = TopicStats(name, info.type, len(state.recv), decoded=name in decodable)
        topic_stats.append(stats)
        order = sorted(range(len(state.recv)), key=state.recv.__getitem__)
        times = [state.recv[i] for i in order]
        if is_static_tf(name):
            stats.first_ns = times[0] if times else None
            stats.last_ns = times[-1] if times else None
        else:
            periodic = name in opts.expectations or not is_event_topic(name, info.type)
            findings.extend(analyze_rates(stats, times, origin, end, opts, periodic))
        stats.frame_ids = dict(sorted(state.frames.items()))
        if state.stamps and not is_static_tf(name):
            pairs = sorted(zip(state.stamp_recv, state.stamps, strict=True), key=lambda p: p[0])
            findings.extend(
                analyze_stamps(stats, [p[0] for p in pairs], [p[1] for p in pairs], origin, opts)
            )
        findings.extend(_topic_findings(bag, name, state, opts))

    for name in sorted(opts.expectations):
        if name not in bag.topics or not states[name].recv:
            findings.append(
                Finding(
                    "expected_topic_missing",
                    ERROR,
                    name,
                    "Expected topic has no messages",
                    f"--expect asked for {name} at {opts.expectations[name]:g} Hz, but the bag "
                    + ("does not contain it." if name not in bag.topics else "has no messages."),
                )
            )

    rates = {s.name: s.rate_avg_hz for s in topic_stats}
    for name, lcol in lasers.items():
        findings.extend(laser_findings(name, lcol, rates.get(name), opts))
    for name, icol in imus.items():
        findings.extend(imu_findings(name, icol, opts))
    for name, ocol in odoms.items():
        findings.extend(odom_findings(name, ocol, opts, origin))
    for name, imcol in images.items():
        findings.extend(image_findings(name, imcol))
    for name, jcol in joints.items():
        findings.extend(joint_state_findings(name, jcol))
    if odoms:
        odom_topic = "/odom" if "/odom" in odoms else sorted(odoms)[0]
        for name, samples in sorted(cmds.items()):
            if name.rstrip("/").endswith("cmd_vel"):
                findings.extend(
                    cmd_response_findings(
                        name, samples, odom_topic, odoms[odom_topic].samples, opts, origin
                    )
                )
    tf_summary, tf_findings = analyze_tf(tf)
    findings.extend(tf_findings)

    return Report(
        tool_version=__version__,
        bag=str(bag.path),
        storage=bag.storage,
        files=[f.name for f in bag.files],
        start_ns=bag.start_ns,
        end_ns=end if any(s.recv for s in states.values()) else bag.end_ns,
        message_count=sum(len(s.recv) for s in states.values()),
        topics=topic_stats,
        findings=sort_findings(findings),
        tf=tf_summary,
        details=dict(bag.details),
        options=asdict(opts),
    )


def _topic_findings(bag: Bag, name: str, state: _TopicState, opts: Options) -> list[Finding]:
    info = bag.topics[name]
    findings: list[Finding] = []
    if info.metadata_count is not None and info.metadata_count != len(state.recv):
        findings.append(
            Finding(
                "metadata_count_mismatch",
                WARNING,
                name,
                "metadata.yaml message count differs from storage",
                f"metadata.yaml says {info.metadata_count} messages, the storage file holds "
                f"{len(state.recv)}. The metadata is stale (bag edited or not closed cleanly).",
                data={"metadata": info.metadata_count, "storage": len(state.recv)},
            )
        )
    if not state.recv:
        findings.append(
            Finding(
                "empty_topic",
                WARNING if name in opts.expectations else INFO,
                name,
                "Topic has no messages",
                "The topic was subscribed to but nothing was recorded on it (publisher absent, "
                "or a QoS mismatch between publisher and recorder).",
                0,
            )
        )
    if not bag.decodable(name):
        if info.serialization_format != "cdr":
            reason = f"serialization format {info.serialization_format!r}"
        elif not info.type:
            reason = "the channel has no schema, so its message type is unknown"
        else:
            missing = bag.registry.missing_dependencies(info.type)
            reason = "no message definition for " + ", ".join(missing or [info.type])
        findings.append(
            Finding(
                "not_decoded",
                INFO,
                name,
                "Message contents not checked",
                f"Only timing was checked: {reason}. MCAP files and .db3 files from Iron on "
                "(message_definitions table) carry definitions; for older .db3 bags only the "
                "built-in common types are decoded.",
            )
        )
    if state.decode_errors:
        findings.append(
            Finding(
                "decode_error",
                ERROR,
                name,
                "Messages could not be decoded",
                f"{state.decode_errors} message(s) did not match the {info.type} definition. "
                f"First error: {state.first_error}",
                state.decode_errors,
            )
        )
    if len(state.frames) > 1:
        findings.append(
            Finding(
                "frame_id_changes",
                WARNING,
                name,
                "Inconsistent frame_id",
                "Messages on one topic use different frame_ids; usually two publishers share "
                "the topic or a parameter changed mid-recording.",
                len(state.frames),
                [f"'{f}': {n} message(s)" for f, n in sorted(state.frames.items())],
            )
        )
    if state.frames.get("", 0) and info.type in EMPTY_FRAME_TYPES:
        findings.append(
            Finding(
                "empty_frame_id",
                WARNING,
                name,
                "Empty frame_id",
                f"{state.frames['']} message(s) have an empty header.frame_id, so they cannot "
                "be placed in the TF tree.",
                state.frames[""],
            )
        )
    return findings


__all__ = ["ERROR", "INFO", "WARNING", "analyze"]
