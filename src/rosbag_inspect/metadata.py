"""rosbag2 ``metadata.yaml`` model (only the keys this tool needs)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from .yamlmini import YamlError, loads


class MetadataError(ValueError):
    """``metadata.yaml`` exists but cannot be understood."""


@dataclass
class TopicMetadata:
    name: str
    type: str
    serialization_format: str
    message_count: int
    offered_qos_profiles: str = ""


@dataclass
class BagMetadata:
    version: int
    storage_identifier: str
    duration_ns: int
    starting_time_ns: int
    message_count: int
    topics: list[TopicMetadata] = field(default_factory=list)
    relative_file_paths: list[str] = field(default_factory=list)
    compression_format: str = ""
    compression_mode: str = ""
    ros_distro: str = ""


def _get(mapping: object, key: str, where: str) -> object:
    if not isinstance(mapping, dict):
        raise MetadataError(f"{where}: expected a mapping")
    if key not in mapping:
        raise MetadataError(f"{where}: missing key {key!r}")
    return mapping[key]


def _int(value: object, where: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise MetadataError(f"{where}: expected an integer, got {value!r}")
    return value


def _str(value: object) -> str:
    return "" if value is None else str(value)


def _qos_text(value: object) -> str:
    """rosbag2 writes QoS as a YAML string; newer versions may write a list."""
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    return repr(value)


def parse_metadata(text: str) -> BagMetadata:
    """Parse the text of a rosbag2 ``metadata.yaml``."""
    try:
        doc = loads(text)
    except YamlError as exc:
        raise MetadataError(f"metadata.yaml: {exc}") from exc
    info = _get(doc, "rosbag2_bagfile_information", "metadata.yaml")
    where = "rosbag2_bagfile_information"
    if not isinstance(info, dict):
        raise MetadataError(f"{where}: expected a mapping")
    version = _int(info.get("version", 1), f"{where}.version")
    duration = _int(_get(info.get("duration"), "nanoseconds", f"{where}.duration"), "duration")
    start = _int(
        _get(info.get("starting_time"), "nanoseconds_since_epoch", f"{where}.starting_time"),
        "starting_time",
    )
    topics: list[TopicMetadata] = []
    raw_topics = info.get("topics_with_message_count") or []
    if not isinstance(raw_topics, list):
        raise MetadataError(f"{where}.topics_with_message_count: expected a list")
    for index, entry in enumerate(raw_topics):
        loc = f"{where}.topics_with_message_count[{index}]"
        meta = _get(entry, "topic_metadata", loc)
        if not isinstance(meta, dict):
            raise MetadataError(f"{loc}.topic_metadata: expected a mapping")
        topics.append(
            TopicMetadata(
                name=_str(_get(meta, "name", loc)),
                type=_str(_get(meta, "type", loc)),
                serialization_format=_str(meta.get("serialization_format", "cdr")),
                message_count=_int(_get(entry, "message_count", loc), f"{loc}.message_count"),
                offered_qos_profiles=_qos_text(meta.get("offered_qos_profiles")),
            )
        )
    files = info.get("relative_file_paths") or []
    if not isinstance(files, list):
        raise MetadataError(f"{where}.relative_file_paths: expected a list")
    return BagMetadata(
        version=version,
        storage_identifier=_str(info.get("storage_identifier", "")),
        duration_ns=duration,
        starting_time_ns=start,
        message_count=_int(info.get("message_count", 0), f"{where}.message_count"),
        topics=topics,
        relative_file_paths=[_str(f) for f in files],
        compression_format=_str(info.get("compression_format", "")),
        compression_mode=_str(info.get("compression_mode", "")),
        ros_distro=_str(info.get("ros_distro", "")),
    )


def read_metadata(path: Path) -> BagMetadata:
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError as exc:
        raise MetadataError(f"{path}: not UTF-8 text") from exc
    return parse_metadata(text)
