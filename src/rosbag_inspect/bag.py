"""Open a rosbag2 bag (directory, ``.db3`` or ``.mcap``) behind one interface."""

from __future__ import annotations

import heapq
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass, field
from pathlib import Path
from types import TracebackType
from typing import NamedTuple

from .mcap import ZSTD_HELP, McapError, McapReader, _zstd_decompressor
from .metadata import BagMetadata, MetadataError, read_metadata
from .msgdef import MsgDefError, Registry, normalize_type
from .sqlite_storage import SqliteBagError, SqliteReader

IMAGE_TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")


class BagError(ValueError):
    """The bag cannot be opened or read."""


class RawMessage(NamedTuple):
    topic: str
    log_time: int
    data: bytes


@dataclass
class Topic:
    name: str
    type: str
    serialization_format: str = "cdr"
    message_count: int = 0
    first_ns: int | None = None
    last_ns: int | None = None
    offered_qos_profiles: str = ""
    metadata_count: int | None = None
    definition_source: str = ""


@dataclass
class Bag:
    path: Path
    storage: str
    files: list[Path]
    metadata: BagMetadata | None
    topics: dict[str, Topic]
    registry: Registry
    issues: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    details: dict[str, str] = field(default_factory=dict)
    _sqlite: list[SqliteReader] = field(default_factory=list)
    _mcap: list[McapReader] = field(default_factory=list)
    _decompress: Callable[[bytes], bytes] | None = None

    # -- context management ------------------------------------------------------------------
    def close(self) -> None:
        for reader in self._sqlite:
            reader.close()
        self._sqlite.clear()

    def __enter__(self) -> Bag:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    # -- summary -----------------------------------------------------------------------------
    @property
    def message_count(self) -> int:
        return sum(t.message_count for t in self.topics.values())

    @property
    def start_ns(self) -> int | None:
        values = [t.first_ns for t in self.topics.values() if t.first_ns is not None]
        values += [span[0] for span in map(_mcap_span, self._mcap) if span is not None]
        return min(values) if values else None

    @property
    def end_ns(self) -> int | None:
        values = [t.last_ns for t in self.topics.values() if t.last_ns is not None]
        values += [span[1] for span in map(_mcap_span, self._mcap) if span is not None]
        return max(values) if values else None

    def decodable(self, topic: str) -> bool:
        info = self.topics[topic]
        return (
            info.serialization_format == "cdr"
            and info.type in self.registry
            and not self.registry.missing_dependencies(info.type)
        )

    # -- messages ----------------------------------------------------------------------------
    def messages(self, topics: Iterable[str] | None = None) -> Iterator[RawMessage]:
        """Yield raw messages in receive-time order (per file order for unindexed MCAP)."""
        wanted = None if topics is None else set(topics)
        unknown = (wanted or set()) - set(self.topics)
        if unknown:
            raise BagError(f"topic(s) not in bag: {', '.join(sorted(unknown))}")
        streams: list[Iterator[RawMessage]] = []
        for reader in self._sqlite:
            streams.append(self._sqlite_stream(reader, wanted))
        for mreader in self._mcap:
            streams.append(self._mcap_stream(mreader, wanted))
        stream: Iterator[RawMessage] = (
            streams[0] if len(streams) == 1 else heapq.merge(*streams, key=lambda m: m.log_time)
        )
        try:
            for message in stream:
                if self._decompress is not None:
                    message = RawMessage(
                        message.topic, message.log_time, self._decompress(message.data)
                    )
                yield message
        except (SqliteBagError, McapError) as exc:
            raise BagError(str(exc)) from exc

    def _sqlite_stream(self, reader: SqliteReader, wanted: set[str] | None) -> Iterator[RawMessage]:
        names = {t.id: t.name for t in reader.topics}
        ids = None if wanted is None else {i for i, n in names.items() if n in wanted}
        prefix = (
            {t.id for t in reader.topics if normalize_type_safe(t.type) in IMAGE_TYPES}
            if self._decompress is None
            else set()
        )
        for topic_id, timestamp, data in reader.messages(ids, prefix):
            name = names.get(topic_id)
            if name is not None:
                yield RawMessage(name, timestamp, data)

    def _mcap_stream(self, reader: McapReader, wanted: set[str] | None) -> Iterator[RawMessage]:
        names = {c.id: c.topic for c in reader.channels.values()}
        ids = None if wanted is None else {i for i, n in names.items() if n in wanted}
        for msg in reader.messages(ids):
            name = names.get(msg.channel_id)
            if name is not None:
                yield RawMessage(name, msg.log_time, msg.data)
        for issue in reader.issues:
            if issue not in self.issues:
                self.issues.append(issue)


def _mcap_span(reader: McapReader) -> tuple[int, int] | None:
    stats = reader.statistics
    if stats is None or stats.message_count == 0:
        return None
    return stats.message_start_time, stats.message_end_time


def normalize_type_safe(name: str) -> str:
    try:
        return normalize_type(name)
    except MsgDefError:
        return name


def _message_decompressor(fmt: str) -> Callable[[bytes], bytes]:
    if fmt != "zstd":
        raise BagError(f"unsupported per-message compression {fmt!r}")
    func = _zstd_decompressor()
    if func is None:
        raise BagError(
            f"this bag uses per-message zstd compression, whose decompression {ZSTD_HELP}; "
            "or decompress it with 'ros2 bag convert' first"
        )
    return func


def _find_files(directory: Path, metadata: BagMetadata | None, issues: list[str]) -> list[Path]:
    if metadata is not None and metadata.relative_file_paths:
        files: list[Path] = []
        for rel in metadata.relative_file_paths:
            candidate = directory / rel
            if candidate.is_file():
                files.append(candidate)
            else:
                issues.append(f"metadata.yaml lists {rel}, which does not exist")
        if files:
            return files
    found = sorted(p for p in directory.iterdir() if p.suffix in (".db3", ".mcap"))
    compressed = sorted(
        p for p in directory.iterdir() if p.name.endswith((".db3.zstd", ".mcap.zstd"))
    )
    if not found and compressed:
        raise BagError(
            f"{directory}: the bag uses file-level compression ({compressed[0].name}); "
            "decompress it first (for example with 'zstd -d')"
        )
    return found


def open_bag(path: str | Path) -> Bag:
    """Open a bag directory (with or without ``metadata.yaml``), a ``.db3`` or an ``.mcap``."""
    path = Path(path)
    issues: list[str] = []
    notes: list[str] = []
    metadata: BagMetadata | None = None
    if path.is_dir():
        meta_path = path / "metadata.yaml"
        if meta_path.is_file():
            try:
                metadata = read_metadata(meta_path)
            except MetadataError as exc:
                issues.append(f"metadata.yaml could not be read: {exc}")
        else:
            issues.append(
                "metadata.yaml is missing: the recorder was probably killed before it could "
                "finish the bag (run 'ros2 bag reindex' to recreate it)"
            )
        files = _find_files(path, metadata, issues)
        if not files:
            raise BagError(f"{path}: no .db3 or .mcap files found")
    elif path.is_file():
        files = [path]
    else:
        raise BagError(f"{path}: no such file or directory")

    if metadata is not None and metadata.compression_mode.lower() == "file":
        raise BagError(
            f"{path}: the bag uses file-level {metadata.compression_format} compression; "
            "decompress the files first"
        )
    decompress = None
    if metadata is not None and metadata.compression_mode.lower() == "message":
        decompress = _message_decompressor(metadata.compression_format)

    registry = Registry()
    topics: dict[str, Topic] = {}
    sqlite_readers: list[SqliteReader] = []
    mcap_readers: list[McapReader] = []
    storages: set[str] = set()
    details: dict[str, str] = {}
    try:
        for file in files:
            if file.suffix == ".mcap" or _has_mcap_magic(file):
                mreader = McapReader(file)
                mcap_readers.append(mreader)
                storages.add("mcap")
                _add_mcap_topics(mreader, registry, topics, issues, notes)
                details["mcap"] = (
                    f"profile {mreader.profile or '-'}, library {mreader.library or '-'}, "
                    f"{len(mreader.chunk_indexes)} indexed chunk(s), "
                    f"summary {'yes' if mreader.has_summary else 'no'}"
                )
                comps = sorted(c or "none" for c in mreader.compressions)
                if comps:
                    details["mcap"] += f", compression {'/'.join(comps)}"
                issues.extend(i for i in mreader.issues if i not in issues)
            else:
                sreader = SqliteReader(file)
                sqlite_readers.append(sreader)
                storages.add("sqlite3")
                _add_sqlite_topics(sreader, topics)
                details["sqlite3"] = f"schema version {sreader.schema_version}" + (
                    f", ros_distro {sreader.ros_distro}" if sreader.ros_distro else ""
                )
    except (SqliteBagError, McapError, OSError) as exc:
        for reader in sqlite_readers:
            reader.close()
        raise BagError(str(exc)) from exc

    if metadata is not None:
        for tmeta in metadata.topics:
            topic = topics.get(tmeta.name)
            if topic is None:
                topics[tmeta.name] = Topic(
                    tmeta.name,
                    normalize_type_safe(tmeta.type),
                    tmeta.serialization_format,
                    0,
                    offered_qos_profiles=tmeta.offered_qos_profiles,
                    metadata_count=tmeta.message_count,
                )
            else:
                topic.metadata_count = tmeta.message_count
                if not topic.offered_qos_profiles:
                    topic.offered_qos_profiles = tmeta.offered_qos_profiles
    for topic in topics.values():
        if not topic.definition_source and topic.type in registry:
            topic.definition_source = "built-in"
    return Bag(
        path=path,
        storage="+".join(sorted(storages)),
        files=files,
        metadata=metadata,
        topics=dict(sorted(topics.items())),
        registry=registry,
        issues=issues,
        notes=notes,
        details=details,
        _sqlite=sqlite_readers,
        _mcap=mcap_readers,
        _decompress=decompress,
    )


def _has_mcap_magic(path: Path) -> bool:
    with path.open("rb") as f:
        return f.read(8) == b"\x89MCAP0\r\n"


def _merge_topic(topics: dict[str, Topic], new: Topic) -> Topic:
    existing = topics.get(new.name)
    if existing is None:
        topics[new.name] = new
        return new
    existing.message_count += new.message_count
    for value in (new.first_ns,):
        if value is not None:
            existing.first_ns = (
                value if existing.first_ns is None else min(existing.first_ns, value)
            )
    if new.last_ns is not None:
        existing.last_ns = (
            new.last_ns if existing.last_ns is None else max(existing.last_ns, new.last_ns)
        )
    return existing


def _add_sqlite_topics(reader: SqliteReader, topics: dict[str, Topic]) -> None:
    stats = reader.topic_stats()
    for t in reader.topics:
        s = stats.get(t.id)
        _merge_topic(
            topics,
            Topic(
                t.name,
                normalize_type_safe(t.type),
                t.serialization_format,
                s.count if s else 0,
                s.first if s else None,
                s.last if s else None,
                t.offered_qos_profiles,
            ),
        )


def _add_mcap_topics(
    reader: McapReader,
    registry: Registry,
    topics: dict[str, Topic],
    issues: list[str],
    notes: list[str],
) -> None:
    counts = reader.statistics.channel_message_counts if reader.statistics else {}
    for channel in reader.channels.values():
        schema = reader.schemas.get(channel.schema_id)
        type_name = normalize_type_safe(schema.name) if schema else ""
        source = ""
        if schema is not None:
            if schema.encoding == "ros2msg":
                try:
                    registry.add_bundle(schema.name, schema.data.decode("utf-8"))
                    source = "mcap schema"
                except (MsgDefError, UnicodeDecodeError) as exc:
                    issue = f"schema {schema.name} could not be parsed: {exc}"
                    if issue not in issues:
                        issues.append(issue)
            elif schema.encoding:
                note = (
                    f"schema {schema.name} uses encoding {schema.encoding!r}; only ros2msg "
                    "schemas are parsed (built-in definitions are used if available)"
                )
                if note not in notes:
                    notes.append(note)
        count = counts.get(channel.id, 0)
        topic = _merge_topic(
            topics,
            Topic(
                channel.topic,
                type_name,
                channel.message_encoding,
                count,
                None,
                None,
                channel.metadata.get("offered_qos_profiles", ""),
                definition_source=source,
            ),
        )
        if source:
            topic.definition_source = source
