"""Minimal writers for rosbag2 SQLite3 bags and MCAP files, used to build test fixtures.

They are written independently from the readers, from the same public specifications, so a
round trip exercises both sides. Only what the tests need is implemented.
"""

from __future__ import annotations

import sqlite3
import struct
import zlib
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

from rosbag_inspect.msgdef import BUILTIN_DEFINITIONS, SEPARATOR, Registry

MAGIC = b"\x89MCAP0\r\n"
QOS = (
    "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n  deadline:\n    sec: "
    "9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n"
    "  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n"
    "  avoid_ros_namespace_conventions: false"
)


@dataclass
class Record:
    """One serialized message to write."""

    topic: str
    type: str
    log_time: int
    data: bytes


# -- ros2msg bundles ---------------------------------------------------------------------------


def ros2msg_bundle(type_name: str, registry: Registry | None = None) -> str:
    """Concatenate a type and its dependencies the way rosbag2's MCAP plugin does."""
    registry = registry or Registry()
    order: list[str] = []
    stack = [type_name]
    while stack:
        current = stack.pop(0)
        if current in order:
            continue
        order.append(current)
        for fld in registry.get(current).fields:
            if not fld.type.is_primitive and fld.type.base not in order:
                stack.append(fld.type.base)
    parts = [BUILTIN_DEFINITIONS[order[0]].rstrip("\n")]
    for dep in order[1:]:
        pkg, _, name = dep.split("/")
        parts.append(f"{SEPARATOR}\nMSG: {pkg}/{name}\n{BUILTIN_DEFINITIONS[dep].rstrip(chr(10))}")
    return "\n".join(parts) + "\n"


# -- LZ4 ---------------------------------------------------------------------------------------


def _lz4_lengths(value: int) -> bytes:
    out = bytearray()
    while value >= 255:
        out.append(255)
        value -= 255
    out.append(value)
    return bytes(out)


def lz4_compress_block(data: bytes) -> bytes:
    """Greedy LZ4 block compressor (hash of 4-byte sequences) following the end-of-block
    rules: the last 5 bytes are literals and no match starts in the last 12 bytes."""
    out = bytearray()
    n = len(data)
    anchor = i = 0
    table: dict[bytes, int] = {}
    limit = n - 12
    while i < limit:
        key = data[i : i + 4]
        candidate = table.get(key)
        table[key] = i
        if candidate is not None and i - candidate <= 0xFFFF:
            length = 4
            while i + length < n - 5 and data[candidate + length] == data[i + length]:
                length += 1
            literals = data[anchor:i]
            lit, match = len(literals), length - 4
            out.append((min(lit, 15) << 4) | min(match, 15))
            if lit >= 15:
                out += _lz4_lengths(lit - 15)
            out += literals
            out += struct.pack("<H", i - candidate)
            if match >= 15:
                out += _lz4_lengths(match - 15)
            i += length
            anchor = i
        else:
            i += 1
    literals = data[anchor:]
    out.append(min(len(literals), 15) << 4)
    if len(literals) >= 15:
        out += _lz4_lengths(len(literals) - 15)
    out += literals
    return bytes(out)


def lz4_compress_frame(data: bytes, block_size: int = 65536, raw_blocks: bool = False) -> bytes:
    """LZ4 frame with independent blocks. The header checksum byte is left as zero (the reader
    does not verify it)."""
    out = bytearray(struct.pack("<I", 0x184D2204))
    out += bytes([0x60, 0x40, 0x00])  # FLG: version 1, independent blocks; BD: 64 KiB
    for start in range(0, len(data), block_size):
        chunk = data[start : start + block_size]
        block = chunk if raw_blocks else lz4_compress_block(chunk)
        if raw_blocks or len(block) >= len(chunk):
            out += struct.pack("<I", len(chunk) | 0x80000000) + chunk
        else:
            out += struct.pack("<I", len(block)) + block
    out += struct.pack("<I", 0)
    return bytes(out)


# -- MCAP --------------------------------------------------------------------------------------


def _string(text: str) -> bytes:
    raw = text.encode("utf-8")
    return struct.pack("<I", len(raw)) + raw


def _record(opcode: int, content: bytes) -> bytes:
    return struct.pack("<BQ", opcode, len(content)) + content


def _string_map(values: dict[str, str]) -> bytes:
    body = b"".join(_string(k) + _string(v) for k, v in sorted(values.items()))
    return struct.pack("<I", len(body)) + body


def _u16_u64_map(values: dict[int, int]) -> bytes:
    body = b"".join(struct.pack("<HQ", k, v) for k, v in sorted(values.items()))
    return struct.pack("<I", len(body)) + body


def _compress(compression: str, data: bytes) -> bytes:
    if compression == "":
        return data
    if compression == "lz4":
        return lz4_compress_frame(data)
    if compression == "zstd":
        try:
            from compression import zstd  # type: ignore[import-not-found,unused-ignore]
        except ImportError:
            import zstandard

            out: bytes = zstandard.ZstdCompressor().compress(data)
            return out
        result: bytes = zstd.compress(data)
        return result
    raise ValueError(compression)


def write_mcap(
    path: Path,
    records: Iterable[Record],
    *,
    chunked: bool = True,
    compression: str = "",
    chunk_size: int = 64 * 1024,
    summary: bool = True,
    schemas: bool = True,
    crc: bool = True,
) -> None:
    """Write an MCAP file with the ros2 profile and ros2msg schemas."""
    out = bytearray(MAGIC)
    out += _record(0x01, _string("ros2") + _string("rosbag-inspect test writer"))
    schema_ids: dict[str, int] = {}
    channel_ids: dict[str, int] = {}
    schema_records: list[bytes] = []
    channel_records: list[bytes] = []
    chunk_indexes: list[bytes] = []
    counts: dict[int, int] = {}
    total = 0
    first: int | None = None
    last: int | None = None
    chunk_buf = bytearray()
    chunk_index: dict[int, list[tuple[int, int]]] = {}
    chunk_times: list[int] = []
    chunk_count = 0

    def flush_chunk() -> None:
        nonlocal chunk_buf, chunk_index, chunk_times, chunk_count
        if not chunk_buf:
            return
        records_raw = bytes(chunk_buf)
        compressed = _compress(compression, records_raw)
        start, end = min(chunk_times), max(chunk_times)
        content = (
            struct.pack(
                "<QQQI", start, end, len(records_raw), zlib.crc32(records_raw) if crc else 0
            )
            + _string(compression)
            + struct.pack("<Q", len(compressed))
            + compressed
        )
        chunk_offset = len(out)
        out.extend(_record(0x06, content))
        chunk_length = len(out) - chunk_offset
        index_offsets: dict[int, int] = {}
        index_start = len(out)
        for channel_id, entries in sorted(chunk_index.items()):
            index_offsets[channel_id] = len(out)
            body = b"".join(struct.pack("<QQ", t, o) for t, o in entries)
            out.extend(
                _record(0x07, struct.pack("<H", channel_id) + struct.pack("<I", len(body)) + body)
            )
        chunk_indexes.append(
            _record(
                0x08,
                struct.pack("<QQQQ", start, end, chunk_offset, chunk_length)
                + _u16_u64_map(index_offsets)
                + struct.pack("<Q", len(out) - index_start)
                + _string(compression)
                + struct.pack("<QQ", len(compressed), len(records_raw)),
            )
        )
        chunk_count += 1
        chunk_buf = bytearray()
        chunk_index = {}
        chunk_times = []

    for sequence, rec in enumerate(records, start=1):
        if rec.topic not in channel_ids:
            schema_id = 0
            if schemas:
                if rec.type not in schema_ids:
                    schema_ids[rec.type] = len(schema_ids) + 1
                    body = ros2msg_bundle(rec.type).encode("utf-8")
                    content = (
                        struct.pack("<H", schema_ids[rec.type])
                        + _string(rec.type)
                        + _string("ros2msg")
                        + struct.pack("<I", len(body))
                        + body
                    )
                    schema_records.append(_record(0x03, content))
                    out.extend(schema_records[-1])
                schema_id = schema_ids[rec.type]
            channel_ids[rec.topic] = len(channel_ids)
            content = (
                struct.pack("<HH", channel_ids[rec.topic], schema_id)
                + _string(rec.topic)
                + _string("cdr")
                + _string_map({"offered_qos_profiles": QOS})
            )
            channel_records.append(_record(0x04, content))
            out.extend(channel_records[-1])
        channel_id = channel_ids[rec.topic]
        message = _record(
            0x05, struct.pack("<HIQQ", channel_id, sequence, rec.log_time, rec.log_time) + rec.data
        )
        counts[channel_id] = counts.get(channel_id, 0) + 1
        total += 1
        first = rec.log_time if first is None else min(first, rec.log_time)
        last = rec.log_time if last is None else max(last, rec.log_time)
        if chunked:
            chunk_index.setdefault(channel_id, []).append((rec.log_time, len(chunk_buf)))
            chunk_times.append(rec.log_time)
            chunk_buf += message
            if len(chunk_buf) >= chunk_size:
                flush_chunk()
        else:
            out.extend(message)
    flush_chunk()
    out.extend(_record(0x0F, struct.pack("<I", 0)))  # DataEnd without CRC

    summary_start = 0
    summary_offset_start = 0
    if summary:
        summary_start = len(out)
        groups: list[tuple[int, int, int]] = []
        for opcode, recs in ((0x03, schema_records), (0x04, channel_records)):
            if recs:
                start = len(out)
                for r in recs:
                    out.extend(r)
                groups.append((opcode, start, len(out) - start))
        start = len(out)
        out.extend(
            _record(
                0x0B,
                struct.pack(
                    "<QHIIIIQQ",
                    total,
                    len(schema_records),
                    len(channel_records),
                    0,
                    0,
                    chunk_count,
                    first or 0,
                    last or 0,
                )
                + _u16_u64_map(counts),
            )
        )
        groups.append((0x0B, start, len(out) - start))
        if chunk_indexes:
            start = len(out)
            for r in chunk_indexes:
                out.extend(r)
            groups.append((0x08, start, len(out) - start))
        summary_offset_start = len(out)
        for opcode, start, length in groups:
            out.extend(_record(0x0E, struct.pack("<BQQ", opcode, start, length)))
    footer_head = struct.pack("<BQQQ", 0x02, 20, summary_start, summary_offset_start)
    summary_crc = zlib.crc32(bytes(out[summary_start:]) + footer_head) if summary else 0
    out.extend(footer_head + struct.pack("<I", summary_crc if crc else 0))
    out.extend(MAGIC)
    path.write_bytes(bytes(out))


# -- rosbag2 SQLite3 ---------------------------------------------------------------------------


def write_sqlite_bag(
    directory: Path,
    records: list[Record],
    *,
    metadata: bool = True,
    distro: str = "humble",
    metadata_counts: dict[str, int] | None = None,
) -> Path:
    """Write ``<directory>/<name>_0.db3`` (+ ``metadata.yaml``) like rosbag2 on Humble."""
    directory.mkdir(parents=True, exist_ok=True)
    db_path = directory / f"{directory.name}_0.db3"
    if db_path.exists():
        db_path.unlink()
    con = sqlite3.connect(db_path)
    con.executescript(
        "CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);"
        "CREATE TABLE metadata(id INTEGER PRIMARY KEY, metadata_version INTEGER NOT NULL,"
        " metadata TEXT NOT NULL);"
        "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,"
        " serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);"
        "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,"
        " timestamp INTEGER NOT NULL, data BLOB NOT NULL);"
        "CREATE INDEX timestamp_idx ON messages (timestamp ASC);"
    )
    con.execute("INSERT INTO schema VALUES (3, ?)", (distro,))
    topic_ids: dict[str, int] = {}
    types: dict[str, str] = {}
    counts: dict[str, int] = {}
    for rec in records:
        if rec.topic not in topic_ids:
            topic_ids[rec.topic] = len(topic_ids) + 1
            types[rec.topic] = rec.type
            con.execute(
                "INSERT INTO topics VALUES (?, ?, ?, 'cdr', ?)",
                (topic_ids[rec.topic], rec.topic, rec.type, QOS),
            )
        counts[rec.topic] = counts.get(rec.topic, 0) + 1
        con.execute(
            "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
            (topic_ids[rec.topic], rec.log_time, rec.data),
        )
    con.commit()
    con.close()
    if metadata:
        start = min(r.log_time for r in records)
        end = max(r.log_time for r in records)
        overrides = metadata_counts or {}
        lines = [
            "rosbag2_bagfile_information:",
            "  version: 5",
            "  storage_identifier: sqlite3",
            "  duration:",
            f"    nanoseconds: {end - start}",
            "  starting_time:",
            f"    nanoseconds_since_epoch: {start}",
            f"  message_count: {len(records)}",
            "  topics_with_message_count:",
        ]
        qos_yaml = QOS.replace("\n", "\\n")
        for topic, _tid in sorted(topic_ids.items(), key=lambda kv: kv[1]):
            lines += [
                "    - topic_metadata:",
                f"        name: {topic}",
                f"        type: {types[topic]}",
                "        serialization_format: cdr",
                f'        offered_qos_profiles: "{qos_yaml}"',
                f"      message_count: {overrides.get(topic, counts[topic])}",
            ]
        lines += [
            '  compression_format: ""',
            '  compression_mode: ""',
            "  relative_file_paths:",
            f"    - {db_path.name}",
            "  files:",
            f"    - path: {db_path.name}",
            "      starting_time:",
            f"        nanoseconds_since_epoch: {start}",
            "      duration:",
            f"        nanoseconds: {end - start}",
            f"      message_count: {len(records)}",
        ]
        (directory / "metadata.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return db_path
