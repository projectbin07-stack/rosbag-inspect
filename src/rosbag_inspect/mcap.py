"""MCAP reader (format version 0) written from the public specification at https://mcap.dev/spec.

The file is ``magic, Header, data section, [DataEnd], [summary section], [summary offsets],
Footer, magic``. Every record is ``opcode:uint8, length:uint64, content``; integers are
little-endian, strings and byte arrays are prefixed with a ``uint32`` length.

When the footer points to a valid summary section, schemas, channels, statistics and chunk
indexes come from it and messages are read chunk by chunk in ``log_time`` order (chunks
without the requested channels are skipped using the chunk index). Otherwise the data
section is scanned linearly, which also works for files whose recording was cut off.

Chunk compression: ``""`` (none) and ``lz4`` (pure-Python decoder) are always supported;
``zstd`` is supported when the interpreter ships :mod:`compression.zstd` (Python 3.14+).
"""

from __future__ import annotations

import heapq
import importlib
import os
import struct
import zlib
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import BinaryIO, NamedTuple

from .lz4 import LZ4Error, decompress_frame

MAGIC = b"\x89MCAP0\r\n"

OP_HEADER = 0x01
OP_FOOTER = 0x02
OP_SCHEMA = 0x03
OP_CHANNEL = 0x04
OP_MESSAGE = 0x05
OP_CHUNK = 0x06
OP_MESSAGE_INDEX = 0x07
OP_CHUNK_INDEX = 0x08
OP_ATTACHMENT = 0x09
OP_ATTACHMENT_INDEX = 0x0A
OP_STATISTICS = 0x0B
OP_METADATA = 0x0C
OP_METADATA_INDEX = 0x0D
OP_SUMMARY_OFFSET = 0x0E
OP_DATA_END = 0x0F

_FOOTER_SIZE = 1 + 8 + 8 + 8 + 4
MAX_RECORD = 1 << 34  # refuse single records above 16 GiB (corrupt length fields)


class McapError(ValueError):
    """The file is not MCAP, is corrupt, or uses an unsupported feature."""


class McapMessage(NamedTuple):
    channel_id: int
    sequence: int
    log_time: int
    publish_time: int
    data: bytes


@dataclass
class McapSchema:
    id: int
    name: str
    encoding: str
    data: bytes


@dataclass
class McapChannel:
    id: int
    schema_id: int
    topic: str
    message_encoding: str
    metadata: dict[str, str] = field(default_factory=dict)


@dataclass
class McapStatistics:
    message_count: int
    schema_count: int
    channel_count: int
    attachment_count: int
    metadata_count: int
    chunk_count: int
    message_start_time: int
    message_end_time: int
    channel_message_counts: dict[int, int]


@dataclass
class ChunkIndex:
    message_start_time: int
    message_end_time: int
    chunk_start_offset: int
    chunk_length: int
    message_index_offsets: dict[int, int]
    message_index_length: int
    compression: str
    compressed_size: int
    uncompressed_size: int


class _Cursor:
    """Little-endian field reader over one record's content."""

    __slots__ = ("data", "pos", "what")

    def __init__(self, data: bytes, what: str) -> None:
        self.data = data
        self.pos = 0
        self.what = what

    def _take(self, size: int) -> int:
        start = self.pos
        if start + size > len(self.data):
            raise McapError(f"{self.what}: record content is too short")
        self.pos += size
        return start

    def u8(self) -> int:
        return self.data[self._take(1)]

    def u16(self) -> int:
        return int(struct.unpack_from("<H", self.data, self._take(2))[0])

    def u32(self) -> int:
        return int(struct.unpack_from("<I", self.data, self._take(4))[0])

    def u64(self) -> int:
        return int(struct.unpack_from("<Q", self.data, self._take(8))[0])

    def raw(self, size: int) -> bytes:
        start = self._take(size)
        return self.data[start : start + size]

    def string(self) -> str:
        try:
            return self.raw(self.u32()).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise McapError(f"{self.what}: string is not UTF-8") from exc

    def string_map(self) -> dict[str, str]:
        end = self.u32() + self.pos
        result: dict[str, str] = {}
        while self.pos < end:
            key = self.string()
            result[key] = self.string()
        return result

    def u16_u64_map(self) -> dict[int, int]:
        end = self.u32() + self.pos
        result: dict[int, int] = {}
        while self.pos < end:
            key = self.u16()
            result[key] = self.u64()
        return result

    def rest(self) -> bytes:
        return self.data[self.pos :]


def _parse_schema(content: bytes) -> McapSchema:
    c = _Cursor(content, "Schema")
    return McapSchema(c.u16(), c.string(), c.string(), c.raw(c.u32()))


def _parse_channel(content: bytes) -> McapChannel:
    c = _Cursor(content, "Channel")
    return McapChannel(c.u16(), c.u16(), c.string(), c.string(), c.string_map())


def _parse_message(content: bytes) -> McapMessage:
    c = _Cursor(content, "Message")
    return McapMessage(c.u16(), c.u32(), c.u64(), c.u64(), c.rest())


def _parse_statistics(content: bytes) -> McapStatistics:
    c = _Cursor(content, "Statistics")
    return McapStatistics(
        c.u64(), c.u16(), c.u32(), c.u32(), c.u32(), c.u32(), c.u64(), c.u64(), c.u16_u64_map()
    )


def _parse_chunk_index(content: bytes) -> ChunkIndex:
    c = _Cursor(content, "ChunkIndex")
    return ChunkIndex(
        c.u64(), c.u64(), c.u64(), c.u64(), c.u16_u64_map(), c.u64(), c.string(), c.u64(), c.u64()
    )


def _zstd_decompressor() -> Callable[[bytes], bytes] | None:
    """zstd decompression: the standard library's on Python 3.14+, else ``zstandard`` if
    installed (``pip install rosbag-inspect[zstd]``), else None."""
    try:
        module = importlib.import_module("compression.zstd")
    except ImportError:
        pass
    else:
        func: Callable[[bytes], bytes] = module.decompress
        return func
    try:
        zstandard = importlib.import_module("zstandard")
    except ImportError:
        return None

    def decompress(data: bytes) -> bytes:
        # Frames written without a content size need the streaming reader.
        out: bytes = zstandard.ZstdDecompressor().decompressobj().decompress(data)
        return out

    return decompress


ZSTD_HELP = (
    "needs Python 3.14 or newer (standard-library compression.zstd) or the zstandard package "
    "(pip install 'rosbag-inspect[zstd]')"
)


def decompress_chunk(compression: str, data: bytes, uncompressed_size: int) -> bytes:
    """Return the uncompressed records of a chunk."""
    if compression == "":
        return data
    if compression == "lz4":
        try:
            return decompress_frame(data, uncompressed_size)
        except LZ4Error as exc:
            raise McapError(f"lz4 chunk: {exc}") from exc
    if compression == "zstd":
        func = _zstd_decompressor()
        if func is None:
            raise McapError(
                f"zstd-compressed chunks: decompression {ZSTD_HELP}; or convert the bag with "
                "'mcap compress --compression none' / 'ros2 bag convert'"
            )
        try:
            out = func(data)
        except Exception as exc:
            raise McapError(f"zstd chunk: {exc}") from exc
        if len(out) != uncompressed_size:
            raise McapError(f"zstd chunk: got {len(out)} bytes, expected {uncompressed_size}")
        return out
    raise McapError(f"unsupported chunk compression {compression!r}")


class _ParsedChunk(NamedTuple):
    message_start_time: int
    message_end_time: int
    uncompressed_size: int
    uncompressed_crc: int
    compression: str
    records: bytes


def _parse_chunk(content: bytes) -> _ParsedChunk:
    c = _Cursor(content, "Chunk")
    start, end, size, crc = c.u64(), c.u64(), c.u64(), c.u32()
    compression = c.string()
    records = c.raw(c.u64())
    return _ParsedChunk(start, end, size, crc, compression, records)


def _iter_records(data: bytes, what: str) -> Iterator[tuple[int, bytes]]:
    pos = 0
    while pos < len(data):
        if pos + 9 > len(data):
            raise McapError(f"{what}: truncated record header")
        opcode = data[pos]
        length = struct.unpack_from("<Q", data, pos + 1)[0]
        pos += 9
        if pos + length > len(data):
            raise McapError(f"{what}: record 0x{opcode:02x} runs past the end")
        yield opcode, data[pos : pos + length]
        pos += length


def _widen(spans: dict[int, tuple[int, int]], channel_id: int, log_time: int) -> None:
    span = spans.get(channel_id)
    if span is None:
        spans[channel_id] = (log_time, log_time)
    elif log_time < span[0] or log_time > span[1]:
        spans[channel_id] = (min(span[0], log_time), max(span[1], log_time))


class McapReader:
    """Read schemas, channels and messages from an MCAP file."""

    def __init__(self, path: str | os.PathLike[str]) -> None:
        self.path = Path(path)
        self.size = self.path.stat().st_size
        self.profile = ""
        self.library = ""
        self.schemas: dict[int, McapSchema] = {}
        self.channels: dict[int, McapChannel] = {}
        self.statistics: McapStatistics | None = None
        self.chunk_indexes: list[ChunkIndex] = []
        self.has_summary = False
        self.issues: list[str] = []
        self.compressions: set[str] = set()
        self._scanned = False
        self._spans: dict[int, tuple[int, int]] | None = None
        with self.path.open("rb") as f:
            if f.read(8) != MAGIC:
                raise McapError(f"{self.path.name}: not an MCAP file (bad magic)")
            record = self._read_record(f, 8)
            if record is None or record[0] != OP_HEADER:
                raise McapError(f"{self.path.name}: missing Header record")
            header = _Cursor(record[1], "Header")
            self.profile = header.string()
            self.library = header.string()
            self._read_summary(f)
        if not self.has_summary:
            self._linear_metadata()

    # -- low-level ---------------------------------------------------------------------------
    def _read_record(self, f: BinaryIO, offset: int) -> tuple[int, bytes, int] | None:
        """Read the record at ``offset``; ``None`` at EOF; records an issue if truncated."""
        f.seek(offset)
        head = f.read(9)
        if len(head) == 0:
            return None
        if len(head) < 9:
            self._truncated(offset)
            return None
        opcode = head[0]
        length = struct.unpack_from("<Q", head, 1)[0]
        if length > MAX_RECORD or offset + 9 + length > self.size:
            self._truncated(offset)
            return None
        content = f.read(length)
        return opcode, content, offset + 9 + length

    def _truncated(self, offset: int) -> None:
        message = (
            f"file ends inside a record at byte {offset} of {self.size}: the recording was "
            "probably interrupted; data after this point is lost"
        )
        if message not in self.issues:
            self.issues.append(message)

    # -- summary -----------------------------------------------------------------------------
    def _read_summary(self, f: BinaryIO) -> None:
        if self.size < 8 + _FOOTER_SIZE + 8:
            return
        footer_pos = self.size - 8 - _FOOTER_SIZE
        f.seek(footer_pos)
        tail = f.read(_FOOTER_SIZE + 8)
        if tail[-8:] != MAGIC or tail[0] != OP_FOOTER:
            self.issues.append(
                "no footer at the end of the file: the recording was not closed cleanly; "
                "reading the data section linearly"
            )
            return
        summary_start, _summary_offset_start, summary_crc = struct.unpack_from("<QQI", tail, 9)
        if summary_start == 0:
            return
        if summary_start >= footer_pos:
            self.issues.append("footer points to a summary section outside the file")
            return
        f.seek(summary_start)
        summary = f.read(footer_pos - summary_start)
        if summary_crc != 0:
            crc = zlib.crc32(summary + tail[: 1 + 8 + 16])
            if crc != summary_crc:
                self.issues.append("summary section CRC mismatch; ignoring the summary")
                return
        schemas: dict[int, McapSchema] = {}
        channels: dict[int, McapChannel] = {}
        chunk_indexes: list[ChunkIndex] = []
        statistics: McapStatistics | None = None
        try:
            for opcode, content in _iter_records(summary, "summary section"):
                if opcode == OP_SCHEMA:
                    schema = _parse_schema(content)
                    schemas[schema.id] = schema
                elif opcode == OP_CHANNEL:
                    channel = _parse_channel(content)
                    channels[channel.id] = channel
                elif opcode == OP_CHUNK_INDEX:
                    chunk_indexes.append(_parse_chunk_index(content))
                elif opcode == OP_STATISTICS:
                    statistics = _parse_statistics(content)
        except McapError as exc:
            self.issues.append(f"unreadable summary section ({exc}); reading linearly")
            return
        if not channels and (statistics is None or statistics.message_count > 0):
            # Some writers put only statistics in the summary; channels must come from data.
            self.statistics = statistics
            return
        self.schemas, self.channels = schemas, channels
        self.chunk_indexes, self.statistics = chunk_indexes, statistics
        self.compressions = {ci.compression for ci in chunk_indexes}
        self.has_summary = True

    # -- linear scan -------------------------------------------------------------------------
    def _linear_records(self) -> Iterator[tuple[int, bytes]]:
        """Yield data-section records, expanding chunks; stops at DataEnd/summary/EOF."""
        with self.path.open("rb") as f:
            offset = 8
            while True:
                record = self._read_record(f, offset)
                if record is None:
                    return
                opcode, content, offset = record
                if opcode in (OP_DATA_END, OP_FOOTER):
                    return
                if opcode == OP_CHUNK:
                    chunk = _parse_chunk(content)
                    self.compressions.add(chunk.compression)
                    records = self._chunk_records(chunk, offset)
                    if records is None:
                        continue
                    yield from _iter_records(records, f"chunk ending at byte {offset}")
                else:
                    yield opcode, content

    def _chunk_records(self, chunk: _ParsedChunk, where: int) -> bytes | None:
        records = decompress_chunk(chunk.compression, chunk.records, chunk.uncompressed_size)
        if chunk.uncompressed_crc != 0 and zlib.crc32(records) != chunk.uncompressed_crc:
            message = f"chunk near byte {where} failed its CRC check; its messages were skipped"
            if message not in self.issues:
                self.issues.append(message)
            return None
        return records

    def _linear_metadata(self) -> None:
        counts: dict[int, int] = {}
        spans: dict[int, tuple[int, int]] = {}
        first = last = None
        total = 0
        for opcode, content in self._linear_records():
            if opcode == OP_SCHEMA:
                schema = _parse_schema(content)
                self.schemas[schema.id] = schema
            elif opcode == OP_CHANNEL:
                channel = _parse_channel(content)
                self.channels[channel.id] = channel
            elif opcode == OP_MESSAGE:
                c = _Cursor(content, "Message")
                channel_id = c.u16()
                c.u32()
                log_time = c.u64()
                counts[channel_id] = counts.get(channel_id, 0) + 1
                _widen(spans, channel_id, log_time)
                total += 1
                first = log_time if first is None else min(first, log_time)
                last = log_time if last is None else max(last, log_time)
        self._scanned = True
        self._spans = spans
        if self.statistics is None:
            self.statistics = McapStatistics(
                total,
                len(self.schemas),
                len(self.channels),
                0,
                0,
                0,
                first or 0,
                last or 0,
                counts,
            )

    # -- per-channel time spans --------------------------------------------------------------
    def channel_time_spans(self) -> dict[int, tuple[int, int]]:
        """First and last ``log_time`` of every channel that has messages.

        Taken from the MessageIndex records when every chunk in the chunk index has them (no
        chunk is read or decompressed), from the linear scan done at opening for files without
        a summary, and otherwise from one pass over the messages."""
        if self._spans is None:
            try:
                spans = self._message_index_spans()
            except McapError:  # malformed MessageIndex record
                spans = None
            if spans is None:
                spans = {}
                for msg in self.messages():
                    _widen(spans, msg.channel_id, msg.log_time)
            self._spans = spans
        return self._spans

    def _message_index_spans(self) -> dict[int, tuple[int, int]] | None:
        if not (self.has_summary and self.chunk_indexes):
            return None
        spans: dict[int, tuple[int, int]] = {}
        indexed: dict[int, int] = {}
        with self.path.open("rb") as f:
            for ci in self.chunk_indexes:
                if not ci.message_index_offsets:
                    return None  # chunk written without message indexes
                for channel_id, offset in ci.message_index_offsets.items():
                    f.seek(offset)
                    head = f.read(9)
                    if len(head) < 9 or head[0] != OP_MESSAGE_INDEX:
                        return None  # not a MessageIndex: fall back to reading the messages
                    length = struct.unpack_from("<Q", head, 1)[0]
                    if offset + 9 + length > self.size:
                        return None
                    c = _Cursor(f.read(length), "MessageIndex")
                    if c.u16() != channel_id:
                        return None
                    entries = c.raw(c.u32())
                    count = len(entries) // 16
                    indexed[channel_id] = indexed.get(channel_id, 0) + count
                    if count == 0:
                        continue
                    times = struct.unpack_from(f"<{2 * count}Q", entries)[0::2]
                    _widen(spans, channel_id, min(times))
                    _widen(spans, channel_id, max(times))
        stats = self.statistics
        if stats is not None and any(
            indexed.get(channel_id, 0) != count
            for channel_id, count in stats.channel_message_counts.items()
        ):
            return None  # the indexes miss messages the statistics count: read the messages
        return spans

    # -- messages ----------------------------------------------------------------------------
    def messages(self, channel_ids: set[int] | None = None) -> Iterator[McapMessage]:
        """Yield messages. With a chunk index they come in ``log_time`` order, otherwise in
        file order."""
        if self.has_summary and self.chunk_indexes:
            yield from self._indexed_messages(channel_ids)
            return
        for opcode, content in self._linear_records():
            if opcode == OP_MESSAGE:
                msg = _parse_message(content)
                if channel_ids is None or msg.channel_id in channel_ids:
                    yield msg

    def _indexed_messages(self, channel_ids: set[int] | None) -> Iterator[McapMessage]:
        chunks = sorted(
            self.chunk_indexes, key=lambda ci: (ci.message_start_time, ci.chunk_start_offset)
        )
        if channel_ids is not None:
            chunks = [
                ci
                for ci in chunks
                if not ci.message_index_offsets or channel_ids & set(ci.message_index_offsets)
            ]
        heap: list[tuple[int, int, McapMessage]] = []
        counter = 0
        with self.path.open("rb") as f:
            for index, ci in enumerate(chunks):
                record = self._read_record(f, ci.chunk_start_offset)
                if record is None or record[0] != OP_CHUNK:
                    self.issues.append(
                        f"chunk index points to byte {ci.chunk_start_offset}, which is not a chunk"
                    )
                    continue
                chunk = _parse_chunk(record[1])
                records = self._chunk_records(chunk, record[2])
                if records is not None:
                    for opcode, content in _iter_records(records, "chunk"):
                        if opcode != OP_MESSAGE:
                            continue
                        msg = _parse_message(content)
                        if channel_ids is None or msg.channel_id in channel_ids:
                            heapq.heappush(heap, (msg.log_time, counter, msg))
                            counter += 1
                horizon = chunks[index + 1].message_start_time if index + 1 < len(chunks) else None
                while heap and (horizon is None or heap[0][0] < horizon):
                    yield heapq.heappop(heap)[2]
        while heap:
            yield heapq.heappop(heap)[2]
