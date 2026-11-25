"""MCAP, SQLite3 and bag-directory readers on generated files."""

from __future__ import annotations

import importlib
import shutil
import sqlite3
import struct
from pathlib import Path
from types import ModuleType

import pytest

from bagwriter import Record, write_mcap, write_sqlite_bag
from rosbag_inspect import bag as bag_module
from rosbag_inspect import mcap as mcap_module
from rosbag_inspect.analysis import analyze
from rosbag_inspect.bag import BagError, open_bag
from rosbag_inspect.cdr import encode
from rosbag_inspect.mcap import McapError, McapReader, decompress_chunk
from rosbag_inspect.msgdef import Registry
from rosbag_inspect.sqlite_storage import SqliteBagError, SqliteReader

REG = Registry()
S = 1_000_000_000


def _string_records(topic: str, times: list[int]) -> list[Record]:
    return [
        Record(
            topic, "std_msgs/msg/String", t, encode(REG, "std_msgs/msg/String", {"data": f"m{t}"})
        )
        for t in times
    ]


def _payloads(path: Path, topic: str | None = None) -> list[tuple[str, int]]:
    with open_bag(path) as bag:
        return [(m.topic, m.log_time) for m in bag.messages(None if topic is None else [topic])]


# -- MCAP ----------------------------------------------------------------------------------------


def test_mcap_summary_statistics_and_index(fixtures: dict[str, Path]) -> None:
    reader = McapReader(fixtures["mcap"])
    assert reader.has_summary
    assert reader.profile == "ros2"
    assert reader.statistics is not None
    assert reader.statistics.message_count == 5482
    assert len(reader.chunk_indexes) == reader.statistics.chunk_count > 1
    assert {c.topic for c in reader.channels.values()} >= {"/scan", "/imu", "/tf_static"}
    assert {s.encoding for s in reader.schemas.values()} == {"ros2msg"}
    times = [m.log_time for m in reader.messages()]
    assert times == sorted(times)
    assert reader.issues == []


def test_mcap_layouts_yield_the_same_messages(fixtures: dict[str, Path]) -> None:
    reference = sorted(_payloads(fixtures["sqlite"]))
    for layout in ("mcap", "lz4", "unchunked", "nosummary"):
        assert sorted(_payloads(fixtures[layout])) == reference, layout


def test_mcap_topic_filter_uses_chunk_index(tmp_path: Path) -> None:
    # Two topics written in separate time ranges, one chunk each: filtering by topic must
    # skip the other chunk entirely.
    recs = _string_records("/a", [S + i for i in range(50)]) + _string_records(
        "/b", [2 * S + i for i in range(50)]
    )
    path = tmp_path / "two.mcap"
    write_mcap(path, recs, chunk_size=1)
    reader = McapReader(path)
    assert len(reader.chunk_indexes) == 100
    b_id = next(c.id for c in reader.channels.values() if c.topic == "/b")
    assert [m.channel_id for m in reader.messages({b_id})] == [b_id] * 50
    assert _payloads(path, "/a") == [("/a", S + i) for i in range(50)]


def test_mcap_overlapping_chunks_are_merged_in_time_order(tmp_path: Path) -> None:
    times = [5, 1, 7, 3, 9, 2, 8, 4, 6, 0]
    recs = _string_records("/x", [S + t for t in times])
    path = tmp_path / "unordered.mcap"
    write_mcap(path, recs, chunk_size=200)
    reader = McapReader(path)
    assert len(reader.chunk_indexes) > 1
    assert [m.log_time - S for m in reader.messages()] == sorted(times)


def _rate_records() -> list[Record]:
    """Hand-checked rates: /a 11 messages every 100 ms (span 1.0 s -> 10 / 1.0 = 10 Hz),
    /b 4 irregular messages at +0.25, +0.5, +1.0, +1.75 s (span 1.5 s -> 3 / 1.5 = 2 Hz),
    /c a single message (no rate)."""
    recs = _string_records("/a", [S + k * S // 10 for k in range(11)])
    recs += _string_records("/b", [S + S // 4, S + S // 2, 2 * S, S + 7 * S // 4])
    recs += _string_records("/c", [S + S // 3])
    return sorted(recs, key=lambda r: r.log_time)


RATE_SPANS = {
    "/a": (S, 2 * S),
    "/b": (S + S // 4, S + 7 * S // 4),
    "/c": (S + S // 3, S + S // 3),
}


@pytest.mark.parametrize(
    "layout",
    [
        {"chunk_size": 1},  # one chunk per message: spans come from many message indexes
        {},
        {"compression": "lz4", "chunk_size": 200},
        {"chunked": False},
        {"summary": False},
    ],
)
def test_mcap_per_topic_time_spans_and_rates(tmp_path: Path, layout: dict[str, object]) -> None:
    path = tmp_path / "rates.mcap"
    write_mcap(path, _rate_records(), **layout)  # type: ignore[arg-type]
    with open_bag(path) as bag:
        assert all(t.first_ns is None for t in bag.topics.values())
        bag.load_time_spans()
        bag.load_time_spans()  # idempotent
        assert {n: (t.first_ns, t.last_ns) for n, t in bag.topics.items()} == RATE_SPANS
        assert bag.rate_hz("/a") == pytest.approx(10.0)
        assert bag.rate_hz("/b") == pytest.approx(2.0)
        assert bag.rate_hz("/c") is None
        assert bag.issues == []


def test_mcap_time_spans_from_message_index_read_no_chunk(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "rates.mcap"
    write_mcap(path, _rate_records(), chunk_size=200)
    reader = McapReader(path)
    assert len(reader.chunk_indexes) > 1

    def no_chunks(*args: object) -> None:
        raise AssertionError("a chunk was read")

    monkeypatch.setattr(reader, "messages", no_chunks)
    names = {c.id: c.topic for c in reader.channels.values()}
    spans = {names[i]: span for i, span in reader.channel_time_spans().items()}
    assert spans == RATE_SPANS


@pytest.mark.parametrize(
    "damage",
    ["no_index", "bad_offset", "wrong_channel", "short", "past_end", "missing_channel"],
)
def test_mcap_time_spans_fall_back_to_messages(tmp_path: Path, damage: str) -> None:
    path = tmp_path / "rates.mcap"
    write_mcap(path, _rate_records(), chunk_size=200)
    reader = McapReader(path)
    ci = reader.chunk_indexes[0]
    channel_id, offset = next(iter(ci.message_index_offsets.items()))
    if damage == "no_index":
        ci.message_index_offsets = {}  # writers may omit message indexes
    elif damage == "bad_offset":
        ci.message_index_offsets[channel_id] = ci.chunk_start_offset  # points at the chunk
    elif damage == "wrong_channel":
        ci.message_index_offsets[channel_id + 100] = offset
    elif damage == "missing_channel":
        del ci.message_index_offsets[channel_id]  # statistics count messages no index lists
    elif damage == "past_end":
        data = bytearray(path.read_bytes())
        struct.pack_into("<Q", data, offset + 1, len(data))  # record length runs past the file
        path.write_bytes(bytes(data))
    else:
        data = bytearray(path.read_bytes())
        struct.pack_into("<I", data, offset + 9 + 2, 1 << 20)  # entry list runs past the record
        path.write_bytes(bytes(data))
    names = {c.id: c.topic for c in reader.channels.values()}
    spans = {names[i]: span for i, span in reader.channel_time_spans().items()}
    assert spans == RATE_SPANS


def test_split_mcap_time_spans_are_merged(tmp_path: Path) -> None:
    directory = tmp_path / "split"
    directory.mkdir()
    write_mcap(
        directory / "split_0.mcap", _string_records("/a", [S + k * S // 10 for k in range(6)])
    )
    write_mcap(
        directory / "split_1.mcap", _string_records("/a", [S + k * S // 10 for k in range(6, 11)])
    )
    with open_bag(directory) as bag:
        bag.load_time_spans()
        assert (bag.topics["/a"].first_ns, bag.topics["/a"].last_ns) == (S, 2 * S)
        assert bag.topics["/a"].message_count == 11
        assert bag.rate_hz("/a") == pytest.approx(10.0)


def test_mcap_truncated_file_is_read_linearly(fixtures: dict[str, Path], tmp_path: Path) -> None:
    data = fixtures["mcap"].read_bytes()
    path = tmp_path / "cut.mcap"
    path.write_bytes(data[: len(data) // 2])
    with open_bag(path) as bag:
        report = analyze(bag)
    assert any("interrupted" in i for i in bag.issues)
    assert ("recording_incomplete", "") in report.codes()
    assert 0 < report.message_count < 5482


def test_mcap_crc_mismatch_skips_chunk(tmp_path: Path) -> None:
    recs = _string_records("/x", [S + i for i in range(10)])
    path = tmp_path / "crc.mcap"
    write_mcap(path, recs, chunk_size=10_000, summary=False)
    raw = bytearray(path.read_bytes())
    index = raw.index(b"m1000000000")
    raw[index] ^= 0xFF  # corrupt a message payload inside the only chunk
    path.write_bytes(bytes(raw))
    reader = McapReader(path)
    assert list(reader.messages()) == []
    assert any("CRC" in issue for issue in reader.issues)


def test_mcap_summary_crc_mismatch_falls_back(tmp_path: Path) -> None:
    recs = _string_records("/x", [S + i for i in range(10)])
    path = tmp_path / "scrc.mcap"
    write_mcap(path, recs)
    raw = bytearray(path.read_bytes())
    raw[-12] ^= 0xFF  # inside summary_crc of the footer
    path.write_bytes(bytes(raw))
    reader = McapReader(path)
    assert not reader.has_summary
    assert any("summary section CRC" in issue for issue in reader.issues)
    assert len(list(reader.messages())) == 10


def test_mcap_without_schemas_and_without_crc(tmp_path: Path) -> None:
    recs = _string_records("/x", [S + i for i in range(3)])
    path = tmp_path / "noschema.mcap"
    write_mcap(path, recs, schemas=False, crc=False)
    with open_bag(path) as bag:
        assert bag.topics["/x"].type == ""
        assert not bag.decodable("/x")
        report = analyze(bag)
    assert ("not_decoded", "/x") in report.codes()


def test_mcap_bad_files(tmp_path: Path) -> None:
    bad = tmp_path / "bad.mcap"
    bad.write_bytes(b"not an mcap file at all")
    with pytest.raises(McapError, match="bad magic"):
        McapReader(bad)
    bad.write_bytes(mcap_module.MAGIC + b"\x02" + struct.pack("<Q", 0))
    with pytest.raises(McapError, match="missing Header"):
        McapReader(bad)
    junk = tmp_path / "junk.mcap"
    junk.write_bytes(b"xx")
    with pytest.raises(BagError, match="bad magic"):
        open_bag(junk)


def test_chunk_compression_support(monkeypatch: pytest.MonkeyPatch) -> None:
    assert decompress_chunk("", b"abc", 3) == b"abc"
    with pytest.raises(McapError, match="unsupported chunk compression 'brotli'"):
        decompress_chunk("brotli", b"", 0)
    with pytest.raises(McapError, match="lz4 chunk"):
        decompress_chunk("lz4", b"garbage", 10)
    monkeypatch.setattr(mcap_module, "_zstd_decompressor", lambda: None)
    with pytest.raises(
        McapError, match=r"zstd-compressed chunks: decompression needs Python 3\.14 .* zstandard"
    ):
        decompress_chunk("zstd", b"", 0)


def _without_stdlib_zstd(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make ``compression.zstd`` unimportable, as on Python 3.10-3.13."""
    real = importlib.import_module

    def fake(name: str, package: str | None = None) -> ModuleType:
        if name == "compression.zstd":
            raise ImportError(name)
        return real(name, package)

    monkeypatch.setattr(importlib, "import_module", fake)


def test_zstd_chunks_with_zstandard_package(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    zstandard = pytest.importorskip("zstandard")
    recs = _string_records("/z", [S + i for i in range(20)])
    path = tmp_path / "zstd.mcap"
    write_mcap(path, recs, compression="zstd", chunk_size=100)
    _without_stdlib_zstd(monkeypatch)
    func = mcap_module._zstd_decompressor()
    assert func is not None
    # A frame without a content size in its header (streaming writers) is read too.
    stream = zstandard.ZstdCompressor(write_content_size=False).compressobj()
    frame = stream.compress(b"abc" * 50) + stream.flush()
    assert func(frame) == b"abc" * 50
    assert [t for _, t in _payloads(path)] == [S + i for i in range(20)]


def test_zstd_unavailable_message_compression(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(mcap_module, "_zstd_decompressor", lambda: None)
    monkeypatch.setattr(bag_module, "_zstd_decompressor", lambda: None)
    with pytest.raises(BagError, match=r"per-message zstd compression.*rosbag-inspect\[zstd\]"):
        bag_module._message_decompressor("zstd")


def test_zstd_chunks_with_stdlib_decoder(tmp_path: Path) -> None:
    pytest.importorskip("compression.zstd")
    recs = _string_records("/z", [S + i for i in range(20)])
    path = tmp_path / "zstd.mcap"
    write_mcap(path, recs, compression="zstd", chunk_size=100)
    assert [t for _, t in _payloads(path)] == [S + i for i in range(20)]
    with pytest.raises(McapError, match="expected 99"):
        from compression import zstd  # type: ignore[import-not-found,unused-ignore]

        decompress_chunk("zstd", zstd.compress(b"x" * 10), 99)


# -- SQLite --------------------------------------------------------------------------------------


def test_sqlite_reader_basics(fixtures: dict[str, Path]) -> None:
    reader = SqliteReader(fixtures["sqlite"] / "drive_sqlite_0.db3")
    try:
        assert reader.schema_version == 3
        assert reader.ros_distro == "humble"
        names = {t.name: t for t in reader.topics}
        assert names["/scan"].type == "sensor_msgs/msg/LaserScan"
        assert "reliability" in names["/scan"].offered_qos_profiles
        stats = reader.topic_stats()
        assert stats[names["/scan"].id].count == 181
        image_id = names["/camera/image_raw"].id
        sizes = {len(d) for _, _, d in reader.messages({image_id}, {image_id})}
        assert sizes == {4096}  # only the prefix of each image is loaded
        assert list(reader.messages(set())) == []
    finally:
        reader.close()


def test_sqlite_foxy_schema_without_qos(tmp_path: Path) -> None:
    db = tmp_path / "foxy.db3"
    con = sqlite3.connect(db)
    con.executescript(
        "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT,"
        " serialization_format TEXT);"
        "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER,"
        " data BLOB);"
    )
    con.execute("INSERT INTO topics VALUES (1, '/chatter', 'std_msgs/String', 'cdr')")
    data = encode(REG, "std_msgs/msg/String", {"data": "hi"})
    con.execute("INSERT INTO messages VALUES (1, 1, 20, ?)", (data,))
    con.execute("INSERT INTO messages VALUES (2, 1, 10, ?)", (data,))
    con.commit()
    con.close()
    with open_bag(db) as bag:
        assert bag.details["sqlite3"] == "schema version 1"
        assert bag.topics["/chatter"].type == "std_msgs/msg/String"
        assert [m.log_time for m in bag.messages()] == [10, 20]


def test_sqlite_not_a_bag(tmp_path: Path) -> None:
    db = tmp_path / "other.db3"
    con = sqlite3.connect(db)
    con.execute("CREATE TABLE things(x)")
    con.close()
    with pytest.raises(SqliteBagError, match="missing table"):
        SqliteReader(db)
    junk = tmp_path / "junk.db3"
    junk.write_bytes(b"this is not sqlite" * 100)
    with pytest.raises(BagError, match="not a readable SQLite database"):
        open_bag(junk)


# -- bag directories -----------------------------------------------------------------------------


def test_directory_without_metadata(fixtures: dict[str, Path], tmp_path: Path) -> None:
    target = tmp_path / "nometa"
    target.mkdir()
    shutil.copy(fixtures["sqlite"] / "drive_sqlite_0.db3", target)
    with open_bag(target) as bag:
        assert bag.metadata is None
        assert any("metadata.yaml is missing" in i for i in bag.issues)
        assert bag.message_count == 5482


def test_metadata_count_mismatch_and_missing_file(tmp_path: Path) -> None:
    recs = _string_records("/a", [S + i * S // 10 for i in range(20)])
    directory = tmp_path / "bag"
    write_sqlite_bag(directory, recs, metadata_counts={"/a": 25})
    meta = directory / "metadata.yaml"
    meta.write_text(
        meta.read_text(encoding="utf-8").replace(
            "  relative_file_paths:\n", "  relative_file_paths:\n    - gone_1.db3\n"
        ),
        encoding="utf-8",
    )
    with open_bag(directory) as bag:
        report = analyze(bag)
    assert ("metadata_count_mismatch", "/a") in report.codes()
    assert any("gone_1.db3" in f.detail for f in report.findings)


def test_split_bag_is_merged_in_time_order(tmp_path: Path) -> None:
    directory = tmp_path / "split"
    write_sqlite_bag(directory, _string_records("/a", [S, 3 * S]), metadata=False)
    second = tmp_path / "tmp2"
    db = write_sqlite_bag(second, _string_records("/a", [2 * S, 4 * S]), metadata=False)
    shutil.move(db, directory / "split_1.db3")
    write_mcap(directory / "split_2.mcap", _string_records("/b", [int(2.5 * S)]))
    with open_bag(directory) as bag:
        assert bag.storage == "mcap+sqlite3"
        assert [m.log_time for m in bag.messages()] == [S, 2 * S, int(2.5 * S), 3 * S, 4 * S]
        assert bag.topics["/a"].message_count == 4
        assert (bag.start_ns, bag.end_ns) == (S, 4 * S)
        with pytest.raises(BagError, match="not in bag"):
            list(bag.messages(["/nope"]))


def test_compressed_and_missing_bags(tmp_path: Path) -> None:
    with pytest.raises(BagError, match="no such file"):
        open_bag(tmp_path / "missing")
    empty = tmp_path / "empty"
    empty.mkdir()
    with pytest.raises(BagError, match=r"no \.db3 or \.mcap"):
        open_bag(empty)
    zdir = tmp_path / "zbag"
    zdir.mkdir()
    (zdir / "zbag_0.db3.zstd").write_bytes(b"\x28\xb5\x2f\xfd")
    with pytest.raises(BagError, match="file-level compression"):
        open_bag(zdir)


def test_file_and_message_compression_modes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    directory = tmp_path / "cbag"
    write_sqlite_bag(directory, _string_records("/a", [S, 2 * S]))
    meta = directory / "metadata.yaml"
    text = meta.read_text(encoding="utf-8")
    meta.write_text(
        text.replace('compression_format: ""', "compression_format: zstd").replace(
            'compression_mode: ""', "compression_mode: FILE"
        ),
        encoding="utf-8",
    )
    with pytest.raises(BagError, match="file-level zstd"):
        open_bag(directory)
    meta.write_text(
        text.replace('compression_format: ""', "compression_format: lzma").replace(
            'compression_mode: ""', "compression_mode: MESSAGE"
        ),
        encoding="utf-8",
    )
    with pytest.raises(BagError, match="unsupported per-message compression 'lzma'"):
        open_bag(directory)


def test_message_compressed_sqlite_bag(tmp_path: Path) -> None:
    zstd = pytest.importorskip("compression.zstd")
    recs = [
        Record(r.topic, r.type, r.log_time, zstd.compress(r.data))
        for r in _string_records("/a", [S, 2 * S])
    ]
    directory = tmp_path / "mbag"
    write_sqlite_bag(directory, recs)
    meta = directory / "metadata.yaml"
    meta.write_text(
        meta.read_text(encoding="utf-8")
        .replace('compression_format: ""', "compression_format: zstd")
        .replace('compression_mode: ""', "compression_mode: MESSAGE"),
        encoding="utf-8",
    )
    with open_bag(directory) as bag:
        payloads = [m.data for m in bag.messages()]
    assert payloads == [r.data for r in _string_records("/a", [S, 2 * S])]


def test_bad_metadata_is_reported_not_fatal(fixtures: dict[str, Path], tmp_path: Path) -> None:
    target = tmp_path / "badmeta"
    shutil.copytree(fixtures["sqlite"], target)
    (target / "metadata.yaml").write_text("rosbag2_bagfile_information: [", encoding="utf-8")
    with open_bag(target) as bag:
        assert any("metadata.yaml could not be read" in i for i in bag.issues)
        assert bag.message_count == 5482
