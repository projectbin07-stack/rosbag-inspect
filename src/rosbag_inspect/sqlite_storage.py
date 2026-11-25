"""Reader for rosbag2's ``sqlite3`` storage plugin (``.db3`` files).

Schema (rosbag2 sqlite_storage.cpp; ``offered_qos_profiles`` since schema version 2,
``type_description_hash`` in newer distributions)::

    topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT,
           offered_qos_profiles TEXT [, type_description_hash TEXT])
    messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)
    schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT)      -- optional
    metadata(id INTEGER PRIMARY KEY, metadata_version INTEGER, metadata TEXT)  -- optional
    message_definitions(id INTEGER PRIMARY KEY, topic_type TEXT, encoding TEXT,
           encoded_message_definition TEXT, type_description_hash TEXT)  -- schema 4+, optional

The database is opened read-only. Messages are read in ``(timestamp, id)`` order, which is
the order rosbag2 itself replays them in.
"""

from __future__ import annotations

import sqlite3
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

PREFIX_BYTES = 4096


class SqliteBagError(ValueError):
    """The file is not a readable rosbag2 SQLite database."""


@dataclass
class SqliteTopic:
    id: int
    name: str
    type: str
    serialization_format: str
    offered_qos_profiles: str


@dataclass
class SqliteTopicStats:
    count: int
    first: int
    last: int


class SqliteReader:
    def __init__(self, path: Path) -> None:
        self.path = path
        uri = path.resolve().as_uri() + "?mode=ro"
        try:
            self.connection = sqlite3.connect(uri, uri=True)
        except sqlite3.DatabaseError as exc:
            raise SqliteBagError(f"{path.name}: cannot open SQLite database ({exc})") from exc
        try:
            tables = {
                row[0]
                for row in self.connection.execute(
                    "SELECT name FROM sqlite_master WHERE type = 'table'"
                )
            }
        except sqlite3.DatabaseError as exc:
            self.connection.close()
            raise SqliteBagError(f"{path.name}: not a readable SQLite database ({exc})") from exc
        missing = {"topics", "messages"} - tables
        if missing:
            self.connection.close()
            raise SqliteBagError(
                f"{path.name}: not a rosbag2 database "
                f"(missing table(s) {', '.join(sorted(missing))})"
            )
        self.schema_version = 1
        self.ros_distro = ""
        columns = [row[1] for row in self.connection.execute("PRAGMA table_info(topics)")]
        if "offered_qos_profiles" in columns:
            self.schema_version = 2
        if "schema" in tables:
            row = self.connection.execute(
                "SELECT schema_version, ros_distro FROM schema ORDER BY schema_version DESC"
            ).fetchone()
            if row is not None:
                self.schema_version, self.ros_distro = int(row[0]), str(row[1])
        self.has_message_definitions = "message_definitions" in tables
        qos = "offered_qos_profiles" if "offered_qos_profiles" in columns else "''"
        self.topics = [
            SqliteTopic(int(r[0]), str(r[1]), str(r[2]), str(r[3]), str(r[4]))
            for r in self.connection.execute(
                f"SELECT id, name, type, serialization_format, {qos} FROM topics ORDER BY id"
            )
        ]

    def close(self) -> None:
        self.connection.close()

    def topic_stats(self) -> dict[int, SqliteTopicStats]:
        try:
            rows = self.connection.execute(
                "SELECT topic_id, COUNT(*), MIN(timestamp), MAX(timestamp) "
                "FROM messages GROUP BY topic_id"
            ).fetchall()
        except sqlite3.DatabaseError as exc:
            raise SqliteBagError(f"{self.path.name}: cannot read messages ({exc})") from exc
        return {int(r[0]): SqliteTopicStats(int(r[1]), int(r[2]), int(r[3])) for r in rows}

    def message_definitions(self) -> list[tuple[str, str, str]]:
        """``(topic_type, encoding, definition)`` rows of the ``message_definitions`` table that
        rosbag2 writes since schema version 4 (Iron); empty for older databases."""
        if not self.has_message_definitions:
            return []
        try:
            rows = self.connection.execute(
                "SELECT topic_type, encoding, encoded_message_definition "
                "FROM message_definitions ORDER BY id"
            ).fetchall()
        except sqlite3.DatabaseError as exc:
            raise SqliteBagError(
                f"{self.path.name}: cannot read message_definitions ({exc})"
            ) from exc
        return [(str(r[0]), str(r[1] or ""), str(r[2] or "")) for r in rows]

    def first_message(self, topic_id: int) -> bytes | None:
        """The complete payload of the earliest message on ``topic_id``, if any."""
        try:
            row = self.connection.execute(
                "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp, id LIMIT 1",
                (topic_id,),
            ).fetchone()
        except sqlite3.DatabaseError as exc:
            raise SqliteBagError(f"{self.path.name}: cannot read messages ({exc})") from exc
        return None if row is None else bytes(row[0])

    def messages(
        self, topic_ids: set[int] | None = None, prefix_only: set[int] | None = None
    ) -> Iterator[tuple[int, int, bytes]]:
        """Yield ``(topic_id, timestamp, data)``. For topics in ``prefix_only`` only the first
        :data:`PREFIX_BYTES` bytes of each message are loaded (used for image payloads)."""
        prefix = sorted(prefix_only or ())
        data_expr = "data"
        if prefix:
            ids = ", ".join(str(int(i)) for i in prefix)
            data_expr = (
                f"CASE WHEN topic_id IN ({ids}) THEN substr(data, 1, {PREFIX_BYTES}) ELSE data END"
            )
        query = f"SELECT topic_id, timestamp, {data_expr} FROM messages"
        if topic_ids is not None:
            if not topic_ids:
                return
            query += f" WHERE topic_id IN ({', '.join(str(int(i)) for i in sorted(topic_ids))})"
        query += " ORDER BY timestamp, id"
        try:
            for topic_id, timestamp, data in self.connection.execute(query):
                yield int(topic_id), int(timestamp), bytes(data)
        except sqlite3.DatabaseError as exc:
            raise SqliteBagError(f"{self.path.name}: cannot read messages ({exc})") from exc
