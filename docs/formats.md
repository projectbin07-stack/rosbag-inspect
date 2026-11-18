# File formats

`rosbag-inspect` reads ROS 2 bags without ROS. Everything below was implemented from the public
descriptions listed under each heading. No code was taken from `rosbags`, the `mcap` libraries,
`rosbag2` or other readers; their source was not copied, and where the rosbag2 C++ source was
read it was only to learn the database schema and YAML keys it writes.

## rosbag2 bag directory and `metadata.yaml`

A bag is a directory with one or more storage files (`<name>_0.db3`, `<name>_0.mcap`, ... when
split) and a `metadata.yaml` written when the recorder closes cleanly. If the recorder is killed,
`metadata.yaml` is missing; `ros2 bag reindex` can rebuild it. `rosbag-inspect` reports this and
reads the storage files directly.

Keys used (all under `rosbag2_bagfile_information`):

| Key | Meaning |
| --- | --- |
| `version` | metadata version (4 on Foxy/Galactic, 5 on Humble, higher on newer releases) |
| `storage_identifier` | `sqlite3` or `mcap` |
| `duration.nanoseconds`, `starting_time.nanoseconds_since_epoch` | time span |
| `message_count` | total messages |
| `topics_with_message_count[].topic_metadata.{name,type,serialization_format,offered_qos_profiles}` | topics; `offered_qos_profiles` is a YAML string on Humble, a list in some newer versions |
| `topics_with_message_count[].message_count` | per-topic count, compared with the storage |
| `compression_format`, `compression_mode` | `""`, or `zstd` with `FILE` / `MESSAGE` (version >= 3) |
| `relative_file_paths` | storage files in order |

The YAML is parsed by a small purpose-built parser (`yamlmini.py`) that only builds dicts, lists,
strings, numbers, booleans and `None`. It supports the block style written by yaml-cpp,
quoted strings with escapes (including multi-line double-quoted strings), block scalars and flat
flow collections, and rejects anchors, aliases and tags.

References:
- rosbag2 README, "Bag file metadata": <https://github.com/ros2/rosbag2#readme>
- rosbag2 `metadata_io.cpp` (keys written and the version gates when reading),
  <https://github.com/ros2/rosbag2/blob/humble/rosbag2_storage/src/rosbag2_storage/metadata_io.cpp>
- YAML 1.2.2 specification, <https://yaml.org/spec/1.2.2/>

## SQLite3 storage (`.db3`)

Tables written by the `sqlite3` storage plugin (Humble):

```sql
CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
CREATE TABLE metadata(id INTEGER PRIMARY KEY, metadata_version INTEGER NOT NULL,
                      metadata TEXT NOT NULL);
CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,
                    serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,
                      timestamp INTEGER NOT NULL, data BLOB NOT NULL);
CREATE INDEX timestamp_idx ON messages (timestamp ASC);
```

- Foxy (schema version 1) has no `offered_qos_profiles` column and no `schema` table; schema
  version 2 adds the column; version 3 adds the `schema` table with the distribution name. Newer
  releases add `type_description_hash` to `topics`. The reader picks columns by name.
- `messages.timestamp` is the receive time at the recorder in nanoseconds since the epoch.
  `data` is the CDR-serialized message (with its 4-byte encapsulation header).
- The database carries **no message definitions**, so only types in the built-in table can be
  decoded from `.db3` files (see below).
- Messages are read with `ORDER BY timestamp, id`, the order rosbag2 replays them in. For
  `sensor_msgs/Image` and `CompressedImage` only the first 4 KiB of each blob is loaded
  (`substr(data, 1, 4096)`): enough for the header and size fields, never the pixels.

Reference: rosbag2 `sqlite_storage.cpp`,
<https://github.com/ros2/rosbag2/blob/humble/rosbag2_storage_default_plugins/src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.cpp>

## MCAP

Specification: <https://mcap.dev/spec> (format version 0); well-known encodings:
<https://mcap.dev/spec/registry>.

Layout: `magic (0x89 "MCAP0" \r \n)`, `Header`, data section, optional `DataEnd`, optional summary
section, optional summary offset section, `Footer`, magic. Every record is `opcode: uint8`,
`length: uint64`, content. Integers are little-endian; strings are `uint32` length + UTF-8;
maps and arrays are prefixed with their `uint32` byte length.

| Opcode | Record | Fields used |
| --- | --- | --- |
| 0x01 | Header | profile, library |
| 0x02 | Footer | summary_start, summary_offset_start, summary_crc |
| 0x03 | Schema | id, name, encoding, data |
| 0x04 | Channel | id, schema_id, topic, message_encoding, metadata (`offered_qos_profiles`) |
| 0x05 | Message | channel_id, sequence, log_time, publish_time, data |
| 0x06 | Chunk | message_start/end_time, uncompressed_size, uncompressed_crc, compression, records |
| 0x07 | Message Index | (written by the test writer; the reader uses chunk indexes) |
| 0x08 | Chunk Index | message start/end time, chunk_start_offset, chunk_length, message_index_offsets, compression, sizes |
| 0x0B | Statistics | message_count, chunk_count, start/end time, channel_message_counts |
| 0x0E | Summary Offset | (skipped) |
| 0x0F | Data End | marks the end of the data section |

Attachment (0x09), Attachment Index (0x0A), Metadata (0x0C) and Metadata Index (0x0D) records are
skipped.

Reading strategy:
1. Check the leading magic and the `Header`.
2. Read the `Footer` 37 bytes before the end. If `summary_start` is non-zero, verify
   `summary_crc` (CRC-32 of the summary section plus the footer up to `summary_offset_start`) when
   it is non-zero, then load schemas, channels, statistics and chunk indexes from the summary.
3. With chunk indexes, messages are read chunk by chunk in `message_start_time` order. Chunks
   whose `message_index_offsets` contain none of the requested channels are not read at all.
   Overlapping chunks are merged with a heap so messages come out in `log_time` order.
4. Without a usable summary (no footer, `summary_start = 0`, CRC mismatch, or a file cut off by
   an interrupted recording) the data section is scanned linearly up to `DataEnd` or the end of
   the readable data. A record whose length runs past the end of the file is reported as an
   interrupted recording.
5. `uncompressed_crc` of each chunk is verified when non-zero; a failing chunk is skipped and
   reported.

Chunk compression:
- `""`: none.
- `lz4`: LZ4 frame format, decoded by a pure-Python implementation of the LZ4 frame and block
  formats (<https://github.com/lz4/lz4/blob/dev/doc/lz4_Frame_format.md>,
  <https://github.com/lz4/lz4/blob/dev/doc/lz4_Block_format.md>). Header and content checksums
  (xxHash32) are not verified; the MCAP chunk CRC-32 is. Linked and independent blocks,
  uncompressed blocks, block checksums and skippable frames are handled; dictionaries are not.
  The decoder was checked locally against frames produced by the reference `lz4` library
  (python-lz4 4.4.5) with several block/checksum settings; that library is not a dependency.
- `zstd`: Zstandard has no practical pure-Python decoder in the standard library before Python
  3.14. On Python 3.14+ the standard-library `compression.zstd` module (PEP 784) is used; on
  older interpreters a zstd chunk raises a clear error that names the alternatives
  (run with Python 3.14+, or convert the bag to uncompressed chunks first).

ROS 2 profile: header profile `ros2`, channel `message_encoding` `cdr`, schema encoding
`ros2msg` (or `ros2idl`, which is not parsed). A `ros2msg` schema is the top-level `.msg` text
followed by each dependency as a line of 80 `=`, a line `MSG: package/Name`, and the dependency's
`.msg` text.

## `.msg` definitions

Reference: ROS 2 interface definition design article,
<https://design.ros2.org/articles/legacy_interface_definition.html>, and
<https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html>.

- Field line: `type name [default]`; constant line: `type NAME=value`; `#` starts a comment.
- Primitive types: `bool`, `byte`, `char`, `int8`..`int64`, `uint8`..`uint64`, `float32`,
  `float64`, `string`, `wstring`. In ROS 2 `char` is an unsigned 8-bit value and `byte` an octet.
- Arrays: `T[N]` (fixed), `T[]` (unbounded sequence), `T[<=N]` (bounded sequence); bounded
  strings `string<=N`.
- Type names are normalised to `package/msg/Name`; a bare `Name` refers to the same package
  (and `Header` to `std_msgs/msg/Header`, for old definitions).
- A message with no fields is given one `uint8 structure_needs_at_least_one_member` field,
  matching what the ROS 2 IDL generators do, so empty messages (e.g. `test_msgs/Constants`)
  occupy one byte on the wire.

Built-in definitions (for `.db3` bags, which carry none): `builtin_interfaces/Time`,
`Duration`; `std_msgs/Header`, `String`; `geometry_msgs/Vector3`, `Point`, `Quaternion`, `Pose`,
`PoseStamped`, `PoseWithCovariance`, `Twist`, `TwistStamped`, `TwistWithCovariance`,
`Transform`, `TransformStamped`; `sensor_msgs/LaserScan`, `Imu`, `JointState`, `Image`,
`CompressedImage`; `nav_msgs/Odometry`; `tf2_msgs/TFMessage`. Definitions from MCAP schemas take
precedence over these.

## CDR (as used by ROS 2)

References:
- OMG DDS-XTypes 1.3, section 7.4 (data representation, XCDR1/XCDR2),
  <https://www.omg.org/spec/DDS-XTypes/1.3/>
- OMG DDSI-RTPS 2.5, chapter 10 (serialized payload and representation identifiers),
  <https://www.omg.org/spec/DDSI-RTPS/2.5/>
- OMG CORBA 3.4 Part 2, section 9.3 (Common Data Representation and alignment),
  <https://www.omg.org/spec/CORBA/3.4/>

Rules implemented:
- Every payload starts with a 4-byte encapsulation header: a 2-byte representation identifier
  (big-endian) and 2 option bytes. Supported: `0x0000` CDR_BE, `0x0001` CDR_LE (what ROS 2 with
  Fast DDS or Cyclone DDS writes), `0x0006` CDR2_BE and `0x0007` CDR2_LE (plain XCDR2, for final
  types). Parameter-list (`PL_CDR*`) and delimited (`D_CDR2*`) encodings are rejected.
- Alignment is measured from the first byte after the header. Each primitive is aligned to its
  size; the largest alignment is 8 in XCDR1 and 4 in XCDR2.
- `bool`, `byte`, `char`, `int8`, `uint8`: 1 byte. Nested messages add no padding of their own.
- `string`: `uint32` length counting the terminating NUL, the bytes, the NUL.
- Sequences: `uint32` element count, then the elements; fixed arrays have no count. An empty
  sequence adds no padding after its count.
- `wstring` is not supported (it is rare in robot data and its encoding differs between
  middleware implementations).
- `uint8[]`/`byte[]` arrays longer than 4096 bytes are returned as their length only, so image
  and point-cloud payloads are never copied into Python objects.

Compatibility evidence: the decoder consumes each message of the rosbag2 test bags listed in the
README exactly (at most 3 bytes of end padding left over), including `test_msgs/Arrays` with
nested, fixed-size, string and empty-message arrays.
