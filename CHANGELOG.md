# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [0.2.0] - 2026-11-25

### Added

- `export --arrays`: every numeric variable-length array (LaserScan `ranges` and `intensities`,
  JointState `position` / `velocity` / `effort`, ...) is also written in full, in a column of its
  own name, as space-separated numbers. Floats use Python's shortest round-trip form, so `inf`,
  `-inf` and `nan` are kept and `float()` reads every value back exactly. The default export is
  unchanged. Library: `flatten(..., arrays=True)`, `flat_columns(..., arrays=True)`,
  `array_cell()`, `number_text()`.
- `echo --nonfinite null`: NaN and infinities as JSON `null` instead of the strings `"NaN"`,
  `"Infinity"`, `"-Infinity"` (the default, unchanged). Library: `to_jsonable(value, nonfinite=)`.
- Optional `zstd` extra (`pip install "rosbag-inspect[zstd]"`, the `zstandard` package): zstd
  MCAP chunks and per-message zstd compression on Python 3.10-3.13 (3.14+ keeps using the
  standard library's `compression.zstd`). Frames without a content size are read too.

### Fixed

- A LaserScan could not leave the tool with its beams: `export` kept only `ranges.length` and
  `echo` wrote infinities as text (ecosystem audit finding 20).
- The zstd error message now names both ways to read such bags.

## [0.1.0] - 2026-11-19

### Added

- Readers for rosbag2 bag directories, SQLite3 (`.db3`, schema versions 1 to current) and MCAP
  files (summary and chunk index when present, linear scan otherwise, chunk CRC verification,
  recovery of files cut off by an interrupted recording), with split bags merged in time order.
- Pure-Python LZ4 frame decoder for LZ4-compressed MCAP chunks; zstd chunks and per-message
  zstd compression through `compression.zstd` on Python 3.14+, with a clear error elsewhere.
- Safe YAML subset parser for `metadata.yaml`.
- `.msg` parser (constants, defaults, fixed/bounded/unbounded arrays, bounded strings, empty
  messages), `ros2msg` schema bundles and built-in definitions for common robot types.
- CDR decoder and encoder (XCDR1 little/big endian, plain XCDR2) driven by the definitions;
  large byte arrays are skipped instead of copied.
- Checks: rates, jitter, gaps, expected rates, stamp latency, future/zero/duplicate/out-of-order
  stamps, simulation-time detection, frame_id consistency, TF tree (missing frames, broken
  chains, multiple parents, cycles, static vs. dynamic), LaserScan, IMU (gravity, quaternion,
  covariance conventions), odometry (teleports, twist vs. pose motion), cmd_vel response,
  Image size fields and JointState array lengths, incomplete recordings and stale metadata.
- `rosbag-inspect info | check | echo | export` with text and JSON output and CI exit codes.
- Self-contained HTML report with inline-SVG rate timelines, gap markers, TF tree diagram and
  latency histograms, light and dark themes, no JavaScript.
- Test fixture writers (SQLite3, MCAP, LZ4) and a simulated drive with deliberate faults.

[0.2.0]: https://github.com/projectbin07-stack/rosbag-inspect/releases/tag/v0.2.0
[0.1.0]: https://github.com/projectbin07-stack/rosbag-inspect/releases/tag/v0.1.0
