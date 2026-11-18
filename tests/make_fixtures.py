"""Write the simulated-drive fixtures to a directory.

Usage: ``python tests/make_fixtures.py OUT_DIR``. Creates ``drive_sqlite/`` (rosbag2 SQLite3
bag with metadata.yaml), ``drive.mcap`` (chunked, indexed), ``drive_lz4.mcap``,
``drive_unchunked.mcap`` and ``drive_nosummary.mcap``. Bags are generated, never committed.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from bagwriter import write_mcap, write_sqlite_bag
from drive import build_drive


def write_all(out: Path) -> dict[str, Path]:
    out.mkdir(parents=True, exist_ok=True)
    records = build_drive()
    paths = {
        "sqlite": out / "drive_sqlite",
        "mcap": out / "drive.mcap",
        "lz4": out / "drive_lz4.mcap",
        "unchunked": out / "drive_unchunked.mcap",
        "nosummary": out / "drive_nosummary.mcap",
    }
    write_sqlite_bag(paths["sqlite"], records)
    write_mcap(paths["mcap"], records)
    write_mcap(paths["lz4"], records, compression="lz4")
    write_mcap(paths["unchunked"], records, chunked=False)
    write_mcap(paths["nosummary"], records, summary=False)
    return paths


def main() -> int:
    if len(sys.argv) != 2:
        sys.stderr.write("usage: python tests/make_fixtures.py OUT_DIR\n")
        return 2
    for name, path in write_all(Path(sys.argv[1])).items():
        print(f"{name:10} {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
