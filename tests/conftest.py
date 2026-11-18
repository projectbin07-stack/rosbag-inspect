from __future__ import annotations

from pathlib import Path

import pytest

from bagwriter import Record
from drive import build_drive
from make_fixtures import write_all


@pytest.fixture(scope="session")
def drive_records() -> list[Record]:
    return build_drive()


@pytest.fixture(scope="session")
def fixtures(tmp_path_factory: pytest.TempPathFactory) -> dict[str, Path]:
    """The simulated drive written in every supported storage layout."""
    return write_all(tmp_path_factory.mktemp("drive"))
