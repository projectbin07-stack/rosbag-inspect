"""Optional checks against bags produced by real rosbag2 recorders.

These files are not part of the repository. To run the tests, download the rosbag2 test
resources listed in README.md ("Compatibility") into one directory and set
``ROSBAG_INSPECT_SAMPLES`` to it. Without the variable the tests are skipped.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from rosbag_inspect.analysis import analyze
from rosbag_inspect.bag import open_bag
from rosbag_inspect.cdr import decode, decode_with_size

SAMPLES = os.environ.get("ROSBAG_INSPECT_SAMPLES")
pytestmark = pytest.mark.skipif(not SAMPLES, reason="ROSBAG_INSPECT_SAMPLES not set")

BAGS = {
    "humble_cdr_test": 7,
    "sqlite_cdr_test": 7,
    "mcap_cdr_test": 7,
    "mcap_talker": 20,
    "sqlite_talker": 20,
    "mcap_events": 218,
    "wbag": 6074,
}


@pytest.mark.parametrize("name", sorted(BAGS))
def test_real_bag_reads_and_decodes(name: str) -> None:
    path = Path(str(SAMPLES)) / name
    if not path.exists():
        pytest.skip(f"{path} not downloaded")
    with open_bag(path) as bag:
        assert bag.message_count == BAGS[name]
        for raw in bag.messages():
            if bag.decodable(raw.topic):
                _, size = decode_with_size(
                    bag.registry, bag.topics[raw.topic].type, raw.data, max_bytes=None
                )
                # the definition must account for the whole payload (up to end padding)
                assert 0 <= len(raw.data) - size <= 3, raw.topic
        report = analyze(bag)
    codes = {f.code for f in report.findings}
    assert "decode_error" not in codes
    assert "metadata_count_mismatch" not in codes


def test_real_arrays_message_content() -> None:
    path = Path(str(SAMPLES)) / "mcap_cdr_test"
    if not path.exists():
        pytest.skip(f"{path} not downloaded")
    with open_bag(path) as bag:
        raw = next(iter(bag.messages(["/array_topic"])))
        msg = decode(bag.registry, bag.topics["/array_topic"].type, raw.data)
    assert msg["bool_values"] == [True, False, True]
    assert msg["string_values"] == ["Complex Hello1", "Complex Hello2", "Complex Hello3"]
    assert msg["string_values_default"] == ["", "max value", "min value"]
    assert msg["alignment_check"] == 0
