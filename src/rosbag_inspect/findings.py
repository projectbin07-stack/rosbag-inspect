"""Findings, per-topic statistics and the overall report model."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

ERROR = "error"
WARNING = "warning"
INFO = "info"
SEVERITY_ORDER = {ERROR: 0, WARNING: 1, INFO: 2}

PASS = "pass"
WARN = "warn"
FAIL = "fail"


@dataclass
class Finding:
    """One problem or observation. ``topic`` is empty for bag-level findings."""

    code: str
    severity: str
    topic: str
    title: str
    detail: str
    count: int = 1
    items: list[str] = field(default_factory=list)
    data: dict[str, Any] = field(default_factory=dict)


@dataclass
class Gap:
    start_ns: int
    end_ns: int

    @property
    def duration_s(self) -> float:
        return (self.end_ns - self.start_ns) / 1e9


@dataclass
class Histogram:
    edges: list[float]
    counts: list[int]


@dataclass
class LatencyStats:
    """Receive time minus ``header.stamp`` in seconds, over the usable samples."""

    samples: int
    mean_s: float
    min_s: float
    max_s: float
    p50_s: float
    p95_s: float
    histogram: Histogram


@dataclass
class TopicStats:
    name: str
    type: str
    count: int
    decoded: bool = False
    first_ns: int | None = None
    last_ns: int | None = None
    rate_avg_hz: float | None = None
    rate_min_hz: float | None = None
    rate_max_hz: float | None = None
    period_median_s: float | None = None
    jitter_s: float | None = None
    expected_hz: float | None = None
    gaps: list[Gap] = field(default_factory=list)
    gap_threshold_s: float | None = None
    latency: LatencyStats | None = None
    frame_ids: dict[str, int] = field(default_factory=dict)
    timeline_bin_s: float = 1.0
    timeline: list[float] = field(default_factory=list)


@dataclass
class TfEdge:
    parent: str
    child: str
    static: bool
    count: int
    topics: list[str]


@dataclass
class TfSummary:
    edges: list[TfEdge] = field(default_factory=list)
    frames: list[str] = field(default_factory=list)
    components: list[list[str]] = field(default_factory=list)
    referenced: dict[str, list[str]] = field(default_factory=dict)
    missing: list[str] = field(default_factory=list)
    unreachable: list[str] = field(default_factory=list)
    main_root: str = ""


@dataclass
class Report:
    tool_version: str
    bag: str
    storage: str
    files: list[str]
    start_ns: int | None
    end_ns: int | None
    message_count: int
    topics: list[TopicStats]
    findings: list[Finding]
    tf: TfSummary
    details: dict[str, str] = field(default_factory=dict)
    options: dict[str, Any] = field(default_factory=dict)

    @property
    def duration_s(self) -> float:
        if self.start_ns is None or self.end_ns is None:
            return 0.0
        return (self.end_ns - self.start_ns) / 1e9

    def counts(self) -> dict[str, int]:
        result = {ERROR: 0, WARNING: 0, INFO: 0}
        for finding in self.findings:
            result[finding.severity] += 1
        return result

    @property
    def verdict(self) -> str:
        counts = self.counts()
        if counts[ERROR]:
            return FAIL
        if counts[WARNING]:
            return WARN
        return PASS

    def codes(self) -> list[tuple[str, str]]:
        """``(code, topic)`` pairs, handy in tests."""
        return [(f.code, f.topic) for f in self.findings]

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["duration_s"] = self.duration_s
        data["verdict"] = self.verdict
        data["summary"] = self.counts()
        return data


def sort_findings(findings: list[Finding]) -> list[Finding]:
    return sorted(findings, key=lambda f: (SEVERITY_ORDER[f.severity], f.topic, f.code))
