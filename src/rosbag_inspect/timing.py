"""Per-topic timing: rates, jitter, gaps, header-stamp latency and stamp sanity."""

from __future__ import annotations

import itertools
import math
from collections.abc import Sequence

from .findings import ERROR, INFO, WARNING, Finding, Gap, Histogram, LatencyStats, TopicStats
from .options import Options

MAX_ITEMS = 20
TIMELINE_BINS = 600
HISTOGRAM_BINS = 20
WALL_CLOCK_MIN_NS = 1_000_000_000 * 1_000_000_000  # 2001-09-09 in ns since the Unix epoch


def rel(ns: int, origin: int) -> str:
    """Format a time as seconds after ``origin``."""
    return f"+{(ns - origin) / 1e9:.3f} s"


def percentile(sorted_values: Sequence[float], q: float) -> float:
    """Linear-interpolated percentile of an ascending, non-empty sequence."""
    if len(sorted_values) == 1:
        return sorted_values[0]
    pos = q * (len(sorted_values) - 1)
    low = math.floor(pos)
    high = min(low + 1, len(sorted_values) - 1)
    frac = pos - low
    return sorted_values[low] * (1 - frac) + sorted_values[high] * frac


def median(values: Sequence[float]) -> float:
    return percentile(sorted(values), 0.5)


def histogram(values: Sequence[float], bins: int = HISTOGRAM_BINS) -> Histogram:
    low, high = min(values), max(values)
    if high - low <= 0:
        return Histogram([low, low], [len(values)])
    width = (high - low) / bins
    counts = [0] * bins
    for v in values:
        counts[min(int((v - low) / width), bins - 1)] += 1
    return Histogram([low + i * width for i in range(bins + 1)], counts)


def timeline(recv: Sequence[int], start: int, end: int) -> tuple[float, list[float]]:
    """Message rate per time bin between ``start`` and ``end`` (receive times in ns)."""
    duration = max(end - start, 1)
    bin_ns = max(1_000_000_000, math.ceil(duration / TIMELINE_BINS))
    nbins = max(1, math.ceil(duration / bin_ns))
    counts = [0] * nbins
    for t in recv:
        counts[min(max((t - start) // bin_ns, 0), nbins - 1)] += 1
    rates = []
    for i, c in enumerate(counts):
        width = min(bin_ns, duration - i * bin_ns) if i == nbins - 1 else bin_ns
        rates.append(c / (max(width, 1) / 1e9))
    return bin_ns / 1e9, rates


def looks_like_sim_time(stamps: Sequence[int], recv: Sequence[int]) -> bool:
    """Header stamps far before 2001 while the recorder clock is wall time."""
    nonzero = [s for s in stamps if s > 0]
    if not nonzero or not recv:
        return False
    return median(nonzero) < WALL_CLOCK_MIN_NS <= median(recv)


MIN_PERIODIC_MESSAGES = 10


def duration_text(seconds: float) -> str:
    """Human-readable duration with a unit that fits the magnitude."""
    magnitude = abs(seconds)
    if magnitude >= 1:
        return f"{seconds:.3f} s"
    if magnitude >= 1e-3:
        return f"{seconds * 1e3:.1f} ms"
    if magnitude >= 1e-6:
        return f"{seconds * 1e6:.1f} us"
    return f"{seconds * 1e9:.0f} ns"


def analyze_rates(
    stats: TopicStats,
    recv: Sequence[int],
    origin: int,
    end: int,
    opts: Options,
    periodic: bool = True,
) -> list[Finding]:
    """Fill rate/jitter/gap fields of ``stats`` from ascending receive times.

    Gap and jitter checks need a period, so they only run for ``periodic`` topics with at
    least :data:`MIN_PERIODIC_MESSAGES` messages (or an expected rate)."""
    findings: list[Finding] = []
    n = len(recv)
    if n == 0:
        return findings
    stats.first_ns, stats.last_ns = recv[0], recv[-1]
    stats.timeline_bin_s, stats.timeline = timeline(recv, origin, end)
    if n < 2:
        return findings
    dts = [(b - a) / 1e9 for a, b in itertools.pairwise(recv)]
    span = (recv[-1] - recv[0]) / 1e9
    stats.rate_avg_hz = (n - 1) / span if span > 0 else None
    stats.period_median_s = median(dts)
    longest, shortest = max(dts), min(dts)
    stats.rate_min_hz = 1 / longest if longest > 0 else None
    stats.rate_max_hz = 1 / shortest if shortest > 0 else None
    expected = opts.expectations.get(stats.name)
    stats.expected_hz = expected
    period = 1 / expected if expected else stats.period_median_s
    if period <= 0 or not periodic or (n < MIN_PERIODIC_MESSAGES and not expected):
        findings.extend(_expectation(stats, expected, opts))
        return findings
    threshold = max(opts.gap_factor * period, opts.min_gap_s)
    stats.gap_threshold_s = threshold
    regular: list[float] = []
    for i, dt in enumerate(dts):
        if dt > threshold:
            stats.gaps.append(Gap(recv[i], recv[i + 1]))
        else:
            regular.append(dt)
    if len(regular) >= 2:
        mean = sum(regular) / len(regular)
        stats.jitter_s = math.sqrt(sum((d - mean) ** 2 for d in regular) / len(regular))
    if stats.gaps:
        worst = max(stats.gaps, key=lambda g: g.duration_s)
        basis = "expected" if expected else "median"
        findings.append(
            Finding(
                "gap",
                WARNING,
                stats.name,
                "Gaps in the message stream",
                f"{len(stats.gaps)} inter-arrival time(s) above {duration_text(threshold)} "
                f"({opts.gap_factor:g} x the {basis} period of {duration_text(period)}). "
                f"Longest: {duration_text(worst.duration_s)} starting at "
                f"{rel(worst.start_ns, origin)}.",
                len(stats.gaps),
                [
                    f"{rel(g.start_ns, origin)} -> {rel(g.end_ns, origin)}: "
                    f"{g.duration_s:.3f} s without messages "
                    f"(~{max(round(g.duration_s / period) - 1, 1)} expected messages missing)"
                    for g in stats.gaps[:MAX_ITEMS]
                ],
                {
                    "threshold_s": threshold,
                    "gaps": [[g.start_ns, g.end_ns] for g in stats.gaps],
                },
            )
        )
    if stats.jitter_s is not None and stats.jitter_s > 0.5 * period and len(regular) >= 10:
        findings.append(
            Finding(
                "high_jitter",
                WARNING,
                stats.name,
                "Irregular message timing",
                f"Inter-arrival standard deviation is {duration_text(stats.jitter_s)}, more "
                f"than half the {duration_text(period)} period (gaps excluded). Look for CPU load, "
                "Wi-Fi links or QoS depth problems between publisher and recorder.",
                data={"jitter_s": stats.jitter_s, "period_s": period},
            )
        )
    findings.extend(_expectation(stats, expected, opts))
    return findings


def _expectation(stats: TopicStats, expected: float | None, opts: Options) -> list[Finding]:
    findings: list[Finding] = []
    if expected and stats.rate_avg_hz is not None:
        deviation = abs(stats.rate_avg_hz - expected) / expected
        if deviation > opts.rate_tolerance:
            findings.append(
                Finding(
                    "rate_mismatch",
                    ERROR,
                    stats.name,
                    "Rate differs from expectation",
                    f"Average rate {stats.rate_avg_hz:.2f} Hz, expected {expected:g} Hz "
                    f"+/- {opts.rate_tolerance * 100:.0f}% (off by {deviation * 100:.0f}%).",
                    data={"rate_hz": stats.rate_avg_hz, "expected_hz": expected},
                )
            )
    return findings


def analyze_stamps(
    stats: TopicStats,
    recv: Sequence[int],
    stamps: Sequence[int],
    origin: int,
    opts: Options,
) -> list[Finding]:
    """Check header stamps against each other and against receive times."""
    findings: list[Finding] = []
    if not stamps:
        return findings
    name = stats.name
    zero = [r for r, s in zip(recv, stamps, strict=False) if s == 0]
    if zero:
        findings.append(
            Finding(
                "zero_stamp",
                ERROR,
                name,
                "Messages with header.stamp = 0",
                f"{len(zero)} of {len(stamps)} messages have a zero stamp: the publisher never "
                "set header.stamp (use node->now() or the driver's acquisition time).",
                len(zero),
                [f"received at {rel(r, origin)}" for r in zero[:MAX_ITEMS]],
            )
        )
    if looks_like_sim_time(stamps, recv):
        findings.append(
            Finding(
                "sim_time_stamps",
                INFO,
                name,
                "Stamps look like simulation time",
                "header.stamp values are far earlier than the recorder's wall-clock receive "
                "times, as with use_sim_time publishers; latency and future-stamp checks were "
                "skipped for this topic.",
            )
        )
        nonzero = [(r, s) for r, s in zip(recv, stamps, strict=True) if s > 0]
        findings.extend(_ordering(name, nonzero, origin))
        return findings

    future: list[tuple[int, int]] = []
    ahead: list[tuple[int, int]] = []
    valid: list[tuple[int, int]] = []
    latencies: list[float] = []
    for r, s in zip(recv, stamps, strict=True):
        if s == 0:
            continue
        lead = (s - r) / 1e9
        if lead > opts.future_s:
            future.append((r, s))
            continue
        if lead > opts.clock_skew_s:
            ahead.append((r, s))
        valid.append((r, s))
        latencies.append(-lead)
    if future:
        findings.append(
            Finding(
                "future_stamp",
                ERROR,
                name,
                "Stamps in the future",
                f"{len(future)} message(s) carry a header.stamp more than {opts.future_s:g} s "
                "later than the time they were recorded. Typical causes: a sensor computer "
                "whose clock is not synchronised (chrony/NTP), or a driver adding an offset "
                "twice. tf2 and message_filters will hold or drop such messages.",
                len(future),
                [
                    f"received at {rel(r, origin)}: stamp is {(s - r) / 1e9:.3f} s ahead"
                    for r, s in future[:MAX_ITEMS]
                ],
            )
        )
    if ahead:
        worst = max((s - r) for r, s in ahead) / 1e9
        findings.append(
            Finding(
                "stamp_after_receive",
                WARNING,
                name,
                "Stamps later than receive time",
                f"{len(ahead)} message(s) are stamped up to {worst * 1000:.1f} ms after they "
                "were received, which is only possible with clock offset between the "
                "publishing and recording machines.",
                len(ahead),
                data={"max_ahead_s": worst},
            )
        )
    if latencies:
        ordered = sorted(latencies)
        stats.latency = LatencyStats(
            samples=len(ordered),
            mean_s=sum(ordered) / len(ordered),
            min_s=ordered[0],
            max_s=ordered[-1],
            p50_s=percentile(ordered, 0.5),
            p95_s=percentile(ordered, 0.95),
            histogram=histogram(ordered),
        )
        if stats.latency.p95_s > opts.max_latency_s:
            findings.append(
                Finding(
                    "high_latency",
                    WARNING,
                    name,
                    "High stamp-to-record latency",
                    f"95% of messages arrive within {stats.latency.p95_s * 1000:.1f} ms of their "
                    f"header.stamp (limit {opts.max_latency_s * 1000:.0f} ms; median "
                    f"{stats.latency.p50_s * 1000:.1f} ms, max {stats.latency.max_s * 1000:.1f} "
                    "ms). Consumers using a TF buffer or time synchroniser may drop these.",
                    data={"p95_s": stats.latency.p95_s},
                )
            )
    findings.extend(_ordering(name, valid, origin))
    return findings


def _ordering(name: str, valid: Sequence[tuple[int, int]], origin: int) -> list[Finding]:
    findings: list[Finding] = []
    backwards: list[str] = []
    duplicates: list[str] = []
    for (_, prev), (r, s) in itertools.pairwise(valid):
        if s < prev:
            backwards.append(
                f"received at {rel(r, origin)}: stamp {(prev - s) / 1e9:.6f} s before "
                "the previous one"
            )
        elif s == prev:
            duplicates.append(
                f"received at {rel(r, origin)}: stamp {s // 10**9}.{s % 10**9:09d} repeated"
            )
    if backwards:
        findings.append(
            Finding(
                "stamp_out_of_order",
                WARNING,
                name,
                "header.stamp goes backwards",
                f"{len(backwards)} message(s) are stamped earlier than the message received "
                "before them. Filters and tf2 assume increasing stamps per topic.",
                len(backwards),
                backwards[:MAX_ITEMS],
            )
        )
    if duplicates:
        findings.append(
            Finding(
                "duplicate_stamp",
                WARNING,
                name,
                "Repeated header.stamp",
                f"{len(duplicates)} message(s) repeat the stamp of the previous message: a "
                "driver republishing stale data, or a stamp taken from a coarse clock.",
                len(duplicates),
                duplicates[:MAX_ITEMS],
            )
        )
    return findings
