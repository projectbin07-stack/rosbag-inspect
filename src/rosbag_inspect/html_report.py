"""Self-contained HTML report: inline CSS and SVG, light/dark via ``prefers-color-scheme``.

No JavaScript and no external resources. Hover details use SVG ``<title>`` elements and
long finding lists use ``<details>``.
"""

from __future__ import annotations

import html
import math
from datetime import datetime, timezone

from .findings import ERROR, FAIL, INFO, PASS, WARNING, Report, TfSummary, TopicStats

_CSS = """
:root {
  color-scheme: light;
  --surface-0: #f3f2ef; --surface-1: #fcfcfb; --border: #dddcd7;
  --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #77766f;
  --series-1: #2a78d6; --series-1-fill: rgba(42,120,214,.16); --grid: #e7e6e1;
  --critical: #d03b3b; --critical-fill: rgba(208,59,59,.16); --warning: #b97c00;
  --warning-fill: rgba(250,178,25,.22); --good: #0b850b; --info: #52514e;
  --node: #ffffff; --edge: #77766f;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    color-scheme: dark;
    --surface-0: #121211; --surface-1: #1a1a19; --border: #33332f;
    --text-primary: #f0efec; --text-secondary: #c3c2b7; --text-muted: #9a998f;
    --series-1: #3987e5; --series-1-fill: rgba(57,135,229,.22); --grid: #2b2b28;
    --critical: #e06060; --critical-fill: rgba(224,96,96,.22); --warning: #fab219;
    --warning-fill: rgba(250,178,25,.18); --good: #0ca30c; --info: #c3c2b7;
    --node: #232321; --edge: #9a998f;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --surface-0: #121211; --surface-1: #1a1a19; --border: #33332f;
  --text-primary: #f0efec; --text-secondary: #c3c2b7; --text-muted: #9a998f;
  --series-1: #3987e5; --series-1-fill: rgba(57,135,229,.22); --grid: #2b2b28;
  --critical: #e06060; --critical-fill: rgba(224,96,96,.22); --warning: #fab219;
  --warning-fill: rgba(250,178,25,.18); --good: #0ca30c; --info: #c3c2b7;
  --node: #232321; --edge: #9a998f;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--surface-0); color: var(--text-primary);
  font: 14px/1.45 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }
main { max-width: 1180px; margin: 0 auto; padding: 24px 16px 48px; }
h1 { font-size: 22px; margin: 0 0 4px; overflow-wrap: anywhere; }
h2 { font-size: 17px; margin: 28px 0 10px; }
h3 { font-size: 15px; margin: 0 0 2px; overflow-wrap: anywhere; }
.muted { color: var(--text-muted); }
.secondary { color: var(--text-secondary); }
.card { background: var(--surface-1); border: 1px solid var(--border); border-radius: 8px;
  padding: 14px 16px; margin-bottom: 12px; }
.head { display: flex; flex-wrap: wrap; gap: 16px; align-items: center;
  justify-content: space-between; }
.verdict { font-weight: 700; font-size: 18px; padding: 6px 14px; border-radius: 6px;
  border: 2px solid currentColor; white-space: nowrap; }
.v-pass { color: var(--good); } .v-warn { color: var(--warning); }
.v-fail { color: var(--critical); }
.facts { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px;
  margin-top: 12px; }
.fact b { display: block; font-size: 18px; font-variant-numeric: tabular-nums; }
.fact span { color: var(--text-secondary); font-size: 12px; }
.finding { border-left: 4px solid var(--border); }
.finding.error { border-left-color: var(--critical); }
.finding.warning { border-left-color: var(--warning); }
.finding.info { border-left-color: var(--info); }
.sev { font-weight: 700; font-size: 12px; letter-spacing: .04em; margin-right: 6px; }
.sev.error { color: var(--critical); } .sev.warning { color: var(--warning); }
.sev.info { color: var(--info); }
a { color: inherit; text-decoration-color: var(--border); }
code { font: 12px ui-monospace, SFMono-Regular, Consolas, monospace; overflow-wrap: anywhere; }
.finding p { margin: 4px 0; }
details summary { cursor: pointer; color: var(--text-secondary); }
details ul { margin: 6px 0 0; padding-left: 20px; font: 12px ui-monospace, Consolas, monospace; }
.table-wrap { overflow-x: auto; }
table { border-collapse: collapse; width: 100%; font-variant-numeric: tabular-nums; }
th, td { text-align: left; padding: 6px 8px; border-bottom: 1px solid var(--border);
  white-space: nowrap; }
th { color: var(--text-secondary); font-weight: 600; font-size: 12px; }
td.num, th.num { text-align: right; }
.charts { display: grid; grid-template-columns: 2fr 1fr; gap: 16px; margin-top: 8px; }
@media (max-width: 760px) { .charts { grid-template-columns: 1fr; } }
svg { width: 100%; height: auto; display: block; }
svg text { fill: var(--text-secondary); font-size: 11px;
  font-family: system-ui, -apple-system, "Segoe UI", sans-serif; }
.axis { stroke: var(--border); stroke-width: 1; }
.gridline { stroke: var(--grid); stroke-width: 1; }
.rate-line { fill: none; stroke: var(--series-1); stroke-width: 2; stroke-linejoin: round; }
.rate-area { fill: var(--series-1-fill); stroke: none; }
.expected { stroke: var(--text-muted); stroke-width: 1.5; stroke-dasharray: 4 4; }
.gap { fill: var(--critical-fill); stroke: var(--critical); stroke-width: 1; }
.bar { fill: var(--series-1); }
.edge { stroke: var(--edge); stroke-width: 1.5; fill: none; }
.edge.static { stroke-dasharray: 5 4; }
.node rect { fill: var(--node); stroke: var(--edge); stroke-width: 1.2; }
.node.used rect { stroke: var(--series-1); stroke-width: 2; }
.node.bad rect { stroke: var(--critical); stroke-width: 2; fill: var(--critical-fill); }
.node text { fill: var(--text-primary); font-size: 12px; }
.legend { display: flex; flex-wrap: wrap; gap: 14px; font-size: 12px; color: var(--text-secondary);
  margin-top: 6px; }
.key { display: inline-block; width: 22px; height: 0; border-top: 2px solid var(--edge);
  vertical-align: middle; margin-right: 4px; }
.key.static { border-top-style: dashed; }
.key.box { width: 12px; height: 12px; border: 2px solid var(--series-1); border-radius: 3px; }
.key.box.bad { border-color: var(--critical); background: var(--critical-fill); }
.key.box.gapkey { border: 1px solid var(--critical); background: var(--critical-fill); }
footer { margin-top: 32px; color: var(--text-muted); font-size: 12px; }
"""

_MARK = {ERROR: "FAIL", WARNING: "WARN", INFO: "INFO"}


def _e(text: object) -> str:
    return html.escape(str(text), quote=True)


def _fmt(value: float | None, digits: int = 2, scale: float = 1.0) -> str:
    if value is None or not math.isfinite(value):
        return "-"
    return f"{value * scale:.{digits}f}"


def _when(ns: int | None) -> str:
    if ns is None:
        return "-"
    return datetime.fromtimestamp(ns / 1e9, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")


def _nice_max(value: float) -> float:
    if value <= 0:
        return 1.0
    exponent = math.floor(math.log10(value))
    for step in (1, 2, 2.5, 5, 10):
        candidate = step * 10**exponent
        if candidate >= value:
            return float(candidate)
    return float(10 ** (exponent + 1))  # pragma: no cover - loop always returns


def rate_timeline_svg(stats: TopicStats, start_ns: int, duration_s: float) -> str:
    """Line chart of message rate per time bin, with gap bands and the expected rate."""
    width, height = 640, 150
    left, right, top, bottom = 44, 10, 10, 26
    plot_w, plot_h = width - left - right, height - top - bottom
    rates = stats.timeline
    peak = max([*rates, stats.expected_hz or 0, 1e-9])
    ymax = _nice_max(peak * 1.1)
    span = max(duration_s, stats.timeline_bin_s * len(rates), 1e-9)

    def x(seconds: float) -> float:
        return left + plot_w * min(max(seconds / span, 0), 1)

    def y(rate: float) -> float:
        return top + plot_h * (1 - rate / ymax)

    parts = [
        f'<svg viewBox="0 0 {width} {height}" role="img" '
        f'aria-label="{_e(stats.name)} message rate over time">'
    ]
    for frac in (0, 0.5, 1):
        gy = y(ymax * frac)
        parts.append(
            f'<line class="gridline" x1="{left}" x2="{width - right}" y1="{gy:.1f}" y2="{gy:.1f}"/>'
        )
        parts.append(
            f'<text x="{left - 6}" y="{gy + 4:.1f}" text-anchor="end">{ymax * frac:g}</text>'
        )
    for gap in stats.gaps:
        gx0 = x((gap.start_ns - start_ns) / 1e9)
        gx1 = x((gap.end_ns - start_ns) / 1e9)
        parts.append(
            f'<rect class="gap" x="{gx0:.1f}" y="{top}" width="{max(gx1 - gx0, 2):.1f}" '
            f'height="{plot_h}"><title>gap {gap.duration_s:.3f} s from '
            f"+{(gap.start_ns - start_ns) / 1e9:.3f} s</title></rect>"
        )
    if rates:
        points = []
        for i, rate in enumerate(rates):
            t0 = i * stats.timeline_bin_s
            t1 = min((i + 1) * stats.timeline_bin_s, span)
            points.append((x(t0), y(rate)))
            points.append((x(t1), y(rate)))
        path = " ".join(f"{px:.1f},{py:.1f}" for px, py in points)
        base = top + plot_h
        parts.append(
            f'<polygon class="rate-area" points="{points[0][0]:.1f},{base} {path} '
            f'{points[-1][0]:.1f},{base}"/>'
        )
        parts.append(f'<polyline class="rate-line" points="{path}"/>')
        for i, rate in enumerate(rates):
            t0 = i * stats.timeline_bin_s
            bx0, bx1 = x(t0), x(min(t0 + stats.timeline_bin_s, span))
            parts.append(
                f'<rect x="{bx0:.1f}" y="{top}" width="{max(bx1 - bx0, 1):.1f}" '
                f'height="{plot_h}" fill="transparent"><title>+{t0:.0f} s: {rate:.1f} Hz'
                "</title></rect>"
            )
    if stats.expected_hz:
        ey = y(stats.expected_hz)
        parts.append(
            f'<line class="expected" x1="{left}" x2="{width - right}" y1="{ey:.1f}" '
            f'y2="{ey:.1f}"><title>expected {stats.expected_hz:g} Hz</title></line>'
        )
    base = top + plot_h
    parts.append(f'<line class="axis" x1="{left}" x2="{width - right}" y1="{base}" y2="{base}"/>')
    for frac in (0, 0.25, 0.5, 0.75, 1):
        tx = left + plot_w * frac
        anchor = "start" if frac == 0 else ("end" if frac == 1 else "middle")
        parts.append(
            f'<text x="{tx:.1f}" y="{height - 8}" text-anchor="{anchor}">{span * frac:.0f} s</text>'
        )
    parts.append(f'<text x="{left + 4}" y="{top + 12}">Hz</text>')
    parts.append("</svg>")
    return "".join(parts)


def latency_histogram_svg(stats: TopicStats) -> str:
    """Histogram of receive time - header.stamp in milliseconds."""
    lat = stats.latency
    if lat is None:
        return '<p class="muted">No header stamps to measure latency.</p>'
    width, height = 320, 150
    left, right, top, bottom = 34, 10, 10, 26
    plot_w, plot_h = width - left - right, height - top - bottom
    counts = lat.histogram.counts
    edges = lat.histogram.edges
    ymax = _nice_max(max(counts) * 1.1)
    bar_w = plot_w / len(counts) if len(counts) > 1 else plot_w / 5
    offset = 0.0 if len(counts) > 1 else (plot_w - bar_w) / 2
    parts = [
        f'<svg viewBox="0 0 {width} {height}" role="img" '
        f'aria-label="{_e(stats.name)} latency histogram">'
    ]
    for frac in (0, 0.5, 1):
        gy = top + plot_h * (1 - frac)
        parts.append(
            f'<line class="gridline" x1="{left}" x2="{width - right}" y1="{gy:.1f}" y2="{gy:.1f}"/>'
        )
        parts.append(
            f'<text x="{left - 6}" y="{gy + 4:.1f}" text-anchor="end">{ymax * frac:g}</text>'
        )
    for i, count in enumerate(counts):
        if count == 0:
            continue
        bh = plot_h * count / ymax
        bx = left + offset + i * bar_w + 1
        by = top + plot_h - bh
        lo = edges[i] * 1000
        hi = edges[min(i + 1, len(edges) - 1)] * 1000
        parts.append(
            f'<rect class="bar" x="{bx:.1f}" y="{by:.1f}" width="{max(bar_w - 2, 1):.1f}" '
            f'height="{bh:.1f}" rx="2"><title>{lo:.1f} to {hi:.1f} ms: {count} message(s)'
            "</title></rect>"
        )
    base = top + plot_h
    parts.append(f'<line class="axis" x1="{left}" x2="{width - right}" y1="{base}" y2="{base}"/>')
    parts.append(f'<text x="{left}" y="{height - 8}">{edges[0] * 1000:.1f} ms</text>')
    parts.append(
        f'<text x="{width - right}" y="{height - 8}" text-anchor="end">'
        f"{edges[-1] * 1000:.1f} ms</text>"
    )
    parts.append("</svg>")
    return "".join(parts)


def _layout_tf(tf: TfSummary) -> tuple[dict[str, tuple[int, int]], int, int]:
    """Assign (column, row) to frames: one column block per tree, rows by depth."""
    children: dict[str, list[str]] = {}
    has_parent: set[str] = set()
    for edge in tf.edges:
        if edge.child not in children.setdefault(edge.parent, []):
            children[edge.parent].append(edge.child)
        has_parent.add(edge.child)
    positions: dict[str, tuple[int, int]] = {}
    column = 0
    max_row = 0
    for group in tf.components:
        roots = [f for f in group if f not in has_parent] or group[:1]
        for root in roots:
            column = _place(root, 0, column, children, positions)
        for frame in group:
            if frame not in positions:
                positions[frame] = (column, 0)
                column += 1
        max_row = max([max_row, *(positions[f][1] for f in group)])
    for frame in tf.missing:
        positions[frame] = (column, 0)
        column += 1
    return positions, max(column, 1), max_row + 1


def _place(
    node: str,
    depth: int,
    column: int,
    children: dict[str, list[str]],
    positions: dict[str, tuple[int, int]],
) -> int:
    if node in positions:
        return column
    kids = [k for k in sorted(children.get(node, [])) if k not in positions]
    positions[node] = (column, depth)
    if not kids:
        return column + 1
    start = column
    for kid in kids:
        column = _place(kid, depth + 1, column, children, positions)
    # centre the parent over its children
    positions[node] = ((start + column - 1) // 2, depth)
    return column


def tf_tree_svg(tf: TfSummary) -> str:
    if not tf.frames and not tf.missing:
        return '<p class="muted">No /tf or /tf_static messages in this bag.</p>'
    positions, columns, rows = _layout_tf(tf)
    node_w, node_h, col_w, row_h = 150, 30, 170, 70
    width = max(columns * col_w + 20, 300)
    height = rows * row_h + 20

    def centre(frame: str) -> tuple[float, float]:
        col, row = positions[frame]
        return 10 + col * col_w + col_w / 2, 10 + row * row_h + node_h / 2

    bad = set(tf.missing) | set(tf.unreachable)
    parts = [
        f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="TF tree" '
        f'style="max-width:{width}px">'
    ]
    for edge in tf.edges:
        (x0, y0), (x1, y1) = centre(edge.parent), centre(edge.child)
        y0 += node_h / 2
        y1 -= node_h / 2
        mid = (y0 + y1) / 2
        kind = "static" if edge.static else "dynamic"
        parts.append(
            f'<path class="edge {kind}" d="M{x0:.1f},{y0:.1f} C{x0:.1f},{mid:.1f} '
            f'{x1:.1f},{mid:.1f} {x1:.1f},{y1:.1f}"><title>{_e(edge.parent)} -&gt; '
            f"{_e(edge.child)} ({kind}, {edge.count} transform(s) on "
            f"{_e(', '.join(edge.topics))})</title></path>"
        )
    for frame in sorted(positions):
        cx, cy = centre(frame)
        used = tf.referenced.get(frame)
        classes = "node" + (" bad" if frame in bad else (" used" if used else ""))
        tip = frame
        if used:
            tip += f" - frame_id of {', '.join(used)}"
        if frame in tf.missing:
            tip += " - MISSING from TF"
        elif frame in tf.unreachable:
            tip += f" - not connected to {tf.main_root}"
        label = frame if len(frame) <= 20 else frame[:19] + "…"
        parts.append(
            f'<g class="{classes}"><title>{_e(tip)}</title>'
            f'<rect x="{cx - node_w / 2:.1f}" y="{cy - node_h / 2:.1f}" width="{node_w}" '
            f'height="{node_h}" rx="6"/>'
            f'<text x="{cx:.1f}" y="{cy + 4:.1f}" text-anchor="middle">{_e(label)}</text></g>'
        )
    parts.append("</svg>")
    return "".join(parts)


def _anchor(name: str) -> str:
    return "".join(c if c.isalnum() else "-" for c in name).strip("-") or "topic"


def _topic_row(t: TopicStats) -> str:
    lat = t.latency
    return (
        f'<tr><td><a href="#topic-{_anchor(t.name)}"><code>{_e(t.name)}</code></a></td>'
        f"<td><code>{_e(t.type or '?')}</code></td>"
        f'<td class="num">{t.count}</td>'
        f'<td class="num">{_fmt(t.rate_avg_hz)}</td>'
        f'<td class="num">{_fmt(t.rate_min_hz)} / {_fmt(t.rate_max_hz)}</td>'
        f'<td class="num">{_fmt(t.jitter_s, 1, 1000)}</td>'
        f'<td class="num">{len(t.gaps)}</td>'
        f'<td class="num">{_fmt(lat.p50_s if lat else None, 1, 1000)} / '
        f"{_fmt(lat.p95_s if lat else None, 1, 1000)}</td>"
        f"<td>{_e(', '.join(f or '(empty)' for f in t.frame_ids) or '-')}</td></tr>"
    )


def render_html(report: Report) -> str:
    counts = report.counts()
    verdict = report.verdict
    verdict_text = {PASS: "✓ PASS", FAIL: "✗ FAIL"}.get(verdict, "! WARN")
    start = report.start_ns or 0
    out: list[str] = [
        "<!DOCTYPE html>",
        '<html lang="en"><head><meta charset="utf-8">',
        '<meta name="viewport" content="width=device-width, initial-scale=1">',
        f"<title>rosbag-inspect: {_e(report.bag)}</title>",
        f"<style>{_CSS}</style></head><body><main>",
        '<section class="card"><div class="head"><div>',
        f"<h1>{_e(report.bag)}</h1>",
        f'<div class="secondary">{_e(report.storage)} &middot; {_e(", ".join(report.files))}</div>',
        f'<div class="muted">{_e(_when(report.start_ns))} &rarr; {_e(_when(report.end_ns))}</div>',
        f'</div><div class="verdict v-{verdict}">{verdict_text}</div></div>',
        '<div class="facts">',
        f'<div class="fact"><b>{report.duration_s:.2f} s</b><span>duration</span></div>',
        f'<div class="fact"><b>{report.message_count}</b><span>messages</span></div>',
        f'<div class="fact"><b>{len(report.topics)}</b><span>topics</span></div>',
        f'<div class="fact"><b>{counts[ERROR]}</b><span>errors</span></div>',
        f'<div class="fact"><b>{counts[WARNING]}</b><span>warnings</span></div>',
        f'<div class="fact"><b>{counts[INFO]}</b><span>notes</span></div>',
        "</div></section>",
        '<h2 id="findings">Findings</h2>',
    ]
    if not report.findings:
        out.append('<div class="card">No findings.</div>')
    for f in report.findings:
        where = f"<code>{_e(f.topic)}</code> " if f.topic else ""
        out.append(
            f'<div class="card finding {f.severity}"><h3><span class="sev {f.severity}">'
            f'{_MARK[f.severity]}</span>{where}{_e(f.title)} <code class="muted">'
            f"[{_e(f.code)}]</code></h3><p>{_e(f.detail)}</p>"
        )
        if f.items:
            out.append(f"<details><summary>{len(f.items)} detail line(s)</summary><ul>")
            out.extend(f"<li>{_e(item)}</li>" for item in f.items)
            out.append("</ul></details>")
        out.append("</div>")
    out += [
        '<h2 id="topics">Topics</h2>',
        '<div class="card table-wrap"><table><thead><tr><th>Topic</th><th>Type</th>'
        '<th class="num">Msgs</th><th class="num">Avg Hz</th><th class="num">Min / max Hz</th>'
        '<th class="num">Jitter ms</th><th class="num">Gaps</th>'
        '<th class="num">Latency p50 / p95 ms</th><th>frame_id</th></tr></thead><tbody>',
        *(_topic_row(t) for t in report.topics),
        "</tbody></table></div>",
        '<h2 id="tf">TF tree</h2>',
        '<div class="card table-wrap">',
        tf_tree_svg(report.tf),
        '<div class="legend"><span><i class="key"></i>dynamic (/tf)</span>'
        '<span><i class="key static"></i>static (/tf_static)</span>'
        '<span><i class="key box"></i>frame used by a sensor topic</span>'
        '<span><i class="key box bad"></i>missing or disconnected</span></div>',
        "</div>",
        '<h2 id="timelines">Timelines</h2>',
    ]
    for t in report.topics:
        if not t.count:
            continue
        expected = f", expected {t.expected_hz:g} Hz" if t.expected_hz else ""
        out += [
            f'<section class="card" id="topic-{_anchor(t.name)}">',
            f"<h3><code>{_e(t.name)}</code></h3>",
            f'<div class="muted">{_e(t.type or "?")} &middot; {t.count} messages &middot; '
            + (f"{_fmt(t.rate_avg_hz)} Hz average" if t.rate_avg_hz else "no rate")
            + f"{_e(expected)}</div>",
            '<div class="charts"><div>',
            f'<div class="secondary">Message rate per {t.timeline_bin_s:g} s bin</div>',
            rate_timeline_svg(t, start, report.duration_s)
            if t.timeline and t.count > 1
            else '<p class="muted">A single message (latched or published once): no rate '
            "to plot.</p>",
            '<div class="legend"><span><i class="key" style="border-color:var(--series-1)"></i>'
            "rate</span>"
            + ('<span><i class="key box gapkey"></i>gap</span>' if t.gaps else "")
            + ('<span><i class="key static"></i>expected</span>' if t.expected_hz else "")
            + "</div></div><div>",
            '<div class="secondary">Receive time &minus; header.stamp</div>',
            latency_histogram_svg(t),
            "</div></div></section>",
        ]
    out.append(
        f"<footer>Generated by rosbag-inspect {_e(report.tool_version)}. "
        "Times on the charts are seconds after the first message.</footer>"
    )
    out.append("</main></body></html>")
    return "\n".join(out)
