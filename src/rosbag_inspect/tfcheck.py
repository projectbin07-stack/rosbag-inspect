"""TF tree checks from ``tf2_msgs/TFMessage`` topics and the frame_ids used by sensors."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field

from .findings import ERROR, INFO, WARNING, Finding, TfEdge, TfSummary


@dataclass
class TfCollector:
    """Accumulates transforms and frame references while messages are read."""

    edges: dict[tuple[str, str, bool], tuple[int, set[str]]] = field(default_factory=dict)
    referenced: dict[str, set[str]] = field(default_factory=dict)
    tf_topics: set[str] = field(default_factory=set)
    slashed: set[str] = field(default_factory=set)

    def add_transform(self, parent: str, child: str, static: bool, topic: str) -> None:
        self.tf_topics.add(topic)
        for name in (parent, child):
            if name.startswith("/"):
                self.slashed.add(name)
        key = (parent.lstrip("/"), child.lstrip("/"), static)
        count, topics = self.edges.get(key, (0, set()))
        topics.add(topic)
        self.edges[key] = (count + 1, topics)

    def reference(self, frame: str, topic: str) -> None:
        if frame:
            self.referenced.setdefault(frame.lstrip("/"), set()).add(topic)


def _components(frames: Iterable[str], pairs: Iterable[tuple[str, str]]) -> list[list[str]]:
    parent: dict[str, str] = {f: f for f in frames}

    def find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b in pairs:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    groups: dict[str, list[str]] = {}
    for f in parent:
        groups.setdefault(find(f), []).append(f)
    return sorted((sorted(g) for g in groups.values()), key=lambda g: (-len(g), g))


def _find_cycle(children: dict[str, set[str]]) -> list[str] | None:
    state: dict[str, int] = {}
    stack: list[str] = []

    def visit(node: str) -> list[str] | None:
        state[node] = 1
        stack.append(node)
        for nxt in sorted(children.get(node, ())):
            if state.get(nxt) == 1:
                return [*stack[stack.index(nxt) :], nxt]
            if nxt not in state:
                found = visit(nxt)
                if found:
                    return found
        stack.pop()
        state[node] = 2
        return None

    for node in sorted(children):
        if node not in state:
            found = visit(node)
            if found:
                return found
    return None


def analyze_tf(collector: TfCollector) -> tuple[TfSummary, list[Finding]]:
    findings: list[Finding] = []
    summary = TfSummary()
    summary.referenced = {k: sorted(v) for k, v in sorted(collector.referenced.items())}
    if not collector.edges:
        if collector.referenced:
            findings.append(
                Finding(
                    "tf_absent",
                    INFO,
                    "",
                    "No TF data in the bag",
                    "The bag has no /tf or /tf_static messages, so the frames used by sensors "
                    f"({', '.join(sorted(collector.referenced))}) cannot be checked. Record "
                    "/tf and /tf_static to debug transforms offline.",
                )
            )
        return summary, findings

    merged: dict[tuple[str, str], dict[bool, tuple[int, set[str]]]] = {}
    for (p, c, static), value in collector.edges.items():
        merged.setdefault((p, c), {})[static] = value
    for (p, c), kinds in sorted(merged.items()):
        for static, (count, topics) in sorted(kinds.items()):
            summary.edges.append(TfEdge(p, c, static, count, sorted(topics)))
        if len(kinds) == 2:
            findings.append(
                Finding(
                    "tf_static_and_dynamic",
                    WARNING,
                    "",
                    "Transform published as both static and dynamic",
                    f"{p} -> {c} appears on /tf_static and on /tf. Two publishers disagree on "
                    "whether this transform moves; remove one of them.",
                )
            )
    frames = sorted({p for p, _ in merged} | {c for _, c in merged})
    summary.frames = frames

    if collector.slashed:
        findings.append(
            Finding(
                "tf_leading_slash",
                WARNING,
                "",
                "Frame names with a leading slash",
                "tf2 rejects frame ids that start with '/': "
                + ", ".join(sorted(collector.slashed)),
                len(collector.slashed),
            )
        )

    parents: dict[str, set[str]] = {}
    children: dict[str, set[str]] = {}
    for p, c in merged:
        parents.setdefault(c, set()).add(p)
        children.setdefault(p, set()).add(c)
    multi = {c: ps for c, ps in parents.items() if len(ps) > 1}
    if multi:
        findings.append(
            Finding(
                "tf_multiple_parents",
                ERROR,
                "",
                "Frames with more than one parent",
                "TF must be a tree: each frame has exactly one parent. Two nodes publish a "
                "transform to the same child (for example robot_state_publisher and a "
                "localisation node both publishing to base_link).",
                len(multi),
                [f"{c}: parents {', '.join(sorted(ps))}" for c, ps in sorted(multi.items())],
            )
        )
    cycle = _find_cycle(children)
    if cycle:
        findings.append(
            Finding(
                "tf_cycle",
                ERROR,
                "",
                "Loop in the TF graph",
                "Transforms form a cycle: " + " -> ".join(cycle),
            )
        )

    components = _components(frames, merged)
    summary.components = components

    def score(group: list[str]) -> tuple[int, int]:
        return (sum(1 for f in group if f in collector.referenced), len(group))

    main = max(components, key=score)
    roots = [f for f in main if f not in parents]
    summary.main_root = roots[0] if roots else main[0]
    if len(components) > 1:
        items = []
        for group in components:
            group_roots = [f for f in group if f not in parents] or group[:1]
            items.append(f"root {group_roots[0]}: {', '.join(group)}")
        findings.append(
            Finding(
                "tf_disconnected",
                WARNING,
                "",
                "TF is split into separate trees",
                f"The transforms form {len(components)} unconnected trees; there is no "
                "transform between frames of different trees. The main tree (most frames "
                f"used by sensors) is rooted at {summary.main_root}.",
                len(components),
                items,
            )
        )

    main_set = set(main)
    for frame, topics in sorted(collector.referenced.items()):
        users = ", ".join(sorted(topics))
        if frame not in frames:
            summary.missing.append(frame)
            findings.append(
                Finding(
                    "tf_missing_frame",
                    ERROR,
                    "",
                    f"Frame '{frame}' has no transform",
                    f"'{frame}' is used as frame_id by {users} but never appears in /tf or "
                    "/tf_static, so these messages cannot be transformed. Check the URDF / "
                    "static_transform_publisher and the driver's frame_id parameter.",
                    data={"frame": frame, "topics": sorted(topics)},
                )
            )
        elif frame not in main_set:
            summary.unreachable.append(frame)
            findings.append(
                Finding(
                    "tf_broken_chain",
                    ERROR,
                    "",
                    f"No TF path from '{frame}' to '{summary.main_root}'",
                    f"'{frame}' (used by {users}) is in a tree that is not connected to "
                    f"'{summary.main_root}'. A link in the chain is missing, often the static "
                    "mount transform from the robot body to the sensor.",
                    data={"frame": frame, "topics": sorted(topics)},
                )
            )
    return summary, findings
