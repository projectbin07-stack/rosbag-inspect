"""Turn decoded messages into JSON-friendly values and flat numeric CSV rows."""

from __future__ import annotations

import math
from typing import Any

from .cdr import Message, SkippedBytes
from .msgdef import PRIMITIVE_SIZES, FieldType, Registry

MAX_INLINE_BYTES = 64


def to_jsonable(value: Any) -> Any:
    """Convert decoded values: non-finite floats become strings, skipped payloads a marker."""
    if isinstance(value, dict):
        return {k: to_jsonable(v) for k, v in value.items()}
    if isinstance(value, list):
        return [to_jsonable(v) for v in value]
    if isinstance(value, SkippedBytes):
        return {"skipped_bytes": value.length}
    if isinstance(value, (bytes, bytearray)):
        if len(value) > MAX_INLINE_BYTES:
            return {"bytes": len(value)}
        return list(value)
    if isinstance(value, float) and not math.isfinite(value):
        if value != value:
            return "NaN"
        return "Infinity" if value > 0 else "-Infinity"
    return value


def flat_columns(registry: Registry, type_name: str) -> list[str]:
    """Column names of :func:`flatten` for a type (sequences contribute ``name.length``)."""
    columns: list[str] = []

    def walk(name: str, prefix: str) -> None:
        for fld in registry.get(name).fields:
            path = f"{prefix}{fld.name}"
            _columns_for(fld.type, path, columns, walk)

    walk(type_name, "")
    return columns


def _columns_for(ftype: FieldType, path: str, columns: list[str], walk: Any) -> None:
    if ftype.array == "sequence":
        columns.append(f"{path}.length")
        return
    if ftype.array == "fixed":
        assert ftype.length is not None
        single = FieldType(ftype.base, string_bound=ftype.string_bound)
        for i in range(ftype.length):
            _columns_for(single, f"{path}.{i}", columns, walk)
        return
    if ftype.base in PRIMITIVE_SIZES:
        columns.append(path)
    elif ftype.base not in ("string", "wstring"):
        walk(ftype.base, f"{path}.")


def flatten(registry: Registry, type_name: str, msg: Message) -> dict[str, Any]:
    """Numeric leaves of a message keyed by dotted path (bools as 0/1, strings dropped)."""
    row: dict[str, Any] = {}

    def walk(name: str, value: Message, prefix: str) -> None:
        for fld in registry.get(name).fields:
            _flatten_value(fld.type, value[fld.name], f"{prefix}{fld.name}", row, walk)

    walk(type_name, msg, "")
    return row


def _flatten_value(ftype: FieldType, value: Any, path: str, row: dict[str, Any], walk: Any) -> None:
    if ftype.array == "sequence":
        row[f"{path}.length"] = value.length if isinstance(value, SkippedBytes) else len(value)
        return
    if ftype.array == "fixed":
        single = FieldType(ftype.base, string_bound=ftype.string_bound)
        items = list(value) if not isinstance(value, SkippedBytes) else [None] * value.length
        for i, item in enumerate(items):
            _flatten_value(single, item, f"{path}.{i}", row, walk)
        return
    if ftype.base in PRIMITIVE_SIZES:
        row[path] = int(value) if isinstance(value, bool) else value
    elif ftype.base not in ("string", "wstring"):
        walk(ftype.base, value, f"{path}.")
