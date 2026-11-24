"""Turn decoded messages into JSON-friendly values and flat numeric CSV rows.

Non-finite floats have no JSON spelling. :func:`to_jsonable` writes them as the strings
``"NaN"``, ``"Infinity"`` and ``"-Infinity"`` by default, or as ``null`` with
``nonfinite="null"`` (for consumers that treat every non-finite value alike, such as a
LaserScan reader for which inf, -inf and NaN all mean "no usable return").

In CSV, :func:`flatten` with ``arrays=True`` also writes every numeric variable-length array
(``LaserScan.ranges``, ``JointState.position``, ...) in full, as one cell of space-separated
numbers. Floats use Python's ``repr`` (shortest round-trip form), so non-finite values are
``inf``, ``-inf`` and ``nan`` and ``float()`` reads every value back exactly.
"""

from __future__ import annotations

import math
from typing import Any

from .cdr import Message, SkippedBytes
from .msgdef import PRIMITIVE_SIZES, FieldType, Registry

MAX_INLINE_BYTES = 64


NONFINITE_MODES = ("string", "null")


def to_jsonable(value: Any, nonfinite: str = "string") -> Any:
    """Convert decoded values for JSON.

    Non-finite floats become strings (``nonfinite="string"``) or ``None``
    (``nonfinite="null"``); skipped payloads become a marker.
    """
    if nonfinite not in NONFINITE_MODES:
        raise ValueError(f"nonfinite must be one of {NONFINITE_MODES}, not {nonfinite!r}")
    return _jsonable(value, nonfinite == "null")


def _jsonable(value: Any, as_null: bool) -> Any:
    if isinstance(value, dict):
        return {k: _jsonable(v, as_null) for k, v in value.items()}
    if isinstance(value, list):
        return [_jsonable(v, as_null) for v in value]
    if isinstance(value, SkippedBytes):
        return {"skipped_bytes": value.length}
    if isinstance(value, (bytes, bytearray)):
        if len(value) > MAX_INLINE_BYTES:
            return {"bytes": len(value)}
        return list(value)
    if isinstance(value, float) and not math.isfinite(value):
        if as_null:
            return None
        if value != value:
            return "NaN"
        return "Infinity" if value > 0 else "-Infinity"
    return value


def flat_columns(registry: Registry, type_name: str, arrays: bool = False) -> list[str]:
    """Column names of :func:`flatten` for a type.

    Sequences contribute ``name.length``; with ``arrays=True`` a sequence of numbers also
    contributes a ``name`` column holding every element.
    """
    columns: list[str] = []

    def walk(name: str, prefix: str) -> None:
        for fld in registry.get(name).fields:
            path = f"{prefix}{fld.name}"
            _columns_for(fld.type, path, columns, walk, arrays)

    walk(type_name, "")
    return columns


def _columns_for(ftype: FieldType, path: str, columns: list[str], walk: Any, arrays: bool) -> None:
    if ftype.array == "sequence":
        columns.append(f"{path}.length")
        if arrays and ftype.base in PRIMITIVE_SIZES:
            columns.append(path)
        return
    if ftype.array == "fixed":
        assert ftype.length is not None
        single = FieldType(ftype.base, string_bound=ftype.string_bound)
        for i in range(ftype.length):
            _columns_for(single, f"{path}.{i}", columns, walk, arrays)
        return
    if ftype.base in PRIMITIVE_SIZES:
        columns.append(path)
    elif ftype.base not in ("string", "wstring"):
        walk(ftype.base, f"{path}.")


def flatten(
    registry: Registry, type_name: str, msg: Message, arrays: bool = False
) -> dict[str, Any]:
    """Numeric leaves of a message keyed by dotted path (bools as 0/1, strings dropped).

    With ``arrays=True`` each numeric sequence is also written in full under its own path as
    space-separated numbers (see :func:`array_cell`).
    """
    row: dict[str, Any] = {}

    def walk(name: str, value: Message, prefix: str) -> None:
        for fld in registry.get(name).fields:
            _flatten_value(fld.type, value[fld.name], f"{prefix}{fld.name}", row, walk, arrays)

    walk(type_name, msg, "")
    return row


def number_text(value: Any) -> str:
    """One number as text that ``float()``/``int()`` read back exactly (``inf``, ``nan``)."""
    if isinstance(value, bool):
        return str(int(value))
    if isinstance(value, float):
        return repr(value)
    return str(value)


def array_cell(values: Any) -> str:
    """A numeric array as one CSV cell: elements separated by single spaces.

    Skipped byte payloads (larger than the decoder's inline limit) give an empty cell.
    """
    if isinstance(values, SkippedBytes):
        return ""
    return " ".join(number_text(v) for v in values)


def _flatten_value(
    ftype: FieldType, value: Any, path: str, row: dict[str, Any], walk: Any, arrays: bool
) -> None:
    if ftype.array == "sequence":
        row[f"{path}.length"] = value.length if isinstance(value, SkippedBytes) else len(value)
        if arrays and ftype.base in PRIMITIVE_SIZES:
            row[path] = array_cell(value)
        return
    if ftype.array == "fixed":
        single = FieldType(ftype.base, string_bound=ftype.string_bound)
        items = list(value) if not isinstance(value, SkippedBytes) else [None] * value.length
        for i, item in enumerate(items):
            _flatten_value(single, item, f"{path}.{i}", row, walk, arrays)
        return
    if ftype.base in PRIMITIVE_SIZES:
        row[path] = int(value) if isinstance(value, bool) else value
    elif ftype.base not in ("string", "wstring"):
        walk(ftype.base, value, f"{path}.")
