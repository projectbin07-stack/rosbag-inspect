"""OMG CDR (XCDR1, and plain XCDR2 for final types) driven by parsed ``.msg`` definitions.

A serialized ROS 2 message starts with a 4-byte encapsulation header: two bytes of
representation identifier (big-endian on the wire) and two option bytes. The body follows.
Primitive values are aligned to their own size, measured from the first byte after the
header. In XCDR1 the largest alignment is 8; in XCDR2 it is 4. Strings are a ``uint32``
length that counts the terminating NUL, then the bytes and the NUL. Sequences are a
``uint32`` element count followed by the elements; fixed arrays have no count. Nested
messages are serialized field by field with no padding of their own.

Supported identifiers: ``0x0000`` CDR_BE, ``0x0001`` CDR_LE, ``0x0006`` CDR2_BE and
``0x0007`` CDR2_LE. Parameter-list and delimited encodings are rejected with
:class:`CdrError`, as is ``wstring``.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any

from .msgdef import PRIMITIVE_SIZES, FieldType, MsgDefError, Registry

Message = dict[str, Any]

FORMATS: dict[str, str] = {
    "bool": "?",
    "byte": "B",
    "char": "B",
    "int8": "b",
    "uint8": "B",
    "int16": "h",
    "uint16": "H",
    "int32": "i",
    "uint32": "I",
    "int64": "q",
    "uint64": "Q",
    "float32": "f",
    "float64": "d",
}
BYTE_TYPES = ("uint8", "byte", "char")

ENCAPSULATIONS: dict[int, tuple[str, int, str]] = {
    0x0000: (">", 8, "CDR_BE"),
    0x0001: ("<", 8, "CDR_LE"),
    0x0006: (">", 4, "CDR2_BE"),
    0x0007: ("<", 4, "CDR2_LE"),
}
_UNSUPPORTED = {
    0x0002: "PL_CDR_BE",
    0x0003: "PL_CDR_LE",
    0x0008: "D_CDR2_BE",
    0x0009: "D_CDR2_LE",
    0x000A: "PL_CDR2_BE",
    0x000B: "PL_CDR2_LE",
}

DEFAULT_MAX_BYTES = 4096


class CdrError(ValueError):
    """The payload does not match the definition or uses an unsupported encoding."""


@dataclass(frozen=True)
class SkippedBytes:
    """A ``uint8[]``/``byte[]`` payload that was deliberately not copied (e.g. image pixels).

    ``available`` is ``False`` when the buffer ended before the payload did, which happens
    when only a message prefix was loaded.
    """

    length: int
    available: bool = True


class _Reader:
    __slots__ = ("buf", "end", "endian", "max_align", "max_bytes", "pos", "registry")

    def __init__(
        self, registry: Registry, buf: bytes, endian: str, max_align: int, max_bytes: int | None
    ) -> None:
        self.registry = registry
        self.buf = buf
        self.pos = 4
        self.end = len(buf)
        self.endian = endian
        self.max_align = max_align
        self.max_bytes = max_bytes

    def align(self, size: int) -> None:
        size = min(size, self.max_align)
        self.pos += (-(self.pos - 4)) % size

    def need(self, count: int, what: str) -> None:
        if self.pos + count > self.end:
            raise CdrError(
                f"truncated payload: need {count} bytes for {what} at offset {self.pos - 4}, "
                f"{max(self.end - self.pos, 0)} left"
            )

    def primitive(self, base: str, what: str) -> Any:
        size = PRIMITIVE_SIZES[base]
        self.align(size)
        self.need(size, what)
        value = struct.unpack_from(self.endian + FORMATS[base], self.buf, self.pos)[0]
        self.pos += size
        return value

    def count(self, what: str) -> int:
        return int(self.primitive("uint32", f"length of {what}"))

    def string(self, what: str, bound: int | None) -> str:
        length = self.count(what)
        if length == 0:
            return ""
        self.need(length, what)
        raw = self.buf[self.pos : self.pos + length]
        self.pos += length
        if raw[-1:] == b"\0":
            raw = raw[:-1]
        if bound is not None and len(raw) > bound:
            raise CdrError(f"{what}: string of {len(raw)} bytes exceeds bound {bound}")
        return raw.decode("utf-8", errors="replace")

    def primitive_array(self, base: str, count: int, what: str) -> Any:
        if count == 0:
            return b"" if base in BYTE_TYPES else []
        size = PRIMITIVE_SIZES[base]
        self.align(size)
        if base in BYTE_TYPES:
            if self.max_bytes is not None and count > self.max_bytes:
                available = self.pos + count <= self.end
                self.pos += count
                return SkippedBytes(count, available)
            self.need(count, what)
            raw = bytes(self.buf[self.pos : self.pos + count])
            self.pos += count
            return raw
        self.need(size * count, what)
        values = struct.unpack_from(f"{self.endian}{count}{FORMATS[base]}", self.buf, self.pos)
        self.pos += size * count
        return list(values)

    def value(self, ftype: FieldType, what: str) -> Any:
        if ftype.array == "":
            return self.single(ftype, what)
        if ftype.array == "fixed":
            assert ftype.length is not None
            count = ftype.length
        else:
            count = self.count(what)
            if ftype.length is not None and count > ftype.length:
                raise CdrError(f"{what}: sequence of {count} exceeds bound {ftype.length}")
            skipped = (
                ftype.base in BYTE_TYPES and self.max_bytes is not None and count > self.max_bytes
            )
            if count > self.end and not skipped:
                raise CdrError(f"{what}: implausible sequence length {count}")
        if ftype.base in PRIMITIVE_SIZES:
            return self.primitive_array(ftype.base, count, what)
        element = FieldType(ftype.base, string_bound=ftype.string_bound)
        return [self.single(element, f"{what}[{i}]") for i in range(count)]

    def single(self, ftype: FieldType, what: str) -> Any:
        base = ftype.base
        if base in PRIMITIVE_SIZES:
            return self.primitive(base, what)
        if base == "string":
            return self.string(what, ftype.string_bound)
        if base == "wstring":
            raise CdrError(f"{what}: wstring is not supported")
        return self.message(base, what)

    def message(self, type_name: str, what: str) -> Message:
        try:
            definition = self.registry.get(type_name)
        except MsgDefError as exc:
            raise CdrError(str(exc)) from exc
        result: Message = {}
        for fld in definition.fields:
            result[fld.name] = self.value(fld.type, f"{what}.{fld.name}" if what else fld.name)
        return result


def encapsulation(data: bytes) -> tuple[str, int, str]:
    """Return ``(struct endian prefix, max alignment, name)`` for a payload header."""
    if len(data) < 4:
        raise CdrError(f"payload of {len(data)} bytes is shorter than the encapsulation header")
    ident = (data[0] << 8) | data[1]
    if ident in ENCAPSULATIONS:
        return ENCAPSULATIONS[ident]
    name = _UNSUPPORTED.get(ident, f"0x{ident:04x}")
    raise CdrError(f"unsupported CDR encapsulation {name}")


def decode(
    registry: Registry,
    type_name: str,
    data: bytes,
    max_bytes: int | None = DEFAULT_MAX_BYTES,
) -> Message:
    """Decode a serialized message.

    ``uint8``/``byte``/``char`` arrays longer than ``max_bytes`` are not copied and come back
    as :class:`SkippedBytes`; pass ``None`` to copy them all.
    """
    return decode_with_size(registry, type_name, data, max_bytes)[0]


def decode_with_size(
    registry: Registry,
    type_name: str,
    data: bytes,
    max_bytes: int | None = DEFAULT_MAX_BYTES,
) -> tuple[Message, int]:
    """Like :func:`decode`, also returning how many payload bytes the definition consumed
    (including the 4-byte header). Serializers may pad the end to a multiple of 4, so a
    correct definition leaves 0-3 bytes unread."""
    endian, max_align, _ = encapsulation(data)
    reader = _Reader(registry, data, endian, max_align, max_bytes)
    message = reader.message(type_name, "")
    return message, reader.pos


# -- encoder ----------------------------------------------------------------------------------


class _Writer:
    def __init__(self, registry: Registry, endian: str, max_align: int) -> None:
        self.registry = registry
        self.endian = endian
        self.max_align = max_align
        self.out = bytearray()

    def align(self, size: int) -> None:
        size = min(size, self.max_align)
        self.out.extend(b"\0" * ((-len(self.out)) % size))

    def primitive(self, base: str, value: Any) -> None:
        self.align(PRIMITIVE_SIZES[base])
        if base == "bool":
            value = bool(value)
        self.out.extend(struct.pack(self.endian + FORMATS[base], value))

    def string(self, value: str) -> None:
        raw = value.encode("utf-8")
        self.primitive("uint32", len(raw) + 1)
        self.out.extend(raw + b"\0")

    def value(self, ftype: FieldType, value: Any) -> None:
        if ftype.array == "":
            self.single(ftype, value)
            return
        items = list(value) if value is not None else []
        if ftype.array == "fixed":
            assert ftype.length is not None
            if len(items) != ftype.length:
                raise CdrError(f"fixed array needs {ftype.length} elements, got {len(items)}")
        else:
            self.primitive("uint32", len(items))
        if not items:
            return
        if ftype.base in PRIMITIVE_SIZES:
            self.align(PRIMITIVE_SIZES[ftype.base])
            if ftype.base == "bool":
                items = [bool(v) for v in items]
            self.out.extend(struct.pack(f"{self.endian}{len(items)}{FORMATS[ftype.base]}", *items))
            return
        element = FieldType(ftype.base, string_bound=ftype.string_bound)
        for item in items:
            self.single(element, item)

    def single(self, ftype: FieldType, value: Any) -> None:
        if ftype.base in PRIMITIVE_SIZES:
            self.primitive(ftype.base, 0 if value is None else value)
        elif ftype.base == "string":
            self.string("" if value is None else str(value))
        elif ftype.base == "wstring":
            raise CdrError("wstring is not supported")
        else:
            self.message(ftype.base, value or {})

    def message(self, type_name: str, value: Message) -> None:
        definition = self.registry.get(type_name)
        for fld in definition.fields:
            self.value(fld.type, value.get(fld.name, default_value(self.registry, fld.type)))


def default_value(registry: Registry, ftype: FieldType) -> Any:
    """Zero value for a field (empty sequences, zeroed fixed arrays, nested defaults)."""
    if ftype.array == "sequence":
        return []
    if ftype.array == "fixed":
        assert ftype.length is not None
        single = FieldType(ftype.base, string_bound=ftype.string_bound)
        return [default_value(registry, single) for _ in range(ftype.length)]
    if ftype.base in PRIMITIVE_SIZES:
        return False if ftype.base == "bool" else (0.0 if "float" in ftype.base else 0)
    if ftype.base in ("string", "wstring"):
        return ""
    definition = registry.get(ftype.base)
    return {f.name: default_value(registry, f.type) for f in definition.fields}


def encode(
    registry: Registry,
    type_name: str,
    value: Message,
    little_endian: bool = True,
    xcdr2: bool = False,
) -> bytes:
    """Serialize ``value`` (missing fields take their zero value) with an encapsulation header."""
    endian = "<" if little_endian else ">"
    writer = _Writer(registry, endian, 4 if xcdr2 else 8)
    writer.message(type_name, value)
    ident = (0x0006 if xcdr2 else 0x0000) | (1 if little_endian else 0)
    return bytes([ident >> 8, ident & 0xFF, 0, 0]) + bytes(writer.out)
