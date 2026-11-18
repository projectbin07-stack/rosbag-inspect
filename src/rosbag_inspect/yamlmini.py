"""A deliberately small YAML subset parser for rosbag2 ``metadata.yaml``.

It never constructs arbitrary objects: the result is built only from ``dict``, ``list``,
``str``, ``int``, ``float``, ``bool`` and ``None``. Supported:

- block mappings (``key: value``) and block sequences (``- item``), including the compact
  form ``- key: value`` that starts a mapping inside a sequence item
- plain, single-quoted and double-quoted scalars (double-quoted ones may span lines and use
  the usual backslash escapes)
- literal (``|``) and folded (``>``) block scalars
- empty flow collections ``[]`` / ``{}`` and flat flow sequences/maps of scalars
- comments, ``~``/``null``, booleans, integers and floats

Anchors, aliases, tags, multi-document streams and complex keys are rejected with
:class:`YamlError`, which is enough for files written by rosbag2's yaml-cpp emitter.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

YamlValue = object

_INT_RE = re.compile(r"[-+]?(0|[1-9][0-9]*)$")
_FLOAT_RE = re.compile(r"[-+]?([0-9]+\.[0-9]*|\.[0-9]+|[0-9]+)([eE][-+]?[0-9]+)?$")
_ESCAPES = {
    "0": "\0",
    "a": "\a",
    "b": "\b",
    "t": "\t",
    "\t": "\t",
    "n": "\n",
    "v": "\v",
    "f": "\f",
    "r": "\r",
    "e": "\x1b",
    " ": " ",
    '"': '"',
    "/": "/",
    "\\": "\\",
    "N": "\x85",
    "_": "\xa0",
}


class YamlError(ValueError):
    """Raised for input outside the supported subset or malformed YAML."""


@dataclass
class _Line:
    number: int
    indent: int
    text: str


def _strip_comment(text: str) -> str:
    """Remove a trailing ``# comment`` that is outside quotes."""
    quote = ""
    for i, ch in enumerate(text):
        if quote:
            if ch == quote:
                if quote == "'" and i + 1 < len(text) and text[i + 1] == "'":
                    continue
                quote = ""
            elif ch == "\\" and quote == '"':
                continue
        elif ch in "\"'":
            if i == 0 or text[i - 1] in " \t:-[{,":
                quote = ch
        elif ch == "#" and (i == 0 or text[i - 1] in " \t"):
            return text[:i].rstrip()
    return text.rstrip()


def scalar(token: str) -> YamlValue:
    """Convert a plain scalar to ``None``/bool/int/float/str following YAML 1.2 core rules."""
    if token in ("", "~", "null", "Null", "NULL"):
        return None
    if token in ("true", "True", "TRUE"):
        return True
    if token in ("false", "False", "FALSE"):
        return False
    if _INT_RE.match(token):
        return int(token)
    if token.startswith(("0x", "0X")):
        try:
            return int(token, 16)
        except ValueError:
            return token
    if _FLOAT_RE.match(token):
        return float(token)
    if token.lower() in (".inf", "+.inf"):
        return float("inf")
    if token.lower() == "-.inf":
        return float("-inf")
    if token.lower() == ".nan":
        return float("nan")
    return token


def _unescape_double(body: str, line: int) -> str:
    out: list[str] = []
    i = 0
    while i < len(body):
        ch = body[i]
        if ch != "\\":
            out.append(ch)
            i += 1
            continue
        if i + 1 >= len(body):
            raise YamlError(f"line {line}: dangling backslash in quoted string")
        code = body[i + 1]
        if code in _ESCAPES:
            out.append(_ESCAPES[code])
            i += 2
        elif code in "xuU":
            width = {"x": 2, "u": 4, "U": 8}[code]
            digits = body[i + 2 : i + 2 + width]
            if len(digits) != width:
                raise YamlError(f"line {line}: short \\{code} escape")
            try:
                out.append(chr(int(digits, 16)))
            except ValueError as exc:
                raise YamlError(f"line {line}: bad \\{code} escape") from exc
            i += 2 + width
        else:
            raise YamlError(f"line {line}: unknown escape \\{code}")
    return "".join(out)


def _fold_lines(parts: list[str]) -> str:
    """Join the physical lines of a multi-line flow scalar (empty line -> newline)."""
    result = ""
    pending_newlines = 0
    for index, part in enumerate(parts):
        piece = part.strip() if 0 < index < len(parts) - 1 else part
        if index == 0:
            piece = part.rstrip()
        elif index == len(parts) - 1:
            piece = part.lstrip()
        if index > 0 and piece == "" and index < len(parts) - 1:
            pending_newlines += 1
            continue
        if index > 0:
            if pending_newlines:
                result += "\n" * pending_newlines
            elif result and not result.endswith("\n"):
                result += " "
            pending_newlines = 0
        result += piece
    return result


class _Parser:
    def __init__(self, text: str) -> None:
        self.raw = text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
        self.pos = 0

    # -- line helpers -------------------------------------------------------------------
    def _peek(self) -> _Line | None:
        while self.pos < len(self.raw):
            raw = self.raw[self.pos]
            if "\t" in raw[: len(raw) - len(raw.lstrip())]:
                raise YamlError(f"line {self.pos + 1}: tab used for indentation")
            stripped = _strip_comment(raw)
            if stripped.strip() == "" or stripped.strip() in ("---",):
                self.pos += 1
                continue
            if stripped.strip() == "...":
                self.pos = len(self.raw)
                return None
            indent = len(stripped) - len(stripped.lstrip(" "))
            return _Line(self.pos + 1, indent, stripped.strip())
        return None

    # -- values -------------------------------------------------------------------------
    def parse_document(self) -> YamlValue:
        first = self._peek()
        if first is None:
            return None
        value = self._parse_node(first.indent)
        rest = self._peek()
        if rest is not None:
            raise YamlError(f"line {rest.number}: unexpected content {rest.text!r}")
        return value

    def _parse_node(self, indent: int) -> YamlValue:
        line = self._peek()
        assert line is not None
        if line.text.startswith("- ") or line.text == "-":
            return self._parse_sequence(line.indent)
        if self._split_key(line.text, line.number) is not None:
            return self._parse_mapping(line.indent)
        self.pos += 1
        return self._inline_value(line.text, line.number, line.indent)

    def _parse_sequence(self, indent: int) -> list[YamlValue]:
        items: list[YamlValue] = []
        while True:
            line = self._peek()
            if line is None or line.indent < indent:
                break
            if line.indent > indent:
                raise YamlError(f"line {line.number}: bad indentation")
            if not (line.text.startswith("- ") or line.text == "-"):
                break
            rest = line.text[1:].lstrip(" ")
            if rest == "":
                self.pos += 1
                nxt = self._peek()
                if nxt is None or nxt.indent <= indent:
                    items.append(None)
                else:
                    items.append(self._parse_node(nxt.indent))
                continue
            # The item content starts at this column.
            item_indent = line.indent + (len(line.text) - len(rest))
            if rest.startswith("- ") or rest == "-":
                self.raw[self.pos] = " " * item_indent + rest
                items.append(self._parse_sequence(item_indent))
            elif self._split_key(rest, line.number) is not None:
                self.raw[self.pos] = " " * item_indent + rest
                items.append(self._parse_mapping(item_indent))
            else:
                self.pos += 1
                items.append(self._inline_value(rest, line.number, item_indent))
        return items

    def _parse_mapping(self, indent: int) -> dict[str, YamlValue]:
        result: dict[str, YamlValue] = {}
        while True:
            line = self._peek()
            if line is None or line.indent < indent:
                break
            if line.indent > indent:
                raise YamlError(f"line {line.number}: bad indentation")
            split = self._split_key(line.text, line.number)
            if split is None:
                break
            key, rest = split
            if key in result:
                raise YamlError(f"line {line.number}: duplicate key {key!r}")
            self.pos += 1
            if rest == "":
                nxt = self._peek()
                if nxt is not None and (
                    nxt.indent > indent
                    or (nxt.indent == indent and (nxt.text.startswith("- ") or nxt.text == "-"))
                ):
                    result[key] = self._parse_node(nxt.indent)
                else:
                    result[key] = None
            else:
                result[key] = self._inline_value(rest, line.number, indent)
        return result

    @staticmethod
    def _split_key(text: str, number: int) -> tuple[str, str] | None:
        """Split ``key: rest``; returns ``None`` if the line is not a mapping entry."""
        if text.startswith(("&", "*", "!", "? ", "%")):
            raise YamlError(f"line {number}: anchors, aliases, tags and complex keys unsupported")
        if text[0] in "\"'":
            quote = text[0]
            end = 1
            while end < len(text):
                if text[end] == "\\" and quote == '"':
                    end += 2
                    continue
                if text[end] == quote:
                    if quote == "'" and end + 1 < len(text) and text[end + 1] == "'":
                        end += 2
                        continue
                    break
                end += 1
            after = text[end + 1 :]
            if after.startswith(":") and (len(after) == 1 or after[1] == " "):
                raw_key = text[: end + 1]
                key = _Parser._quoted(raw_key, number)
                return key, after[1:].strip()
            return None
        if text[0] in "[{":
            return None
        match = re.match(r"([^:#]*?[^ :])?:( |$)", text)
        if match is None or match.group(1) is None:
            return None
        return match.group(1), text[match.end() :].strip()

    @staticmethod
    def _quoted(token: str, number: int) -> str:
        if token[0] == "'":
            return token[1:-1].replace("''", "'")
        return _unescape_double(token[1:-1], number)

    def _collect_continuation(self, first: str, number: int, quote: str) -> str:
        """Collect a quoted scalar that continues on following physical lines."""
        parts = [first]
        while True:
            joined = "\n".join(parts)
            if self._closed(joined, quote):
                return _fold_lines(parts)
            if self.pos >= len(self.raw):
                raise YamlError(f"line {number}: unterminated quoted string")
            parts.append(self.raw[self.pos])
            self.pos += 1

    @staticmethod
    def _closed(text: str, quote: str) -> bool:
        i = 1
        while i < len(text):
            ch = text[i]
            if quote == '"' and ch == "\\":
                i += 2
                continue
            if ch == quote:
                if quote == "'" and i + 1 < len(text) and text[i + 1] == "'":
                    i += 2
                    continue
                return text[i + 1 :].strip() == ""
            i += 1
        return False

    def _block_scalar(self, header: str, number: int, parent_indent: int) -> str:
        style = header[0]
        chomp = header[1:].strip()
        lines: list[str] = []
        block_indent: int | None = None
        while self.pos < len(self.raw):
            raw = self.raw[self.pos]
            if raw.strip() == "":
                lines.append("")
                self.pos += 1
                continue
            indent = len(raw) - len(raw.lstrip(" "))
            if indent <= parent_indent:
                break
            if block_indent is None:
                block_indent = indent
            if indent < block_indent:
                break
            lines.append(raw[block_indent:])
            self.pos += 1
        while lines and lines[-1] == "":
            lines.pop()
        text = "\n".join(lines) if style == "|" else _fold_lines(lines) if lines else ""
        if chomp == "-" or not lines:
            return text
        if chomp in ("", "+"):
            return text + "\n"
        raise YamlError(f"line {number}: unsupported block scalar header {header!r}")

    def _inline_value(self, text: str, number: int, indent: int) -> YamlValue:
        """Parse a value that begins on the current line (already consumed)."""
        if text.startswith(("&", "*", "!")):
            raise YamlError(f"line {number}: anchors, aliases and tags are not supported")
        if text[0] in "\"'":
            full = self._collect_continuation(text, number, text[0])
            if text[0] == "'":
                return full.strip()[1:-1].replace("''", "'")
            return _unescape_double(full.strip()[1:-1], number)
        if text[0] in "|>":
            return self._block_scalar(text, number, indent)
        if text[0] == "[":
            return self._flow_sequence(text, number)
        if text[0] == "{":
            return self._flow_mapping(text, number)
        return scalar(text)

    def _flow_items(self, body: str, number: int) -> list[str]:
        if "[" in body or "{" in body:
            raise YamlError(f"line {number}: nested flow collections are not supported")
        items: list[str] = []
        current = ""
        quote = ""
        for ch in body:
            if quote:
                current += ch
                if ch == quote:
                    quote = ""
            elif ch in "\"'":
                quote = ch
                current += ch
            elif ch == ",":
                items.append(current.strip())
                current = ""
            else:
                current += ch
        if current.strip():
            items.append(current.strip())
        return items

    def _flow_scalar(self, token: str, number: int) -> YamlValue:
        if token and token[0] in "\"'":
            return self._quoted(token, number)
        return scalar(token)

    def _flow_sequence(self, text: str, number: int) -> list[YamlValue]:
        if not text.endswith("]"):
            raise YamlError(f"line {number}: multi-line flow sequences are not supported")
        return [self._flow_scalar(t, number) for t in self._flow_items(text[1:-1], number)]

    def _flow_mapping(self, text: str, number: int) -> dict[str, YamlValue]:
        if not text.endswith("}"):
            raise YamlError(f"line {number}: multi-line flow mappings are not supported")
        result: dict[str, YamlValue] = {}
        for item in self._flow_items(text[1:-1], number):
            key, sep, value = item.partition(":")
            if not sep:
                raise YamlError(f"line {number}: flow mapping entry without ':'")
            result[str(self._flow_scalar(key.strip(), number))] = self._flow_scalar(
                value.strip(), number
            )
        return result


def loads(text: str) -> YamlValue:
    """Parse a YAML document in the supported subset."""
    return _Parser(text).parse_document()
