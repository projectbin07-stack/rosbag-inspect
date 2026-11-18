"""Pure-Python LZ4 frame decompression (used for LZ4-compressed MCAP chunks).

Written from the public LZ4 Frame Format (v1.6.x) and LZ4 Block Format descriptions. Header
and content checksums (xxHash32) are not verified; the MCAP chunk CRC32 is checked by the
caller instead. This is slow compared to the C library but needs nothing outside the standard
library.
"""

from __future__ import annotations

import struct

FRAME_MAGIC = 0x184D2204
_SKIPPABLE_MASK = 0xFFFFFFF0
_SKIPPABLE_MAGIC = 0x184D2A50


class LZ4Error(ValueError):
    """Malformed LZ4 data."""


def decompress_block(src: bytes | memoryview, out: bytearray, limit: int | None = None) -> None:
    """Decode one LZ4 block, appending to ``out`` (which may hold earlier blocks' data)."""
    pos = 0
    end = len(src)
    while pos < end:
        token = src[pos]
        pos += 1
        literal_len = token >> 4
        if literal_len == 15:
            while True:
                if pos >= end:
                    raise LZ4Error("truncated literal length")
                extra = src[pos]
                pos += 1
                literal_len += extra
                if extra != 255:
                    break
        if pos + literal_len > end:
            raise LZ4Error("literal run past end of block")
        out += src[pos : pos + literal_len]
        pos += literal_len
        if pos >= end:
            break  # the last sequence has literals only
        if pos + 2 > end:
            raise LZ4Error("truncated match offset")
        offset = src[pos] | (src[pos + 1] << 8)
        pos += 2
        if offset == 0 or offset > len(out):
            raise LZ4Error(f"invalid match offset {offset}")
        match_len = token & 0x0F
        if match_len == 15:
            while True:
                if pos >= end:
                    raise LZ4Error("truncated match length")
                extra = src[pos]
                pos += 1
                match_len += extra
                if extra != 255:
                    break
        match_len += 4
        start = len(out) - offset
        if offset >= match_len:
            out += out[start : start + match_len]
        else:
            # Overlapping copy: repeat the last ``offset`` bytes.
            chunk = bytes(out[start:])
            repeats, rest = divmod(match_len, offset)
            out += chunk * repeats + chunk[:rest]
        if limit is not None and len(out) > limit:
            raise LZ4Error("decompressed data exceeds the declared size")


def decompress_frame(data: bytes, expected_size: int | None = None) -> bytes:
    """Decompress one or more concatenated LZ4 frames."""
    out = bytearray()
    view = memoryview(data)
    pos = 0
    while pos < len(data):
        if pos + 4 > len(data):
            raise LZ4Error("truncated frame magic")
        magic = struct.unpack_from("<I", data, pos)[0]
        if (magic & _SKIPPABLE_MASK) == _SKIPPABLE_MAGIC:
            size = struct.unpack_from("<I", data, pos + 4)[0]
            pos += 8 + size
            continue
        if magic != FRAME_MAGIC:
            raise LZ4Error(f"bad LZ4 frame magic 0x{magic:08x}")
        pos += 4
        if pos + 3 > len(data):
            raise LZ4Error("truncated frame descriptor")
        flg = data[pos]
        if (flg >> 6) != 1:
            raise LZ4Error(f"unsupported LZ4 frame version {flg >> 6}")
        block_checksum = bool(flg & 0x10)
        content_size_flag = bool(flg & 0x08)
        content_checksum = bool(flg & 0x04)
        dict_id = bool(flg & 0x01)
        pos += 2  # FLG and BD
        if content_size_flag:
            pos += 8
        if dict_id:
            raise LZ4Error("LZ4 frames with a dictionary are not supported")
        pos += 1  # header checksum (not verified)
        frame_start = len(out)
        while True:
            if pos + 4 > len(data):
                raise LZ4Error("truncated block size")
            word = struct.unpack_from("<I", data, pos)[0]
            pos += 4
            if word == 0:
                break
            size = word & 0x7FFFFFFF
            if pos + size > len(data):
                raise LZ4Error("truncated block")
            block = view[pos : pos + size]
            if word & 0x80000000:
                out += block
            else:
                limit = None if expected_size is None else expected_size
                # Blocks may reference earlier blocks of the same frame (linked mode).
                decompress_block(block, out, limit)
            pos += size + (4 if block_checksum else 0)
        if content_checksum:
            pos += 4
        if len(out) < frame_start:  # pragma: no cover - defensive
            raise LZ4Error("frame shrank output")
    if expected_size is not None and len(out) != expected_size:
        raise LZ4Error(f"decompressed {len(out)} bytes, expected {expected_size}")
    return bytes(out)
