"""JPEG framing for concatenated MJPEG streams.

UVC and rpicam both write a raw sequence of JPEG files. Baseline JPEG byte-stuffs
0xFF as 0xFF 0x00, so 0xFFD8 / 0xFFD9 are the real start and end markers.
"""

from __future__ import annotations


class JpegSplitter:
    def __init__(self, max_bytes: int = 12_000_000) -> None:
        self._buf = bytearray()
        self._max = max_bytes

    def feed(self, data: bytes) -> list[bytes]:
        if not data:
            return []
        self._buf.extend(data)
        frames: list[bytes] = []
        while True:
            start = self._buf.find(b"\xff\xd8")
            if start < 0:
                if len(self._buf) > 1:
                    del self._buf[:-1]
                break
            if start:
                del self._buf[:start]
            end = self._buf.find(b"\xff\xd9", 2)
            if end < 0:
                if len(self._buf) > self._max:
                    del self._buf[:2]
                    continue
                break
            frames.append(bytes(self._buf[: end + 2]))
            del self._buf[: end + 2]
        return frames


def split_jpegs(data: bytes) -> list[bytes]:
    return JpegSplitter().feed(data)


def jpeg_size(data: bytes) -> tuple[int, int] | None:
    """Return (width, height) from a SOF marker, or None if the header is incomplete."""
    if len(data) < 4 or data[0:2] != b"\xff\xd8":
        return None
    i = 2
    size = len(data)
    while i < size - 8:
        if data[i] != 0xFF:
            i += 1
            continue
        while i < size and data[i] == 0xFF:
            i += 1
        if i >= size:
            return None
        marker = data[i]
        i += 1
        if marker in (0xD8, 0xD9, 0x01) or 0xD0 <= marker <= 0xD7:
            continue
        if i + 2 > size:
            return None
        seglen = int.from_bytes(data[i : i + 2], "big")
        if seglen < 2 or i + seglen > size:
            return None
        if marker in (0xC0, 0xC1, 0xC2):
            height = int.from_bytes(data[i + 3 : i + 5], "big")
            width = int.from_bytes(data[i + 5 : i + 7], "big")
            if width > 0 and height > 0:
                return width, height
            return None
        i += seglen
    return None
