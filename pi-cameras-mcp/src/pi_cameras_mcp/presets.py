"""Resolution presets.

The C920's useful modes are discrete MJPEG sizes. Picking one of those lets the
Pi copy frames instead of decoding and encoding them. 1280x720 at 15 fps is the
default because it is one of those native modes and about 4 Mbps, which is
comfortable over Tailscale.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Preset:
    id: str
    width: int
    height: int
    fps: int
    quality: int
    summary: str
    stream_default: bool = False
    snapshot_default: bool = False


PRESETS: dict[str, Preset] = {
    "tiny": Preset(
        "tiny",
        640,
        480,
        10,
        60,
        "Status checks and slow links. About 1 Mbps.",
    ),
    "low": Preset(
        "low",
        640,
        480,
        15,
        70,
        "Remote viewing when the link is tight. About 1.5 Mbps.",
    ),
    "balanced": Preset(
        "balanced",
        1280,
        720,
        15,
        75,
        "Sweet spot over Tailscale. Native C920 MJPEG, no Pi re-encode, about 4 Mbps.",
        stream_default=True,
    ),
    "smooth": Preset(
        "smooth",
        1280,
        720,
        30,
        80,
        "Same 720p picture at 30 fps for a LAN. About 8 Mbps.",
    ),
    "detail": Preset(
        "detail",
        1920,
        1080,
        15,
        82,
        "1080p for snapshots and reading small text. About 8 Mbps as a stream.",
        snapshot_default=True,
    ),
    "max": Preset(
        "max",
        1920,
        1080,
        30,
        85,
        "1080p30. LAN only, about 18 Mbps. The C920 can do this natively.",
    ),
    "full": Preset(
        "full",
        0,
        0,
        5,
        90,
        "Largest still the sensor can produce. Streams above 1080p are capped at 10 fps.",
    ),
}

SWEET_SPOT = "balanced"
SNAPSHOT_DEFAULT = "detail"

# Documented C920 MJPEG modes. Used when v4l2 probing has not run yet.
C920_MJPEG_MODES: tuple[tuple[int, int, tuple[int, ...]], ...] = (
    (640, 480, (30, 15, 10)),
    (800, 600, (30,)),
    (960, 720, (30,)),
    (1280, 720, (30, 15)),
    (1600, 896, (30,)),
    (1920, 1080, (30, 15)),
)


@dataclass(frozen=True)
class Mode:
    width: int
    height: int
    fps: tuple[int, ...]
    pixel_format: str = "MJPG"


def fallback_c920_modes() -> tuple[Mode, ...]:
    return tuple(Mode(width, height, fps, "MJPG") for width, height, fps in C920_MJPEG_MODES)


def merge_modes(modes: list[Mode]) -> list[Mode]:
    merged: dict[tuple[int, int], set[int]] = {}
    for mode in modes:
        if mode.pixel_format != "MJPG":
            continue
        merged.setdefault((mode.width, mode.height), set()).update(mode.fps)
    return [
        Mode(width, height, tuple(sorted(fps)), "MJPG")
        for (width, height), fps in sorted(merged.items())
        if fps
    ]


def even(value: int) -> int:
    value = int(value)
    if value % 2:
        value -= 1
    return max(2, value)


def clamp_dim(value: int, *, low: int, high: int) -> int:
    return even(min(high, max(low, value)))


def estimate_mbps(width: int, height: int, fps: int, quality: int) -> float:
    """Rough MJPEG bitrate. Tuned so 720p15 q75 lands near 4 Mbps."""
    bits_per_pixel = 0.12 + (quality / 100) * 0.22
    return round(width * height * fps * bits_per_pixel / 1_000_000, 1)


def choose_mode(modes: list[Mode], width: int, height: int, fps: int) -> tuple[Mode, int, str]:
    """Pick a native MJPEG mode and a frame rate the camera actually lists.

    Prefers an exact size, otherwise the smallest mode that covers the request,
    otherwise the closest mode by pixel count. Frame rate is the highest listed
    rate that does not exceed the request, or the camera's minimum if the
    request is slower than every listed rate.
    """
    if not modes:
        raise ValueError("no modes")
    exact = [mode for mode in modes if mode.width == width and mode.height == height]
    if exact:
        mode = exact[0]
        note = "exact native MJPEG mode"
    else:
        covering = [mode for mode in modes if mode.width >= width and mode.height >= height]
        pool = covering or modes
        mode = min(pool, key=lambda item: abs(item.width * item.height - width * height))
        if covering:
            note = f"snapped {width}x{height} to native {mode.width}x{mode.height} so the Pi does not re-encode"
        else:
            note = f"camera max is {mode.width}x{mode.height}"
    choices = sorted(set(mode.fps))
    under = [item for item in choices if item <= fps]
    if under:
        chosen = under[-1]
        if chosen != fps:
            note = f"{note}; fps {fps} is not a listed rate, using {chosen}"
    else:
        chosen = choices[0]
        note = f"{note}; fps raised to {chosen}, the slowest rate this mode lists"
    return mode, chosen, note


def ffmpeg_qscale(quality: int) -> str:
    """Map a 1-100 JPEG quality onto ffmpeg's mjpeg qscale (2 is best)."""
    quality = min(100, max(1, quality))
    qscale = 2 + (100 - quality) * 5 // 50
    return str(min(15, max(2, qscale)))


def preset_catalog() -> list[dict[str, object]]:
    rows = []
    for preset in PRESETS.values():
        width = preset.width or None
        height = preset.height or None
        rows.append(
            {
                "id": preset.id,
                "width": width,
                "height": height,
                "fps": preset.fps,
                "quality": preset.quality,
                "summary": preset.summary,
                "estimated_mbps": (
                    estimate_mbps(preset.width, preset.height, preset.fps, preset.quality)
                    if preset.width and preset.height
                    else None
                ),
                "stream_default": preset.stream_default,
                "snapshot_default": preset.snapshot_default,
            }
        )
    return rows
