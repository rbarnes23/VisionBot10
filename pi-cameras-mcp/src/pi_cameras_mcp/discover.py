"""Find the C920 and the Pi camera without opening them for long."""

from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass, field

from pi_cameras_mcp.presets import Mode, fallback_c920_modes, merge_modes
from pi_cameras_mcp.settings import Settings

_ISP_MARKERS = ("pisp", "unicam", "rp1-cfe", "rpi-hevc", "bcm2835-codec", "bcm2835-isp")
_DEVICE_HEADER = re.compile(r"^(\S.*):\s*$")
_FORMAT_HEADER = re.compile(r"\[(\d+)\]:\s+'([A-Z0-9]+)'")
_SIZE = re.compile(r"Size:\s+Discrete\s+(\d+)x(\d+)")
_FPS = re.compile(r"\((\d+(?:\.\d+)?)\s+fps\)")
_RPICAM = re.compile(r"^\s*(\d+)\s*:\s*(\S+)\s+\[(\d+)x(\d+)")


@dataclass
class CameraInfo:
    id: str
    label: str
    kind: str
    device: str | None
    modes: tuple[Mode, ...] = ()
    sensor: str | None = None
    max_still: tuple[int, int] | None = None
    detail: str = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "id": self.id,
            "label": self.label,
            "kind": self.kind,
            "device": self.device,
            "sensor": self.sensor,
            "max_still": list(self.max_still) if self.max_still else None,
            "modes": [
                {"width": mode.width, "height": mode.height, "fps": list(mode.fps)}
                for mode in self.modes
            ],
            "detail": self.detail,
        }


@dataclass
class Discovery:
    cameras: list[CameraInfo] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, object]:
        return {
            "cameras": [camera.to_dict() for camera in self.cameras],
            "notes": self.notes,
        }


def run_text(argv: list[str], timeout: float = 4) -> str | None:
    try:
        completed = subprocess.run(
            argv,
            capture_output=True,
            timeout=timeout,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return None
    return (completed.stdout + b"\n" + completed.stderr).decode("utf-8", "replace")


def is_isp_node(name: str) -> bool:
    lowered = name.lower()
    return any(marker in lowered for marker in _ISP_MARKERS)


def parse_v4l2_devices(text: str) -> list[tuple[str, list[str]]]:
    groups: list[tuple[str, list[str]]] = []
    name: str | None = None
    nodes: list[str] = []
    for raw in text.splitlines():
        if not raw.strip():
            continue
        if raw[:1].isspace():
            node = raw.strip()
            if name is not None and node.startswith("/dev/"):
                nodes.append(node)
            continue
        header = _DEVICE_HEADER.match(raw.strip())
        if header:
            if name is not None:
                groups.append((name, nodes))
            name = header.group(1)
            nodes = []
    if name is not None:
        groups.append((name, nodes))
    return groups


def parse_v4l2_formats(text: str) -> list[Mode]:
    modes: list[Mode] = []
    fmt: str | None = None
    size: tuple[int, int] | None = None
    fps: list[int] = []

    def finish() -> None:
        nonlocal size, fps
        if fmt == "MJPG" and size and fps:
            modes.append(Mode(size[0], size[1], tuple(fps), "MJPG"))
        size = None
        fps = []

    for raw in text.splitlines():
        line = raw.strip()
        fmt_match = _FORMAT_HEADER.search(line)
        if fmt_match:
            finish()
            fmt = fmt_match.group(2)
            continue
        size_match = _SIZE.search(line)
        if size_match:
            finish()
            size = (int(size_match.group(1)), int(size_match.group(2)))
            continue
        fps_match = _FPS.search(line)
        if fps_match and size is not None:
            fps.append(int(round(float(fps_match.group(1)))))
    finish()
    return merge_modes(modes)


def parse_rpicam_cameras(text: str) -> list[dict[str, object]]:
    found = []
    for line in text.splitlines():
        match = _RPICAM.match(line)
        if not match:
            continue
        found.append(
            {
                "index": int(match.group(1)),
                "sensor": match.group(2),
                "width": int(match.group(3)),
                "height": int(match.group(4)),
                "line": line.strip(),
            }
        )
    return found


def discover_cameras(settings: Settings, run=run_text) -> Discovery:
    discovery = Discovery()
    if shutil_which(settings.ffmpeg) is None and settings.ffmpeg == "ffmpeg":
        discovery.notes.append("ffmpeg is not on PATH. Install it with: sudo apt install -y ffmpeg")
    if shutil_which(settings.v4l2_ctl) is None and settings.v4l2_ctl == "v4l2-ctl":
        discovery.notes.append("v4l2-ctl is not on PATH. Install it with: sudo apt install -y v4l-utils")

    devices_text = run([settings.v4l2_ctl, "--list-devices"])
    groups = parse_v4l2_devices(devices_text) if devices_text else []
    usb_index = 0
    for name, nodes in groups:
        if is_isp_node(name):
            continue
        video_nodes = [node for node in nodes if node.startswith("/dev/video")]
        best_node: str | None = None
        best_modes: list[Mode] = []
        for node in video_nodes:
            formats = run([settings.v4l2_ctl, "-d", node, "--list-formats-ext"])
            modes = parse_v4l2_formats(formats or "")
            if len(modes) > len(best_modes):
                best_node = node
                best_modes = modes
        if best_node is None:
            continue
        kind = "c920" if ("c920" in name.lower() or "046d:082d" in name.lower()) else "usb"
        if kind == "c920":
            camera_id = "c920"
            label = "Logitech C920"
        else:
            camera_id = "usb" if usb_index == 0 else f"usb{usb_index}"
            usb_index += 1
            label = name.split(" (")[0]
        device = settings.c920_device if kind == "c920" and settings.c920_device else best_node
        modes = tuple(best_modes) if best_modes else (fallback_c920_modes() if kind == "c920" else ())
        discovery.cameras.append(
            CameraInfo(
                id=camera_id,
                label=label,
                kind=kind,
                device=device,
                modes=modes,
                max_still=_largest(modes),
                detail=name,
            )
        )

    if settings.c920_device and not any(camera.kind == "c920" for camera in discovery.cameras):
        discovery.cameras.insert(
            0,
            CameraInfo(
                id="c920",
                label="Logitech C920",
                kind="c920",
                device=settings.c920_device,
                modes=fallback_c920_modes(),
                max_still=(1920, 1080),
                detail="device override",
            ),
        )
        discovery.notes.append(
            f"C920 was not in v4l2 device list; using PI_CAMERAS_C920_DEVICE={settings.c920_device}"
        )

    hello = settings.rpicam_hello
    if hello:
        listed = run([hello, "--list-cameras"])
        parsed = parse_rpicam_cameras(listed or "")
        if listed is None:
            discovery.notes.append(f"{hello} did not run")
        elif not parsed:
            discovery.notes.append("rpicam did not report a CSI camera")
        for item in parsed:
            index = int(item["index"])
            discovery.cameras.append(
                CameraInfo(
                    id="pi" if index == 0 else f"pi{index}",
                    label=f"Pi camera ({item['sensor']})",
                    kind="pi",
                    device=None,
                    sensor=str(item["sensor"]),
                    max_still=(int(item["width"]), int(item["height"])),
                    detail=str(item["line"]),
                )
            )
    else:
        discovery.notes.append(
            "rpicam-hello / libcamera-hello not found. The CSI camera will be hidden until "
            "rpicam-apps (Bookworm) or libcamera-apps is installed."
        )
    return discovery


def _largest(modes: tuple[Mode, ...] | list[Mode]) -> tuple[int, int] | None:
    if not modes:
        return None
    mode = max(modes, key=lambda item: item.width * item.height)
    return mode.width, mode.height


def shutil_which(name: str) -> str | None:
    import shutil

    return shutil.which(name)
