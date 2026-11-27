from __future__ import annotations

import os
import shutil
from dataclasses import dataclass, field


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    return int(raw)


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    return float(raw)


@dataclass
class Settings:
    host: str = "100.109.182.59"
    port: int = 8765
    token: str | None = None
    public_base_url: str | None = None
    idle_seconds: float = 120
    ffmpeg: str = "ffmpeg"
    v4l2_ctl: str = "v4l2-ctl"
    rpicam_vid: str | None = None
    rpicam_still: str | None = None
    rpicam_hello: str | None = None
    c920_device: str | None = None
    allowed_hosts: list[str] = field(default_factory=list)

    @classmethod
    def from_env(cls) -> Settings:
        token = os.environ.get("PI_CAMERAS_TOKEN", "").strip() or None
        extra = os.environ.get("PI_CAMERAS_ALLOWED_HOSTS", "")
        allowed = [item.strip() for item in extra.split(",") if item.strip()]
        return cls(
            host=os.environ.get("PI_CAMERAS_HOST", "100.109.182.59").strip() or "100.109.182.59",
            port=_env_int("PI_CAMERAS_PORT", 8765),
            token=token,
            public_base_url=os.environ.get("PI_CAMERAS_PUBLIC_URL", "").strip() or None,
            idle_seconds=_env_float("PI_CAMERAS_IDLE_SECONDS", 120),
            ffmpeg=os.environ.get("PI_CAMERAS_FFMPEG", "ffmpeg").strip() or "ffmpeg",
            v4l2_ctl=os.environ.get("PI_CAMERAS_V4L2_CTL", "v4l2-ctl").strip() or "v4l2-ctl",
            rpicam_vid=os.environ.get("PI_CAMERAS_RPICAM_VID", "").strip() or None,
            rpicam_still=os.environ.get("PI_CAMERAS_RPICAM_STILL", "").strip() or None,
            rpicam_hello=os.environ.get("PI_CAMERAS_RPICAM_HELLO", "").strip() or None,
            c920_device=os.environ.get("PI_CAMERAS_C920_DEVICE", "").strip() or None,
            allowed_hosts=allowed,
        )

    def autodetect_binaries(self) -> None:
        self.rpicam_vid = self.rpicam_vid or _first_which("rpicam-vid", "libcamera-vid")
        self.rpicam_still = self.rpicam_still or _first_which("rpicam-still", "libcamera-still")
        self.rpicam_hello = self.rpicam_hello or _first_which("rpicam-hello", "libcamera-hello")

    @property
    def base_url(self) -> str:
        if self.public_base_url:
            return self.public_base_url.rstrip("/")
        host = "127.0.0.1" if self.host in ("0.0.0.0", "::") else self.host
        return f"http://{host}:{self.port}"

    def stream_urls(self, camera_id: str) -> dict[str, str]:
        base = self.base_url
        return {
            "watch": f"{base}/",
            "mjpeg": f"{base}/stream/{camera_id}/live.mjpg",
            "frame": f"{base}/stream/{camera_id}/frame.jpg",
            "snapshot": f"{base}/snapshot/{camera_id}",
        }


def _first_which(*names: str) -> str | None:
    for name in names:
        found = shutil.which(name)
        if found:
            return found
    return None
