from pi_cameras_mcp.capture import StreamSession
from pi_cameras_mcp.discover import CameraInfo, Discovery
from pi_cameras_mcp.errors import CameraError
from pi_cameras_mcp.presets import fallback_c920_modes
from pi_cameras_mcp.service import CameraService
from pi_cameras_mcp.settings import Settings


def make_jpeg(width: int, height: int, marker: bytes = b"\x10", pad: int = 80) -> bytes:
    sof = bytes(
        [
            0xFF,
            0xC0,
            0x00,
            0x0B,
            0x08,
            (height >> 8) & 0xFF,
            height & 0xFF,
            (width >> 8) & 0xFF,
            width & 0xFF,
            0x01,
            0x01,
            0x11,
            0x00,
        ]
    )
    return b"\xff\xd8" + sof + marker * pad + b"\xff\xd9"


def c920() -> CameraInfo:
    return CameraInfo(
        id="c920",
        label="Logitech C920",
        kind="c920",
        device="/dev/video0",
        modes=fallback_c920_modes(),
        max_still=(1920, 1080),
    )


def pi_cam() -> CameraInfo:
    return CameraInfo(
        id="pi",
        label="Pi camera (imx708)",
        kind="pi",
        device=None,
        sensor="imx708",
        max_still=(4608, 2592),
        detail="imx708",
    )


class FakeRunner:
    def __init__(self, jpeg: bytes) -> None:
        self.jpeg = jpeg
        self.started = []
        self.once_calls = []

    def start(self, spec):
        self.started.append(spec)
        return StreamSession.manual(self.jpeg)

    def once(self, spec):
        self.once_calls.append(spec)
        return self.jpeg


def service(jpeg: bytes | None = None, token: str | None = None) -> tuple[CameraService, FakeRunner]:
    frame = jpeg or make_jpeg(320, 240)
    runner = FakeRunner(frame)
    settings = Settings(host="127.0.0.1", port=8765, token=token)
    cameras = Discovery(cameras=[c920(), pi_cam()])

    def discover():
        return cameras

    return CameraService(settings, runner=runner, discover=discover), runner
