from pi_cameras_mcp.discover import (
    discover_cameras,
    parse_rpicam_cameras,
    parse_v4l2_devices,
    parse_v4l2_formats,
)
from pi_cameras_mcp.settings import Settings

DEVICES = """
HD Pro Webcam C920 (usb-0000:01:00.0-1.1):
\t/dev/video0
\t/dev/video1
\t/dev/media4

pispbe (platform:1000880000.pisp_be):
\t/dev/video20
\t/dev/video21
"""

FORMATS = """
ioctl: VIDIOC_ENUM_FMT
\tType: Video Capture

\t[0]: 'YUYV' (YUYV 4:2:2)
\t\tSize: Discrete 640x480
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t[1]: 'MJPG' (Motion-JPEG, compressed)
\t\tSize: Discrete 640x480
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t\t\tInterval: Discrete 0.067s (15.000 fps)
\t\tSize: Discrete 1280x720
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t\tSize: Discrete 1920x1080
\t\t\tInterval: Discrete 0.033s (30.000 fps)
"""

RPICAM = """
Available cameras
-----------------
0 : imx708 [4608x2592 10-bit RGGB] (/base/soc/i2c0mux/i2c@1/imx708@1a)
    Modes: 'SRGGB10_CSI2P' : 1536x864 [120.13 fps - (0, 0)/4608x2592 crop]
"""


def test_parse_devices_skips_blank_lines():
    groups = parse_v4l2_devices(DEVICES)
    assert groups[0][0].startswith("HD Pro Webcam C920")
    assert groups[0][1] == ["/dev/video0", "/dev/video1", "/dev/media4"]
    assert "pispbe" in groups[1][0]


def test_parse_formats_keeps_mjpeg_only():
    modes = {(mode.width, mode.height): mode.fps for mode in parse_v4l2_formats(FORMATS)}
    assert modes[(640, 480)] == (15, 30)
    assert modes[(1280, 720)] == (30,)
    assert (640, 480) in modes
    assert all(mode.pixel_format == "MJPG" for mode in parse_v4l2_formats(FORMATS))


def test_parse_rpicam():
    found = parse_rpicam_cameras(RPICAM)
    assert found[0]["sensor"] == "imx708"
    assert found[0]["width"] == 4608
    assert found[0]["height"] == 2592


def test_discover_picks_c920_and_pi_and_ignores_isp():
    settings = Settings(rpicam_hello="rpicam-hello", v4l2_ctl="v4l2-ctl", ffmpeg="ffmpeg")

    def run(argv, timeout=4):
        if argv[-1] == "--list-devices":
            return DEVICES
        if "--list-formats-ext" in argv and argv[argv.index("-d") + 1] == "/dev/video0":
            return FORMATS
        if "--list-formats-ext" in argv:
            return "no video"
        if argv[:1] == ["rpicam-hello"]:
            return RPICAM
        return ""

    found = discover_cameras(settings, run=run)
    ids = [camera.id for camera in found.cameras]
    assert ids == ["c920", "pi"]
    c920 = found.cameras[0]
    assert c920.device == "/dev/video0"
    assert c920.kind == "c920"
    assert any(mode.width == 1920 for mode in c920.modes)
    pi = found.cameras[1]
    assert pi.max_still == (4608, 2592)
    assert pi.sensor == "imx708"
