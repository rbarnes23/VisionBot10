import base64
import sys
import time

from pi_cameras_mcp.capture import StreamSession, capture_once, still_argv, stream_argv
from pi_cameras_mcp.errors import CameraError
from pi_cameras_mcp.settings import Settings

from tests.support import c920, make_jpeg, pi_cam, service


def test_c920_balanced_is_a_copy_pipeline():
    svc, _runner = service()
    spec = svc.build_spec(c920(), "balanced", None, None, None, None, False)
    assert (spec.width, spec.height, spec.fps, spec.pipeline) == (1280, 720, 15, "mjpeg-copy")
    argv = stream_argv(spec, Settings(ffmpeg="ffmpeg"))
    assert argv[argv.index("-c:v") + 1] == "copy"
    assert "scale=" not in " ".join(argv)
    assert "-input_format" in argv and "mjpeg" in argv
    still = still_argv(spec, Settings(ffmpeg="ffmpeg"))
    assert still[still.index("-frames:v") + 1] == "2"


def test_custom_size_snaps_unless_exact():
    svc, _runner = service()
    snapped = svc.build_spec(c920(), "balanced", 1000, 700, None, None, False)
    assert (snapped.width, snapped.height, snapped.pipeline) == (1280, 720, "mjpeg-copy")
    exact = svc.build_spec(c920(), "balanced", 1000, 700, None, None, True)
    assert (exact.width, exact.height) == (1000, 700)
    assert (exact.capture_width, exact.capture_height) == (1280, 720)
    assert exact.pipeline == "mjpeg-scale"
    argv = stream_argv(exact, Settings(ffmpeg="ffmpeg"))
    assert any(part.startswith("scale=1000:700") for part in argv)


def test_pi_full_uses_sensor_and_hardware_mjpeg():
    svc, _runner = service()
    spec = svc.build_spec(pi_cam(), "full", None, None, None, None, False)
    assert (spec.width, spec.height, spec.fps, spec.pipeline) == (4608, 2592, 5, "rpicam-mjpeg")
    settings = Settings(rpicam_vid="rpicam-vid", rpicam_still="rpicam-still")
    argv = stream_argv(spec, settings)
    assert argv[0] == "rpicam-vid"
    assert "--flush" in argv
    assert "mjpeg" in argv
    still = still_argv(spec, settings)
    assert still[0] == "rpicam-still"
    assert "--immediate" in still


def test_detail_and_smooth_presets():
    svc, _runner = service()
    detail = svc.build_spec(c920(), "detail", None, None, None, None, False)
    smooth = svc.build_spec(c920(), "smooth", None, None, None, None, False)
    assert (detail.width, detail.height, detail.fps) == (1920, 1080, 15)
    assert (smooth.width, smooth.height, smooth.fps) == (1280, 720, 30)


def test_stream_session_reassembles_chunked_jpegs():
    frame = make_jpeg(32, 16, b"\x21")
    payload = base64.b64encode(frame + frame).decode()
    code = (
        "import base64,os,sys,time\n"
        "data=base64.b64decode(os.environ['PAYLOAD'])\n"
        "for i in range(0,len(data),5):\n"
        "    sys.stdout.buffer.write(data[i:i+5]); sys.stdout.buffer.flush(); time.sleep(0.002)\n"
    )
    session = StreamSession(
        [sys.executable, "-c", code],
        env={"PAYLOAD": payload},
    )
    session.open()
    try:
        first = session.frames.wait(0, 3)
        assert first is not None and first[0] == frame
        deadline = time.monotonic() + 3
        while session.frames.count < 2 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert session.frames.count >= 2
        latest = session.frames.latest()
        assert latest is not None and latest[0] == frame
    finally:
        session.close()


def test_capture_once_returns_last_jpeg_and_times_out():
    frame = make_jpeg(16, 16, b"\x33")
    code = "import base64,os,sys\nsys.stdout.buffer.write(base64.b64decode(os.environ['PAYLOAD']))\n"
    got = capture_once(
        [sys.executable, "-c", code],
        3,
        env={"PAYLOAD": base64.b64encode(frame).decode()},
    )
    assert got == frame
    try:
        capture_once([sys.executable, "-c", "import time; time.sleep(30)"], 0.3)
    except CameraError as exc:
        assert exc.status == 504
    else:
        raise AssertionError("expected a timeout")


def test_missing_both_dimensions():
    svc, _runner = service()
    try:
        svc.build_spec(c920(), "balanced", 640, None, None, None, False)
    except CameraError as exc:
        assert "both" in str(exc)
    else:
        raise AssertionError("expected an error")
