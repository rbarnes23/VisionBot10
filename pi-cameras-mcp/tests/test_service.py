import threading
import time

from pi_cameras_mcp.errors import CameraError

from tests.support import make_jpeg, service


def test_live_snapshot_does_not_reopen_the_camera():
    svc, runner = service()
    started = svc.start_stream("c920", "balanced")
    assert started["width"] == 1280
    assert started["height"] == 720
    assert started["fps"] == 15
    assert started["pipeline"] == "mjpeg-copy"
    assert started["sweet_spot"] is True
    assert started["urls"]["mjpeg"].endswith("/stream/c920/live.mjpg")
    shot = svc.snapshot("c920", "balanced")
    assert shot.source == "live"
    assert runner.once_calls == []
    assert len(runner.started) == 1


def test_different_snapshot_pauses_and_resumes():
    svc, runner = service()
    svc.start_stream("c920", "balanced")
    shot = svc.snapshot("c920", "detail")
    assert shot.source == "one-shot"
    assert shot.width == 320 or shot.jpeg.startswith(b"\xff\xd8")
    assert svc.status()["streams"][0]["preset"] == "balanced"
    assert len(runner.started) == 2
    assert len(runner.once_calls) == 1


def test_grab_opens_once_then_reuses():
    svc, runner = service()
    first = svc.grab_frame("pi")
    second = svc.grab_frame("pi")
    assert first.source == "live"
    assert second.source == "live"
    assert first.pipeline == "rpicam-mjpeg"
    assert (first.width, first.height) == (320, 240)
    assert len(runner.started) == 1
    assert runner.once_calls == []


def test_fresh_frame_waits_for_the_next_one():
    svc, runner = service(make_jpeg(40, 30, b"\x44"))
    svc.grab_frame("c920", preset="balanced")
    newer = make_jpeg(40, 30, b"\x55")

    def publish():
        time.sleep(0.05)
        session = svc._streams["c920"].session
        session.frames.publish(newer)

    threading.Thread(target=publish, daemon=True).start()
    got = svc.grab_frame("c920", preset="balanced", fresh=True)
    assert got.jpeg == newer
    assert runner.once_calls == []


def test_no_frame_reports_stderr():
    from pi_cameras_mcp.capture import StreamSession

    class Silent:
        def start(self, spec):
            session = StreamSession.manual(None)
            session.stderr_text = "Device or resource busy"
            return session

        def once(self, spec):
            raise AssertionError("should not still-capture")

    svc, _runner = service()
    svc.runner = Silent()
    svc.open_timeout["c920"] = 0.2
    try:
        svc.start_stream("c920", "tiny")
    except CameraError as exc:
        assert "no frame" in str(exc)
        assert "busy" in str(exc)
    else:
        raise AssertionError("expected no frame")
    assert svc.status()["streams"] == []


def test_unknown_camera_and_idle_stop():
    svc, _runner = service()
    try:
        svc.start_stream("missing")
    except CameraError as exc:
        assert exc.status == 404
        assert "c920" in str(exc)
    else:
        raise AssertionError("expected unknown camera")

    clock = {"now": 0.0}
    svc._now = lambda: clock["now"]
    svc.settings.idle_seconds = 10
    svc.start_stream("c920", "low")
    clock["now"] = 30
    svc.reap_idle()
    assert svc.status()["streams"] == []


def test_viewer_holds_the_stream_open():
    svc, _runner = service()
    clock = {"now": 0.0}
    svc._now = lambda: clock["now"]
    svc.settings.idle_seconds = 5
    svc.start_stream("pi", "balanced")
    svc.attach_viewer("pi")
    clock["now"] = 100
    svc.reap_idle()
    assert len(svc.status()["streams"]) == 1
    svc.detach_viewer("pi")
    clock["now"] = 200
    svc.reap_idle()
    assert svc.status()["streams"] == []
