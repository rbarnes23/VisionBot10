"""Start and stop camera processes.

The C920 already compresses MJPEG in the webcam. ffmpeg is told to copy those
frames. The Pi camera uses rpicam's hardware JPEG encoder. Either way this
process only keeps the newest frame.
"""

from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass

from pi_cameras_mcp.errors import CameraError
from pi_cameras_mcp.jpegutil import JpegSplitter, jpeg_size
from pi_cameras_mcp.presets import ffmpeg_qscale
from pi_cameras_mcp.settings import Settings

log = logging.getLogger("pi_cameras_mcp.capture")


@dataclass(frozen=True)
class CaptureSpec:
    camera_id: str
    kind: str
    device: str | None
    width: int
    height: int
    fps: int
    quality: int
    preset: str
    pipeline: str
    capture_width: int
    capture_height: int
    note: str

    def same(self, other: CaptureSpec) -> bool:
        return (
            self.camera_id == other.camera_id
            and self.width == other.width
            and self.height == other.height
            and self.fps == other.fps
            and self.quality == other.quality
            and self.pipeline == other.pipeline
            and self.capture_width == other.capture_width
            and self.capture_height == other.capture_height
            and self.device == other.device
        )


class LatestFrame:
    def __init__(self) -> None:
        self._cond = threading.Condition()
        self.frame: bytes | None = None
        self.seq = 0
        self.monotonic = 0.0
        self.count = 0
        self.fps = 0.0
        self._window_start = time.monotonic()
        self._window_count = 0
        self._closed = False

    def publish(self, frame: bytes) -> None:
        now = time.monotonic()
        with self._cond:
            self.frame = frame
            self.seq += 1
            self.monotonic = now
            self.count += 1
            self._window_count += 1
            elapsed = now - self._window_start
            if elapsed >= 1.0:
                self.fps = self._window_count / elapsed
                self._window_start = now
                self._window_count = 0
            self._cond.notify_all()

    def latest(self) -> tuple[bytes, int, float] | None:
        with self._cond:
            if self.frame is None:
                return None
            return self.frame, self.seq, self.monotonic

    def wait(self, after_seq: int, timeout: float) -> tuple[bytes, int] | None:
        deadline = time.monotonic() + timeout
        with self._cond:
            while self.seq <= after_seq and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(timeout=remaining)
            if self.seq > after_seq and self.frame is not None:
                return self.frame, self.seq
            return None

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()


class StreamSession:
    def __init__(
        self,
        argv: list[str],
        *,
        env: dict[str, str] | None = None,
        popen=subprocess.Popen,
        manual: bool = False,
    ) -> None:
        self.argv = argv
        self.frames = LatestFrame()
        self.dead = False
        self.stderr_text = ""
        self._env = env
        self._popen = popen
        self._manual = manual
        self._proc: subprocess.Popen | None = None
        self._stop = threading.Event()
        self._stdout_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._closed = False

    @classmethod
    def manual(cls, frame: bytes | None = None) -> StreamSession:
        session = cls(["fake"], manual=True)
        if frame is not None:
            session.frames.publish(frame)
        return session

    def open(self) -> None:
        if self._manual or self._proc is not None:
            return
        env = None
        if self._env is not None:
            env = os.environ.copy()
            env.update(self._env)
        log.info("start %s", " ".join(self.argv))
        self._proc = self._popen(
            self.argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            env=env,
        )
        self._stdout_thread = threading.Thread(target=self._read_stdout, name="cam-stdout", daemon=True)
        self._stderr_thread = threading.Thread(target=self._read_stderr, name="cam-stderr", daemon=True)
        self._stdout_thread.start()
        self._stderr_thread.start()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        self.frames.close()
        proc = self._proc
        if proc is not None:
            kill_process_group(proc)
        if self._stdout_thread is not None:
            self._stdout_thread.join(timeout=2)
        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=1)
        self.dead = True

    def _read_stdout(self) -> None:
        splitter = JpegSplitter()
        proc = self._proc
        assert proc is not None and proc.stdout is not None
        try:
            while not self._stop.is_set():
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                for frame in splitter.feed(chunk):
                    if len(frame) >= 64 and frame.startswith(b"\xff\xd8"):
                        self.frames.publish(frame)
        finally:
            self.dead = True
            self.frames.close()

    def _read_stderr(self) -> None:
        proc = self._proc
        assert proc is not None and proc.stderr is not None
        chunks: deque[bytes] = deque()
        total = 0
        while True:
            data = proc.stderr.read(4096)
            if not data:
                break
            chunks.append(data)
            total += len(data)
            while total > 8000 and chunks:
                total -= len(chunks.popleft())
        self.stderr_text = b"".join(chunks).decode("utf-8", "replace")[-2000:]


def kill_process_group(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    for sig, wait_s in ((signal.SIGINT, 1.0), (signal.SIGTERM, 0.5), (signal.SIGKILL, 0.5)):
        if proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, sig)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                proc.send_signal(sig)
            except OSError:
                return
        try:
            proc.wait(timeout=wait_s)
            return
        except subprocess.TimeoutExpired:
            continue


def stream_argv(spec: CaptureSpec, settings: Settings) -> list[str]:
    if spec.kind == "pi":
        binary = settings.rpicam_vid or "rpicam-vid"
        return [
            binary,
            "-t",
            "0",
            "-n",
            "--width",
            str(spec.width),
            "--height",
            str(spec.height),
            "--framerate",
            str(spec.fps),
            "--codec",
            "mjpeg",
            "--quality",
            str(spec.quality),
            "--denoise",
            "off",
            "--flush",
            "-o",
            "-",
        ]
    if not spec.device:
        raise CameraError(f"{spec.camera_id} has no video device")
    command = [
        settings.ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-nostdin",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-f",
        "v4l2",
        "-input_format",
        "mjpeg",
        "-video_size",
        f"{spec.capture_width}x{spec.capture_height}",
        "-framerate",
        str(spec.fps),
        "-i",
        spec.device,
        "-an",
    ]
    if spec.pipeline == "mjpeg-copy":
        command += ["-c:v", "copy"]
    else:
        command += [
            "-vf",
            f"scale={spec.width}:{spec.height}:flags=fast_bilinear",
            "-pix_fmt",
            "yuvj420p",
            "-c:v",
            "mjpeg",
            "-q:v",
            ffmpeg_qscale(spec.quality),
        ]
    command += ["-flush_packets", "1", "-f", "mjpeg", "pipe:1"]
    return command


def still_argv(spec: CaptureSpec, settings: Settings) -> list[str]:
    if spec.kind == "pi":
        binary = settings.rpicam_still or "rpicam-still"
        return [
            binary,
            "-n",
            "-o",
            "-",
            "--immediate",
            "--width",
            str(spec.width),
            "--height",
            str(spec.height),
            "--quality",
            str(spec.quality),
            "--denoise",
            "off",
            "-t",
            "1",
        ]
    command = stream_argv(spec, settings)
    # One extra frame: the first JPEG from some UVC cameras is truncated.
    insert_at = command.index("-an") + 1
    return command[:insert_at] + ["-frames:v", "2"] + command[insert_at:]


def capture_once(argv: list[str], timeout: float, *, env: dict[str, str] | None = None) -> bytes:
    merged = None
    if env is not None:
        merged = os.environ.copy()
        merged.update(env)
    proc = subprocess.Popen(
        argv,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
        env=merged,
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        kill_process_group(proc)
        raise CameraError(f"capture timed out after {timeout:.0f}s: {' '.join(argv)}", status=504) from None
    frames = [frame for frame in JpegSplitter().feed(stdout) if len(frame) >= 64]
    if frames:
        return frames[-1]
    message = stderr.decode("utf-8", "replace").strip()[-800:]
    raise CameraError(
        "camera returned no JPEG. " + (message or f"exit {proc.returncode}: {' '.join(argv)}"),
        status=502,
    )


def frame_size(frame: bytes, fallback: tuple[int, int]) -> tuple[int, int]:
    return jpeg_size(frame) or fallback


class DirectRunner:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings

    def start(self, spec: CaptureSpec) -> StreamSession:
        if spec.kind == "pi":
            _require_binary(self.settings.rpicam_vid, "rpicam-vid", "rpicam-apps")
        else:
            _require_ffmpeg(self.settings)
            _set_compression_quality(spec, self.settings)
        session = StreamSession(stream_argv(spec, self.settings))
        session.open()
        return session

    def once(self, spec: CaptureSpec) -> bytes:
        if spec.kind == "pi":
            _require_binary(self.settings.rpicam_still, "rpicam-still", "rpicam-apps")
        else:
            _require_ffmpeg(self.settings)
            _set_compression_quality(spec, self.settings)
        timeout = 12 if spec.kind == "pi" else 8
        return capture_once(still_argv(spec, self.settings), timeout)


def _require_binary(path: str | None, label: str, package: str) -> None:
    if path:
        return
    raise CameraError(
        f"{label} is not installed. On Raspberry Pi OS Bookworm: sudo apt install -y {package}",
        status=500,
    )


def _require_ffmpeg(settings: Settings) -> None:
    import shutil

    if shutil.which(settings.ffmpeg) is None and settings.ffmpeg == "ffmpeg":
        raise CameraError(
            "ffmpeg is not installed. On the Pi: sudo apt install -y ffmpeg v4l-utils",
            status=500,
        )


def _set_compression_quality(spec: CaptureSpec, settings: Settings) -> None:
    if not spec.device:
        return
    import shutil

    if shutil.which(settings.v4l2_ctl) is None and settings.v4l2_ctl == "v4l2-ctl":
        return
    try:
        subprocess.run(
            [settings.v4l2_ctl, "-d", spec.device, "-c", f"compression_quality={spec.quality}"],
            capture_output=True,
            timeout=2,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return
