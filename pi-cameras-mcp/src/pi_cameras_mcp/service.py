"""One capture process per camera, shared by every viewer and tool call."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

from pi_cameras_mcp.capture import CaptureSpec, DirectRunner, StreamSession, frame_size
from pi_cameras_mcp.discover import CameraInfo, Discovery, discover_cameras
from pi_cameras_mcp.errors import CameraError
from pi_cameras_mcp.presets import (
    PRESETS,
    SNAPSHOT_DEFAULT,
    SWEET_SPOT,
    Mode,
    choose_mode,
    clamp_dim,
    estimate_mbps,
    fallback_c920_modes,
    preset_catalog,
)
from pi_cameras_mcp.settings import Settings

log = logging.getLogger("pi_cameras_mcp.service")

_ALIASES = {
    "c920": "c920",
    "logitech": "c920",
    "logitech-c920": "c920",
    "webcam": "c920",
    "usb": "usb",
    "pi": "pi",
    "pi-camera": "pi",
    "picamera": "pi",
    "picam": "pi",
    "csi": "pi",
    "camera-module": "pi",
    "auto": "auto",
    "default": "auto",
    "camera": "auto",
}


def fallback_modes_for(kind: str) -> tuple[Mode, ...]:
    if kind == "c920":
        return fallback_c920_modes()
    return ()


@dataclass
class CaptureResult:
    jpeg: bytes
    width: int
    height: int
    camera: str
    preset: str
    elapsed_ms: int
    source: str
    pipeline: str
    note: str
    fps: int

    def summary(self) -> str:
        return (
            f"{self.camera} {self.width}x{self.height} preset={self.preset} "
            f"fps={self.fps} pipeline={self.pipeline} source={self.source} "
            f"in {self.elapsed_ms}ms. {self.note}"
        ).strip()


@dataclass
class RunningStream:
    spec: CaptureSpec
    session: StreamSession
    viewers: int = 0
    last_touch: float | None = None

    def __post_init__(self) -> None:
        if self.last_touch is None:
            self.last_touch = time.monotonic()

    @property
    def alive(self) -> bool:
        return not self.session.dead

    def touch(self, now: float | None = None) -> None:
        self.last_touch = time.monotonic() if now is None else now


class CameraService:
    def __init__(self, settings: Settings, runner=None, discover=None, clock=None) -> None:
        self.settings = settings
        self.runner = runner or DirectRunner(settings)
        self._discover = discover or (lambda: discover_cameras(settings))
        self._now = clock or time.monotonic
        self._discovery: Discovery | None = None
        self._streams: dict[str, RunningStream] = {}
        self._mu = threading.Lock()
        self._camera_locks: dict[str, threading.Lock] = {}
        self._reaper_stop = threading.Event()
        self._reaper: threading.Thread | None = None
        self.open_timeout = {"pi": 8.0, "c920": 5.0, "usb": 5.0}

    def start_reaper(self) -> None:
        if self.settings.idle_seconds <= 0 or self._reaper is not None:
            return
        self._reaper = threading.Thread(target=self._reap_loop, name="cam-idle", daemon=True)
        self._reaper.start()

    def close(self) -> None:
        self._reaper_stop.set()
        with self._mu:
            streams = list(self._streams.values())
            self._streams.clear()
        for stream in streams:
            stream.session.close()

    def _reap_loop(self) -> None:
        while not self._reaper_stop.wait(2.0):
            try:
                self.reap_idle()
            except Exception:
                log.exception("idle reaper failed")

    def reap_idle(self) -> None:
        if self.settings.idle_seconds <= 0:
            return
        now = self._now()
        stale: list[str] = []
        with self._mu:
            for camera_id, stream in self._streams.items():
                if stream.viewers > 0:
                    continue
                touched = 0.0 if stream.last_touch is None else stream.last_touch
                if now - touched >= self.settings.idle_seconds:
                    stale.append(camera_id)
        for camera_id in stale:
            log.info("stopping idle stream %s", camera_id)
            self.stop_stream(camera_id)

    def _lock(self, camera_id: str) -> threading.Lock:
        with self._mu:
            lock = self._camera_locks.get(camera_id)
            if lock is None:
                lock = threading.Lock()
                self._camera_locks[camera_id] = lock
            return lock

    def discovery(self, rescan: bool = False) -> Discovery:
        if rescan or self._discovery is None:
            self._discovery = self._discover()
        return self._discovery

    def list_cameras(self, rescan: bool = False) -> dict[str, object]:
        found = self.discovery(rescan)
        return {
            "cameras": [self._camera_row(camera) for camera in found.cameras],
            "notes": found.notes,
            "sweet_spot": SWEET_SPOT,
            "streams": self.status()["streams"],
        }

    def list_resolutions(self) -> dict[str, object]:
        return {
            "sweet_spot": SWEET_SPOT,
            "snapshot_default": SNAPSHOT_DEFAULT,
            "why": (
                "balanced is 1280x720 at 15 fps. On the C920 that size is a native MJPEG "
                "mode, so the Pi copies frames and does not encode. It is about 4 Mbps, "
                "which is the comfortable point over Tailscale. Use smooth on a LAN, "
                "detail for a 1080p still, and max only on a fast LAN."
            ),
            "presets": preset_catalog(),
        }

    def resolve(self, name: str) -> CameraInfo:
        cameras = self.discovery().cameras
        key = (name or "auto").strip().lower().replace("_", "-")
        for camera in cameras:
            if camera.id == key or (camera.device and camera.device == name):
                return camera
        target = _ALIASES.get(key, key)
        if target == "auto":
            if len(cameras) == 1:
                return cameras[0]
            for camera in cameras:
                if camera.kind == "c920":
                    return camera
        if target == "c920":
            for camera in cameras:
                if camera.kind == "c920":
                    return camera
            usbs = [camera for camera in cameras if camera.kind == "usb"]
            if len(usbs) == 1:
                return usbs[0]
        if target == "usb":
            usbs = [camera for camera in cameras if camera.kind in ("c920", "usb")]
            if len(usbs) == 1:
                return usbs[0]
        if target == "pi":
            pis = [camera for camera in cameras if camera.kind == "pi"]
            if len(pis) == 1:
                return pis[0]
            if pis:
                return pis[0]
        available = ", ".join(camera.id for camera in cameras) or "none discovered"
        raise CameraError(
            f"unknown camera {name!r}. Available: {available}. "
            "Use list_cameras, or set PI_CAMERAS_C920_DEVICE if the webcam name does not contain C920.",
            status=404,
        )

    def start_stream(
        self,
        camera: str,
        preset: str = SWEET_SPOT,
        width: int | None = None,
        height: int | None = None,
        fps: int | None = None,
        quality: int | None = None,
        exact: bool = False,
    ) -> dict[str, object]:
        cam = self.resolve(camera)
        spec = self.build_spec(cam, preset, width, height, fps, quality, exact)
        return self.start_spec(spec)

    def start_spec(self, spec: CaptureSpec) -> dict[str, object]:
        lock = self._lock(spec.camera_id)
        with lock:
            with self._mu:
                running = self._streams.get(spec.camera_id)
                if running and running.alive and running.spec.same(spec):
                    running.touch(self._now())
                    return self._describe(running)
                old = self._streams.pop(spec.camera_id, None)
        if old:
            old.session.close()
        session = self.runner.start(spec)
        timeout = self.open_timeout.get(spec.kind, 5.0)
        item = session.frames.wait(0, timeout)
        if item is None:
            err = session.stderr_text.strip()
            session.close()
            detail = f" {err}" if err else ""
            raise CameraError(
                f"no frame from {spec.camera_id} ({spec.width}x{spec.height}@{spec.fps} {spec.pipeline}).{detail}",
                status=502,
            )
        running = RunningStream(spec=spec, session=session, last_touch=self._now())
        with lock:
            with self._mu:
                displaced = self._streams.get(spec.camera_id)
                self._streams[spec.camera_id] = running
        if displaced:
            displaced.session.close()
        return self._describe(running)

    def stop_stream(self, camera: str) -> dict[str, object]:
        cam = self.resolve(camera)
        lock = self._lock(cam.id)
        with lock:
            with self._mu:
                running = self._streams.pop(cam.id, None)
        if running:
            running.session.close()
            return {"camera": cam.id, "stopped": True}
        return {"camera": cam.id, "stopped": False, "note": "it was not streaming"}

    def stop_all(self) -> dict[str, object]:
        with self._mu:
            items = list(self._streams.items())
            self._streams.clear()
        for _, stream in items:
            stream.session.close()
        return {"stopped": [camera_id for camera_id, _ in items]}

    def snapshot(
        self,
        camera: str,
        preset: str = SNAPSHOT_DEFAULT,
        width: int | None = None,
        height: int | None = None,
        fps: int | None = None,
        quality: int | None = None,
        exact: bool = False,
        fresh: bool = False,
    ) -> CaptureResult:
        cam = self.resolve(camera)
        spec = self.build_spec(cam, preset, width, height, fps, quality, exact)
        return self._capture(spec, fresh=fresh, allow_oneshot=True)

    def grab_frame(
        self,
        camera: str,
        preset: str | None = None,
        width: int | None = None,
        height: int | None = None,
        fps: int | None = None,
        quality: int | None = None,
        exact: bool = False,
        fresh: bool = False,
        keep_open: bool = True,
    ) -> CaptureResult:
        cam = self.resolve(camera)
        running = self._running(cam.id)
        if preset is None and width is None and height is None and running and running.alive:
            return self._capture(running.spec, fresh=fresh, allow_oneshot=False)
        spec = self.build_spec(cam, preset or SWEET_SPOT, width, height, fps, quality, exact)
        if keep_open:
            self.start_spec(spec)
            return self._capture(spec, fresh=fresh, allow_oneshot=False)
        return self._capture(spec, fresh=True, allow_oneshot=True)

    def sample_frames(
        self,
        camera: str,
        count: int = 3,
        interval_ms: int = 400,
        preset: str | None = None,
        width: int | None = None,
        height: int | None = None,
        fps: int | None = None,
        quality: int | None = None,
        exact: bool = False,
    ) -> list[CaptureResult]:
        count = max(1, min(6, int(count)))
        interval_ms = max(0, min(2000, int(interval_ms)))
        frames = [
            self.grab_frame(
                camera,
                preset=preset,
                width=width,
                height=height,
                fps=fps,
                quality=quality,
                exact=exact,
                fresh=False,
                keep_open=True,
            )
        ]
        for _ in range(count - 1):
            if interval_ms:
                time.sleep(interval_ms / 1000)
            frames.append(
                self.grab_frame(
                    camera,
                    preset=preset,
                    width=width,
                    height=height,
                    fps=fps,
                    quality=quality,
                    exact=exact,
                    fresh=True,
                    keep_open=True,
                )
            )
        return frames

    def attach_viewer(self, camera: str) -> RunningStream:
        cam = self.resolve(camera)
        lock = self._lock(cam.id)
        with lock:
            with self._mu:
                running = self._streams.get(cam.id)
            if running is None or not running.alive:
                raise CameraError(
                    f"{cam.id} is not streaming. Start it first.",
                    status=409,
                )
            running.viewers += 1
            running.touch(self._now())
            return running

    def detach_viewer(self, camera_id: str) -> None:
        lock = self._lock(camera_id)
        with lock:
            with self._mu:
                running = self._streams.get(camera_id)
            if running and running.viewers > 0:
                running.viewers -= 1
                running.touch(self._now())

    def latest_jpeg(self, camera: str, *, fresh: bool = False, wait_s: float = 2.0) -> bytes:
        cam = self.resolve(camera)
        running = self.attach_viewer(cam.id)
        try:
            frame = self._read_live(running, fresh=fresh, wait_s=wait_s)
        finally:
            self.detach_viewer(cam.id)
        if frame is None:
            raise CameraError(f"no frame available from {cam.id}", status=504)
        return frame

    def status(self) -> dict[str, object]:
        with self._mu:
            streams = list(self._streams.values())
        rows = []
        for stream in streams:
            rows.append(self._describe(stream))
        return {"streams": rows, "idle_seconds": self.settings.idle_seconds}

    def build_spec(
        self,
        camera: CameraInfo,
        preset_id: str,
        width: int | None,
        height: int | None,
        fps: int | None,
        quality: int | None,
        exact: bool,
    ) -> CaptureSpec:
        if (width is None) ^ (height is None):
            raise CameraError("pass both width and height, or neither")
        try:
            preset = PRESETS[preset_id]
        except KeyError:
            known = ", ".join(PRESETS)
            raise CameraError(f"unknown preset {preset_id!r}. Choose one of: {known}") from None
        if preset.id == "full":
            if camera.max_still:
                req_w, req_h = camera.max_still
            elif camera.modes:
                biggest = max(camera.modes, key=lambda mode: mode.width * mode.height)
                req_w, req_h = biggest.width, biggest.height
            else:
                req_w, req_h = (1920, 1080)
            req_fps = preset.fps
        else:
            req_w = width if width is not None else preset.width
            req_h = height if height is not None else preset.height
            req_fps = fps if fps is not None else preset.fps
        req_w = clamp_dim(req_w, low=160, high=4672)
        req_h = clamp_dim(req_h, low=120, high=3500)
        req_fps = max(1, min(60, int(req_fps if fps is None else fps)))
        if preset.id == "full" and fps is not None:
            req_fps = max(1, min(60, int(fps)))
        jpeg_q = preset.quality if quality is None else max(1, min(100, int(quality)))
        note = preset.summary
        pipeline = "rpicam-mjpeg"
        cap_w, cap_h = req_w, req_h
        out_w, out_h = req_w, req_h
        out_fps = req_fps
        if camera.kind == "pi":
            if out_w * out_h > 1920 * 1080 and out_fps > 10:
                out_fps = 10
                note = f"{note} Frame rate capped at 10 above 1080p."
            pipeline = "rpicam-mjpeg"
        else:
            modes = list(camera.modes) or list(fallback_modes_for(camera.kind))
            if modes:
                mode, chosen_fps, mode_note = choose_mode(modes, req_w, req_h, req_fps)
                note = f"{note} {mode_note}".strip()
                if exact and (mode.width != req_w or mode.height != req_h):
                    cap_w, cap_h = mode.width, mode.height
                    out_w, out_h = req_w, req_h
                    out_fps = chosen_fps
                    pipeline = "mjpeg-scale"
                    note = (
                        f"{note} Scaling on the Pi from {cap_w}x{cap_h} to {out_w}x{out_h}. "
                        "Leave exact off to stay on the native mode."
                    )
                else:
                    cap_w = out_w = mode.width
                    cap_h = out_h = mode.height
                    out_fps = chosen_fps
                    pipeline = "mjpeg-copy"
                    if pipeline == "mjpeg-copy":
                        note = f"{note} Native MJPEG copy: the Pi does not encode.".strip()
            else:
                pipeline = "mjpeg-copy"
                note = f"{note} No probed modes; asking the camera for this size directly."
        if out_w * out_h > 1920 * 1080 and out_fps > 10 and camera.kind != "pi":
            out_fps = min(out_fps, 10)
        return CaptureSpec(
            camera_id=camera.id,
            kind=camera.kind,
            device=camera.device,
            width=out_w,
            height=out_h,
            fps=out_fps,
            quality=jpeg_q,
            preset=preset.id,
            pipeline=pipeline,
            capture_width=cap_w,
            capture_height=cap_h,
            note=note,
        )

    def _capture(self, spec: CaptureSpec, *, fresh: bool, allow_oneshot: bool) -> CaptureResult:
        started = time.perf_counter()
        lock = self._lock(spec.camera_id)
        with lock:
            with self._mu:
                running = self._streams.get(spec.camera_id)
            if running and running.alive and running.spec.same(spec):
                frame = self._read_live(running, fresh=fresh, wait_s=2.5)
                if frame is None:
                    raise CameraError(
                        f"{spec.camera_id} is streaming but no frame arrived. Check camera_status.",
                        status=504,
                    )
                running.touch(self._now())
                return self._result(spec, frame, "live", started)
            paused = running.spec if running and running.alive else None
            if running:
                with self._mu:
                    self._streams.pop(spec.camera_id, None)
        if running:
            running.session.close()
        if not allow_oneshot:
            if paused is not None:
                self.start_spec(paused)
            raise CameraError(
                f"{spec.camera_id} is not streaming at {spec.width}x{spec.height}. "
                "Call grab_frame with keep_open or start_stream first.",
                status=409,
            )
        try:
            frame = self.runner.once(spec)
            source = "one-shot"
        finally:
            if paused is not None:
                try:
                    self.start_spec(paused)
                except CameraError:
                    log.exception("failed to resume %s", spec.camera_id)
        return self._result(spec, frame, source, started)

    def _read_live(self, running: RunningStream, *, fresh: bool, wait_s: float) -> bytes | None:
        latest = running.session.frames.latest()
        window = max(2.0, 3.0 / max(running.spec.fps, 1))
        if latest is None:
            item = running.session.frames.wait(0, wait_s)
            return item[0] if item else None
        frame, seq, ts = latest
        age = self._now() - ts
        if not fresh and age <= window:
            return frame
        item = running.session.frames.wait(seq, wait_s)
        if item:
            return item[0]
        if not fresh:
            return frame
        return None

    def _running(self, camera_id: str) -> RunningStream | None:
        with self._mu:
            # camera locks guard mutations; a snapshot of the reference is enough
            return self._streams.get(camera_id)

    def _result(self, spec: CaptureSpec, frame: bytes, source: str, started: float) -> CaptureResult:
        width, height = frame_size(frame, (spec.width, spec.height))
        return CaptureResult(
            jpeg=frame,
            width=width,
            height=height,
            camera=spec.camera_id,
            preset=spec.preset,
            elapsed_ms=int((time.perf_counter() - started) * 1000),
            source=source,
            pipeline=spec.pipeline,
            note=spec.note,
            fps=spec.fps,
        )

    def _describe(self, stream: RunningStream) -> dict[str, object]:
        spec = stream.spec
        latest = stream.session.frames.latest()
        age_ms = None
        if latest is not None:
            age_ms = int(max(0.0, self._now() - latest[2]) * 1000)
        return {
            "camera": spec.camera_id,
            "preset": spec.preset,
            "width": spec.width,
            "height": spec.height,
            "fps": spec.fps,
            "fps_actual": round(stream.session.frames.fps, 1),
            "frames": stream.session.frames.count,
            "quality": spec.quality,
            "pipeline": spec.pipeline,
            "estimated_mbps": estimate_mbps(spec.width, spec.height, spec.fps, spec.quality),
            "viewers": stream.viewers,
            "age_ms": age_ms,
            "alive": stream.alive,
            "note": spec.note,
            "urls": self.settings.stream_urls(spec.camera_id),
            "sweet_spot": spec.preset == SWEET_SPOT,
        }

    def _camera_row(self, camera: CameraInfo) -> dict[str, object]:
        row = camera.to_dict()
        running = self._running(camera.id)
        row["streaming"] = bool(running and running.alive)
        return row
