"""Talk to a camera service that is already running on this Pi."""

from __future__ import annotations

from urllib.parse import unquote

import httpx2

from pi_cameras_mcp.errors import CameraError
from pi_cameras_mcp.jpegutil import jpeg_size
from pi_cameras_mcp.service import CaptureResult
from pi_cameras_mcp.settings import Settings


class RemoteCameraService:
    """Same calls as CameraService, over HTTP, so a stdio session does not open the device twice."""

    def __init__(self, settings: Settings, client: httpx2.Client | None = None) -> None:
        self.settings = settings
        headers = {}
        if settings.token:
            headers["Authorization"] = f"Bearer {settings.token}"
        self._client = client or httpx2.Client(
            base_url=settings.base_url,
            headers=headers,
            timeout=httpx2.Timeout(20.0, connect=2.0),
        )

    def list_cameras(self, rescan: bool = False) -> dict:
        return self._json("GET", "/api/cameras", params={"rescan": "1" if rescan else "0"})

    def list_resolutions(self) -> dict:
        return self._json("GET", "/api/presets")

    def status(self) -> dict:
        return self._json("GET", "/api/status")

    def start_stream(
        self,
        camera: str,
        preset: str = "balanced",
        width: int | None = None,
        height: int | None = None,
        fps: int | None = None,
        quality: int | None = None,
        exact: bool = False,
    ) -> dict:
        return self._json(
            "POST",
            "/api/stream/start",
            json=_omit_none(
                camera=camera,
                preset=preset,
                width=width,
                height=height,
                fps=fps,
                quality=quality,
                exact=exact,
            ),
        )

    def stop_stream(self, camera: str) -> dict:
        return self._json("POST", "/api/stream/stop", json={"camera": camera})

    def stop_all(self) -> dict:
        return self.stop_stream("all")

    def snapshot(
        self,
        camera: str,
        preset: str = "detail",
        width: int | None = None,
        height: int | None = None,
        fps: int | None = None,
        quality: int | None = None,
        exact: bool = False,
        fresh: bool = False,
    ) -> CaptureResult:
        response = self._send(
            "POST",
            "/api/snapshot",
            json=_omit_none(
                camera=camera,
                preset=preset,
                width=width,
                height=height,
                fps=fps,
                quality=quality,
                exact=exact,
                fresh=fresh,
            ),
        )
        return self._capture(response)

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
        response = self._send(
            "POST",
            "/api/grab",
            json=_omit_none(
                camera=camera,
                preset=preset,
                width=width,
                height=height,
                fps=fps,
                quality=quality,
                exact=exact,
                fresh=fresh,
                keep_open=keep_open,
            ),
        )
        return self._capture(response)

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
        frames = [
            self.grab_frame(
                camera,
                preset,
                width,
                height,
                fps,
                quality,
                exact,
                fresh=False,
                keep_open=True,
            )
        ]
        for _ in range(count - 1):
            if interval_ms:
                import time

                time.sleep(interval_ms / 1000)
            frames.append(
                self.grab_frame(
                    camera,
                    preset,
                    width,
                    height,
                    fps,
                    quality,
                    exact,
                    fresh=True,
                    keep_open=True,
                )
            )
        return frames

    def _json(self, method: str, path: str, **kwargs) -> dict:
        response = self._send(method, path, **kwargs)
        data = response.json()
        if not isinstance(data, dict):
            raise CameraError("camera service returned a non-object response", status=502)
        return data

    def _send(self, method: str, path: str, **kwargs) -> httpx2.Response:
        try:
            response = self._client.request(method, path, **kwargs)
        except httpx2.ConnectError as exc:
            raise CameraError(
                f"camera service is not running at {self.settings.base_url}. "
                "On the Pi: sudo systemctl start pi-cameras-mcp",
                status=503,
            ) from exc
        except httpx2.HTTPError as exc:
            raise CameraError(f"camera service request failed: {exc}", status=502) from exc
        if response.status_code >= 400:
            message = _error_message(response)
            raise CameraError(message, status=response.status_code)
        return response

    def _capture(self, response: httpx2.Response) -> CaptureResult:
        jpeg = response.content
        if not jpeg.startswith(b"\xff\xd8"):
            raise CameraError("camera service did not return a JPEG", status=502)
        header = response.headers
        width = _header_int(header, "x-width")
        height = _header_int(header, "x-height")
        parsed = jpeg_size(jpeg)
        if parsed:
            width, height = parsed
        note = unquote(header.get("x-note", ""))
        return CaptureResult(
            jpeg=jpeg,
            width=width or 0,
            height=height or 0,
            camera=header.get("x-camera", ""),
            preset=header.get("x-preset", ""),
            elapsed_ms=_header_int(header, "x-elapsed-ms") or 0,
            source=header.get("x-source", "remote"),
            pipeline=header.get("x-pipeline", ""),
            note=note,
            fps=_header_int(header, "x-fps") or 0,
        )


def _omit_none(**kwargs):
    return {key: value for key, value in kwargs.items() if value is not None}


def probe_daemon(settings: Settings) -> bool:
    try:
        response = httpx2.get(f"{settings.base_url}/health", timeout=0.5)
    except httpx2.HTTPError:
        return False
    if response.status_code != 200:
        return False
    try:
        body = response.json()
    except ValueError:
        return False
    return body.get("service") == "pi-cameras-mcp"


def _header_int(headers: httpx2.Headers, name: str) -> int | None:
    raw = headers.get(name)
    if not raw:
        return None
    try:
        return int(raw)
    except ValueError:
        return None


def _error_message(response: httpx2.Response) -> str:
    try:
        body = response.json()
    except ValueError:
        body = None
    if isinstance(body, dict) and body.get("error"):
        return str(body["error"])
    text = response.text.strip()
    return text or f"camera service returned HTTP {response.status_code}"
