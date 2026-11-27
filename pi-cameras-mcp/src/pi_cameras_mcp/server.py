"""MCP tools plus the HTTP routes that actually carry video."""

from __future__ import annotations

import hmac
import re
from typing import Annotated, Literal
from urllib.parse import parse_qs, quote, unquote

import anyio
from mcp.server import MCPServer
from mcp.server.mcpserver import Image
from mcp.server.transport_security import TransportSecuritySettings
from pydantic import Field
from starlette.requests import Request
from starlette.responses import HTMLResponse, JSONResponse, Response, StreamingResponse

from pi_cameras_mcp import __version__
from pi_cameras_mcp.errors import CameraError
from pi_cameras_mcp.preview import render_index
from pi_cameras_mcp.presets import SNAPSHOT_DEFAULT, SWEET_SPOT
from pi_cameras_mcp.service import CaptureResult
from pi_cameras_mcp.settings import Settings

PresetName = Literal["tiny", "low", "balanced", "smooth", "detail", "max", "full"]
_CAMERA_ID = re.compile(r"^[A-Za-z0-9_.-]{1,32}$")

_INSTRUCTIONS = (
    "Cameras on this Raspberry Pi: 'c920' is the Logitech USB webcam and 'pi' is the CSI camera. "
    "Call list_cameras first if you are not sure which ids are connected. "
    "The stream sweet spot is preset 'balanced': 1280x720 at 15 fps, about 4 Mbps. "
    "On the C920 that mode is native MJPEG, so the Pi copies frames and does not encode. "
    "Use 'smooth' (720p30) on a LAN, 'detail' (1080p) for a still or to read small text, "
    "and 'max' (1080p30) only on a fast LAN. "
    "grab_frame returns a picture you can see and leaves the camera open so the next grab is instant. "
    "take_snapshot defaults to detail and does not have to leave a stream running. "
    "start_stream returns a browser URL; it does not embed the video in chat. "
    "sample_frames returns a few pictures a fraction of a second apart when you need motion."
)


def build_mcp(service, settings: Settings, *, http: bool) -> tuple[MCPServer, object | None]:
    mcp = MCPServer(
        "pi-cameras",
        instructions=_INSTRUCTIONS,
        version=__version__,
    )
    _register_tools(mcp, service)
    app = None
    if http:
        _register_routes(mcp, service, settings)
        app = mcp.streamable_http_app(
            json_response=True,
            stateless_http=True,
            transport_security=TransportSecuritySettings(enable_dns_rebinding_protection=False),
            host=_security_host(settings),
        )
        app = TokenGate(app, settings.token)
    return mcp, app


def _security_host(settings: Settings) -> str:
    if settings.host in ("0.0.0.0", "::", ""):
        return "127.0.0.1"
    return settings.host


def _register_tools(mcp: MCPServer, service) -> None:
    @mcp.tool()
    def list_cameras(
        rescan: Annotated[bool, Field(description="Probe v4l2 and rpicam again.")] = False,
    ) -> dict:
        """List the C920 and Pi camera, their native modes, and whether a stream is running."""
        return service.list_cameras(rescan=rescan)

    @mcp.tool()
    def list_resolutions() -> dict:
        """List resolution presets. balanced (720p15) is the Tailscale sweet spot. detail is the snapshot default."""
        return service.list_resolutions()

    @mcp.tool()
    def camera_status() -> dict:
        """Show running streams, measured frame rate, and watch URLs."""
        return service.status()

    @mcp.tool()
    def start_stream(
        camera: Annotated[str, Field(description="c920, pi, or an id from list_cameras.")] = "c920",
        preset: Annotated[PresetName, Field(description="balanced is the sweet spot.")] = "balanced",
        width: Annotated[int | None, Field(description="Custom width. Pass height too.")] = None,
        height: Annotated[int | None, Field(description="Custom height. Pass width too.")] = None,
        fps: Annotated[int | None, Field(description="Override the preset frame rate, 1-60.")] = None,
        quality: Annotated[int | None, Field(description="JPEG quality 1-100. Ignored for C920 copy mode except as a camera hint.")] = None,
        exact: Annotated[bool, Field(description="Scale to a non-native size. Slower on the C920. Leave false.")] = False,
    ) -> dict:
        """Start a live MJPEG stream and return browser URLs. One stream per camera, shared by every viewer."""
        return service.start_stream(camera, preset, width, height, fps, quality, exact)

    @mcp.tool()
    def stop_stream(
        camera: Annotated[str, Field(description="c920, pi, or all.")] = "c920",
    ) -> dict:
        """Stop one camera stream, or all of them."""
        if camera.strip().lower() in ("all", "*"):
            return service.stop_all()
        return service.stop_stream(camera)

    @mcp.tool()
    def take_snapshot(
        camera: Annotated[str, Field(description="c920, pi, or an id from list_cameras.")] = "c920",
        preset: Annotated[PresetName, Field(description="detail is 1080p. balanced is faster.")] = "detail",
        width: Annotated[int | None, Field(description="Custom width. Pass height too.")] = None,
        height: Annotated[int | None, Field(description="Custom height. Pass width too.")] = None,
        fps: Annotated[int | None, Field(description="Used only if a stream has to start.")] = None,
        quality: Annotated[int | None, Field(description="JPEG quality 1-100.")] = None,
        exact: Annotated[bool, Field(description="Scale to a non-native size instead of snapping.")] = False,
        fresh: Annotated[bool, Field(description="Wait for a new frame when a matching stream is already running.")] = False,
    ) -> tuple[str, Image]:
        """Take one JPEG. If a stream is already running at this size, the frame comes from memory."""
        result = service.snapshot(camera, preset, width, height, fps, quality, exact, fresh)
        return _image_result(result)

    @mcp.tool()
    def grab_frame(
        camera: Annotated[str, Field(description="c920, pi, or an id from list_cameras.")] = "c920",
        preset: Annotated[PresetName | None, Field(description="Omit to use the stream that is already running, else balanced.")] = None,
        width: Annotated[int | None, Field(description="Custom width. Pass height too.")] = None,
        height: Annotated[int | None, Field(description="Custom height. Pass width too.")] = None,
        fps: Annotated[int | None, Field(description="Override frame rate when opening the camera.")] = None,
        quality: Annotated[int | None, Field(description="JPEG quality 1-100.")] = None,
        exact: Annotated[bool, Field(description="Scale to a non-native size instead of snapping.")] = False,
        fresh: Annotated[bool, Field(description="Wait for a frame newer than the one already buffered.")] = False,
        keep_open: Annotated[bool, Field(description="Leave the stream running so the next grab is instant.")] = True,
    ) -> tuple[str, Image]:
        """Return a JPEG the model can see. The first call opens the camera; later calls reuse it."""
        result = service.grab_frame(camera, preset, width, height, fps, quality, exact, fresh, keep_open)
        return _image_result(result)

    @mcp.tool()
    def sample_frames(
        camera: Annotated[str, Field(description="c920, pi, or an id from list_cameras.")] = "c920",
        count: Annotated[int, Field(description="How many frames, 1 to 6.", ge=1, le=6)] = 3,
        interval_ms: Annotated[int, Field(description="Gap between frames, up to 2000 ms.", ge=0, le=2000)] = 400,
        preset: Annotated[PresetName | None, Field(description="Omit to keep the current stream, else balanced.")] = None,
    ) -> list[str | Image]:
        """Return a short burst of JPEGs so motion is visible without opening a video URL."""
        frames = service.sample_frames(camera, count=count, interval_ms=interval_ms, preset=preset)
        content: list[str | Image] = []
        if frames:
            content.append(frames[0].summary() + f" series={len(frames)} interval_ms={interval_ms}")
        for result in frames:
            content.append(Image(data=result.jpeg, format="jpeg"))
        return content


def _image_result(result: CaptureResult) -> tuple[str, Image]:
    return result.summary(), Image(data=result.jpeg, format="jpeg")


def _register_routes(mcp: MCPServer, service, settings: Settings) -> None:
    def guarded(fn):
        async def wrapper(request: Request) -> Response:
            try:
                return await fn(request)
            except CameraError as exc:
                return JSONResponse({"error": str(exc)}, status_code=exc.status)
            except (TypeError, ValueError) as exc:
                return JSONResponse({"error": str(exc)}, status_code=400)

        return wrapper

    @mcp.custom_route("/health", methods=["GET"], include_in_schema=False)
    async def health(_: Request) -> Response:
        return JSONResponse({"ok": True, "service": "pi-cameras-mcp", "version": __version__})

    @mcp.custom_route("/", methods=["GET"], include_in_schema=False)
    async def index(_: Request) -> Response:
        return HTMLResponse(render_index(), headers={"Cache-Control": "no-store"})

    @mcp.custom_route("/api/cameras", methods=["GET"], include_in_schema=False)
    @guarded
    async def cameras(request: Request) -> Response:
        rescan = request.query_params.get("rescan") in ("1", "true", "yes")
        return JSONResponse(service.list_cameras(rescan=rescan))

    @mcp.custom_route("/api/presets", methods=["GET"], include_in_schema=False)
    async def presets(_: Request) -> Response:
        return JSONResponse(service.list_resolutions())

    @mcp.custom_route("/api/status", methods=["GET"], include_in_schema=False)
    async def status(_: Request) -> Response:
        return JSONResponse(service.status())

    @mcp.custom_route("/api/stream/start", methods=["POST"], include_in_schema=False)
    @guarded
    async def stream_start(request: Request) -> Response:
        body = await _json_object(request)
        result = service.start_stream(
            _camera(body),
            _preset(body, SWEET_SPOT),
            _opt_int(body, "width"),
            _opt_int(body, "height"),
            _opt_int(body, "fps"),
            _opt_int(body, "quality"),
            bool(body.get("exact", False)),
        )
        return JSONResponse(result)

    @mcp.custom_route("/api/stream/stop", methods=["POST"], include_in_schema=False)
    @guarded
    async def stream_stop(request: Request) -> Response:
        body = await _json_object(request)
        camera = str(body.get("camera") or "c920")
        if camera.strip().lower() in ("all", "*"):
            return JSONResponse(service.stop_all())
        return JSONResponse(service.stop_stream(camera))

    @mcp.custom_route("/api/snapshot", methods=["POST"], include_in_schema=False)
    @guarded
    async def snapshot_post(request: Request) -> Response:
        body = await _json_object(request)
        result = service.snapshot(
            _camera(body),
            _preset(body, SNAPSHOT_DEFAULT),
            _opt_int(body, "width"),
            _opt_int(body, "height"),
            _opt_int(body, "fps"),
            _opt_int(body, "quality"),
            bool(body.get("exact", False)),
            bool(body.get("fresh", False)),
        )
        return _jpeg(result)

    @mcp.custom_route("/api/grab", methods=["POST"], include_in_schema=False)
    @guarded
    async def grab_post(request: Request) -> Response:
        body = await _json_object(request)
        preset = body.get("preset")
        result = service.grab_frame(
            _camera(body),
            None if preset in (None, "") else str(preset),
            _opt_int(body, "width"),
            _opt_int(body, "height"),
            _opt_int(body, "fps"),
            _opt_int(body, "quality"),
            bool(body.get("exact", False)),
            bool(body.get("fresh", False)),
            bool(body.get("keep_open", True)),
        )
        return _jpeg(result)

    @mcp.custom_route("/snapshot/{camera}", methods=["GET"], include_in_schema=False)
    @guarded
    async def snapshot_get(request: Request) -> Response:
        camera = _path_camera(request)
        params = request.query_params
        result = service.snapshot(
            camera,
            params.get("preset") or SNAPSHOT_DEFAULT,
            _query_int(params, "width"),
            _query_int(params, "height"),
            _query_int(params, "fps"),
            _query_int(params, "quality"),
            params.get("exact") in ("1", "true"),
            params.get("fresh") in ("1", "true"),
        )
        return _jpeg(result)

    @mcp.custom_route("/stream/{camera}/frame.jpg", methods=["GET"], include_in_schema=False)
    @guarded
    async def frame(request: Request) -> Response:
        camera = _path_camera(request)
        fresh = request.query_params.get("fresh") in ("1", "true")
        jpeg = service.latest_jpeg(camera, fresh=fresh)
        return Response(jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @mcp.custom_route("/stream/{camera}/live.mjpg", methods=["GET"], include_in_schema=False)
    @guarded
    async def live(request: Request) -> Response:
        camera = _path_camera(request)
        running = service.attach_viewer(camera)

        async def frames():
            seq = 0
            try:
                while True:
                    item = await anyio.to_thread.run_sync(running.session.frames.wait, seq, 5.0)
                    if item is None:
                        if running.session.dead:
                            break
                        continue
                    frame_bytes, seq = item
                    header = (
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(frame_bytes)).encode()
                        + b"\r\n\r\n"
                    )
                    yield header + frame_bytes + b"\r\n"
            finally:
                service.detach_viewer(running.spec.camera_id)

        return StreamingResponse(
            frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "Pragma": "no-cache",
                "X-Accel-Buffering": "no",
            },
        )


def _jpeg(result: CaptureResult) -> Response:
    return Response(
        result.jpeg,
        media_type="image/jpeg",
        headers={
            "Cache-Control": "no-store",
            "X-Width": str(result.width),
            "X-Height": str(result.height),
            "X-Elapsed-Ms": str(result.elapsed_ms),
            "X-Source": result.source,
            "X-Pipeline": result.pipeline,
            "X-Preset": result.preset,
            "X-Fps": str(result.fps),
            "X-Camera": result.camera,
            "X-Note": quote(result.note),
        },
    )


async def _json_object(request: Request) -> dict:
    if not request.headers.get("content-type", "").lower().startswith("application/json"):
        raise CameraError("expected a JSON body")
    payload = await request.json()
    if not isinstance(payload, dict):
        raise CameraError("expected a JSON object")
    return payload


def _camera(body: dict) -> str:
    return str(body.get("camera") or "c920")


def _preset(body: dict, default: str) -> str:
    value = body.get("preset")
    if value in (None, ""):
        return default
    return str(value)


def _opt_int(body: dict, key: str) -> int | None:
    value = body.get(key)
    if value in (None, ""):
        return None
    return int(value)


def _query_int(params, key: str) -> int | None:
    value = params.get(key)
    if value in (None, ""):
        return None
    return int(value)


def _path_camera(request: Request) -> str:
    camera = request.path_params["camera"]
    if not _CAMERA_ID.match(camera):
        raise CameraError("invalid camera id", status=400)
    return camera


class TokenGate:
    """Bearer token, ?access_token=, or the pi_cam cookie. /health stays open.

    Tailscale is the real boundary. The token stops other machines on the tailnet
    from opening the cameras if you set PI_CAMERAS_TOKEN.
    """

    def __init__(self, app, token: str | None) -> None:
        self.app = app
        self.token = token or ""

    async def __call__(self, scope, receive, send) -> None:
        if scope["type"] != "http" or not self.token:
            await self.app(scope, receive, send)
            return
        if scope.get("path") == "/health":
            await self.app(scope, receive, send)
            return
        provided = token_from_scope(scope)
        if not provided or not hmac.compare_digest(provided.encode(), self.token.encode()):
            body = b'{"error":"unauthorized"}'
            await send(
                {
                    "type": "http.response.start",
                    "status": 401,
                    "headers": [
                        (b"content-type", b"application/json"),
                        (b"www-authenticate", b'Bearer realm="pi-cameras"'),
                        (b"content-length", str(len(body)).encode()),
                        (b"cache-control", b"no-store"),
                    ],
                }
            )
            await send({"type": "http.response.body", "body": body})
            return

        async def send_cookie(message):
            if message["type"] == "http.response.start":
                headers = list(message.get("headers") or [])
                cookie = f"pi_cam={quote(self.token, safe='')}; Path=/; HttpOnly; SameSite=Lax"
                headers.append((b"set-cookie", cookie.encode()))
                message = {**message, "headers": headers}
            await send(message)

        await self.app(scope, receive, send_cookie)


def token_from_scope(scope) -> str:
    headers: dict[str, str] = {}
    for key, value in scope.get("headers") or []:
        headers[key.decode("latin1").lower()] = value.decode("latin1")
    auth = headers.get("authorization", "")
    if auth[:7].lower() == "bearer ":
        return auth[7:].strip()
    query = parse_qs(scope.get("query_string", b"").decode("latin1"))
    if query.get("access_token"):
        return query["access_token"][0]
    for part in headers.get("cookie", "").split(";"):
        part = part.strip()
        if part.startswith("pi_cam="):
            return unquote(part.split("=", 1)[1])
    return ""
