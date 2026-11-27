"""Run the camera MCP on the Pi."""

from __future__ import annotations

import argparse
import logging
import shutil
import socket
import sys
import threading
import time

import uvicorn

from pi_cameras_mcp import __version__
from pi_cameras_mcp.remote import probe_daemon
from pi_cameras_mcp.server import build_mcp
from pi_cameras_mcp.service import CameraService
from pi_cameras_mcp.settings import Settings


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="pi-cameras-mcp", description="C920 and Pi camera MCP")
    parser.add_argument("--version", action="version", version=__version__)
    sub = parser.add_subparsers(dest="command", required=True)

    serve = sub.add_parser("serve", help="HTTP MCP and MJPEG on the Tailscale address")
    _add_net(serve)
    stdio = sub.add_parser("stdio", help="stdio MCP; uses the running service if there is one")
    _add_net(stdio)
    check = sub.add_parser("check", help="print cameras and binaries, then exit")
    _add_net(check)

    args = parser.parse_args(argv)
    settings = Settings.from_env()
    if args.host:
        settings.host = args.host
    if args.port:
        settings.port = args.port
    settings.autodetect_binaries()
    _reject_bad_token(settings)

    if args.command == "check":
        raise SystemExit(run_check(settings))
    if args.command == "serve":
        serve_forever(settings)
        return
    run_stdio(settings)


def _add_net(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--host", default=None, help="bind address, default PI_CAMERAS_HOST or 100.109.182.59")
    parser.add_argument("--port", type=int, default=None, help="default PI_CAMERAS_PORT or 8765")


def _reject_bad_token(settings: Settings) -> None:
    if settings.token and any(char in settings.token for char in "\r\n;"):
        raise SystemExit("PI_CAMERAS_TOKEN cannot contain semicolons or newlines")


def run_check(settings: Settings) -> int:
    from pi_cameras_mcp.discover import discover_cameras

    print(f"pi-cameras-mcp {__version__}")
    print(f"listen {settings.host}:{settings.port}")
    print(f"public {settings.base_url}")
    print(f"ffmpeg {shutil.which(settings.ffmpeg) or 'MISSING'}")
    print(f"v4l2-ctl {shutil.which(settings.v4l2_ctl) or 'MISSING'}")
    print(f"rpicam-vid {settings.rpicam_vid or 'MISSING'}")
    print(f"rpicam-still {settings.rpicam_still or 'MISSING'}")
    found = discover_cameras(settings)
    if not found.cameras:
        print("cameras: none")
    for camera in found.cameras:
        modes = ", ".join(f"{mode.width}x{mode.height}" for mode in camera.modes[:8]) or "hardware scaled"
        print(f"camera {camera.id} kind={camera.kind} device={camera.device or '-'} modes={modes}")
    for note in found.notes:
        print(f"note: {note}")
    print("sweet spot: balanced = 1280x720 @ 15 fps (native C920 MJPEG, about 4 Mbps)")
    print("snapshot default: detail = 1920x1080")
    missing = shutil.which(settings.ffmpeg) is None and settings.ffmpeg == "ffmpeg"
    return 1 if missing else 0


def serve_forever(settings: Settings) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", stream=sys.stderr)
    ensure_bindable(settings.host, settings.port)
    service = CameraService(settings)
    service.start_reaper()
    _mcp, app = build_mcp(service, settings, http=True)
    assert app is not None
    uvicorn.run(app, host=settings.host, port=settings.port, log_level="info", access_log=False)


def run_stdio(settings: Settings) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", stream=sys.stderr)
    if probe_daemon(settings):
        from pi_cameras_mcp.remote import RemoteCameraService

        logging.info("using camera service at %s", settings.base_url)
        service = RemoteCameraService(settings)
        mcp, _app = build_mcp(service, settings, http=False)
        mcp.run(transport="stdio")
        return
    ensure_bindable(settings.host, settings.port)
    service = CameraService(settings)
    service.start_reaper()
    _http_mcp, app = build_mcp(service, settings, http=True)
    start_http(app, settings)
    stdio_mcp, _app = build_mcp(service, settings, http=False)
    stdio_mcp.run(transport="stdio")


def start_http(app, settings: Settings) -> None:
    config = uvicorn.Config(
        app,
        host=settings.host,
        port=settings.port,
        log_level="warning",
        access_log=False,
    )
    server = uvicorn.Server(config)
    server.install_signal_handlers = False
    thread = threading.Thread(target=server.run, name="pi-cameras-http", daemon=True)
    thread.start()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if server.started:
            return
        time.sleep(0.05)
    logging.warning("HTTP server did not report started; stream URLs may fail")


def ensure_bindable(host: str, port: int) -> None:
    if host in ("0.0.0.0", "::"):
        return
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind((host, port))
    except OSError as exc:
        raise SystemExit(
            f"Cannot bind {host}:{port} ({exc}). "
            "On the Pi run `tailscale ip -4` and set PI_CAMERAS_HOST to that address, "
            "or set PI_CAMERAS_HOST=0.0.0.0 to listen on every interface."
        ) from exc
    finally:
        sock.close()


if __name__ == "__main__":
    main()
