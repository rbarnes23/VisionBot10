import asyncio

from starlette.testclient import TestClient

from pi_cameras_mcp.remote import RemoteCameraService
from pi_cameras_mcp.server import build_mcp
from pi_cameras_mcp.settings import Settings

from tests.support import make_jpeg, service


def test_tools_return_text_and_jpeg():
    svc, _runner = service(make_jpeg(160, 120, b"\x07"))
    mcp, _app = build_mcp(svc, svc.settings, http=False)
    names = {tool.name for tool in asyncio.run(mcp.list_tools())}
    assert {
        "list_cameras",
        "list_resolutions",
        "camera_status",
        "start_stream",
        "stop_stream",
        "take_snapshot",
        "grab_frame",
        "sample_frames",
    } <= names
    result = asyncio.run(mcp.call_tool("grab_frame", {"camera": "c920", "preset": "balanced"}))
    assert result.is_error is not True
    kinds = [block.type for block in result.content]
    assert kinds == ["text", "image"]
    assert result.content[1].mime_type == "image/jpeg"
    text = result.content[0].text
    assert "preset=balanced" in text
    assert "fps=15" in text
    assert "mjpeg-copy" in text
    catalog = asyncio.run(mcp.call_tool("list_resolutions", {}))
    assert "balanced" in catalog.content[0].text
    assert "sweet spot" in catalog.content[0].text.lower() or "Sweet spot" in catalog.content[0].text


def test_http_preview_stream_and_snapshot():
    frame = make_jpeg(200, 100, b"\x08")
    svc, _runner = service(frame)
    _mcp, app = build_mcp(svc, svc.settings, http=True)
    with TestClient(app) as client:
        health = client.get("/health")
        assert health.status_code == 200
        assert health.json()["service"] == "pi-cameras-mcp"
        page = client.get("/")
        assert page.status_code == 200
        assert "Sweet spot" in page.text or "sweet spot" in page.text.lower()
        assert "balanced" in page.text
        presets = client.get("/api/presets").json()
        assert presets["sweet_spot"] == "balanced"
        cameras = client.get("/api/cameras").json()
        assert {row["id"] for row in cameras["cameras"]} == {"c920", "pi"}

        started = client.post("/api/stream/start", json={"camera": "c920", "preset": "balanced"})
        assert started.status_code == 200
        body = started.json()
        assert body["estimated_mbps"] < 6
        assert body["pipeline"] == "mjpeg-copy"

        with client.stream("GET", "/stream/c920/live.mjpg") as response:
            assert response.status_code == 200
            assert "multipart" in response.headers["content-type"]
            chunk = b""
            for part in response.iter_bytes():
                chunk += part
                if b"\xff\xd9" in chunk:
                    break
        assert b"--frame" in chunk
        assert frame in chunk

        shot = client.get("/snapshot/c920?preset=balanced")
        assert shot.status_code == 200
        assert shot.content == frame
        assert shot.headers["x-source"] == "live"

        stopped = client.post("/api/stream/stop", json={"camera": "c920"})
        assert stopped.json()["stopped"] is True
        missing = client.get("/stream/c920/frame.jpg")
        assert missing.status_code == 409


def test_token_and_remote_client():
    frame = make_jpeg(24, 24, b"\x09")
    svc, _runner = service(frame, token="secret-token")
    _mcp, app = build_mcp(svc, svc.settings, http=True)
    with TestClient(app) as client:
        assert client.get("/api/presets").status_code == 401
        assert client.get("/health").status_code == 200
        denied = client.post("/api/grab", json={"camera": "pi", "preset": "low"})
        assert denied.status_code == 401
        ok = client.post(
            "/api/grab",
            json={"camera": "pi", "preset": "low"},
            headers={"Authorization": "Bearer secret-token"},
        )
        assert ok.status_code == 200
        assert ok.content == frame
        assert ok.headers["x-pipeline"] == "rpicam-mjpeg"

        opened = client.get("/?access_token=secret-token")
        assert opened.status_code == 200
        assert "pi_cam=" in opened.headers.get("set-cookie", "")
        assert client.get("/api/status").status_code == 200

    remote = RemoteCameraService(Settings(host="127.0.0.1", port=9), client=_FakeHTTP(frame))
    got = remote.grab_frame("c920", "tiny", None, None, None, None, False, False, True)
    assert got.jpeg == frame
    assert got.pipeline == "mjpeg-copy"
    assert got.width == 24
    listed = remote.list_resolutions()
    assert listed["sweet_spot"] == "balanced"


class _FakeHTTP:
    def __init__(self, frame: bytes) -> None:
        self.frame = frame

    def request(self, method, path, **kwargs):
        if path == "/api/presets":
            return _JSONResponse({"sweet_spot": "balanced", "presets": []})
        if path == "/api/grab":
            body = kwargs.get("json") or {}
            assert body["camera"] == "c920"
            assert body["preset"] == "tiny"
            assert body["exact"] is False
            assert body["keep_open"] is True
            return _JPEGResponse(self.frame)
        raise AssertionError(path)


class _JSONResponse:
    def __init__(self, payload: dict) -> None:
        self.status_code = 200
        self._payload = payload
        self.text = ""

    def json(self):
        return self._payload


class _JPEGResponse:
    def __init__(self, frame: bytes) -> None:
        self.status_code = 200
        self.content = frame
        self.headers = {
            "x-width": "24",
            "x-height": "24",
            "x-elapsed-ms": "3",
            "x-source": "live",
            "x-pipeline": "mjpeg-copy",
            "x-preset": "tiny",
            "x-fps": "10",
            "x-camera": "c920",
            "x-note": "native",
        }
        self.text = ""
