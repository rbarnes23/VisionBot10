# Pi camera MCP

MCP server for a Raspberry Pi 4 with a Logitech C920 and a CSI camera. Ask for a snapshot or a live view. The Pi keeps one capture open per camera and hands the newest JPEG to every caller.

The C920 already compresses MJPEG. At a native size, ffmpeg copies those frames and the Pi does not encode. The CSI camera uses `rpicam-vid` / `rpicam-still`, which encode JPEG in hardware.

## Sweet spot

| Preset | Size | fps | When |
| --- | --- | --- | --- |
| `tiny` | 640×480 | 10 | A quick look on a slow link. About 1 Mbps. |
| `low` | 640×480 | 15 | Tight remote links. About 1.5 Mbps. |
| **`balanced`** | **1280×720** | **15** | **Default stream.** Native C920 MJPEG, no Pi encode, about 4 Mbps. This is the Tailscale sweet spot. |
| `smooth` | 1280×720 | 30 | Same picture, smoother motion, on a LAN. About 8 Mbps. |
| `detail` | 1920×1080 | 15 | Default snapshot. Reading text. About 8 Mbps if you stream it. |
| `max` | 1920×1080 | 30 | LAN only. About 18 Mbps. The C920 can do this natively. |
| `full` | sensor | 5 | Largest still. Pi cameras such as the IMX708 are 4608×2592. Streams above 1080p stay at 10 fps or below. |

A custom width and height is snapped to the nearest native C920 MJPEG mode unless you pass `exact=true`. Exact mode scales on the Pi and costs CPU. Leave it off.

`grab_frame` while a stream is running reads the frame already in memory. The first call pays for opening the camera. The next ones do not. A stream with no viewers and no grabs stops after 120 seconds.

## Tools

- `list_cameras` — ids, devices, native modes
- `list_resolutions` — the table above
- `grab_frame` — one JPEG the model can see. Leaves the camera open.
- `sample_frames` — a few JPEGs a fraction of a second apart
- `take_snapshot` — one JPEG, default `detail`. Uses the live frame when the size matches.
- `start_stream` — starts MJPEG and returns a browser URL
- `stop_stream` — one camera, or `all`
- `camera_status` — frame rate, age of the last frame, URLs

Video does not play inside the MCP session. `start_stream` returns a URL. `grab_frame` and `sample_frames` are how a model looks at the cameras.

## Install on the Pi

Raspberry Pi OS Bookworm. The Pi at `100.109.182.59` must be this machine (`tailscale ip -4`). Your laptop has to be on the same tailnet.

```bash
sudo apt update
sudo apt install -y ffmpeg v4l-utils python3 python3-venv python3-pip
# Bookworm already has the CSI tools. If rpicam-hello is missing:
# sudo apt install -y rpicam-apps
```

From this directory:

```bash
sudo ./deploy/install.sh
```

That creates `.venv`, writes `/etc/pi-cameras-mcp.env` (host, port, a bearer token), and starts `pi-cameras-mcp.service`. The service is in the `video` and `render` groups, so it can open the cameras without a reboot.

Watch:

```text
http://100.109.182.59:8765/?access_token=THE_TOKEN
```

The token is printed by the installer and stored in `/etc/pi-cameras-mcp.env`. Tailscale is the network boundary. The token keeps other machines on the tailnet from opening the cameras. Clear `PI_CAMERAS_TOKEN` if you want the tailnet alone to be the lock.

Check discovery without starting a second server:

```bash
.venv/bin/pi-cameras-mcp check
```

If the Tailscale address is not `100.109.182.59`, set `PI_CAMERAS_HOST` in the env file to the output of `tailscale ip -4`, then `sudo systemctl restart pi-cameras-mcp`. Use `PI_CAMERAS_HOST=0.0.0.0` only when you want every interface, not just the tailnet.

`PI_CAMERAS_C920_DEVICE=/dev/video0` forces the webcam path when the USB name does not contain `C920`.

## Cursor

Put this in `~/.cursor/mcp.json` (the installer prints it with the real token):

```json
{
  "mcpServers": {
    "pi-cameras": {
      "url": "http://100.109.182.59:8765/mcp",
      "headers": {
        "Authorization": "Bearer YOUR_TOKEN"
      }
    }
  }
}
```

If Cursor cannot use a URL, SSH stdio works. When the service is already running, this attaches to it instead of opening the camera twice:

```json
{
  "mcpServers": {
    "pi-cameras": {
      "command": "ssh",
      "args": [
        "pi@100.109.182.59",
        "/path/to/pi-cameras-mcp/.venv/bin/python",
        "-m",
        "pi_cameras_mcp",
        "stdio"
      ]
    }
  }
}
```

## HTTP

| Method | Path | What |
| --- | --- | --- |
| GET | `/` | Viewer |
| GET | `/health` | Open, no token |
| GET | `/api/cameras` | Discovery |
| GET | `/api/presets` | Presets |
| GET | `/api/status` | Running streams |
| POST | `/api/stream/start` | `{"camera":"c920","preset":"balanced"}` |
| POST | `/api/stream/stop` | `{"camera":"c920"}` or `{"camera":"all"}` |
| GET | `/snapshot/{camera}?preset=detail` | One JPEG |
| GET | `/stream/{camera}/live.mjpg` | Multipart MJPEG |
| GET | `/stream/{camera}/frame.jpg` | Latest JPEG |
| POST | `/mcp` | Streamable HTTP MCP |

Send `Authorization: Bearer TOKEN`, `?access_token=TOKEN`, or the `pi_cam` cookie set by a successful request.

## What the model can say

- "Grab a frame from the C920."
- "Stream the Pi camera at the sweet spot and give me the URL."
- "Take a 1080p snapshot of the Pi camera."
- "Sample three frames from the C920 so I can see if it is moving."
- "Switch the C920 to smooth."

## If it fails

- `Cannot bind 100.109.182.59` — that address is not on this Pi. Set `PI_CAMERAS_HOST` from `tailscale ip -4`.
- `Device or resource busy` — another program has the camera. Stop it, or `stop_stream`.
- `Permission denied` on `/dev/video*` — the systemd unit adds the `video` group. A shell user needs `sudo usermod -aG video,render $USER` and a new login.
- No Pi camera — `rpicam-hello --list-cameras`. Reseat the ribbon. The ISP nodes (`pispbe`, `rp1-cfe`) are not the camera and are ignored.
- C920 missing — `v4l2-ctl --list-devices`, then set `PI_CAMERAS_C920_DEVICE`.
- Cursor times out — the laptop is not on the tailnet, or the token header is missing.
- Dark first still — `grab_frame` after the stream has been up for a moment. A cold `rpicam-still --immediate` can fire before exposure settles.
