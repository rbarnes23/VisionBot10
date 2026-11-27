#!/usr/bin/env bash
# Install the camera MCP into a venv and, when run as root, a systemd service.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
PY="${PYTHON:-python3}"

if [[ "${1:-}" != "--service" ]]; then
  "$PY" -m venv .venv
  .venv/bin/pip install --upgrade pip
  .venv/bin/pip install .
  if [[ "$(id -u)" -eq 0 ]]; then
    exec "$0" --service
  fi
  echo
  echo "Venv ready at $REPO/.venv"
  echo "Install the boot service with: sudo $0 --service"
  exit 0
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "sudo $0 --service" >&2
  exit 1
fi

TARGET="$(stat -c '%U' "$REPO")"
if [[ ! -x "$REPO/.venv/bin/pi-cameras-mcp" ]]; then
  sudo -u "$TARGET" "$PY" -m venv "$REPO/.venv"
  sudo -u "$TARGET" "$REPO/.venv/bin/pip" install --upgrade pip
  sudo -u "$TARGET" "$REPO/.venv/bin/pip" install "$REPO"
fi

TS_IP="$(tailscale ip -4 2>/dev/null | head -n1 || true)"
TS_IP="${TS_IP:-100.109.182.59}"
ENV_FILE=/etc/pi-cameras-mcp.env
if [[ ! -f "$ENV_FILE" ]]; then
  TOKEN="$( "$REPO/.venv/bin/python" -c 'import secrets; print(secrets.token_urlsafe(24))' )"
  umask 077
  cat > "$ENV_FILE" <<EOF
PI_CAMERAS_HOST=$TS_IP
PI_CAMERAS_PORT=8765
PI_CAMERAS_TOKEN=$TOKEN
PI_CAMERAS_IDLE_SECONDS=120
EOF
  chmod 600 "$ENV_FILE"
  echo "Wrote $ENV_FILE"
else
  TOKEN="$(sed -n 's/^PI_CAMERAS_TOKEN=//p' "$ENV_FILE" | head -n1)"
  echo "Keeping existing $ENV_FILE"
fi

UNIT=/etc/systemd/system/pi-cameras-mcp.service
sed \
  -e "s|__USER__|$TARGET|g" \
  -e "s|__REPO__|$REPO|g" \
  -e "s|__VENV__|$REPO/.venv|g" \
  "$REPO/deploy/pi-cameras-mcp.service" > "$UNIT"

systemctl daemon-reload
systemctl enable --now pi-cameras-mcp.service
systemctl --no-pager --full status pi-cameras-mcp.service || true

echo
echo "Cursor MCP config (~/.cursor/mcp.json):"
cat <<EOF
{
  "mcpServers": {
    "pi-cameras": {
      "url": "http://${TS_IP}:8765/mcp",
      "headers": { "Authorization": "Bearer ${TOKEN}" }
    }
  }
}
EOF
echo
echo "Watch in a browser on the same tailnet:"
echo "  http://${TS_IP}:8765/?access_token=${TOKEN}"
echo
echo "Token file: sudo cat $ENV_FILE"
