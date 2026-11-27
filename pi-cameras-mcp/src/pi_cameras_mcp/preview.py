"""Single-page viewer served by the camera process. No extra assets."""

from __future__ import annotations


def render_index() -> str:
    return """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pi cameras</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; font: 15px/1.4 ui-sans-serif, system-ui, sans-serif; background: #121416; color: #e8e6e3; }
  header { padding: 16px 20px 8px; }
  h1 { font-size: 20px; margin: 0 0 4px; font-weight: 600; }
  p { margin: 0; color: #b7b3ac; }
  main { display: grid; gap: 16px; padding: 8px 20px 28px; }
  @media (min-width: 900px) { main { grid-template-columns: 1fr 1fr; } }
  article { background: #1c1f22; border-radius: 12px; padding: 14px; }
  h2 { font-size: 16px; margin: 0 0 8px; }
  .row { display: flex; flex-wrap: wrap; gap: 8px; margin: 8px 0; }
  button { background: #2a2f33; color: inherit; border: 1px solid #3a4045; border-radius: 8px; padding: 6px 10px; cursor: pointer; }
  button.active, button.primary { background: #3d5a4c; border-color: #6ea888; }
  img { width: 100%; background: #000; border-radius: 8px; min-height: 180px; object-fit: contain; }
  .meta { color: #b7b3ac; min-height: 1.4em; }
  .err { color: #f0b4a8; }
</style>
</head>
<body>
<header>
  <h1>Pi cameras</h1>
  <p id="why">Sweet spot is balanced: 1280×720 at 15 fps. The C920 sends that as native MJPEG, about 4 Mbps over Tailscale.</p>
</header>
<main id="boards"></main>
<script>
const boards = document.getElementById("boards");
const why = document.getElementById("why");
let presets = [];
let sweet = "balanced";
const choice = {};
const live = {};

async function api(path, options) {
  const response = await fetch(path, options);
  const type = response.headers.get("content-type") || "";
  if (!response.ok) {
    let message = response.statusText;
    if (type.includes("json")) {
      const body = await response.json();
      message = body.error || message;
    }
    throw new Error(message);
  }
  if (type.includes("json")) return response.json();
  return response;
}

function button(label, className, onClick) {
  const el = document.createElement("button");
  el.textContent = label;
  if (className) el.className = className;
  el.addEventListener("click", onClick);
  return el;
}

function render(cameras) {
  boards.replaceChildren();
  if (!cameras.length) {
    const article = document.createElement("article");
    article.innerHTML = "<h2>No cameras found</h2><p class='meta'>Check the cables, then reload. v4l2-ctl --list-devices should show the C920, and rpicam-hello --list-cameras should show the Pi camera.</p>";
    boards.appendChild(article);
    return;
  }
  for (const camera of cameras) {
    if (!choice[camera.id]) choice[camera.id] = sweet;
    const article = document.createElement("article");
    const title = document.createElement("h2");
    title.textContent = camera.label + " · " + camera.id;
    const detail = document.createElement("p");
    detail.className = "meta";
    detail.textContent = [camera.device, camera.sensor, camera.streaming ? "streaming" : "idle"].filter(Boolean).join(" · ");
    const presetRow = document.createElement("div");
    presetRow.className = "row";
    for (const preset of presets) {
      const active = choice[camera.id] === preset.id ? "active" : "";
      const label = preset.width ? preset.id + " " + preset.width + "×" + preset.height + " @" + preset.fps : preset.id;
      presetRow.appendChild(button(label, active, () => { choice[camera.id] = preset.id; render(cameras); }));
    }
    const actions = document.createElement("div");
    actions.className = "row";
    const meta = document.createElement("p");
    meta.className = "meta";
    const img = document.createElement("img");
    img.alt = camera.label;
    img.hidden = true;
    actions.append(
      button("Start stream", "primary", () => stream(camera.id, true, meta, img)),
      button("Stop", "", () => stream(camera.id, false, meta, img)),
      button("Snapshot", "", () => snapshot(camera.id, meta, img))
    );
    article.append(title, detail, presetRow, actions, meta, img);
    boards.appendChild(article);
    if (camera.streaming || live[camera.id]) showStream(camera.id, img);
  }
}

function showStream(id, img) {
  img.hidden = false;
  img.src = "/stream/" + encodeURIComponent(id) + "/live.mjpg?t=" + Date.now();
}

async function stream(id, start, meta, img) {
  meta.className = "meta";
  meta.textContent = start ? "Starting…" : "Stopping…";
  try {
    const body = JSON.stringify({ camera: id, preset: choice[id] || sweet });
    const path = start ? "/api/stream/start" : "/api/stream/stop";
    const result = await api(path, { method: "POST", headers: { "content-type": "application/json" }, body });
    if (start) {
      live[id] = true;
      meta.textContent = result.width + "×" + result.height + " @" + result.fps + " · " + result.pipeline + " · ~" + result.estimated_mbps + " Mbps";
      showStream(id, img);
    } else {
      live[id] = false;
      img.hidden = true;
      img.removeAttribute("src");
      meta.textContent = "Stopped";
    }
  } catch (error) {
    meta.className = "err";
    meta.textContent = error.message;
  }
}

function snapshot(id, meta, img) {
  meta.className = "meta";
  meta.textContent = "Capturing…";
  img.hidden = false;
  img.onload = () => { meta.textContent = "Snapshot"; };
  img.onerror = () => { meta.className = "err"; meta.textContent = "Snapshot failed"; };
  const preset = choice[id] || "detail";
  img.src = "/snapshot/" + encodeURIComponent(id) + "?preset=" + encodeURIComponent(preset) + "&t=" + Date.now();
}

async function boot() {
  try {
    const catalog = await api("/api/presets");
    presets = catalog.presets;
    sweet = catalog.sweet_spot;
    why.textContent = catalog.why;
    const found = await api("/api/cameras");
    render(found.cameras || []);
  } catch (error) {
    why.textContent = error.message;
  }
}
boot();
</script>
</body>
</html>
"""
