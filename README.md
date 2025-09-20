### Rescuer Interface: Live SAR Dashboard (ESP32-CAM + YOLO + ORB‑SLAM2)

One-window web dashboard showing:
- Live camera feed with CV overlays (YOLO person detections)
- Top‑down minimap with robot path and SLAM map points (walls/structure)

Backed by a FastAPI server feeding WebSocket updates, a proxy for the MJPEG video, and a bridge that streams ORB‑SLAM2 pose and map points to the UI.

---

## Quick Start

Prereqs:
- Python 3.10+
- C++ toolchain for ORB‑SLAM2 (already vendored here). To rebuild: `./build.sh`
- ESP32‑CAM reachable at `http://<ip>` and stream at `http://<ip>:81/stream`

Install deps (first time):
```bash
cd /Users/dganjali/GitHub/pennapps
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

Start backend (Terminal 1):
```bash
uvicorn server:app --host 0.0.0.0 --port 8000
```

Start orchestrator (YOLO + SLAM bridge) (Terminal 2):
```bash
source .venv/bin/activate
python orchestrator.py http://localhost:8000 http://<esp32-ip>:81/stream
```

Open dashboard:
- http://localhost:8000/
- Optional: `?stream=http://<esp32-ip>:81/stream`

---

## What’s Running

- `server.py` (FastAPI)
  - Serves dashboard at `/dashboard` (root `/` redirects)
  - Proxies MJPEG video: `GET /video_proxy?url=...`
  - Collects data and broadcasts via WebSocket `/ws`
  - State keys: `detections`, `hazards`, `pose`, `path`, `map_points`

- `esp32_yolo_detection.py`
  - Reads MJPEG stream via OpenCV, runs YOLOv8 (persons), draws locally, and posts detections to `/api/detections?replace=true`

- `Examples/Monocular/mono_esp32cam_live` (C++)
  - Reads the ESP32‑CAM stream directly into ORB‑SLAM2
  - Writes pose lines to stdout: `POSE t x y z qx qy qz qw`
  - Writes sampled map points to stdout: `MAPPOINTS x y z x y z ...`

- `run_esp32_slam.py`
  - Spawns the C++ binary, parses stdout, POSTs:
    - Pose → `POST /api/pose`
    - Map points → `POST /api/map_points`

- `web/index.html`
  - Fullscreen single canvas
  - Draws: contained video frame, detection boxes, top‑right minimap with path + map points + current pose

- `orchestrator.py`
  - Boots YOLO and SLAM bridge together; forwards same `--stream` URL to both

---

## Configuration

Environment variables (optional):
- `ESP32_STREAM_URL` default `http://192.168.43.36:81/stream`
- `DASHBOARD_BASE` default `http://localhost:8000`

CLI flags:
- `orchestrator.py <server_base> <stream_url>`
- `esp32_yolo_detection.py --server <base> --stream <url> [--post-interval 0.5]`
- `run_esp32_slam.py --server <base> --stream <url>`

Camera controls (HTTP GET to `http://<ip>/control?...`), exposed by helper functions in YOLO/SLAM scripts:
- `framesize` (e.g., 6=VGA 640x480)
- `quality` (JPEG 10..63)
- `awb`, `aec`, `agc`, `hmirror`, `vflip`

---

## API

Server endpoints:
- `GET /` → redirects to `/dashboard`
- `GET /dashboard` → static UI
- `GET /video_proxy?url=<mjpeg_url>` → transparent MJPEG proxy
- `GET /ws` (WebSocket) → server broadcasts state updates
- `GET /api/state` → current server state
- `POST /api/detections?replace=true|false` → list of detections
  - item: `{ id, bbox:[x1,y1,x2,y2], confidence, source }`
- `POST /api/hazards` → list of hazards
  - item: `{ id, kind, position:[x,y,z], confidence }`
- `POST /api/pose` → camera pose
  - `{ t, position:[x,y,z], orientation:[qx,qy,qz,qw] }`
- `POST /api/map_points?replace=true|false` → list of points `[ [x,y,z], ... ]`

Broadcast messages (JSON): `type` ∈ `state|detections|hazards|pose|map_points`

---

## Build SLAM (if needed)

Most binaries are included, but to rebuild:
```bash
chmod +x build.sh
./build.sh
```
You should get `Examples/Monocular/mono_esp32cam_live` and others. Vocabulary at `Vocabulary/ORBvoc.txt` must exist.

---

## Troubleshooting

Video not loading / 404:
- Open `http://localhost:8000/dashboard?stream=http://<ip>:81/stream`
- Check the proxy directly: `curl -I "http://localhost:8000/video_proxy?url=http://<ip>:81/stream"`

ESP32 not reachable:
- Verify power/Wi‑Fi, ping the IP
- `curl -I http://<ip>:81/stream`
- Adjust firewall on macOS if needed

SLAM binary missing:
- Build with `./build.sh` or `make mono_esp32cam_live` in `Examples/Monocular`
- Ensure `Vocabulary/ORBvoc.txt` present (see `Vocabulary/ORBvoc.txt.tar.gz`)

Performance:
- Lower camera `framesize` index; increase JPEG `quality` value (lower quality)
- YOLO: increase `detection_skip_frames` in `esp32_yolo_detection.py`
- Use wired network if Wi‑Fi is unstable

UI looks tiny:
- The UI is now full‑screen and scales to the window; avoid browser zoom <100%

Security note:
- `video_proxy` will fetch arbitrary URLs if exposed; restrict via network controls in production

---

## Project Layout (high‑signal)

- `server.py` — FastAPI backend (dashboard, proxy, WS, REST)
- `web/index.html` — Fullscreen UI (video + overlays + minimap)
- `orchestrator.py` — Run YOLO + SLAM bridge together
- `esp32_yolo_detection.py` — Person detection; posts to server
- `run_esp32_slam.py` — Runs C++ SLAM; posts pose/map points
- `Examples/Monocular/mono_esp32cam_live.cc` — ESP32 live SLAM example
- `rescuer/config.py` — Env‑driven defaults
- `Vocabulary/ORBvoc.txt` — ORB‑SLAM2 vocabulary (required)

---

## Credits & License

This project integrates the original [ORB‑SLAM2](https://github.com/raulmur/ORB_SLAM2) (GPLv3). See `LICENSE.txt` and `License-gpl.txt`.
