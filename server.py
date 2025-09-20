from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
from fastapi.responses import RedirectResponse, StreamingResponse
import requests
import os
from rescuer.config import get_default_stream_url
import math
import asyncio
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from typing import List, Dict, Any
import asyncio

app = FastAPI(title="Rescuer Interface Server")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/dashboard", StaticFiles(directory="web", html=True), name="dashboard")

class Detection(BaseModel):
    id: str
    bbox: List[float]
    confidence: float
    source: str = ""

class Hazard(BaseModel):
    id: str
    kind: str
    position: List[float]
    confidence: float = 1.0

class Pose(BaseModel):
    t: float
    position: List[float]
    orientation: List[float]

class ConnectionManager:
    def __init__(self):
        self.active: List[WebSocket] = []
        self.lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self.lock:
            self.active.append(websocket)

    async def disconnect(self, websocket: WebSocket):
        async with self.lock:
            if websocket in self.active:
                self.active.remove(websocket)

    async def broadcast(self, message: Dict[str, Any]):
        async with self.lock:
            websockets = list(self.active)
        for ws in websockets:
            try:
                await ws.send_json(message)
            except Exception:
                try:
                    await ws.close()
                except Exception:
                    pass
                await self.disconnect(ws)

manager = ConnectionManager()

state: Dict[str, Any] = {
    "detections": {},
    "hazards": {},
    "pose": None,
    "path": [],
    "map_points": [],
    "nav": {
        "goal": None,
        "mode": "manual",
        "controller_base": None
    },
    "explore": {
        "enabled": False,
        "grid": None,
        "cell": 0.3,
        "size": 200,  # grid size NxN
        "origin": [-30.0, -30.0]  # meters
    }
}

@app.get("/health")
async def health():
    return {"status": "ok"}

@app.get("/")
async def root_redirect():
    return RedirectResponse(url="/dashboard")

@app.get("/video_proxy")
def video_proxy(url: str = ""):
    if not url:
        url = get_default_stream_url()
    try:
        upstream = requests.get(url, stream=True, timeout=5)
    except Exception as e:
        return {"error": f"failed to connect to stream: {e}"}

    content_type = upstream.headers.get("Content-Type", "multipart/x-mixed-replace")

    def iter_stream():
        try:
            for chunk in upstream.iter_content(chunk_size=4096):
                if chunk:
                    yield chunk
        finally:
            try:
                upstream.close()
            except Exception:
                pass

    return StreamingResponse(iter_stream(), media_type=content_type)

# Simple background-ish planner (polled via client or external cron)
@app.post("/api/nav/step")
async def nav_step():
    goal = state["nav"].get("goal")
    pose = state.get("pose")
    if not goal or not pose:
        return {"ok": False, "error": "no goal or pose"}
    # naive P-controller on XY: command roll/pitch toward goal
    gx, gy = goal["x"], goal["y"]
    px, py = pose["position"][0], pose["position"][1]
    dx, dy = gx - px, gy - py
    dist = math.hypot(dx, dy)
    # gains (tune conservatively!)
    K = 5.0
    roll_cmd = K * dx  # roll right to move +x
    pitch_cmd = -K * dy # pitch forward to move +y
    roll_cmd = max(-10.0, min(10.0, roll_cmd))
    pitch_cmd = max(-10.0, min(10.0, pitch_cmd))
    T = 1200 if dist > 0.5 else 1050
    try:
        requests.get(f"{_controller_base()}/api/rc", params={"T": T, "R": roll_cmd, "P": pitch_cmd, "Y": 0}, timeout=0.5)
        return {"ok": True, "dist": dist, "cmd": {"T": T, "R": roll_cmd, "P": pitch_cmd}}
    except Exception as e:
        return {"ok": False, "error": str(e)}

@app.post("/api/nav/mode")
async def nav_mode(mode: str):
    if mode not in ("manual", "auto"):
        return {"ok": False, "error": "mode must be manual|auto"}
    state["nav"]["mode"] = mode
    await manager.broadcast({"type": "nav_mode", "data": mode})
    return {"ok": True, "mode": mode}

async def _planner_loop():
    while True:
        try:
            if state["nav"].get("mode") == "auto" and state["nav"].get("goal") and state.get("pose"):
                # reuse logic from nav_step
                goal = state["nav"]["goal"]
                pose = state["pose"]
                gx, gy = goal["x"], goal["y"]
                px, py = pose["position"][0], pose["position"][1]
                dx, dy = gx - px, gy - py
                dist = math.hypot(dx, dy)
                K = 5.0
                roll_cmd = max(-10.0, min(10.0, K * dx))
                pitch_cmd = max(-10.0, min(10.0, -K * dy))
                T = 1200 if dist > 0.5 else 1050
                try:
                    requests.get(f"{_controller_base()}/api/rc", params={"T": T, "R": roll_cmd, "P": pitch_cmd, "Y": 0}, timeout=0.3)
                except Exception:
                    pass
            # exploration: select new goal when near target
            if state["explore"]["enabled"] and state.get("pose"):
                _update_grid_from_map()
                g = _choose_frontier_goal()
                if g is not None:
                    state["nav"]["goal"] = {"x": g[0], "y": g[1], "z": 0.0}
                    await manager.broadcast({"type": "nav_goal", "data": state["nav"]["goal"]})
        except Exception:
            pass
        await asyncio.sleep(0.1)

@app.on_event("startup")
async def _on_start():
    asyncio.create_task(_planner_loop())

# ===== Exploration helpers =====
def _grid_init():
    N = state["explore"]["size"]
    state["explore"]["grid"] = [[0 for _ in range(N)] for _ in range(N)]  # 0 unknown, 1 free, 2 occ

def _world_to_grid(x: float, y: float):
    ox, oy = state["explore"]["origin"]
    cell = state["explore"]["cell"]
    gx = int((x - ox) / cell)
    gy = int((y - oy) / cell)
    return gx, gy

def _mark_cell(gx: int, gy: int, val: int):
    grid = state["explore"]["grid"]
    N = state["explore"]["size"]
    if 0 <= gx < N and 0 <= gy < N:
        grid[gy][gx] = val

def _update_grid_from_map():
    if state["explore"]["grid"] is None:
        _grid_init()
    grid = state["explore"]["grid"]
    # mark path as free
    for p in state.get("path", []):
        gx, gy = _world_to_grid(p[0], p[1])
        _mark_cell(gx, gy, 1)
    # mark map points as occupied (sparse)
    step = max(1, int(len(state.get("map_points", [])) / 2000))
    for i in range(0, len(state.get("map_points", [])), step):
        mp = state["map_points"][i]
        gx, gy = _world_to_grid(mp[0], mp[1])
        _mark_cell(gx, gy, 2)

def _choose_frontier_goal():
    grid = state["explore"]["grid"]
    if grid is None:
        return None
    # frontier = free cell with at least one unknown neighbor
    N = state["explore"]["size"]
    poses = state.get("pose")
    if not poses:
        return None
    px, py = poses["position"][0], poses["position"][1]
    pgx, pgy = _world_to_grid(px, py)
    best = None
    best_d = 1e9
    for y in range(max(0, pgy-40), min(N, pgy+40)):
        for x in range(max(0, pgx-40), min(N, pgx+40)):
            if grid[y][x] != 1:
                continue
            # 4-neighborhood unknown check
            unknown = False
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nx, ny = x+dx, y+dy
                if 0 <= nx < N and 0 <= ny < N and grid[ny][nx] == 0:
                    unknown = True; break
            if not unknown:
                continue
            d = (x - pgx)*(x - pgx) + (y - pgy)*(y - pgy)
            if d < best_d:
                best_d = d
                best = (x, y)
    if best is None:
        return None
    # convert back to world
    gx, gy = best
    ox, oy = state["explore"]["origin"]
    cell = state["explore"]["cell"]
    wx = ox + gx * cell
    wy = oy + gy * cell
    return (wx, wy)

@app.post("/api/explore/start")
async def explore_start():
    state["explore"]["enabled"] = True
    _grid_init()
    await manager.broadcast({"type": "explore", "data": True})
    return {"ok": True}

@app.post("/api/explore/stop")
async def explore_stop():
    state["explore"]["enabled"] = False
    await manager.broadcast({"type": "explore", "data": False})
    return {"ok": True}

@app.get("/api/explore/status")
async def explore_status():
    return {"enabled": state["explore"]["enabled"], "cell": state["explore"]["cell"], "origin": state["explore"]["origin"]}

@app.post("/api/detections")
async def post_detections(items: List[Detection], replace: bool = Query(default=False)):
    if replace:
        state["detections"] = {}
    for d in items:
        state["detections"][d.id] = d.model_dump()
    await manager.broadcast({"type": "detections", "data": list(state["detections"].values())})
    return {"ok": True, "count": len(items), "replace": replace}

@app.post("/api/hazards")
async def post_hazards(items: List[Hazard]):
    for h in items:
        state["hazards"][h.id] = h.model_dump()
    await manager.broadcast({"type": "hazards", "data": list(state["hazards"].values())})
    return {"ok": True, "count": len(items)}

@app.post("/api/pose")
async def post_pose(p: Pose):
    state["pose"] = p.model_dump()
    # Append to path history (keep last 200 points)
    pos = state["pose"].get("position", [0,0,0])
    state["path"].append(pos)
    if len(state["path"]) > 200:
        state["path"] = state["path"][ -200: ]
    await manager.broadcast({"type": "pose", "data": state["pose"]})
    return {"ok": True}

@app.post("/api/map_points")
async def post_map_points(points: List[List[float]], replace: bool = Query(default=False)):
    # points: list of [x,y,z]
    if replace:
        state["map_points"] = []
    # keep memory in check
    state["map_points"].extend(points)
    if len(state["map_points"]) > 20000:
        state["map_points"] = state["map_points"][ -20000: ]
    await manager.broadcast({"type": "map_points", "data": points, "replace": replace})
    return {"ok": True, "count": len(points), "total": len(state["map_points"]) }

@app.get("/api/state")
async def get_state():
    return state

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        await websocket.send_json({"type": "state", "data": state})
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        await manager.disconnect(websocket)

# Navigation control endpoints to talk to ESP32 flight controller (HTTP RC API)
class NavGoal(BaseModel):
    x: float
    y: float
    z: float = 0.0

def _controller_base() -> str:
    base = state["nav"].get("controller_base")
    if base:
        return base
    # fallback from env
    return os.getenv("FC_BASE", "http://192.168.4.1")

@app.post("/api/nav/config")
async def nav_config(controller_base: str):
    state["nav"]["controller_base"] = controller_base
    return {"ok": True, "controller_base": controller_base}

@app.post("/api/nav/arm")
async def nav_arm():
    try:
        requests.get(f"{_controller_base()}/api/arm", timeout=1)
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": str(e)}

@app.post("/api/nav/disarm")
async def nav_disarm():
    try:
        requests.get(f"{_controller_base()}/api/disarm", timeout=1)
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": str(e)}

@app.post("/api/nav/rc")
async def nav_rc(T: int | None = None, R: float | None = None, P: float | None = None, Y: float | None = None):
    try:
        params = {}
        if T is not None: params["T"] = T
        if R is not None: params["R"] = R
        if P is not None: params["P"] = P
        if Y is not None: params["Y"] = Y
        requests.get(f"{_controller_base()}/api/rc", params=params, timeout=1)
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": str(e)}

@app.post("/api/nav/goal")
async def nav_goal(g: NavGoal):
    state["nav"]["goal"] = g.model_dump()
    await manager.broadcast({"type": "nav_goal", "data": state["nav"]["goal"]})
    return {"ok": True}
