from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
from fastapi.responses import RedirectResponse, StreamingResponse
import requests
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
}

@app.get("/health")
async def health():
    return {"status": "ok"}

@app.get("/")
async def root_redirect():
    return RedirectResponse(url="/dashboard")

@app.get("/video_proxy")
def video_proxy(url: str = "http://192.168.43.36:81/stream"):
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
