"""Web GUI server for AI module — serves frontend and provides SSE event stream."""

import json
import time
import threading
from queue import Queue, Empty
from pathlib import Path

from flask import Flask, Response, jsonify, send_from_directory

from config import FRONTEND_DIR

app = Flask(__name__, static_folder=None)

# Shared state updated by main demo
state = {
    "status": "INITIALIZING",
    "mic_level": 0,
    "mic_peak": 0,
    "mic_working": False,
    "kws_state": "IDLE",
    "wake_count": 0,
    "last_wake_time": "",
    "asr_partial": "",
    "asr_final": "",
    "last_command": "",
    "last_keyword": "",
    "tts_status": "idle",
    "tts_last_text": "",
    "yolo_running": False,
    "yolo_fps": 0,
    "yolo_detections": [],
    "yolo_detection_count": 0,
    "traffic_light_state": None,
    "log": [],
}

_event_queue = Queue(maxsize=200)


def push_event(event_type, data=None):
    evt = {"type": event_type, "time": time.strftime("%H:%M:%S"), "data": data or {}}
    _event_queue.put(evt, block=False)
    state["log"].append(evt)
    if len(state["log"]) > 100:
        state["log"] = state["log"][-100:]


@app.route("/")
def index():
    return send_from_directory(str(FRONTEND_DIR), "index.html")


@app.route("/<path:path>")
def static_file(path):
    return send_from_directory(str(FRONTEND_DIR), path)


@app.route("/api/state")
def get_state():
    return jsonify(state)


@app.route("/api/voices")
def list_voices():
    try:
        voices_path = FRONTEND_DIR / "voices.json"
        with open(str(voices_path), "r", encoding="utf-8") as f:
            return jsonify(json.load(f))
    except Exception:
        return jsonify([])


@app.route("/api/set_voice", methods=["POST"])
def set_voice():
    from flask import request
    sid = request.json.get("speaker_id", 45)
    state["tts_speaker_id"] = sid
    push_event("tts", {"log_type": "tts", "message": f"Voice changed to speaker_id={sid}"})
    return jsonify({"ok": True, "speaker_id": sid})


@app.route("/api/videos")
def list_videos():
    import glob
    video_dir = state.get("video_dir", "")
    if not video_dir:
        return jsonify([])
    files = sorted(glob.glob(f"{video_dir}/*.mp4") + glob.glob(f"{video_dir}/*.avi"))
    return jsonify([{"file": f, "name": Path(f).name} for f in files])


@app.route("/api/switch_video", methods=["POST"])
def switch_video():
    from flask import request
    path = request.json.get("path", "")
    state["switch_video"] = path
    state["switch_to_camera"] = False
    push_event("sys", {"log_type": "sys", "message": f"Switch video: {Path(path).name}"})
    return jsonify({"ok": True})


@app.route("/api/switch_camera", methods=["POST"])
def switch_camera():
    state["switch_to_camera"] = True
    state["switch_video"] = None
    push_event("sys", {"log_type": "sys", "message": "Switch to live camera + depth"})
    return jsonify({"ok": True})


@app.route("/api/events")
def sse_stream():
    def generate():
        while True:
            try:
                evt = _event_queue.get(timeout=1)
                yield f"data: {json.dumps(evt, ensure_ascii=False)}\n\n"
            except Empty:
                yield ": keepalive\n\n"
    return Response(generate(), mimetype="text/event-stream", headers={
        "Cache-Control": "no-cache",
        "X-Accel-Buffering": "no",
    })


def run_server(host="0.0.0.0", port=8080):
    app.run(host=host, port=port, threaded=True, use_reloader=False)
