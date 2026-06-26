"""
SterileBot 서버 — Flask 없이 내장 HTTP 서버 (가벼움)
"""
from http.server import HTTPServer, BaseHTTPRequestHandler
import json, threading, os, time
from tube_transport import (
    load_config, replay_trajectory, go_home,
    gripper_open, gripper_close,
    downsample, send_safe, TIME_SCALE, REPLAY_SAMP, mc
)

load_config()
_pickups, _drops = {}, {}
if os.path.exists("tube_pickups.json"):
    with open("tube_pickups.json") as f: _pickups = json.load(f)
if os.path.exists("tube_drops.json"):
    with open("tube_drops.json") as f: _drops = json.load(f)

print(f"[로드] 집기: {list(_pickups.keys())}")
print(f"[로드] 꽂기: {list(_drops.keys())}")

_busy = False
_grip_state = {"tube_num": None}

def run_async(fn, *args):
    global _busy
    _busy = True
    try: fn(*args)
    except Exception as e: print(f"[ERROR] {e}")
    finally: _busy = False

def _pickup_move(tube_num):
    key = f"tube_{tube_num}"
    data = _pickups.get(key)
    if not data:
        print(f"[ERROR] {key} 없음"); return
    raw = data["trajectory"]
    evts = data["events"]
    traj = downsample(raw, REPLAY_SAMP)
    grip_t = None
    for ev in evts:
        if ev["action"] == "close":
            grip_t = ev["t"]; break
    mc.stop(); time.sleep(2.0)
    gripper_open()
    send_safe(traj[0]["angles"], 15)
    time.sleep(7.0)
    last_t = 0
    for pt in traj:
        cur_t = pt["t"]
        if grip_t and cur_t >= grip_t - 0.2:
            _grip_state["tube_num"] = tube_num
            print(f"[서버] tube_{tube_num} 위치 도달")
            break
        wait = max(0.08, (cur_t - last_t) * TIME_SCALE)
        last_t = cur_t
        send_safe(pt["angles"])
        time.sleep(wait)

def _pickup_grip():
    gripper_close()
    time.sleep(1.0)
    go_home()
    _grip_state["tube_num"] = None
    print("[서버] 잡기 + 홈 복귀 완료")

def respond(h, data):
    h.send_response(200)
    h.send_header("Content-Type", "application/json")
    h.end_headers()
    h.wfile.write(json.dumps(data).encode())

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print(f"[HTTP] {args[0]}")
    def do_GET(self):
        p = self.path
        if p == "/status":
            respond(self, {"busy": _busy, "positioned": _grip_state["tube_num"] is not None, "tube": _grip_state["tube_num"]})
        elif p == "/home":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"})
            else:
                threading.Thread(target=run_async, args=(go_home,), daemon=True).start()
                respond(self, {"ok": True, "action": "홈"})
        elif p.startswith("/pickup_move/"):
            tube = int(p.split("/")[-1])
            if _busy: respond(self, {"ok": False, "reason": "동작 중"})
            else:
                threading.Thread(target=run_async, args=(_pickup_move, tube), daemon=True).start()
                respond(self, {"ok": True, "action": f"tube_{tube} 이동"})
        elif p == "/pickup_grip":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"})
            elif _grip_state["tube_num"] is None: respond(self, {"ok": False, "reason": "위치 미도달"})
            else:
                threading.Thread(target=run_async, args=(_pickup_grip,), daemon=True).start()
                respond(self, {"ok": True, "action": "잡기+홈"})
        elif p.startswith("/drop/"):
            slot = p.split("/")[-1].upper()
            if _busy: respond(self, {"ok": False, "reason": "동작 중"})
            elif slot not in _drops: respond(self, {"ok": False, "reason": f"{slot} 없음"})
            else:
                threading.Thread(target=run_async, args=(replay_trajectory, _drops[slot], f"{slot} 꽂기"), daemon=True).start()
                respond(self, {"ok": True, "action": f"{slot} 꽂기"})
        elif p == "/grip/open":
            gripper_open(); respond(self, {"ok": True})
        elif p == "/grip/close":
            gripper_close(); respond(self, {"ok": True})
        else:
            respond(self, {"ok": False, "reason": "unknown"})

if __name__ == "__main__":
    server = HTTPServer(("0.0.0.0", 5001), Handler)
    print("=== SterileBot 서버 (포트 5001) ===")
    try: server.serve_forever()
    except KeyboardInterrupt:
        print("\n[종료]"); server.server_close()