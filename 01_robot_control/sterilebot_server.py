
"""
SterileBot HTTP 서버 v2 — tube_transport.py 최신 버전 기반
엔드포인트:
  /status
  /home
  /pickup_move/<tube_num>        집기 위치 이동 (grip 이벤트 직전 대기)
  /pickup_grip                   그리퍼 닫기 + 저장된 복귀 경로 재생
  /drop/<slot>                   꽂기 (A1~B4)
  /pour/<slot>                   붓기 전체 시퀀스 (옆면집기→붓기→꽂기)
  /stir                          섞기 전체 시퀀스
  /reset                         리셋 (action_log 역순 복구)
  /grip/open
  /grip/close
  /play_alarm                    대피 안내 음성 재생
  /stop_alarm                    대피 안내 음성 중단
"""
from http.server import HTTPServer, BaseHTTPRequestHandler
import json, threading, os, time, subprocess

from doors.door_server import start as door_start   # 문 제어
from doors.door_server import servo             # 서보 직접 제어
from service.uno_socket import SocketServer	    # 아두이노 연결

from tube_transport import (
    load_config, replay_trajectory, go_home, go_home_lift,
    gripper_open, gripper_close,
    downsample, send_safe, TIME_SCALE, REPLAY_SAMP,
    run_pour_full, run_reset, run_lift,
    log_action, load_log, clear_log,
    mc, WAIT_STOP, WAIT_GRIP, GRIP_VALUE, SPEED_GRIP
)
import socket as _sock

WPF_IP = "192.168.0.25"
WPF_PORT = 9005

def notify_wpf(msg):
    """WPF(9005)로 신호 전송"""
    try:
        with _sock.socket(_sock.AF_INET, _sock.SOCK_STREAM) as s:
            s.settimeout(2)
            s.connect((WPF_IP, WPF_PORT))
            s.sendall((msg + "\n").encode())
        print(f"[WPF 전달] {msg}")
    except Exception as e:
        print(f"[WPF 전달 실패] {e}")
# ── 음성 파일 경로 ──
ALARM_WAV = "/home/er/SterileBot/alarm.wav"

# ── 대피 안내음성 재생 상태 ──
_alarm = {"proc": None, "thread": None, "running": False}

# ── JSON 로드 ──
def _load(path):
    try:
        return json.load(open(path)) if os.path.exists(path) else {}
    except:
        return {}

load_config()
load_log()
_pickups     = _load("tube_pickups.json")
_drops       = _load("tube_drops.json")
_lifts       = _load("lift_pickups.json")
_side_drops  = _load("side_drops.json")
_pour        = _load("pour_trajectory.json")
_stir_pick   = _load("stir_pickup.json")
_stir_action = _load("stir_action.json")
_stir_drop   = _load("stir_drop.json")
_reset_picks = _load("reset_pickups.json")
_reset_drops = _load("reset_drops.json")

print(f"[로드] 집기: {list(_pickups.keys())}")
print(f"[로드] 꽂기: {list(_drops.keys())}")
print(f"[로드] 붓기 슬롯: {list(_lifts.keys())}")

_busy = False
_stop_flag = False
_grip_state = {"tube_num": None, "remaining_traj": []}
_drop_state = {"slot": None, "remaining_traj": []}

def run_async(fn, *args):
    global _busy, _stop_flag
    _busy = True
    _stop_flag = False
    try:    fn(*args)
    except Exception as e: print(f"[ERROR] {e}")
    finally: _busy = False

def _pickup_move(tube_num):
    key  = f"tube_{tube_num}"
    data = _pickups.get(key)
    if not data:
        print(f"[ERROR] {key} 없음"); return

    raw    = data["trajectory"]
    evts   = data["events"]
    samp   = data.get("replay_samp", REPLAY_SAMP)
    t_sc   = data.get("time_scale",  TIME_SCALE)
    g_val  = data.get("grip_value",  GRIP_VALUE)
    traj   = downsample(raw, samp)

    grip_t = next((ev["t"] for ev in evts if ev["action"] == "close"), None)

    mc.stop(); time.sleep(WAIT_STOP)
    gripper_open()

    first = traj[0]["angles"]
    mc.send_angle(6, first[5], 15); time.sleep(2.5)
    send_safe(first, 15);           time.sleep(4.0)

    last_t = 0; remaining = []
    for pt in traj:
        if _stop_flag: break
        cur_t = pt["t"]
        if grip_t and cur_t >= grip_t - 0.2:
            remaining.append(pt)
            continue
        wait = max(0.08, (cur_t - last_t) * t_sc)
        last_t = cur_t
        send_safe(pt["angles"]); time.sleep(wait)

    _grip_state["tube_num"]       = tube_num
    _grip_state["remaining_traj"] = remaining
    _grip_state["grip_value"]     = g_val
    print(f"[서버] tube_{tube_num} 위치 도달 (잔여 {len(remaining)}프레임)")

def _pickup_grip():
    g_val     = _grip_state.get("grip_value", GRIP_VALUE)
    remaining = _grip_state["remaining_traj"]
    print(f"[DEBUG] pickup_grip 호출 — remaining={len(remaining)}프레임")
    mc.set_gripper_value(g_val, SPEED_GRIP); time.sleep(WAIT_GRIP)

    if remaining:
        print(f"[서버] 복귀 경로 재생 ({len(remaining)}프레임)")
        last_t = remaining[0]["t"]
        for pt in remaining:
            if _stop_flag: break
            cur_t = pt["t"]
            wait  = max(0.08, (cur_t - last_t) * TIME_SCALE)
            last_t = cur_t
            send_safe(pt["angles"]); time.sleep(wait)
    else:
        go_home()

    log_action("pickup", tube=f"tube_{_grip_state['tube_num']}")
    _grip_state["tube_num"]       = None
    _grip_state["remaining_traj"] = []
    print("[서버] 잡기 + 복귀 완료")
    servo.close_for_robot()        # 로봇 빠져나온 뒤 내부문 닫기

def _pickup_grip_lift():
    g_val     = _grip_state.get("grip_value", GRIP_VALUE)
    remaining = _grip_state["remaining_traj"]
    mc.set_gripper_value(g_val, SPEED_GRIP); time.sleep(WAIT_GRIP)
    if remaining:
        print(f"[서버] 복귀 경로 재생 ({len(remaining)}프레임)")
        last_t = remaining[0]["t"]
        for pt in remaining:
            if _stop_flag: break
            cur_t = pt["t"]
            wait  = max(0.08, (cur_t - last_t) * TIME_SCALE)
            last_t = cur_t
            send_safe(pt["angles"]); time.sleep(wait)
    go_home_lift()   # 수평 홈으로 복귀
    log_action("pickup", tube=f"tube_{_grip_state['tube_num']}")
    _grip_state["tube_num"]       = None
    _grip_state["remaining_traj"] = []
    print("[서버] 잡기 + 수평복귀 완료")

def _drop_move(slot):
    data = _drops.get(slot)
    if not data:
        print(f"[ERROR] {slot} 꽂기 데이터 없음"); return

    raw  = data["trajectory"]
    evts = data["events"]
    traj = downsample(raw, REPLAY_SAMP)

    open_t = next((ev["t"] for ev in evts if ev["action"] == "open"), None)

    mc.stop(); time.sleep(WAIT_STOP)

    send_safe(traj[0]["angles"], 15); time.sleep(4.0)

    last_t = 0; remaining = []
    for pt in traj:
        if _stop_flag: break
        cur_t = pt["t"]
        if open_t and cur_t >= open_t - 0.2:
            remaining.append(pt)
            continue
        wait = max(0.08, (cur_t - last_t) * TIME_SCALE)
        last_t = cur_t
        send_safe(pt["angles"]); time.sleep(wait)

    _drop_state["slot"]           = slot
    _drop_state["remaining_traj"] = remaining
    print(f"[서버] {slot} 꽂기 위치 도달 — RELEASE 대기")

def _drop_release():
    gripper_open()
    time.sleep(0.5)
    remaining = _drop_state["remaining_traj"]
    if remaining:
        last_t = remaining[0]["t"]
        for pt in remaining:
            if _stop_flag: break
            cur_t = pt["t"]
            wait  = max(0.08, (cur_t - last_t) * TIME_SCALE)
            last_t = cur_t
            send_safe(pt["angles"]); time.sleep(wait)
    go_home()
    log_action("drop", slot=_drop_state["slot"])
    _drop_state["slot"]           = None
    _drop_state["remaining_traj"] = []
    print("[서버] 꽂기 + 복귀 완료")

_pour_remaining = []

def _beaker_move():
    if not _pour:
        print("[ERROR] 붓기 데이터 없음"); return
    traj = _pour.get("trajectory", _pour) if isinstance(_pour, dict) else _pour
    SPLIT_T = 7.5

    send_safe(traj[0]["angles"], 15); time.sleep(4.0)
    last_t = 0
    global _pour_remaining
    _pour_remaining = []
    for pt in traj:
        if _stop_flag: break
        cur_t = pt["t"]
        if cur_t >= SPLIT_T:
            _pour_remaining.append(pt)
            continue
        wait = max(0.08, (cur_t - last_t) * TIME_SCALE)
        last_t = cur_t
        send_safe(pt["angles"]); time.sleep(wait)
    print(f"[서버] 비커 위치 도달 — POUR 대기 ({len(_pour_remaining)}프레임 남음)")

def _beaker_pour():
    global _pour_remaining
    if not _pour_remaining:
        print("[ERROR] 비커 이동 먼저 필요"); return
    last_t = _pour_remaining[0]["t"]
    for pt in _pour_remaining:
        if _stop_flag: break
        wait = max(0.08, (pt["t"] - last_t) * TIME_SCALE)
        last_t = pt["t"]
        send_safe(pt["angles"]); time.sleep(wait)
    _pour_remaining = []
    go_home_lift()
    print("[서버] 붓기 완료")

_side_drop_state = {"slot": None, "remaining_traj": []}

def _pickup_lift_move(slot):
    data = _lifts.get(slot)
    if not data: print(f"[ERROR] {slot} lift 없음"); return
    if isinstance(data, dict) and "trajectory" in data:
        traj = downsample(data["trajectory"], REPLAY_SAMP)
        evts = data.get("events", [])
    else:
        traj = downsample(data, REPLAY_SAMP); evts = []
    grip_t = next((ev["t"] for ev in evts if ev["action"] == "close"), None)
    mc.stop(); time.sleep(WAIT_STOP)
    gripper_open()
    send_safe(traj[0]["angles"], 15); time.sleep(4.0)
    last_t = 0; remaining = []
    for pt in traj:
        if _stop_flag: break
        cur_t = pt["t"]
        if grip_t and cur_t >= grip_t - 0.2:
            remaining.append(pt); continue
        wait = max(0.08, (cur_t - last_t) * TIME_SCALE)
        last_t = cur_t
        send_safe(pt["angles"]); time.sleep(wait)
    _grip_state["tube_num"] = slot
    _grip_state["remaining_traj"] = remaining
    print(f"[서버] {slot} 수평 집기 위치 도달 — GRAB 대기")

def _side_drop_move(slot):
    data = _side_drops.get(slot)
    if not data: print(f"[ERROR] {slot} side_drop 없음"); return
    if isinstance(data, dict) and "trajectory" in data:
        traj = downsample(data["trajectory"], REPLAY_SAMP)
        evts = data.get("events", [])
    else:
        traj = downsample(data, REPLAY_SAMP); evts = []
    open_t = next((ev["t"] for ev in evts if ev["action"] == "open"), None)
    mc.stop(); time.sleep(WAIT_STOP)
    send_safe(traj[0]["angles"], 15); time.sleep(4.0)
    last_t = 0; remaining = []
    for pt in traj:
        if _stop_flag: break
        cur_t = pt["t"]
        if open_t and cur_t >= open_t - 0.2:
            remaining.append(pt); continue
        wait = max(0.08, (cur_t - last_t) * TIME_SCALE)
        last_t = cur_t
        send_safe(pt["angles"]); time.sleep(wait)
    _side_drop_state["slot"] = slot
    _side_drop_state["remaining_traj"] = remaining
    print(f"[서버] {slot} 수평 꽂기 위치 도달 — RELEASE 대기")

def _side_drop_release():
    gripper_open(); time.sleep(0.5)
    remaining = _side_drop_state["remaining_traj"]
    if remaining:
        last_t = remaining[0]["t"]
        for pt in remaining:
            if _stop_flag: break
            wait = max(0.08, (pt["t"] - last_t) * TIME_SCALE)
            last_t = pt["t"]
            send_safe(pt["angles"]); time.sleep(wait)
    go_home()
    _side_drop_state["slot"] = None
    _side_drop_state["remaining_traj"] = []
    print("[서버] 수평 꽂기 완료")

def _run_stir():
    if not _stir_pick or not _stir_action or not _stir_drop:
        print("[ERROR] 섞기 데이터 없음"); return
    replay_trajectory(_stir_pick,   "막대기 집기")
    replay_trajectory(_stir_action, "섞기 동작")
    replay_trajectory(_stir_drop,   "막대기 내려놓기")
    go_home()
    print("[서버] 섞기 완료")

_stir_grip_state   = {"remaining_traj": [], "grip_value": GRIP_VALUE}
_stir_drop_state   = {"remaining_traj": []}
_stir_beaker_state = {"remaining_traj": []}

def _stir_move():
    if not _stir_pick:
        print("[ERROR] stir_pickup.json 없음"); return
    raw    = _stir_pick["trajectory"]
    evts   = _stir_pick.get("events", [])
    g_val  = _stir_pick.get("grip_value", GRIP_VALUE)
    traj   = downsample(raw, REPLAY_SAMP)
    grip_t = next((ev["t"] for ev in evts if ev["action"] == "close"), None)

    mc.stop(); time.sleep(WAIT_STOP)
    gripper_open()

    first = traj[0]["angles"]
    mc.send_angle(6, first[5], 15); time.sleep(2.5)
    send_safe(first, 15);           time.sleep(4.0)

    last_t = 0; remaining = []
    for pt in traj:
        if _stop_flag: break
        cur_t = pt["t"]
        if grip_t and cur_t >= grip_t - 0.2:
            remaining.append(pt); continue
        wait = max(0.08, (cur_t - last_t) * TIME_SCALE)
        last_t = cur_t
        send_safe(pt["angles"]); time.sleep(wait)

    _stir_grip_state["remaining_traj"] = remaining
    _stir_grip_state["grip_value"]     = g_val
    print(f"[서버] 막대 위치 도달 — GRAB 대기 (잔여 {len(remaining)}프레임)")

def _stir_grip():
    g_val     = _stir_grip_state.get("grip_value", GRIP_VALUE)
    remaining = _stir_grip_state["remaining_traj"]

    mc.set_gripper_value(g_val, SPEED_GRIP); time.sleep(WAIT_GRIP)

    if remaining:
        last_t = remaining[0]["t"]
        for pt in remaining:
            if _stop_flag: break
            wait = max(0.08, (pt["t"] - last_t) * TIME_SCALE)
            last_t = pt["t"]
            send_safe(pt["angles"]); time.sleep(wait)
    else:
        go_home()

    _stir_grip_state["remaining_traj"] = []
    print("[서버] 막대 잡기 + 홈 복귀 완료")

def _stir_beaker_move():
    if not _stir_action:
        print("[ERROR] stir_action.json 없음"); return
    raw  = _stir_action["trajectory"] if isinstance(_stir_action, dict) and "trajectory" in _stir_action else _stir_action
    SPLIT_T = 5.4

    before    = [pt for pt in raw if pt["t"] < SPLIT_T]
    remaining = [pt for pt in raw if pt["t"] >= SPLIT_T]

    before_ds    = downsample(before,    REPLAY_SAMP)
    remaining_ds = downsample(remaining, REPLAY_SAMP)

    if not before_ds:
        print("[ERROR] 이동 구간 없음"); return

    send_safe(before_ds[0]["angles"], 15); time.sleep(4.0)
    last_t = 0
    for pt in before_ds:
        if _stop_flag: break
        wait = max(0.08, (pt["t"] - last_t) * TIME_SCALE)
        last_t = pt["t"]
        send_safe(pt["angles"]); time.sleep(wait)

    if _stop_flag:
        print("[서버] 긴급 정지 — 비커 이동 중단")
        return
    _stir_beaker_state["remaining_traj"] = remaining_ds
    print(f"[서버] 비커 위 도달 — SHAKE 대기 (잔여 {len(remaining_ds)}프레임)")

def _stir_do():
    remaining = _stir_beaker_state.get("remaining_traj", [])
    if remaining:
        last_t = remaining[0]["t"]
        for pt in remaining:
            if _stop_flag: break
            wait = max(0.08, (pt["t"] - last_t) * TIME_SCALE)
            last_t = pt["t"]
            send_safe(pt["angles"]); time.sleep(wait)
    go_home_lift()
    _stir_beaker_state["remaining_traj"] = []
    print("[서버] 섞기 완료 + 홈 복귀")

def _stir_drop_move():
    if not _stir_drop:
        print("[ERROR] stir_drop.json 없음"); return
    raw    = _stir_drop["trajectory"] if isinstance(_stir_drop, dict) and "trajectory" in _stir_drop else _stir_drop
    evts   = _stir_drop.get("events", []) if isinstance(_stir_drop, dict) else []
    traj   = downsample(raw, REPLAY_SAMP)
    open_t = next((ev["t"] for ev in evts if ev["action"] == "open"), None)

    send_safe(traj[0]["angles"], 15); time.sleep(4.0)

    last_t = 0; remaining = []
    for pt in traj:
        if _stop_flag: break
        cur_t = pt["t"]
        if open_t and cur_t >= open_t - 0.2:
            remaining.append(pt); continue
        wait = max(0.08, (cur_t - last_t) * TIME_SCALE)
        last_t = cur_t
        send_safe(pt["angles"]); time.sleep(wait)

    _stir_drop_state["remaining_traj"] = remaining
    print(f"[서버] 막대 원위치 도달 — RELEASE 대기 (잔여 {len(remaining)}프레임)")

def _stir_drop_release():
    gripper_open(); time.sleep(0.5)
    remaining = _stir_drop_state["remaining_traj"]
    if remaining:
        last_t = remaining[0]["t"]
        for pt in remaining:
            if _stop_flag: break
            wait = max(0.08, (pt["t"] - last_t) * TIME_SCALE)
            last_t = pt["t"]
            send_safe(pt["angles"]); time.sleep(wait)
    go_home()
    _stir_drop_state["remaining_traj"] = []
    print("[서버] 막대 내려놓기 + 홈 복귀 완료")

def _run_reset_auto():
    import tube_transport as _tt
    if not _tt.action_log:
        print("[리셋] 기록 없음"); return

    # ★ 리셋 시작 — 내부문 열기
    servo.open_for_robot()

    steps = list(reversed(_tt.action_log))
    i = 0
    while i < len(steps):
        step = steps[i]
        if step["action"] == "drop":
            slot = step["slot"]
            tube = None
            if i + 1 < len(steps) and steps[i+1]["action"] == "pickup":
                tube = steps[i+1]["tube"]
            else:
                i += 1; continue
            if slot not in _reset_picks:
                print(f"[리셋] {slot} 집기 데이터 없음"); i += 2; continue
            replay_trajectory(_reset_picks[slot], f"{slot} 리셋집기")
            go_home()
            if tube not in _reset_drops:
                print(f"[리셋] {tube} 꽂기 데이터 없음"); i += 2; continue
            replay_trajectory(_reset_drops[tube], f"{tube} 리셋꽂기")
            go_home()
            i += 2
        else:
            i += 1
    _tt.clear_log()

    # ★ 리셋 완료 — 내부문 닫기
    servo.close_for_robot()
    print("[리셋] 완료")

# ── HTTP 응답 헬퍼 ──
def respond(h, data, code=200):
    body = json.dumps(data).encode()
    h.send_response(code)
    h.send_header("Content-Type", "application/json")
    h.end_headers()
    h.wfile.write(body)

# ── 핸들러 ──
class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        self.do_GET()

    def log_message(self, fmt, *args):
        print(f"[HTTP] {args[0]}")

    def do_GET(self):
        p = self.path

        if p == "/status":
            respond(self, {
                "busy":       _busy,
                "positioned": _grip_state["tube_num"] is not None,
                "tube":       _grip_state["tube_num"],
                "pickups":    list(_pickups.keys()),
                "drops":      list(_drops.keys()),
                "lifts":      list(_lifts.keys()),
            })

        elif p == "/home":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            threading.Thread(target=run_async, args=(go_home,), daemon=True).start()
            respond(self, {"ok": True, "action": "홈"})
        elif p == "/home_lift":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            threading.Thread(target=run_async, args=(go_home_lift,), daemon=True).start()
            respond(self, {"ok": True, "action": "수평 홈 복귀"})


        elif p == "/stop":
            def _do_stop():
                global _busy, _stop_flag
                _stop_flag = True
                mc.stop()
                _busy = False
                print("[서버] 긴급 정지 실행")
            _do_stop()
            respond(self, {"ok": True, "action": "정지"})

        elif p.startswith("/pickup_lift_move/"):
            slot = p.split("/")[-1].upper()
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if slot not in _lifts: respond(self, {"ok": False, "reason": f"{slot} lift 없음"}); return
            threading.Thread(target=run_async, args=(_pickup_lift_move, slot), daemon=True).start()
            respond(self, {"ok": True, "action": f"{slot} 수평 이동"})

        elif p.startswith("/side_drop_move/"):
            slot = p.split("/")[-1].upper()
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if slot not in _side_drops: respond(self, {"ok": False, "reason": f"{slot} 없음"}); return
            threading.Thread(target=run_async, args=(_side_drop_move, slot), daemon=True).start()
            respond(self, {"ok": True, "action": f"{slot} 수평 꽂기 이동"})

        elif p == "/side_drop_release":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if _side_drop_state["slot"] is None: respond(self, {"ok": False, "reason": "위치 미도달"}); return
            threading.Thread(target=run_async, args=(_side_drop_release,), daemon=True).start()
            respond(self, {"ok": True, "action": "수평 놓기"})

        elif p.startswith("/pickup_lift/"):
            slot = p.split("/")[-1].upper()
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if slot not in _lifts:
                respond(self, {"ok": False, "reason": f"{slot} lift 데이터 없음"}); return
            def _do_lift():
                replay_trajectory(_lifts[slot], f"{slot} 수평 집기")
                go_home_lift()
            threading.Thread(target=run_async, args=(_do_lift,), daemon=True).start()
            respond(self, {"ok": True, "action": f"{slot} 수평 집기"})

        elif p.startswith("/pickup_move/"):
            tube = int(p.split("/")[-1])
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if f"tube_{tube}" not in _pickups:
                respond(self, {"ok": False, "reason": f"tube_{tube} 없음"}); return

            # 잠금 안 돼있으면 거부 + WPF 안내
            if not servo.is_locked():
                notify_wpf("DOOR_UNLOCKED")
                respond(self, {"ok": False, "reason": "DOOR_UNLOCKED", "msg": "먼저 외부문을 잠그세요"})
                return
            # 잠김 → 내부문 열고 집기 이동
            threading.Thread(target=servo.open_for_robot, daemon=True).start()

            threading.Thread(target=run_async, args=(_pickup_move, tube), daemon=True).start()
            respond(self, {"ok": True, "action": f"tube_{tube} 이동"})

        elif p == "/pickup_grip":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if _grip_state["tube_num"] is None:
                respond(self, {"ok": False, "reason": "위치 미도달"}); return
            threading.Thread(target=run_async, args=(_pickup_grip,), daemon=True).start()
            respond(self, {"ok": True, "action": "잡기+복귀"})

        elif p == "/pickup_grip_lift":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if _grip_state["tube_num"] is None:
                respond(self, {"ok": False, "reason": "위치 미도달"}); return
            threading.Thread(target=run_async, args=(_pickup_grip_lift,), daemon=True).start()
            respond(self, {"ok": True, "action": "잡기+수평복귀"})

        elif p.startswith("/side_drop/"):
            slot = p.split("/")[-1].upper()
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if slot not in _side_drops:
                respond(self, {"ok": False, "reason": f"{slot} 옆면 꽂기 없음"}); return
            threading.Thread(target=run_async,
                             args=(replay_trajectory, _side_drops[slot], f"{slot} 옆면 꽂기"),
                             daemon=True).start()
            respond(self, {"ok": True, "action": f"{slot} 옆면 꽂기"})

        elif p.startswith("/drop_move/"):
            slot = p.split("/")[-1].upper()
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if slot not in _drops:
                respond(self, {"ok": False, "reason": f"{slot} 없음"}); return
            threading.Thread(target=run_async, args=(_drop_move, slot), daemon=True).start()
            respond(self, {"ok": True, "action": f"{slot} 꽂기 이동"})

        elif p == "/drop_release":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if _drop_state["slot"] is None:
                respond(self, {"ok": False, "reason": "꽂기 위치 미도달"}); return
            threading.Thread(target=run_async, args=(_drop_release,), daemon=True).start()
            respond(self, {"ok": True, "action": "놓기 + 복귀"})

        elif p.startswith("/drop/"):
            slot = p.split("/")[-1].upper()
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if slot not in _drops:
                respond(self, {"ok": False, "reason": f"{slot} 없음"}); return
            threading.Thread(target=run_async,
                             args=(replay_trajectory, _drops[slot], f"{slot} 꽂기"),
                             daemon=True).start()
            respond(self, {"ok": True, "action": f"{slot} 꽂기"})

        elif p == "/beaker_move":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            threading.Thread(target=run_async, args=(_beaker_move,), daemon=True).start()
            respond(self, {"ok": True, "action": "비커 이동"})

        elif p == "/beaker_pour":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if not _pour_remaining:
                respond(self, {"ok": False, "reason": "비커 이동 먼저"}); return
            threading.Thread(target=run_async, args=(_beaker_pour,), daemon=True).start()
            respond(self, {"ok": True, "action": "붓기"})

        elif p == "/pour_only":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            if not _pour:
                respond(self, {"ok": False, "reason": "붓기 데이터 없음"}); return
            def _do_pour_only():
                replay_trajectory(_pour, "비커 붓기")
                import time; mc.stop(); time.sleep(1.0)
                go_home_lift()
            threading.Thread(target=run_async, args=(_do_pour_only,), daemon=True).start()
            respond(self, {"ok": True, "action": "비커 붓기"})

        elif p.startswith("/pour/"):
            slot = p.split("/")[-1].upper()
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            threading.Thread(target=run_async,
                             args=(run_pour_full, _lifts, _side_drops, _pour, slot),
                             daemon=True).start()
            respond(self, {"ok": True, "action": f"{slot} 붓기"})

        elif p == "/stir":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            threading.Thread(target=run_async, args=(_run_stir,), daemon=True).start()
            respond(self, {"ok": True, "action": "섞기"})

        elif p == "/stir_move":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            def _stir_move_and_release():
                global _stop_flag, _busy
                _busy = True
                _stop_flag = False
                _stir_move()
                _busy = False
            threading.Thread(target=_stir_move_and_release, daemon=True).start()
            respond(self, {"ok": True, "action": "막대 이동"})

        elif p == "/stir_grip":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            threading.Thread(target=run_async, args=(_stir_grip,), daemon=True).start()
            respond(self, {"ok": True, "action": "막대 잡기+복귀"})

        elif p == "/stir_beaker_move":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            def _stir_beaker_move_and_release():
                global _stop_flag, _busy
                _busy = True
                _stop_flag = False
                _stir_beaker_move()
                _busy = False
            threading.Thread(target=_stir_beaker_move_and_release, daemon=True).start()
            respond(self, {"ok": True, "action": "비커 위치 이동"})

        elif p == "/stir_do":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            threading.Thread(target=run_async, args=(_stir_do,), daemon=True).start()
            respond(self, {"ok": True, "action": "섞기 동작"})

        elif p == "/stir_drop_move":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            def _stir_drop_move_and_release():
                global _stop_flag, _busy
                _busy = True
                _stop_flag = False
                _stir_drop_move()
                _busy = False
            threading.Thread(target=_stir_drop_move_and_release, daemon=True).start()
            respond(self, {"ok": True, "action": "막대 원위치 이동"})

        elif p == "/stir_drop_release":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            threading.Thread(target=run_async, args=(_stir_drop_release,), daemon=True).start()
            respond(self, {"ok": True, "action": "막대 놓기+복귀"})

        elif p == "/state":
            import tube_transport as _tt
            log = _tt.action_log
            tubes = {f"tube_{i}": f"bottle_{i}" for i in range(1, 5)}
            for entry in log:
                act = entry.get("action")
                if act == "pickup":
                    tube = entry.get("tube")
                    if tube:
                        tubes[tube] = "HELD"
                elif act == "drop":
                    slot = entry.get("slot")
                    for k, v in tubes.items():
                        if v == "HELD":
                            tubes[k] = slot
                            break
            respond(self, {
                "tubes":   tubes,
                "holding": str(_grip_state["tube_num"]) if _grip_state["tube_num"] else None,
                "busy":    _busy,
            })

        elif p == "/reset":
            if _busy: respond(self, {"ok": False, "reason": "동작 중"}); return
            
            threading.Thread(target=run_async,
                             args=(_run_reset_auto,),
                             daemon=True).start()
            respond(self, {"ok": True, "action": "리셋"})

        elif p == "/clear_log":
            import tube_transport as _tt
            _tt.clear_log()
            respond(self, {"ok": True, "action": "로그 초기화"})

        elif p == "/grip/open":
            gripper_open()
            respond(self, {"ok": True})

        elif p == "/grip/close":
            gripper_close()
            respond(self, {"ok": True})

        # ★ 대피 안내 음성 무한반복 재생
        elif p == "/play_alarm":
            try:
                if not _alarm["running"]:
                    _alarm["running"] = True
                    def _loop():
                        while _alarm["running"]:
                            _alarm["proc"] = subprocess.Popen(
                                ["paplay", "--volume=65536", ALARM_WAV],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL
                            )
                            _alarm["proc"].wait()
                    import threading as _th
                    _alarm["thread"] = _th.Thread(target=_loop, daemon=True)
                    _alarm["thread"].start()
                print("[서버] 대피 안내 음성 무한반복 재생 시작")
                respond(self, {"ok": True, "action": "alarm_start"})
            except Exception as e:
                print(f"[서버] 음성 재생 실패: {e}")
                respond(self, {"ok": False, "reason": str(e)}, 500)

        # ★ 대피 안내 음성 중단
        elif p == "/stop_alarm":
            try:
                _alarm["running"] = False
                if _alarm["proc"]:
                    _alarm["proc"].terminate()
                    _alarm["proc"] = None
                subprocess.run(["pkill", "-f", "paplay"], capture_output=True)
                print("[서버] 대피 안내 음성 중단")
                respond(self, {"ok": True, "action": "alarm_stop"})
            except Exception as e:
                respond(self, {"ok": False, "reason": str(e)}, 500)

        # ★ 인터컴 수신 중 대피음성 볼륨 낮춤
        elif p == "/duck_alarm":
            try:
                # paplay sink-input 찾아서 볼륨 50%로 낮추기
                result = subprocess.run(
                    ["pactl", "list", "sink-inputs"],
                    capture_output=True, text=True
                )
                idx = None
                for line in result.stdout.splitlines():
                    if "Sink Input #" in line:
                        idx = line.split("#")[1].strip()
                    if idx and "paplay" in line:
                        break
                if idx:
                    subprocess.run(["pactl", "set-sink-input-volume", idx, "50%"],
                                   capture_output=True)
                print("[서버] 대피음성 볼륨 50% (인터컴 수신 중)")
                respond(self, {"ok": True, "action": "duck"})
            except Exception as e:
                respond(self, {"ok": False, "reason": str(e)}, 500)

        # ★ 인터컴 종료 후 대피음성 볼륨 복원
        elif p == "/unduck_alarm":
            try:
                result = subprocess.run(
                    ["pactl", "list", "sink-inputs"],
                    capture_output=True, text=True
                )
                idx = None
                for line in result.stdout.splitlines():
                    if "Sink Input #" in line:
                        idx = line.split("#")[1].strip()
                    if idx and "paplay" in line:
                        break
                if idx:
                    subprocess.run(["pactl", "set-sink-input-volume", idx, "100%"],
                                   capture_output=True)
                print("[서버] 대피음성 볼륨 100% 복원")
                respond(self, {"ok": True, "action": "unduck"})
            except Exception as e:
                respond(self, {"ok": False, "reason": str(e)}, 500)

        else:
            respond(self, {"ok": False, "reason": "unknown"}, 404)


if __name__ == "__main__":
    import socketserver, subprocess, sys

    # lab_intercom.py 자동 실행
    intercom_proc = subprocess.Popen(
        [sys.executable, "/home/er/SterileBot/lab_intercom.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    print("[인터컴] lab_intercom.py 시작됨")

    # 서보 서버 (포트 9003) — 별도 스레드
    threading.Thread(target=door_start, daemon=True).start()  # ◀ 추가

    # 비상 시스템 (포트 9002) — 별도 스레드
    eme_server = SocketServer()                                # ◀ 추가
    threading.Thread(target=eme_server.start, daemon=True).start()  # ◀ 추가

    HTTPServer.allow_reuse_address = True
    server = HTTPServer(("0.0.0.0", 5001), Handler)
    print("=== SterileBot 서버 v2 (포트 5001) ===")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[종료]")
        intercom_proc.terminate()
        server.server_close()
