#!/usr/bin/env python3
"""
SterileBot - 시험관 집기/꽂기 통합 제어 v2

파일 구조:
  config.json        ← 영점 각도 (티칭 1회)
  tube_pickups.json  ← 집기 trajectory 4개 (tube_1~4)
  tube_drops.json    ← 꽂기 trajectory 8개 (A1~A4, B1~B4)

실행 흐름:
  사용자 선택 (tube_1, A2)
  → tube_1 집기 trajectory 재생
  → 영점으로 이동
  → A2 꽂기 trajectory 재생
"""

import time, json, os, threading
from pymycobot.mycobot import MyCobot

mc = MyCobot('/dev/ttyAMA0', 1000000)
time.sleep(1)
print("[준비] 연결 완료")

# ── 설정 ──
CONFIG_FILE       = "config.json"
PICKUP_FILE       = "tube_pickups.json"
DROP_FILE         = "tube_drops.json"
LIFT_FILE         = "lift_pickups.json"     # 시험관 거치대 옆면 집기 (붓기 전)
RESET_PICKUP_FILE = "reset_pickups.json"    # 리셋: A1~B4 수직 집기
RESET_DROP_FILE   = "reset_drops.json"      # 리셋: 시약 거치대 원래 위치 꽂기
POUR_FILE         = "pour_trajectory.json"  # 붓기 경로
SIDE_DROP_FILE    = "side_drops.json"       # 옆면 잡은 채로 슬롯에 꽂기
STIR_FILE         = "stir_pickup.json"      # 막대기 집기
STIR_ACTION_FILE  = "stir_action.json"      # 섞기 동작
STIR_DROP_FILE    = "stir_drop.json"        # 막대기 내려놓기

OPEN_VALUE      = 100
GRIP_VALUE      = 20    # 시험관 집기
GRIP_VALUE_STIR = 5     # 막대기 집기 (더 강하게)
SPEED_GRIP      = 15
RECORD_INT       = 0.1
STIR_RECORD_INT  = 0.03  # 섞기 녹화 간격 (0.03 → 0.06)
STIR_REPLAY_SAMP = 0.25  # 섞기 재생 간격 (0.1 → 0.25)
STIR_TIME_SCALE  = 0.3   # 섞기 재생 속도 (빠르게)
REPLAY_SAMP  = 0.8   # 포인트 간격 넓게 → 부드럽게
REPLAY_SPD   = 35
TIME_SCALE   = 0.7
NUM_TUBES    = 4

# ── 대기 시간 설정 (초) ──
WAIT_STOP      = 0.5   # mc.stop() 후 대기
WAIT_HOME      = 3.5   # 영점 이동 완료 대기
WAIT_FIRST_POS = 4.0   # 재생 시작 전 첫 위치 이동 대기
WAIT_GRIP      = 0.8   # 그리퍼 동작 완료 대기

ANGLE_LIMITS = [
    (-168, 168), (-135, 135), (-150, 150),
    (-145, 145), (-165, 165), (-180, 180),
]

# 영점 각도 (config.json에서 로드, 없으면 기본값)
HOME_ANGLES      = [0, 30, -90, 0, 60, 0]
HOME_ANGLES_LIFT = [0, 30, -90, 0, 60, 0]   # 붓기용 영점 (별도 티칭 필요)


# ──────────────────────────────────────────────
#  공통 함수
# ──────────────────────────────────────────────

def load_config():
    global HOME_ANGLES, HOME_ANGLES_LIFT
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as f:
            cfg = json.load(f)
        HOME_ANGLES      = cfg.get("home_angles",      HOME_ANGLES)
        HOME_ANGLES_LIFT = cfg.get("home_angles_lift", HOME_ANGLES_LIFT)
        print(f"[설정] 영점(이동): {[round(a,1) for a in HOME_ANGLES]}")
        print(f"[설정] 영점(붓기): {[round(a,1) for a in HOME_ANGLES_LIFT]}")
    else:
        print(f"[설정] 기본 영점 사용")

def save_config():
    with open(CONFIG_FILE, "w") as f:
        json.dump({
            "home_angles":      HOME_ANGLES,
            "home_angles_lift": HOME_ANGLES_LIFT
        }, f, indent=2)
    print(f"  💾 영점 저장 완료: {CONFIG_FILE}")

def get_angles():
    for _ in range(5):
        a = mc.get_angles()
        if a and len(a) == 6:
            return list(a)
        time.sleep(0.2)
    return None

def clamp(angles):
    return [max(ANGLE_LIMITS[i][0], min(ANGLE_LIMITS[i][1], float(a)))
            for i, a in enumerate(angles)]

def gripper_open():
    mc.set_gripper_value(OPEN_VALUE, SPEED_GRIP)
    time.sleep(WAIT_GRIP)

def gripper_close():
    mc.set_gripper_value(GRIP_VALUE, SPEED_GRIP)
    time.sleep(WAIT_GRIP)

def go_home(speed=20):
    """이동용 영점 (시약통 → 시험관 거치대)"""
    mc.stop()
    time.sleep(WAIT_STOP)
    mc.send_angles(clamp(HOME_ANGLES), speed)
    time.sleep(WAIT_HOME)

def go_home_lift(speed=20):
    """붓기용 영점 (시험관 거치대 → 비커, 시험관 수직 유지)"""
    mc.stop()
    time.sleep(WAIT_STOP)
    mc.send_angles(clamp(HOME_ANGLES_LIFT), speed)
    time.sleep(WAIT_HOME)

def send_safe(angles, speed=REPLAY_SPD):
    try:
        mc.send_angles(clamp(angles), speed)
    except Exception as e:
        print(f"      ⚠ 스킵: {e}")

def downsample(traj, interval):
    if not traj: return traj
    result = [traj[0]]
    last_t = traj[0]["t"]
    for p in traj[1:-1]:
        if p["t"] - last_t >= interval:
            result.append(p)
            last_t = p["t"]
    result.append(traj[-1])
    return result


# ──────────────────────────────────────────────
#  영점 티칭
# ──────────────────────────────────────────────

def teach_home():
    global HOME_ANGLES
    print("\n  📍 이동용 영점 티칭 (시약통 → 시험관 거치대)")
    input("\n  → 서보 해제 [Enter] ")
    mc.release_all_servos()
    input("  → 원하는 영점 위치 정렬 [Enter] (서보 잠금) ")
    mc.power_on(); time.sleep(0.5)
    angles = get_angles()
    if angles is None:
        print("  [ERROR] 각도 읽기 실패"); return
    HOME_ANGLES = angles
    save_config()
    print(f"  ✓ 이동용 영점 저장: {[round(a,1) for a in HOME_ANGLES]}")


def teach_home_lift():
    global HOME_ANGLES_LIFT
    print("\n  📍 붓기용 영점 티칭 (시험관 거치대 → 비커)")
    print("  ※ 시험관이 수직으로 유지되는 자세로 설정해주세요")
    input("\n  → 서보 해제 [Enter] ")
    mc.release_all_servos()
    input("  → 원하는 붓기 영점 위치 정렬 [Enter] (서보 잠금) ")
    mc.power_on(); time.sleep(0.5)
    angles = get_angles()
    if angles is None:
        print("  [ERROR] 각도 읽기 실패"); return
    HOME_ANGLES_LIFT = angles
    save_config()
    print(f"  ✓ 붓기용 영점 저장: {[round(a,1) for a in HOME_ANGLES_LIFT]}")


# ──────────────────────────────────────────────
#  Trajectory 녹화 (집기 / 꽂기 공통)
# ──────────────────────────────────────────────

def record_trajectory(label, close_prompt, open_prompt,
                      start_grip=False, grip_value=None):
    """
    grip_value: None이면 GRIP_VALUE 사용, 지정하면 해당 값으로 닫기
    """
    _grip = grip_value if grip_value is not None else GRIP_VALUE
    print(f"\n  📹 {label} 녹화")

    if start_grip:
        print(f"  순서:")
        print(f"    1. [Enter] → 그리퍼 열기")
        print(f"    2. 시험관을 그리퍼에 끼우기")
        print(f"    3. [Enter] → 그리퍼 닫기 + 녹화 시작")
        print(f"    4. 손으로 꽂는 위치까지 이동")
        print(f"    5. [Enter] → 그리퍼 열기 (꽂기 완료)")
        print(f"    6. 영점으로 복귀 → [Enter] → 녹화 종료")
    else:
        print(f"  순서:")
        print(f"    1. [Enter] → 그리퍼 열기 + 녹화 시작")
        print(f"    2. 손으로 시험관 잡는 위치까지 이동")
        print(f"    3. [Enter] → 그리퍼 닫기 (집기)")
        print(f"    4. 영점으로 복귀 → [Enter] → 녹화 종료")

    traj, events = [], []
    recording = [True]
    t0 = None

    def rec():
        while recording[0]:
            t = time.time() - t0
            a = get_angles()
            if a:
                traj.append({"t": round(t, 2), "angles": clamp(a)})
            time.sleep(RECORD_INT)

    if start_grip:
        # ── 꽂기 녹화 흐름 ──
        input(f"\n  영점에 로봇팔 둔 뒤 [Enter] → 그리퍼 열기 ")
        gripper_open()
        print("  → 그리퍼 열렸습니다. 시험관을 그리퍼에 끼워주세요.")

        input(f"\n  시험관 끼운 뒤 [Enter] → 그리퍼 닫기 + 녹화 시작 ")
        gripper_close()
        print("  → 그리퍼 닫혔습니다. 녹화 시작!")

        t0 = time.time()
        mc.release_all_servos()

        # ★ 그리퍼 힘 유지 스레드 (서보 해제 후에도 그리퍼만 계속 닫기 유지)
        grip_hold = [True]
        def hold_grip():
            while grip_hold[0]:
                try:
                    mc.set_gripper_value(GRIP_VALUE_STIR, SPEED_GRIP)
                except:
                    pass
                time.sleep(0.5)
        threading.Thread(target=hold_grip, daemon=True).start()

        threading.Thread(target=rec, daemon=True).start()
        print("  ▶ 녹화 중 (서보 해제, 그리퍼 힘 유지 중)")

        input(f"\n  → 꽂는 위치 도달 [Enter] → 그리퍼 열기 ")
        grip_hold[0] = False   # 그리퍼 유지 중단
        time.sleep(0.1)
        t1 = time.time() - t0
        events.append({"t": round(t1, 2), "action": "open"})
        gripper_open()
        print(f"  💾 그리퍼 열기 (t={t1:.1f}s)")

    else:
        # ── 집기 녹화 흐름 ──
        input(f"\n  영점에 로봇팔 둔 뒤 [Enter] → 그리퍼 열기 + 녹화 시작 ")
        gripper_open()

        t0 = time.time()
        mc.release_all_servos()
        threading.Thread(target=rec, daemon=True).start()
        print("  ▶ 녹화 중 (서보 해제, 손으로 이동)")

        # 닫기 이벤트 (집기)
        input(f"\n  → 시험관 잡는 위치 도달 [Enter] → 그리퍼 닫기 ")
        t1 = time.time() - t0
        events.append({"t": round(t1, 2), "action": "close"})
        mc.set_gripper_value(_grip, SPEED_GRIP)
        time.sleep(WAIT_GRIP)
        print(f"  💾 그리퍼 닫기 (t={t1:.1f}s, grip={_grip})")

    # 공통 종료
    input(f"\n  → 영점으로 복귀 후 [Enter] (녹화 종료) ")
    recording[0] = False
    time.sleep(0.5)
    mc.power_on()

    print(f"  ✓ 녹화 완료: {len(traj)}포인트, {traj[-1]['t']:.1f}s")
    return {"trajectory": traj, "events": events,
            "start_grip": start_grip, "grip_value": _grip}


# ──────────────────────────────────────────────
#  Trajectory 재생 (집기 / 꽂기 공통)
# ──────────────────────────────────────────────

def replay_trajectory(data, label):
    """집기/꽂기 공통 재생"""
    raw        = data["trajectory"]
    evts       = data["events"]
    start_grip = data.get("start_grip", False)
    grip_val   = data.get("grip_value", GRIP_VALUE)
    samp       = data.get("replay_samp", REPLAY_SAMP)
    t_scale    = data.get("time_scale",  TIME_SCALE)  # 섞기 등 전용 속도

    if not raw:
        print(f"  [WARN] {label} 데이터 없음"); return

    traj = downsample(raw, samp)
    print(f"\n  ▶ {label} 재생 ({len(traj)}포인트)")

    mc.stop(); time.sleep(WAIT_STOP)

    if start_grip:
        print(f"  → 그리퍼 닫기 (grip={grip_val})")
        mc.set_gripper_value(grip_val, SPEED_GRIP)
        time.sleep(WAIT_GRIP)
    else:
        print("  → 그리퍼 열기")
        gripper_open()
        print("  → 그리퍼 열기")
        gripper_open()

    # 첫 위치로 이동 (J6 먼저 이동 후 전체 이동)
    first = clamp(traj[0]["angles"])
    print("  → J6 먼저 정렬 중...")
    mc.send_angle(6, first[5], 15)   # J6만 먼저
    time.sleep(2.5)
    send_safe(first, 15)             # 전체 이동
    time.sleep(WAIT_FIRST_POS)

    evt_idx, last_t = 0, 0
    for pt in traj:
        cur_t = pt["t"]
        wait  = max(0.08, (cur_t - last_t) * t_scale)
        last_t = cur_t

        while evt_idx < len(evts) and evts[evt_idx]["t"] <= cur_t:
            ev = evts[evt_idx]
            if ev["action"] == "close":
                print(f"      🤏 그리퍼 닫기 (t={ev['t']:.1f}s, grip={grip_val})")
                mc.set_gripper_value(grip_val, SPEED_GRIP)
                time.sleep(WAIT_GRIP)
            elif ev["action"] == "open":
                print(f"      ✋ 그리퍼 열기 (t={ev['t']:.1f}s)")
                gripper_open()
            evt_idx += 1

        send_safe(pt["angles"])
        time.sleep(wait)

    while evt_idx < len(evts):
        ev = evts[evt_idx]
        if ev["action"] == "close":
            mc.set_gripper_value(grip_val, SPEED_GRIP)
            time.sleep(WAIT_GRIP)
        elif ev["action"] == "open": gripper_open()
        evt_idx += 1

    print(f"  ✓ {label} 재생 완료")


# ──────────────────────────────────────────────
#  확인 루프 (저장/재생/재녹화)
# ──────────────────────────────────────────────

def confirm_and_save(data, key, file_dict, filepath, label, tube_idx=0):
    """녹화 후 s/p/r 선택 → 즉시 저장"""
    pts = len(data.get("trajectory", []))
    dur = data["trajectory"][-1]["t"] if pts else 0
    print(f"\n  ─ {label}: {pts}포인트, {dur:.1f}s ─")

    while True:
        print("    [s] 저장  [p] 미리 재생  [r] 다시 녹화")
        c = input("  > ").strip().lower()
        if c == "s":
            file_dict[key] = data
            with open(filepath, "w") as f:
                json.dump(file_dict, f, indent=2)
            print(f"  💾 {label} 즉시 저장 완료")
            return True   # 저장됨
        elif c == "p":
            input("  안전 거리 확보 후 [Enter] ")
            replay_trajectory(data, label)
        elif c == "r":
            return False  # 재녹화


def teach_lifts(lifts):
    """시험관 거치대 옆면 집기 녹화 (A1~B4, 8개)"""
    print("\n" + "="*55)
    print("  [옆면 집기 녹화] A1~A4, B1~B4")
    print("="*55)
    print("  ※ 영점에서 시작 → 옆면으로 접근 → 집기 → 영점 대기")

    for rack in ["A", "B"]:
        for num in range(1, 5):
            key = f"{rack}{num}"

            if key in lifts:
                print(f"\n  ✅ 슬롯 {key} 이미 완료")
                if input("     다시 녹화? [y/n] ").strip().lower() != "y":
                    continue

            while True:
                if input(f"\n  [{key}] 영점 이동? [y/n] ").strip().lower() == "y":
                    go_home()
                data = record_trajectory(
                    label=f"{key} 옆면 집기",
                    close_prompt=f"슬롯 {key} 시험관 옆면 잡는 위치",
                    open_prompt=None,   # 집기만 (열기 없음)
                    start_grip=False
                )
                if confirm_and_save(data, key, lifts, LIFT_FILE, f"{key} 옆면집기"):
                    break


def run_lift(lifts, slot):
    """시험관 거치대에서 옆면 집기 후 붓기용 영점 대기"""
    if slot not in lifts:
        print(f"  [ERROR] {slot} 옆면 집기 데이터 없음")
        return False

    print(f"\n  ▶ {slot} 옆면 집기 → 붓기용 영점 대기")
    replay_trajectory(lifts[slot], f"{slot} 옆면 집기")
    go_home_lift()   # ★ 붓기용 영점 (시험관 수직 유지)
    print(f"  ✓ {slot} 시험관 들고 붓기용 영점 대기 완료")
    print("  → 붓기 동작 진행하세요")
    return True


# ──────────────────────────────────────────────
#  집기 녹화
# ──────────────────────────────────────────────

def teach_pickups(pickups):
    print("\n" + "="*55)
    print("  [집기 녹화] tube_1 ~ tube_4")
    print("="*55)

    i = 0
    while i < NUM_TUBES:
        key = f"tube_{i+1}"

        if key in pickups:
            print(f"\n  ✅ {key} 이미 완료")
            if input("     다시 녹화? [y/n] ").strip().lower() != "y":
                i += 1; continue

        while True:
            if input(f"\n  [{key}] 영점 이동? [y/n] ").strip().lower() == "y":
                go_home()
            data = record_trajectory(
                label=f"{key} 집기",
                close_prompt="시험관 잡는 위치",
                open_prompt=None   # 집기는 열기 없음
            )
            if confirm_and_save(data, key, pickups, PICKUP_FILE, key):
                i += 1; break


# ──────────────────────────────────────────────
#  꽂기 녹화
# ──────────────────────────────────────────────

def teach_drops(drops):
    print("\n" + "="*55)
    print("  [꽂기 녹화] A1~A4, B1~B4")
    print("="*55)
    print("  ※ 영점에서 시작 → 꽂는 위치 → 그리퍼 열기 → 영점 복귀")

    for rack in ["A", "B"]:
        for num in range(1, 5):
            key = f"{rack}{num}"

            if key in drops:
                print(f"\n  ✅ 슬롯 {key} 이미 완료")
                if input("     다시 녹화? [y/n] ").strip().lower() != "y":
                    continue

            while True:
                if input(f"\n  [{key}] 영점 이동? [y/n] ").strip().lower() == "y":
                    go_home()
                data = record_trajectory(
                    label=f"{key} 꽂기",
                    close_prompt=f"슬롯 {key} 꽂는 위치 (그리퍼로 시험관 잡힌 상태)",
                    open_prompt=f"슬롯 {key} 구멍에 꽂힌 후",
                    start_grip=True   # ★ 꽂기는 그리퍼 닫힌 상태로 시작
                )
                if confirm_and_save(data, key, drops, DROP_FILE, f"슬롯{key}"):
                    break


# ──────────────────────────────────────────────
#  Action Log (리셋용 실행 기록)
# ──────────────────────────────────────────────

LOG_FILE   = "action_log.json"
action_log = []   # 실험 중 동작 기록

def log_action(action, tube=None, slot=None):
    """동작 기록 (pickup / drop)"""
    entry = {"action": action, "step": len(action_log) + 1}
    if tube: entry["tube"] = tube
    if slot: entry["slot"] = slot
    action_log.append(entry)
    # 즉시 파일 저장 (중간에 꺼져도 복구 가능)
    with open(LOG_FILE, "w") as f:
        json.dump(action_log, f, indent=2)
    print(f"  📝 기록: step {entry['step']} → {action} "
          f"{tube or ''}{slot or ''}")

def clear_log():
    global action_log
    action_log = []
    if os.path.exists(LOG_FILE):
        os.remove(LOG_FILE)
    print("  📝 action_log 초기화")

def load_log():
    global action_log
    if os.path.exists(LOG_FILE):
        with open(LOG_FILE) as f:
            action_log = json.load(f)
        print(f"  📝 기존 로그 로드: {len(action_log)}개 동작")
    else:
        action_log = []


# ──────────────────────────────────────────────
#  Trajectory 역재생
# ──────────────────────────────────────────────

def replay_trajectory_reverse(data, label):
    """trajectory를 역순으로 재생 + 그리퍼 이벤트 반전"""
    raw  = data["trajectory"]
    evts = data["events"]
    start_grip = data.get("start_grip", False)

    if not raw:
        print(f"  [WARN] {label} 데이터 없음"); return

    # 포인트 역순 + 시간 재계산
    rev_traj = list(reversed(raw))
    total_t  = raw[-1]["t"]
    rev_traj = [{"t": round(total_t - p["t"], 2), "angles": p["angles"]}
                for p in rev_traj]

    # 이벤트 반전 (open↔close) + 시간 역산
    rev_evts = []
    for e in reversed(evts):
        rev_evts.append({
            "t":      round(total_t - e["t"], 2),
            "action": "open" if e["action"] == "close" else "close"
        })

    traj = downsample(rev_traj, REPLAY_SAMP)
    print(f"\n  ↩ {label} 역재생 ({len(traj)}포인트)")

    # 시작 그리퍼 상태 반전 (원본의 반대)
    mc.stop(); time.sleep(WAIT_STOP)
    if not start_grip:
        print("  → 그리퍼 닫기 (시험관 잡힌 상태로 시작)")
        gripper_close()
    else:
        print("  → 그리퍼 열기")
        gripper_open()

    first_r = clamp(traj[0]["angles"])
    print("  → J6 먼저 정렬 중...")
    mc.send_angle(6, first_r[5], 15)
    time.sleep(2.5)
    send_safe(first_r, 15)
    time.sleep(WAIT_FIRST_POS)

    evt_idx, last_t = 0, 0
    for pt in traj:
        cur_t = pt["t"]
        wait  = max(0.08, (cur_t - last_t) * TIME_SCALE)
        last_t = cur_t

        while evt_idx < len(rev_evts) and rev_evts[evt_idx]["t"] <= cur_t:
            ev = rev_evts[evt_idx]
            if ev["action"] == "close":
                print(f"      🤏 그리퍼 닫기 (t={ev['t']:.1f}s)")
                gripper_close()
            elif ev["action"] == "open":
                print(f"      ✋ 그리퍼 열기 (t={ev['t']:.1f}s)")
                gripper_open()
            evt_idx += 1

        send_safe(pt["angles"])
        time.sleep(wait)

    while evt_idx < len(rev_evts):
        ev = rev_evts[evt_idx]
        if ev["action"] == "close": gripper_close()
        elif ev["action"] == "open": gripper_open()
        evt_idx += 1

    print(f"  ✓ {label} 역재생 완료")


# ──────────────────────────────────────────────
#  리셋 실행
# ──────────────────────────────────────────────

def teach_pour(pour):
    """붓기 trajectory 녹화 (비커로 이동 → 붓기 → 붓기용 영점 복귀)"""
    print("\n" + "="*55)
    print("  [붓기 녹화] 비커로 이동 후 기울여서 붓기")
    print("="*55)
    print("  ※ 붓기용 영점(시험관 잡힌 상태)에서 시작")
    print("  ※ 그리퍼는 붓는 동안 계속 닫힌 상태 유지")

    while True:
        if input("\n  붓기용 영점 이동? [y/n] ").strip().lower() == "y":
            go_home_lift()

        print("\n  순서:")
        print("    1. [Enter] → 녹화 시작 (그리퍼 닫힌 상태)")
        print("    2. 비커 위로 손으로 이동 → 기울여서 붓기")
        print("    3. 붓기용 영점으로 복귀")
        print("    4. [Enter] → 녹화 종료")

        input("\n  붓기용 영점에 둔 뒤 [Enter] → 그리퍼 열기 ")
        gripper_open()
        print("  → 그리퍼 열렸습니다. 시험관을 그리퍼에 끼워주세요.")

        input("\n  시험관 끼운 뒤 [Enter] → 그리퍼 닫기 + 녹화 시작 ")
        gripper_close()
        print("  → 그리퍼 닫혔습니다. 녹화 시작!")

        traj = []
        recording = [True]
        t0 = time.time()

        # 그리퍼 힘 유지 스레드
        grip_hold = [True]
        def hold_grip():
            while grip_hold[0]:
                try:
                    mc.set_gripper_value(GRIP_VALUE_STIR, SPEED_GRIP)
                except:
                    pass
                time.sleep(0.5)

        def rec():
            while recording[0]:
                t = time.time() - t0
                a = get_angles()
                if a:
                    traj.append({"t": round(t, 2), "angles": clamp(a)})
                time.sleep(RECORD_INT)

        mc.release_all_servos()
        threading.Thread(target=hold_grip, daemon=True).start()
        threading.Thread(target=rec, daemon=True).start()
        print("  ▶ 녹화 중 (서보 해제, 그리퍼 힘 유지)")

        input("\n  → 붓기용 영점 복귀 후 [Enter] (녹화 종료) ")
        grip_hold[0] = False
        recording[0] = False
        time.sleep(0.5)
        mc.power_on()

        pts = len(traj)
        dur = traj[-1]["t"] if pts else 0
        print(f"  ✓ 녹화 완료: {pts}포인트, {dur:.1f}s")

        # 확인
        while True:
            print("    [s] 저장  [p] 미리 재생  [r] 다시 녹화")
            c = input("  > ").strip().lower()
            if c == "s":
                data = {"trajectory": traj, "events": [], "start_grip": True}
                pour.update(data)
                with open(POUR_FILE, "w") as f:
                    json.dump(pour, f, indent=2)
                print("  💾 붓기 저장 완료")
                return
            elif c == "p":
                input("  안전 거리 확보 후 [Enter] ")
                replay_trajectory({"trajectory": traj, "events": [], "start_grip": True}, "붓기 테스트")
            elif c == "r":
                break


def teach_side_drops(side_drops):
    """옆면 잡은 채로 슬롯에 꽂기 녹화 (A1~B4, 8개)"""
    print("\n" + "="*55)
    print("  [옆면 꽂기 녹화] A1~A4, B1~B4")
    print("="*55)
    print("  ※ 붓기용 영점(시험관 옆면 잡힌 상태)에서 시작")
    print("  ※ 슬롯에 꽂고 그리퍼 열기까지 녹화")

    for rack in ["A", "B"]:
        for num in range(1, 5):
            key = f"{rack}{num}"
            if key in side_drops:
                print(f"\n  ✅ {key} 이미 완료")
                if input("     다시 녹화? [y/n] ").strip().lower() != "y":
                    continue
            while True:
                if input(f"\n  [{key}] 붓기용 영점 이동? [y/n] ").strip().lower() == "y":
                    go_home_lift()
                data = record_trajectory(
                    label=f"{key} 옆면 꽂기",
                    close_prompt=f"슬롯 {key} 꽂는 위치 (옆면 잡힌 상태)",
                    open_prompt=f"슬롯 {key} 구멍에 꽂힌 후 그리퍼 열기",
                    start_grip=True
                )
                if confirm_and_save(data, key, side_drops,
                                    SIDE_DROP_FILE, f"{key} 옆면꽂기"):
                    break


def run_pour_full(lifts, side_drops, pour, slot):
    """
    붓기 전체 시퀀스:
    ① 슬롯 옆면 집기 + 붓기용 영점
    ② 비커 붓기 + 붓기용 영점 복귀
    ③ 옆면 잡은 채로 원래 슬롯 되돌려 꽂기
    """
    if slot not in lifts:
        print(f"  [ERROR] {slot} 옆면 집기 데이터 없음"); return
    if slot not in side_drops:
        print(f"  [ERROR] {slot} 옆면 꽂기 데이터 없음 → [D]로 먼저 녹화"); return
    if not pour:
        print("  [ERROR] 붓기 데이터 없음 → [P]로 먼저 녹화"); return

    print(f"\n  ▶ 붓기 전체 시퀀스: {slot}")
    print("  ① 옆면 집기 → 붓기용 영점")
    print("  ② 붓기 → 붓기용 영점 복귀")
    print("  ③ 원래 슬롯 옆면 꽂기")
    input("\n  [Enter] 시작 ")

    # ① 옆면 집기 → 붓기용 영점
    print("\n  [① 집기]")
    replay_trajectory(lifts[slot], f"{slot} 옆면 집기")
    go_home_lift()

    # ② 붓기 → 붓기용 영점 복귀
    print("\n  [② 붓기]")
    replay_trajectory(pour, "비커 붓기")
    mc.stop(); time.sleep(1.0)
    go_home_lift()

    # ③ 옆면 잡은 채로 원래 슬롯 꽂기 (이동용 영점 없이 바로)
    print(f"\n  [③ {slot} 옆면 꽂기]")
    replay_trajectory(side_drops[slot], f"{slot} 옆면 꽂기")

    go_home()
    print("\n  ✓ 붓기 전체 완료")


def teach_reset_drops(reset_drops):
    """리셋용: 시약 거치대 원래 위치에 꽂기 녹화 (tube_1~4)"""
    print("\n" + "="*55)
    print("  [리셋 꽂기 녹화] 시약 거치대 tube_1~4")
    print("="*55)
    print("  ※ 영점(시험관 잡힌 상태) → 시약 거치대 꽂기 → 그리퍼 열기 → 영점")

    for i in range(1, 5):
        key = f"tube_{i}"
        if key in reset_drops:
            print(f"\n  ✅ {key} 이미 완료")
            if input("     다시 녹화? [y/n] ").strip().lower() != "y":
                continue
        while True:
            if input(f"\n  [{key}] 영점 이동? [y/n] ").strip().lower() == "y":
                go_home()
            data = record_trajectory(
                label=f"{key} 리셋 꽂기",
                close_prompt=f"시약 거치대 {key} 꽂는 위치",
                open_prompt=f"시약 거치대 {key} 구멍에 꽂힌 후",
                start_grip=True
            )
            if confirm_and_save(data, key, reset_drops,
                                RESET_DROP_FILE, f"{key} 리셋꽂기"):
                break


def run_reset(pickups, drops, reset_pickups, reset_drops):
    """action_log 역순으로 시험관을 원래 시약 거치대로 복구"""
    if not action_log:
        print("  [리셋] 기록된 동작 없음")
        return

    print("\n" + "="*55)
    print("  ↩ 리셋: 시험관 원래 위치로 복구")
    print("="*55)
    print(f"\n  기록된 동작 {len(action_log)}개:")
    for entry in action_log:
        desc = entry.get("tube") or entry.get("slot") or ""
        print(f"    step {entry['step']}: {entry['action']} {desc}")

    if not reset_pickups or not reset_drops:
        print("\n  [ERROR] 리셋 녹화 데이터 없음")
        print("  → 메뉴 [E]: 리셋 집기 녹화 / 메뉴 [F]: 리셋 꽂기 녹화 먼저 진행")
        return

    print("\n  ⚠ 시험관 위치만 복구됩니다. 혼합된 액체는 복구 불가.")
    if input("  리셋 진행할까요? [y/n] ").strip().lower() != "y":
        return

    # 역순: [drop, pickup] 쌍을 뒤에서부터
    steps = list(reversed(action_log))
    i = 0
    while i < len(steps):
        step = steps[i]
        if step["action"] == "drop":
            slot = step["slot"]
            tube = None
            if i + 1 < len(steps) and steps[i+1]["action"] == "pickup":
                tube = steps[i+1]["tube"]
            else:
                print(f"  [WARN] 쌍 없음, 스킵"); i += 1; continue

            print(f"\n  복구: {slot} → 집기 → 영점 → {tube} 원위치")

            # 1. 슬롯에서 수직 집기
            if slot not in reset_pickups:
                print(f"  [ERROR] {slot} 리셋 집기 데이터 없음"); i += 2; continue
            replay_trajectory(reset_pickups[slot], f"{slot} 리셋집기")

            # 2. 영점
            go_home()

            # 3. 시약 거치대 원래 위치에 꽂기
            if tube not in reset_drops:
                print(f"  [ERROR] {tube} 리셋 꽂기 데이터 없음"); i += 2; continue
            replay_trajectory(reset_drops[tube], f"{tube} 리셋꽂기")

            go_home()
            i += 2
        else:
            i += 1

    clear_log()
    print("\n  ✓ 리셋 완료")


# ──────────────────────────────────────────────
#  통합 실행
# ──────────────────────────────────────────────

def run_transport(pickups, drops):
    print("\n" + "="*55)
    print("  통합 실행: 집기 → 영점 → 꽂기")
    print("="*55)

    # 시험관 선택
    avail_p = list(pickups.keys())
    avail_d = list(drops.keys())
    print(f"\n  집기 가능: {avail_p}")
    tube_no = input("  시험관 번호 (1~4): ").strip()
    key_p = f"tube_{tube_no}"
    if key_p not in pickups:
        print(f"  [ERROR] {key_p} 없음"); return

    # 슬롯 선택
    print(f"\n  꽂기 가능: {avail_d}")
    slot = input("  슬롯 (예: A2, B3): ").strip().upper()
    if slot not in drops:
        print(f"  [ERROR] 슬롯 {slot} 없음"); return

    print(f"\n  → {key_p} 집기 → 영점 → {slot} 꽂기")
    input("  [Enter] 시작 ")

    # 1. 집기
    print("\n  [1단계] 집기")
    replay_trajectory(pickups[key_p], f"{key_p} 집기")
    log_action("pickup", tube=key_p)   # ★ 기록

    # 2. 영점
    print("\n  [2단계] 영점으로 이동")
    go_home()

    # 3. 꽂기
    print("\n  [3단계] 꽂기")
    replay_trajectory(drops[slot], f"{slot} 꽂기")
    log_action("drop", slot=slot)      # ★ 기록

    # 4. 영점 복귀
    print("\n  [완료] 영점 복귀")
    go_home()
    print(f"  ✓ 완료  (누적 동작: {len(action_log)}개)")


# ──────────────────────────────────────────────
#  main
# ──────────────────────────────────────────────

def main():
    load_config()
    load_log()   # 이전 로그 복구

    pickups       = {}
    drops         = {}
    lifts         = {}
    reset_pickups = {}
    reset_drops   = {}

    if os.path.exists(PICKUP_FILE):
        with open(PICKUP_FILE) as f: pickups = json.load(f)
    if os.path.exists(DROP_FILE):
        with open(DROP_FILE) as f: drops = json.load(f)
    if os.path.exists(LIFT_FILE):
        with open(LIFT_FILE) as f: lifts = json.load(f)
    if os.path.exists(RESET_PICKUP_FILE):
        with open(RESET_PICKUP_FILE) as f: reset_pickups = json.load(f)
    if os.path.exists(RESET_DROP_FILE):
        with open(RESET_DROP_FILE) as f: reset_drops = json.load(f)

    stir        = {}
    stir_action = {}
    stir_drop   = {}
    pour        = {}
    side_drops  = {}
    if os.path.exists(STIR_FILE):
        with open(STIR_FILE) as f: stir = json.load(f)
    if os.path.exists(STIR_ACTION_FILE):
        with open(STIR_ACTION_FILE) as f: stir_action = json.load(f)
    if os.path.exists(STIR_DROP_FILE):
        with open(STIR_DROP_FILE) as f: stir_drop = json.load(f)
    if os.path.exists(POUR_FILE):
        with open(POUR_FILE) as f: pour = json.load(f)
    if os.path.exists(SIDE_DROP_FILE):
        with open(SIDE_DROP_FILE) as f: side_drops = json.load(f)

    print("\n" + "="*55)
    print("  SterileBot 시험관 집기/꽂기 v3 (리셋 기능)")
    print("="*55)
    print(f"\n  영점(이동): {[round(a,1) for a in HOME_ANGLES]}")
    print(f"  영점(붓기): {[round(a,1) for a in HOME_ANGLES_LIFT]}")
    print(f"  집기:       {list(pickups.keys()) or '없음'}")
    print(f"  꽂기:       {list(drops.keys()) or '없음'}")
    print(f"  옆면집기:   {list(lifts.keys()) or '없음'}")
    print(f"  붓기:       {'완료' if pour else '없음'}")
    print(f"  옆면꽂기:   {list(side_drops.keys()) or '없음'}")
    print(f"  막대기집기: {'완료' if stir else '없음'}")
    print(f"  섞기:       {'완료' if stir_action else '없음'}")
    print(f"  막대기놓기: {'완료' if stir_drop else '없음'}")
    print(f"  리셋꽂기:   {list(reset_drops.keys()) or '없음'}")
    print(f"  기록된 동작: {len(action_log)}개 "
          f"{'(리셋 가능)' if action_log else ''}")

    while True:
        print("\n  ─── 실행 ───────────────────────────────")
        print("    [1] 통합 실행 (집기 → 영점 → 꽂기)")
        print("    [2] 집기만 테스트 (시약통 거치대)")
        print("    [3] 꽂기만 테스트")
        print("    [9] 옆면 집기 + 붓기용 영점 대기")
        print("    [W] 붓기 전체 실행 (집기→붓기→되돌려꽂기) ★")
        print("    [R] ↩ 리셋 (시험관 원래 위치로 복구)")
        print("  ─── 티칭 ───────────────────────────────")
        print("    [4] 이동용 영점 티칭")
        print("    [C] 붓기용 영점 티칭 ★")
        print("    [5] 집기 녹화 (시약통 거치대)")
        print("    [6] 꽂기 녹화 (A/B 거치대)")
        print("    [A] 옆면 집기 녹화 (붓기 전 준비)")
        print("    [P] 붓기 녹화 ★")
        print("    [D] 옆면 꽂기 녹화 (붓기 후 되돌리기) ★")
        print("    [S] 막대기 집기 녹화 ★")
        print("    [X] 섞기 동작 녹화 ★")
        print("    [Z] 막대기 내려놓기 녹화 ★")
        print("    [M] 섞기 전체 실행 (집기→섞기→놓기) ★")
        print("    [E] 리셋 집기 녹화 (A/B슬롯 수직)")
        print("    [F] 리셋 꽂기 녹화 (시약 거치대)")
        print("  ─── 재녹화 ─────────────────────────────")
        print("    [7] 집기 1개 재녹화")
        print("    [8] 꽂기 1개 재녹화")
        print("    [B] 옆면 집기 1개 재녹화")
        print("    [G] 리셋 집기 1개 재녹화")
        print("    [q] 종료")

        ans = input("\n  선택: ").strip().upper()

        if ans == "Q":
            break

        elif ans == "1":
            if not pickups or not drops:
                print("  [ERROR] 집기/꽂기 데이터 없음"); continue
            run_transport(pickups, drops)

        elif ans == "2":
            t = input("  시험관 번호 (1~4): ").strip()
            key = f"tube_{t}"
            if key in pickups:
                input("  안전 거리 확보 후 [Enter] ")
                replay_trajectory(pickups[key], f"{key} 집기")
            else:
                print(f"  [ERROR] {key} 없음")

        elif ans == "3":
            slot = input("  슬롯 (예: A2): ").strip().upper()
            if slot in drops:
                input("  안전 거리 확보 후 [Enter] ")
                replay_trajectory(drops[slot], f"{slot} 꽂기")
            else:
                print(f"  [ERROR] {slot} 없음")

        elif ans == "9":
            # 옆면 집기 + 영점 대기
            if not lifts:
                print("  [ERROR] 옆면 집기 데이터 없음 → [A]로 먼저 녹화")
                continue
            print(f"\n  옆면 집기 가능: {list(lifts.keys())}")
            slot = input("  슬롯 (예: A1, B2): ").strip().upper()
            run_lift(lifts, slot)

        elif ans == "W":
            if not lifts or not side_drops or not pour:
                print("  [ERROR] 옆면집기/옆면꽂기/붓기 데이터 모두 필요"); continue
            print(f"\n  옆면 집기 가능: {list(lifts.keys())}")
            slot = input("  슬롯 (예: A1): ").strip().upper()
            run_pour_full(lifts, side_drops, pour, slot)

        elif ans == "P":
            teach_pour(pour)

        elif ans == "D":
            teach_side_drops(side_drops)

        elif ans == "S":
            while True:
                if input("\n  영점 이동? [y/n] ").strip().lower() == "y":
                    go_home_lift()
                # 막대기 집기는 GRIP_VALUE_STIR 사용
                mc.set_gripper_value(OPEN_VALUE, SPEED_GRIP)
                time.sleep(0.8)
                print("  막대기 집기에는 강한 그리퍼 힘 적용 (값:", GRIP_VALUE_STIR, ")")

                data = record_trajectory(
                    label="막대기 집기",
                    close_prompt="막대기 잡는 위치",
                    open_prompt=None,
                    start_grip=False,
                    grip_value=GRIP_VALUE_STIR   # ★ 강하게 잡기
                )
                # grip_value는 record_trajectory에서 자동 저장됨
                pts = len(data["trajectory"])
                dur = data["trajectory"][-1]["t"] if pts else 0
                print(f"\n  ─ 결과: {pts}포인트, {dur:.1f}s ─")
                while True:
                    print("    [s] 저장  [p] 미리 재생  [r] 다시 녹화")
                    c = input("  > ").strip().lower()
                    if c == "s":
                        stir.update(data)
                        with open(STIR_FILE, "w") as f:
                            json.dump(stir, f, indent=2)
                        print("  💾 막대기 집기 저장 완료")
                        break
                    elif c == "p":
                        input("  안전 거리 확보 후 [Enter] ")
                        replay_trajectory(data, "막대기 집기 테스트")
                    elif c == "r":
                        break
                if c == "s":
                    break

        elif ans == "X":
            # 섞기 동작 녹화 - 그리퍼 닫힌 채 휘젓고 영점 복귀
            while True:
                if input("\n  붓기용 영점 이동? [y/n] ").strip().lower() == "y":
                    go_home_lift()

                print("\n  순서:")
                print("    1. [Enter] → 그리퍼 닫기 + 서보 해제 + 녹화 시작")
                print("    2. 비커 안에서 막대기로 휘젓기")
                print("    3. 붓기용 영점으로 복귀")
                print("    4. [Enter] → 녹화 종료")

                input("\n  붓기용 영점에서 막대기 끼운 뒤 [Enter] → 그리퍼 닫기 + 녹화 시작 ")
                mc.set_gripper_value(GRIP_VALUE_STIR, SPEED_GRIP)
                time.sleep(WAIT_GRIP)
                print("  → 그리퍼 닫혔습니다. 녹화 시작!")

                traj = []
                recording = [True]
                t0 = time.time()

                grip_hold = [True]
                def hold_stir():
                    while grip_hold[0]:
                        try: mc.set_gripper_value(GRIP_VALUE_STIR, SPEED_GRIP)
                        except: pass
                        time.sleep(0.5)

                def rec_stir():
                    while recording[0]:
                        t = time.time() - t0
                        a = get_angles()
                        if a:
                            traj.append({"t": round(t,2), "angles": clamp(a)})
                        time.sleep(STIR_RECORD_INT)   # ★ 더 촘촘하게

                mc.release_all_servos()
                threading.Thread(target=hold_stir, daemon=True).start()
                threading.Thread(target=rec_stir, daemon=True).start()
                print("  ▶ 녹화 중 (서보 해제, 막대기로 휘젓기)")

                input("\n  → 붓기용 영점 복귀 후 [Enter] (녹화 종료) ")
                grip_hold[0] = False
                recording[0] = False
                time.sleep(0.5)
                mc.power_on()

                pts = len(traj)
                dur = traj[-1]["t"] if pts else 0
                data = {"trajectory": traj, "events": [],
                        "start_grip": True, "grip_value": GRIP_VALUE_STIR,
                        "replay_samp": STIR_REPLAY_SAMP,
                        "time_scale":  STIR_TIME_SCALE}  # ★ 빠른 재생
                print(f"\n  ─ 결과: {pts}포인트, {dur:.1f}s ─")

                while True:
                    print("    [s] 저장  [p] 미리 재생  [r] 다시 녹화")
                    c = input("  > ").strip().lower()
                    if c == "s":
                        stir_action.update(data)
                        with open(STIR_ACTION_FILE, "w") as f:
                            json.dump(stir_action, f, indent=2)
                        print("  💾 섞기 저장 완료")
                        break
                    elif c == "p":
                        input("  안전 거리 확보 후 [Enter] ")
                        replay_trajectory(data, "섞기 테스트")
                    elif c == "r":
                        break
                if c == "s":
                    break

        elif ans == "Z":
            # 막대기 내려놓기 녹화 (그리퍼 닫힌 상태 = 막대기 잡고 시작)
            while True:
                if input("\n  영점 이동? [y/n] ").strip().lower() == "y":
                    go_home_lift()
                data = record_trajectory(
                    label="막대기 내려놓기",
                    close_prompt="막대기 놓는 위치 (그리퍼 닫힌 채 이동)",
                    open_prompt="막대기 원래 위치에 내려놓은 후",
                    start_grip=True,
                    grip_value=GRIP_VALUE_STIR
                )
                pts = len(data["trajectory"])
                dur = data["trajectory"][-1]["t"] if pts else 0
                print(f"\n  ─ 결과: {pts}포인트, {dur:.1f}s ─")
                while True:
                    print("    [s] 저장  [p] 미리 재생  [r] 다시 녹화")
                    c = input("  > ").strip().lower()
                    if c == "s":
                        stir_drop.update(data)
                        with open(STIR_DROP_FILE, "w") as f:
                            json.dump(stir_drop, f, indent=2)
                        print("  💾 막대기 내려놓기 저장 완료")
                        break
                    elif c == "p":
                        input("  안전 거리 확보 후 [Enter] ")
                        replay_trajectory(data, "막대기 놓기 테스트")
                    elif c == "r":
                        break
                if c == "s":
                    break

        elif ans == "M":
            # 섞기 전체 실행
            if not stir or not stir_action or not stir_drop:
                print("  [ERROR] 막대기집기/섞기/내려놓기 데이터 모두 필요")
                print("  → [S] 막대기집기  [X] 섞기  [Z] 내려놓기 먼저 녹화")
                continue
            print("\n  ▶ 섞기 전체 시퀀스")
            print("  ① 막대기 집기")
            print("  ② 비커에서 섞기")
            print("  ③ 막대기 내려놓기")
            input("\n  [Enter] 시작 ")

            print("\n  [① 막대기 집기]")
            replay_trajectory(stir, "막대기 집기")
            go_home_lift()   # ★ 붓기용 영점

            print("\n  [② 섞기]")
            replay_trajectory(stir_action, "섞기 동작")
            go_home_lift()   # ★ 붓기용 영점

            print("\n  [③ 막대기 내려놓기]")
            replay_trajectory(stir_drop, "막대기 내려놓기")
            go_home()
            print("\n  ✓ 섞기 전체 완료")
            # 막대기 집기 녹화
            while True:
                if input("\n  영점 이동? [y/n] ").strip().lower() == "y":
                    go_home()
                data = record_trajectory(
                    label="막대기 집기",
                    close_prompt="막대기 잡는 위치",
                    open_prompt=None,
                    start_grip=False
                )
                pts = len(data["trajectory"])
                dur = data["trajectory"][-1]["t"] if pts else 0
                print(f"\n  ─ 결과: {pts}포인트, {dur:.1f}s ─")
                while True:
                    print("    [s] 저장  [p] 미리 재생  [r] 다시 녹화")
                    c = input("  > ").strip().lower()
                    if c == "s":
                        stir.update(data)
                        with open(STIR_FILE, "w") as f:
                            json.dump(stir, f, indent=2)
                        print("  💾 막대기 집기 저장 완료")
                        break
                    elif c == "p":
                        input("  안전 거리 확보 후 [Enter] ")
                        replay_trajectory(data, "막대기 집기 테스트")
                    elif c == "r":
                        break
                if c == "s":
                    break

        elif ans == "R":
            run_reset(pickups, drops, reset_pickups, reset_drops)

        elif ans == "E":
            teach_reset_pickups(reset_pickups)

        elif ans == "F":
            teach_reset_drops(reset_drops)

        elif ans == "A":
            teach_lifts(lifts)

        elif ans == "B":
            slot = input("  재녹화할 슬롯 (예: A1): ").strip().upper()
            while True:
                if input("  영점 이동? [y/n] ").strip().lower() == "y":
                    go_home()
                data = record_trajectory(
                    label=f"{slot} 옆면 집기",
                    close_prompt=f"슬롯 {slot} 시험관 옆면 잡는 위치",
                    open_prompt=None,
                    start_grip=False
                )
                if confirm_and_save(data, slot, lifts, LIFT_FILE, f"{slot} 옆면집기"):
                    break

        elif ans == "G":
            slot = input("  재녹화할 슬롯 (예: A1): ").strip().upper()
            while True:
                if input("  영점 이동? [y/n] ").strip().lower() == "y":
                    go_home()
                data = record_trajectory(
                    label=f"{slot} 리셋 집기",
                    close_prompt=f"슬롯 {slot} 시험관 위에서 수직으로 잡는 위치",
                    open_prompt=None,
                    start_grip=False
                )
                if confirm_and_save(data, slot, reset_pickups,
                                    RESET_PICKUP_FILE, f"{slot} 리셋집기"):
                    break

        elif ans == "4":
            teach_home()

        elif ans == "C":
            teach_home_lift()

        elif ans == "5":
            teach_pickups(pickups)

        elif ans == "6":
            teach_drops(drops)

        elif ans == "7":
            t = input("  다시 녹화할 시험관 번호 (1~4): ").strip()
            key = f"tube_{t}"
            while True:
                if input("  영점 이동? [y/n] ").strip().lower() == "y":
                    go_home()
                data = record_trajectory(
                    label=f"{key} 집기",
                    close_prompt="시험관 잡는 위치",
                    open_prompt=None
                )
                if confirm_and_save(data, key, pickups, PICKUP_FILE, key):
                    break

        elif ans == "8":
            slot = input("  다시 녹화할 슬롯 (예: A2): ").strip().upper()
            while True:
                if input("  영점 이동? [y/n] ").strip().lower() == "y":
                    go_home()
                data = record_trajectory(
                    label=f"{slot} 꽂기",
                    close_prompt=f"슬롯 {slot} 꽂는 위치",
                    open_prompt=f"슬롯 {slot} 구멍에 꽂힌 후",
                    start_grip=True
                )
                if confirm_and_save(data, slot, drops, DROP_FILE, f"슬롯{slot}"):
                    break

    print("\n[종료]")
    # 종료 시 action_log 삭제 (리셋 경로 초기화)
    clear_log()
    print("  action_log 초기화 완료")


if __name__ == "__main__":
    main()
