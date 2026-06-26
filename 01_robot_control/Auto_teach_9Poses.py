"""
auto_teach_9poses.py
====================
9개 자세 자동 티칭 시퀀스.
 
각 자세마다:
  1. 토크 ON 상태로 5초 대기 (이동 안정화)
  2. 토크 OFF → 3초 대기 (사람이 손으로 자세 잡기)
  3. 토크 ON → 좌표 자동 저장
  4. 다음 자세로
 
실행:
  cd ~/SterileBot
  python3 auto_teach_9poses.py
 
출력 파일:
  data/markers.csv        — 좌표 (X, Y, Z, RX, RY, RZ)
  data/poses_angles.csv   — 관절 각도 (J1~J6)
 
키 조작:
  Ctrl+C : 비상 정지 (저장된 데이터는 유지됨)
"""
 
import csv
import os
import time
from robot_controller import RobotController
 
 
# ─────────────────────────────────────────────────────
# 설정
# ─────────────────────────────────────────────────────
NUM_POSES = 9                # 저장할 자세 개수
WAIT_BEFORE_RELEASE = 5      # 토크 ON 상태에서 대기 (초)
WAIT_FOR_HUMAN = 3           # 토크 OFF 후 사람이 자세 잡는 시간 (초)
WAIT_AFTER_LOCK = 1          # 토크 ON 후 안정화 대기 (초)
 
DATA_DIR = "data"
MARKERS_FILE = os.path.join(DATA_DIR, "markers.csv")
ANGLES_FILE = os.path.join(DATA_DIR, "poses_angles.csv")
 
 
# ─────────────────────────────────────────────────────
# 카운트다운 표시
# ─────────────────────────────────────────────────────
def countdown(seconds, message):
    """카운트다운 진행 (초 단위)"""
    for s in range(seconds, 0, -1):
        print(f"  {message} ({s}초 남음)...", end="\r")
        time.sleep(1)
    # 줄 비우기
    print(" " * 60, end="\r")
 
 
# ─────────────────────────────────────────────────────
# 좌표 / 각도 안전 읽기
# ─────────────────────────────────────────────────────
def safe_get_coords(robot, retries=5, delay=0.3):
    for _ in range(retries):
        try:
            c = robot.get_coords()
            if c and len(c) == 6:
                return [round(v, 2) for v in c]
        except Exception as e:
            print(f"  [WARN] {e}")
        time.sleep(delay)
    return None
 
 
def safe_get_angles(robot, retries=5, delay=0.3):
    for _ in range(retries):
        try:
            a = robot.get_angles()
            if a and len(a) == 6:
                return [round(v, 2) for v in a]
        except Exception as e:
            print(f"  [WARN] {e}")
        time.sleep(delay)
    return None
 
 
# ─────────────────────────────────────────────────────
# 즉시 저장 (중간에 끊겨도 데이터 보존)
# ─────────────────────────────────────────────────────
def save_all(markers, angles):
    """현재까지 모은 데이터를 CSV로 저장"""
    with open(MARKERS_FILE, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["marker_id", "X", "Y", "Z", "RX", "RY", "RZ"])
        w.writerows(markers)
 
    with open(ANGLES_FILE, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["pose_id", "J1", "J2", "J3", "J4", "J5", "J6"])
        w.writerows(angles)
 
 
# ─────────────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────────────
def main():
    os.makedirs(DATA_DIR, exist_ok=True)
 
    print("=" * 60)
    print("  9개 자세 자동 티칭 시퀀스")
    print("=" * 60)
    print(f"\n각 자세마다:")
    print(f"  ① 토크 ON 유지 → {WAIT_BEFORE_RELEASE}초 대기")
    print(f"  ② 토크 OFF → {WAIT_FOR_HUMAN}초 동안 사람이 자세 잡기")
    print(f"  ③ 토크 ON → 좌표 자동 저장")
    print()
 
    print("⚠ 시작 전 확인")
    print("  □ 로봇 첫 자세가 이미 잡혀 있나요? (1번 자세)")
    print("  □ 손으로 팔 잡을 준비 됐나요?")
    print("  □ 케이블이 꼬이지 않았나요?")
    print("  □ 응급 시 Ctrl+C 누를 준비 됐나요?")
 
    confirm = input("\n시작하려면 Enter (취소: Ctrl+C)").strip()
 
    # 로봇 연결
    robot = RobotController()
    if not robot.connect():
        return
 
    markers = []
    angles = []
 
    try:
        for i in range(1, NUM_POSES + 1):
            print("\n" + "═" * 60)
            print(f"  [자세 {i}/{NUM_POSES}]")
            print("═" * 60)
 
            # ① 토크 ON 상태로 대기 (5초)
            print(f"\n① 토크 ON 상태로 대기...")
            countdown(WAIT_BEFORE_RELEASE, "  토크 유지")
 
            # ② 토크 OFF → 3초 동안 사람이 자세 잡기
            print(f"\n② 토크 OFF — 손으로 자세 잡기")
            print("  ⚠ 팔을 손으로 잡으세요!")
            robot.release_servos()
            countdown(WAIT_FOR_HUMAN, "  자세 잡는 중")
 
            # ③ 토크 ON → 좌표 저장
            print(f"\n③ 토크 ON — 자세 고정")
            robot.power_on_servos()
            time.sleep(WAIT_AFTER_LOCK)  # 안정화
 
            # 좌표 / 각도 읽기
            coords = safe_get_coords(robot)
            angles_now = safe_get_angles(robot)
 
            if coords is None or angles_now is None:
                print(f"  ✗ 좌표/각도 읽기 실패 — 자세 {i} 건너뜀")
                continue
 
            # 저장
            markers.append([i] + coords)
            angles.append([i] + angles_now)
            save_all(markers, angles)  # 즉시 저장 (안전)
 
            print(f"\n  ✓ M{i} 저장됨")
            print(f"    좌표 XYZ : ({coords[0]:>7.2f}, {coords[1]:>7.2f}, {coords[2]:>7.2f})")
            print(f"    자세 RPY : ({coords[3]:>7.2f}, {coords[4]:>7.2f}, {coords[5]:>7.2f})")
            print(f"    각도     : {angles_now}")
 
        # ─────────────────────────────────────────
        # 완료 요약
        # ─────────────────────────────────────────
        print("\n" + "═" * 60)
        print(f"  ✓ 완료 — 총 {len(markers)}개 자세 저장")
        print("═" * 60)
        print("\n저장된 자세:")
        for m in markers:
            print(f"  M{m[0]}: ({m[1]:>7.1f}, {m[2]:>7.1f}, {m[3]:>7.1f})")
 
        print(f"\n파일:")
        print(f"  {MARKERS_FILE}")
        print(f"  {ANGLES_FILE}")
 
        if len(markers) >= NUM_POSES:
            print("\n다음 단계:")
            print("  python3 robot_move_saved.py  # 저장된 자세로 자동 이동")
 
    except KeyboardInterrupt:
        print("\n\n!!! 사용자 중단 (Ctrl+C) !!!")
        print(f"지금까지 저장: {len(markers)}개")
        if markers:
            save_all(markers, angles)
            print(f"데이터는 {MARKERS_FILE}에 안전하게 저장됨")
 
    except Exception as e:
        print(f"\n에러: {e}")
        if markers:
            save_all(markers, angles)
            print(f"데이터는 {MARKERS_FILE}에 저장됨")
 
    finally:
        robot.disconnect()
 
 
if __name__ == "__main__":
    main()