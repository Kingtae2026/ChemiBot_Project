#!/usr/bin/env python3
"""
SterileBot - 쓰러짐 감지 모듈 (백그라운드 실행)
MediaPipe Pose 기반, CCTV 대각선 설치 환경

포트:
  9998 UDP - 비상/해제 신호 (양방향)
  9999 TCP - 카메라 영상 스트리밍 (보안실)
  9005 TCP - 통제실 WPF 비상 알림
"""

import cv2
import mediapipe as mp
import time
import argparse
import socket
import threading
import struct
import json

# ── 설정값 ──────────────────────────────────────────────────
CAMERA_INDEX      = 3
FALL_DIFF_Y       = 0.25
FALL_DIFF_Y_STRICT = 0.22
FALL_BODY_SPAN_Y  = 0.7   # 코-발목(또는 무릎) 세로차 < 0.40 이면 누운 것으로 판정
FALL_DELTA_Y      = 0.03
CONFIRM_SEC       = 2.0
VISIBILITY_MIN    = 0.3
CONDITION_REQUIRE = 2

# ── 포트 설정 ────────────────────────────────────────────────
UDP_PORT              = 9998   # 비상/해제 신호 (양방향)
TCP_PORT              = 9999   # 카메라 영상 스트리밍 (보안실)
UDP_INTERVAL          = 0.1    # 비상 신호 재전송 간격
MONITORING_PC_IP      = "192.168.0.25"  # 통제실 PC IP
MONITORING_PC_PORT    = 9005            # EmergencyListenerService 수신 포트

IDX = {
    "nose":        0,
    "l_shoulder": 11,
    "r_shoulder": 12,
    "l_hip":      23,
    "r_hip":      24,
    "l_knee":     25,
    "r_knee":     26,
    "l_ankle":    27,
    "r_ankle":    28,
}


# ── 통제실 WPF 비상 알림 (TCP) ───────────────────────────────
def send_to_monitoring(msg: str):
    """통제실 WPF EmergencyListenerService로 TCP 전송"""
    def _send():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2)
            s.connect((MONITORING_PC_IP, MONITORING_PC_PORT))
            s.sendall(msg.encode())
            s.close()
            print(f"[TCP] 통제실 알림 전송 완료: {msg}")
        except Exception as e:
            print(f"[TCP] 통제실 전송 실패: {e}")
    threading.Thread(target=_send, daemon=True).start()


# ── UDP 브로드캐스터 (통제실 → 보안실) ───────────────────────
class UdpBroadcaster:
    def __init__(self):
        self._running = False
        self._thread  = None

    def start(self, timestamp: str):
        if self._running:
            return
        self._running = True
        self._thread  = threading.Thread(
            target=self._broadcast, args=(timestamp,), daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _broadcast(self, timestamp: str):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        msg = json.dumps({"type": "emergency", "ts": timestamp}).encode()
        print(f"[UDP:{UDP_PORT}] 비상 브로드캐스트 시작")
        while self._running:
            try:
                sock.sendto(msg, ('<broadcast>', UDP_PORT))
            except Exception as e:
                print(f"[UDP] 오류: {e}")
            time.sleep(UDP_INTERVAL)
        sock.close()
        print(f"[UDP:{UDP_PORT}] 브로드캐스트 종료")


# ── UDP 수신기 (보안실 → 통제실 해제 신호) ──────────────────
class UdpReceiver:
    def __init__(self):
        self._thread = None
        self.clear_requested = False

    def start(self):
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", UDP_PORT))
        sock.settimeout(1.0)
        print(f"[UDP:{UDP_PORT}] 해제 신호 수신 대기")
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                msg = json.loads(data.decode())
                if msg.get("type") == "clear":
                    print(f"\n[UDP:{UDP_PORT}] 보안실 해제 수신 ({addr}) → 자동 리셋")
                    self.clear_requested = True
            except socket.timeout:
                continue
            except Exception:
                pass


# ── TCP 스트리밍 서버 (보안실) ────────────────────────────────
class TcpStreamServer:
    def __init__(self):
        self._clients = []
        self._lock    = threading.Lock()
        self._server  = None
        self._running = False
        self._thread  = None
        self.active   = False

    def start(self):
        if self._running:
            return
        self._running = True
        self.active   = True
        self._thread  = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self.active   = False
        with self._lock:
            for c in self._clients:
                try: c.close()
                except: pass
            self._clients.clear()
        if self._server:
            try: self._server.close()
            except: pass

    def send_frame(self, frame_bgr):
        if not self._clients:
            return
        ret, buf = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ret:
            return
        data   = buf.tobytes()
        header = struct.pack('>I', len(data))
        packet = header + data

        dead = []
        with self._lock:
            for c in self._clients:
                try:
                    c.sendall(packet)
                except:
                    dead.append(c)
            for c in dead:
                self._clients.remove(c)
                try: c.close()
                except: pass

    def _serve(self):
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._server.bind(("0.0.0.0", TCP_PORT))
            self._server.listen(5)
            self._server.settimeout(1.0)
            print(f"[TCP:{TCP_PORT}] 서버 시작 → 보안실 연결 대기 중...")
            while self._running:
                try:
                    conn, addr = self._server.accept()
                    print(f"[TCP:{TCP_PORT}] 보안실 연결! {addr}")
                    with self._lock:
                        self._clients.append(conn)
                except socket.timeout:
                    continue
                except Exception as e:
                    if self._running:
                        print(f"[TCP] 오류: {e}")
        except OSError as e:
            print(f"[TCP:{TCP_PORT}] 서버 시작 실패: {e}")
        finally:
            try: self._server.close()
            except: pass


# ── 전역 인스턴스 ─────────────────────────────────────────────
_udp_broadcaster = UdpBroadcaster()
_udp_receiver    = UdpReceiver()
_tcp_server      = TcpStreamServer()


def trigger_emergency(timestamp: str):
    print("\n" + "="*50)
    print("  ⚠  쓰러짐 확정!")
    print("="*50)
    _udp_broadcaster.start(timestamp)          # 보안실 WPF (UDP)
    send_to_monitoring("EMERGENCY:FALL_DOWN")  # 통제실 WPF (TCP)


def do_reset(detector, flash_state_ref):
    detector.fall_start = None
    detector.is_fallen  = False
    detector.alerted    = False
    _udp_broadcaster.stop()
    _udp_receiver.clear_requested = False
    # ★ send_to_monitoring("EMERGENCY_END") 제거
    # 보안실 해제 신호(UDP clear)를 WPF가 먼저 보냈으므로
    # 여기서 다시 WPF로 EMERGENCY_END를 보내면 무한루프 발생


# ── 메인 감지 클래스 ─────────────────────────────────────────
class FallDetector:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.mp_draw = mp.solutions.drawing_utils
        self.pose    = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            model_complexity=0
        )
        self.fall_start      = None
        self.is_fallen       = False
        self.alerted         = False
        self.prev_shoulder_y = None

    def get_landmark(self, lm, key):
        l = lm[IDX[key]]
        return l.x, l.y, l.visibility

    def check_conditions(self, lm):
        debug = {}

        _, l_sh_y, l_sh_v = self.get_landmark(lm, "l_shoulder")
        _, r_sh_y, r_sh_v = self.get_landmark(lm, "r_shoulder")
        _, l_hp_y, l_hp_v = self.get_landmark(lm, "l_hip")
        _, r_hp_y, r_hp_v = self.get_landmark(lm, "r_hip")
        l_sh_x, _, _      = self.get_landmark(lm, "l_shoulder")
        r_sh_x, _, _      = self.get_landmark(lm, "r_shoulder")

        vis_shoulder = (l_sh_v + r_sh_v) / 2
        vis_hip      = (l_hp_v + r_hp_v) / 2

        if vis_shoulder < VISIBILITY_MIN or vis_hip < VISIBILITY_MIN:
            return 0, [False, False, False], {"error": "가시성 부족"}

        shoulder_y = (l_sh_y + r_sh_y) / 2
        hip_y      = (l_hp_y + r_hp_y) / 2

        # cond1: 어깨-엉덩이 높이차가 작음 (누운 자세 핵심 신호)
        diff_y = abs(shoulder_y - hip_y)
        cond1  = diff_y < FALL_DIFF_Y
        debug["diff_y"] = diff_y

        # cond2: 상체(코)와 하체(발목/무릎)의 세로 차이가 작음
        #         서면 큰 값, 누우면 작은 값 → 카메라 각도/머리방향에 강건
        nose_x, nose_y_val, nose_v = self.get_landmark(lm, "nose")
        _, l_ak_y, l_ak_v = self.get_landmark(lm, "l_ankle")
        _, r_ak_y, r_ak_v = self.get_landmark(lm, "r_ankle")
        _, l_kn_y, l_kn_v = self.get_landmark(lm, "l_knee")
        _, r_kn_y, r_kn_v = self.get_landmark(lm, "r_knee")

        # 발목 가시성이 충분하면 발목, 아니면 무릎 사용
        if (l_ak_v + r_ak_v) / 2 >= VISIBILITY_MIN:
            lower_y = (l_ak_y + r_ak_y) / 2
        else:
            lower_y = (l_kn_y + r_kn_y) / 2

        cond2 = diff_y < FALL_DIFF_Y_STRICT
        debug["nose_y"]    = nose_y_val
        # debug["body_span"] = body_span_y

        # cond3: 어깨가 급격히 내려감 (쓰러지는 순간)
        if self.prev_shoulder_y is not None:
            delta_y = shoulder_y - self.prev_shoulder_y
            cond3   = delta_y > FALL_DELTA_Y
        else:
            delta_y = 0
            cond3   = False
        debug["delta_y"] = delta_y

        self.prev_shoulder_y = shoulder_y

        count = sum([cond1, cond2, cond3])
        return count, [cond1, cond2, cond3], debug

    def update(self, lm):
        count, conds, debug = self.check_conditions(lm)

        dy  = debug.get("diff_y",    0)
        bs  = debug.get("body_span", 0)
        dlt = debug.get("delta_y",   0)

        with _tcp_server._lock:
            n = len(_tcp_server._clients)
        conn_str = f"TCP: {'스트리밍 ✅' if n > 0 else '대기 ⏳'}"
        print(f"dy={dy:.3f}(<{FALL_DIFF_Y}) span={bs:.3f}(<{FALL_BODY_SPAN_Y}) 조건={count}/{CONDITION_REQUIRE} [{conn_str}]", end="\r")

        if count >= CONDITION_REQUIRE:
            if self.fall_start is None:
                self.fall_start = time.time()

            elapsed = time.time() - self.fall_start
            pct     = min(1.0, elapsed / CONFIRM_SEC)

            if elapsed >= CONFIRM_SEC and not self.alerted:
                self.is_fallen = True
                self.alerted   = True
                ts = time.strftime("%Y-%m-%d %H:%M:%S")
                trigger_emergency(ts)

            if self.is_fallen:
                return "FALL CONFIRMED", (0, 0, 220), 1.0, True
            else:
                remaining = CONFIRM_SEC - elapsed
                return f"FALL SUSPECTED ({remaining:.1f}s)", (0, 140, 255), pct, False
        else:
            self.fall_start = None
            self.is_fallen  = False
            self.alerted    = False
            txt = f"NORMAL  dy={dy:.2f} span={bs:.2f} ({count}/{CONDITION_REQUIRE})"
            return txt, (0, 200, 80), 0.0, False


# ── 실행 ─────────────────────────────────────────────────────
def main(camera_src):
    cap = cv2.VideoCapture(camera_src, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    detector = FallDetector()
    mp_draw  = mp.solutions.drawing_utils
    mp_pose  = mp.solutions.pose

    _udp_receiver.start()
    _tcp_server.start()

    flash_state  = False
    last_flash_t = time.time()

    print("[Fall Detection Start - Background Mode]")
    print(f"  Camera     : {camera_src}")
    print(f"  UDP        : {UDP_PORT} (비상/해제 신호)")
    print(f"  TCP        : {TCP_PORT} (영상 스트리밍 → 보안실)")
    print(f"  Monitoring : {MONITORING_PC_IP}:{MONITORING_PC_PORT} (통제실 WPF)")
    print(f"  Quit: Ctrl+C\n")

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] No frame")
                break

            # 보안실 해제 신호 수신 시 자동 리셋
            if _udp_receiver.clear_requested:
                do_reset(detector, None)
                flash_state = False
                print("\n[AUTO RESET] 보안실 해제 신호 → 자동 리셋 완료")

            h, w = frame.shape[:2]
            rgb  = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res  = detector.pose.process(rgb)

            status, color, pct, fallen = "NO PERSON", (150, 150, 150), 0.0, False

            if res.pose_landmarks:
                lm = res.pose_landmarks.landmark

                try:
                    mp_draw.draw_landmarks(
                        frame, res.pose_landmarks,
                        mp_pose.POSE_CONNECTIONS,
                        mp_draw.DrawingSpec(color=(0,255,150), thickness=2, circle_radius=3),
                        mp_draw.DrawingSpec(color=(0,180,100), thickness=2)
                    )
                except Exception:
                    pass

                l_sh = lm[IDX["l_shoulder"]]
                r_sh = lm[IDX["r_shoulder"]]
                l_hp = lm[IDX["l_hip"]]
                r_hp = lm[IDX["r_hip"]]
                sh_y = int((l_sh.y + r_sh.y) / 2 * h)
                hp_y = int((l_hp.y + r_hp.y) / 2 * h)
                cv2.line(frame, (0, sh_y), (w, sh_y), (255, 200, 0), 1)
                cv2.line(frame, (0, hp_y), (w, hp_y), (0, 200, 255), 1)

                status, color, pct, fallen = detector.update(lm)

            # TCP 프레임 전송 (보안실 상시 스트리밍)
            if _tcp_server.active:
                _tcp_server.send_frame(frame)
             # ★ 디버깅용 창 표시 (테스트 후 제거)
            # cv2.imshow("Fall Detection Debug", frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
    except KeyboardInterrupt:
        print("\n[종료]")
    finally:
        _udp_broadcaster.stop()
        _tcp_server.stop()
        cap.release()
        # cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default=CAMERA_INDEX,
                        help="카메라 인덱스 또는 스트리밍 URL")
    args = parser.parse_args()

    try:
        src = int(args.camera)
    except ValueError:
        src = args.camera

    main(src)