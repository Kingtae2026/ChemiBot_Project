"""
lab_intercom.py — 실험실 인터컴 (로봇 팔 라즈베리파이에서 실행)

[포트]
  UDP 10000 — 양방향 공용 포트

[볼륨 제어]
  인터컴 음성 수신 중 → 대피 안내음성 볼륨 50%로 낮춤
  수신 종료 후 2초   → 원래 볼륨(100%)으로 복원
  ★ RMS 임계값 이상 음성일 때만 duck (잔음 무시)

[실행]
  python3 lab_intercom.py          (인터컴 시작)
  python3 lab_intercom.py --list   (장치 목록만 출력)
"""

import socket
import struct
import math
import sys
import threading
import time

import sounddevice as sd

# ════════════════════════════════════════════════════════════
#  설정 구역
# ════════════════════════════════════════════════════════════

SECURITY_PC_IP  = "192.168.0.6"   # 보안실 PC IP
VOICE_PORT      = 10000            # 양방향 공용 포트

INPUT_DEVICE    = None             # Pi 마이크 (None=기본)
OUTPUT_DEVICE   = 1                # bcm2835 Headphones (3.5mm AUX)

RATE            = 16000
CHANNELS        = 1
BLOCK           = 640

SILENCE_TIMEOUT = 2.0              # 마지막 패킷 수신 후 복원 대기 시간 (초)
PI_SERVER       = "http://127.0.0.1:5001"  # 로컬 Flask 서버

# ★ RMS 임계값 — 이 값 이상일 때만 duck (0~32768)
# 잔음/노이즈는 보통 100~300, 사람 음성은 500 이상
VOICE_THRESHOLD = 5000

# ════════════════════════════════════════════════════════════
#  볼륨 제어 (Pi 로컬 서버 HTTP 호출)
# ════════════════════════════════════════════════════════════

_is_ducked    = False
_last_recv_t  = 0.0
_duck_lock    = threading.Lock()


def call_server(path: str):
    try:
        import urllib.request
        urllib.request.urlopen(f"{PI_SERVER}{path}", timeout=1)
    except Exception:
        pass


def duck_watchdog():
    """인터컴 패킷이 끊기면 볼륨 복원"""
    global _is_ducked
    while True:
        time.sleep(0.5)
        with _duck_lock:
            if _is_ducked and (time.time() - _last_recv_t) > SILENCE_TIMEOUT:
                _is_ducked = False
                call_server("/unduck_alarm")
                print("[볼륨] 인터컴 종료 감지 → 100% 복원")


def calc_rms(data: bytes) -> float:
    """PCM int16 바이트 데이터의 RMS 계산"""
    samples = len(data) // 2
    if samples == 0:
        return 0.0
    total = sum(struct.unpack_from('<h', data, i * 2)[0] ** 2 for i in range(samples))
    return math.sqrt(total / samples)


def on_packet_received(data: bytes):
    """패킷 수신 시마다 호출 — RMS 임계값 이상일 때만 볼륨 낮추기"""
    global _is_ducked, _last_recv_t

    rms = calc_rms(data)

    # 잔음(RMS < VOICE_THRESHOLD)이면 무시
    if rms < VOICE_THRESHOLD:
        return

    with _duck_lock:
        _last_recv_t = time.time()
        if not _is_ducked:
            _is_ducked = True
            call_server("/duck_alarm")
            print(f"[볼륨] 음성 감지 (RMS={rms:.0f}) → 50% 낮춤")
        else:
            # 이미 duck 중이면 타이머만 갱신
            _last_recv_t = time.time()


# ════════════════════════════════════════════════════════════
#  장치 목록
# ════════════════════════════════════════════════════════════

def print_devices():
    print("\n────── 오디오 장치 목록 ──────")
    for i, dev in enumerate(sd.query_devices()):
        kinds = []
        if dev["max_input_channels"] > 0:
            kinds.append("마이크")
        if dev["max_output_channels"] > 0:
            kinds.append("스피커")
        if kinds:
            print(f"  [{i}] {dev['name']}  ({'/'.join(kinds)})")
    print("──────────────────────────────")
    print("위 번호를 INPUT_DEVICE / OUTPUT_DEVICE 에 지정하세요.\n")


# ════════════════════════════════════════════════════════════
#  스피커 루프 (보안실 음성 수신 → 재생)
# ════════════════════════════════════════════════════════════

def speaker_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", VOICE_PORT))
    print(f"[스피커] UDP {VOICE_PORT} 수신 대기 중")

    stream = sd.RawOutputStream(samplerate=RATE, channels=CHANNELS,
                                 dtype="int16", device=OUTPUT_DEVICE)
    stream.start()

    while True:
        try:
            data, _ = sock.recvfrom(65536)
            on_packet_received(data)   # ★ RMS 기반 볼륨 제어
            stream.write(data)
        except Exception as e:
            print(f"[스피커] 오류: {e}")


# ════════════════════════════════════════════════════════════
#  마이크 루프 (Pi 마이크 → 보안실 전송)
# ════════════════════════════════════════════════════════════

def mic_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[마이크] {SECURITY_PC_IP}:{VOICE_PORT} 로 전송 시작")

    def callback(indata, frames, t, status):
        if status:
            print(f"[마이크] {status}")
        try:
            sock.sendto(bytes(indata), (SECURITY_PC_IP, VOICE_PORT))
        except Exception:
            pass

    with sd.RawInputStream(samplerate=RATE, channels=CHANNELS, dtype="int16",
                           blocksize=BLOCK, device=INPUT_DEVICE,
                           callback=callback):
        threading.Event().wait()


# ════════════════════════════════════════════════════════════
#  메인
# ════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print_devices()
    if "--list" in sys.argv:
        sys.exit(0)

    print("=" * 40)
    print("  실험실 인터컴 시작")
    print(f"  수신(보안실→Pi): UDP {VOICE_PORT}")
    print(f"  송신(Pi→보안실): UDP {VOICE_PORT} → {SECURITY_PC_IP}")
    print(f"  음성 감지 임계값: RMS {VOICE_THRESHOLD}")
    print("=" * 40)

    threading.Thread(target=duck_watchdog, daemon=True).start()
    threading.Thread(target=speaker_loop, daemon=True).start()

    try:
        mic_loop()
    except KeyboardInterrupt:
        print("\n종료")
        call_server("/unduck_alarm")  # 종료 시 볼륨 복원
