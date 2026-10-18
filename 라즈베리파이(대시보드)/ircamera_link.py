import socket
import struct
import threading
import time

import serial
from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QImage

DEFAULT_SERIAL_PORT = "/dev/ircamera"  # udev 심볼릭 링크(/etc/udev/rules.d/99-sew-boards.rules) - ttyACM 번호는 꽂는 순서에 따라 바뀌므로 시리얼 번호로 고정
DEFAULT_SERIAL_BAUD = 921600
DEFAULT_TCP_HOST = "0.0.0.0"
DEFAULT_TCP_PORT = 8090

MODE_BUS_OCR = 0x01
MODE_BRAILLE = 0x02

# TOF(방향+거리)와 AI모드(0xB0, 방향+객체종류)가 같은 방향에서 동시에 뭔가를 보고했을 때만
# RPi가 보내는 확인 트리거 - "이 방향에 있는 게 정확히 이 객체다"를 ESP32에 확정해줘서
# 스피커로 그대로 말하게 한다(2026-08-25). 페이로드는 [object_code 1B] - ESP32가 자체
# s_obs_result로 추측하면 RPi 왕복 시차 동안 다음 300ms 추론 사이클이 돌아 이미 다른
# 객체로 바뀌어있을 수 있어서, RPi가 판단한 시점의 값을 그대로 실어 보낸다.
CMD_OBSTACLE_CONFIRM = 0x10

# 길안내 모드 중 음성 안내 트리거(2026-08-26) - 스피커가 ircamera 쪽에 있어서 guide 보드가 아니라
# 여기로 보낸다. 페이로드는 [subcode 1B]뿐. GUIDE_VOICE_CONFIRM은 목적지에 상관없이 항상
# "버스정류장으로 길안내를 시작합니다"만 재생함(학교 쪽 문구는 아직 구현 안 함 - 의도적으로 그대로 둠).
CMD_GUIDE_VOICE = 0x03
GUIDE_VOICE_ENTER = 0x00       # 길안내모드 진입 - "길안내모드전환" 재생
GUIDE_VOICE_SCHOOL = 0x01      # 목적지 순환: 학교 - "학교" 재생
GUIDE_VOICE_BUS_STOP = 0x02    # 목적지 순환: 버스정류장 - "버스정류장" 재생
GUIDE_VOICE_CONFIRM = 0x03     # 확정 - "버스정류장으로 길안내를 시작합니다" 재생(고정 문구)

# 목적지 도착 트리거(2026-08-26, 페이로드 없음, 단독 1바이트) - "목적지에 도착했습니다" 재생.
# guide 보드가 GPS로 도착 판정을 하고 0xAB(EVT_ARRIVED)를 RPi에 보내오면, dashboard.py가
# 그 시점에 이걸 ircamera로 릴레이한다(guide 보드엔 스피커가 없어서).
CMD_GUIDE_ARRIVED = 0x05

# 긴급상황 알림 트리거(2026-08-26, 페이로드 없음, 단독 1바이트) - "긴급상황발생 119신고요청" 재생.
# 정확히 어떤 조건에서 쏴야 하는지(버튼 롱프레스/낙상감지/별도 키워드 등)는 아직 안 정해져서,
# 일단 기존 window.trigger_emergency() 트리거 지점(비상 버튼 클릭 + 키워드 "four" 인식)에 같이 물려둠.
CMD_EMERGENCY_ALERT = 0x04

# 길안내 경로 이벤트 음성 트리거(2026-08-26, 각각 페이로드 없음, 단독 1바이트) - guide 보드가
# 보내는 원본 신호(smartcane OHT 브랜치 main.c 기준: 좌회전 0x03/우회전 0x04/횡단보도 0x05/계단 0x06)를
# 그 값 그대로 릴레이하면 위 CMD_GUIDE_VOICE(0x03)/CMD_EMERGENCY_ALERT(0x04)/CMD_GUIDE_ARRIVED(0x05)와
# 충돌하므로, ircamera 쪽에서 안 쓰는 값으로 번역해서 보낸다.
CMD_GUIDE_TURN_LEFT = 0x06
CMD_GUIDE_TURN_RIGHT = 0x07
CMD_GUIDE_CROSSWALK = 0x08
CMD_GUIDE_STAIRS = 0x09

# 낙상감지 반복 알림 시작/정지 트리거(2026-08-30, 페이로드 없음, 단독 1바이트) - FD_KWS 보드가
# 낙상 이벤트를 감지(0xFF)하면 START를 보내 ESP32가 자체적으로 반복 알림(음/음성, ESP32 쪽에서
# 별도 구현 예정)을 계속 울리게 하고, 유예시간 안에 다시 집어들어 안전 판정(0xFE)되거나 위급으로
# 확정(0xFD, 이 경우 CMD_EMERGENCY_ALERT로 전환)되면 STOP을 보내 멈춘다.
CMD_FALL_ALERT_START = 0x0A
CMD_FALL_ALERT_STOP = 0x0B

# 키워드스파팅 인식 실패/timeout 시 "다시 말씀해주세요" 재생 트리거(2026-10-16, 페이로드 없음, 단독 1바이트).
# 예전엔 ESP32에 클립이 없어서 이어폰으로만 냈음 - 지금은 ESP32 스피커 + voice_cue로 이어폰 둘 다.
CMD_RETRY_PROMPT = 0x0C

PKT_OBSTACLE = 0xB0
PKT_BUS = 0xB1
PKT_BRAILLE = 0xB2
# ESP32가 장애물모드에 진입할 때마다(버스/점자블록에서 복귀, 부팅 포함) 보내는 신호(페이로드 없음, 2026-08-26).
PKT_REVERT = 0xB3
# 정류장 체크 결과(버스 후보, 2026-10-16). 버스모드 명령 후 ESP32가 버스모드에 들어가기 직전에 1번만 보낸다:
#   [count 1B] + count x ([len 1B][노선번호 UTF-8][도착초 uint16 LE])
# count=0 = 도착 예정 버스 없음(ESP32가 "도착버스없음" 재생, 버스모드 진입 안 함).
# 도착초 0xFFFF = 모름(GPS/정류장DB 없어서 고정 리스트 폴백 - ESP32도 후보 안내 음성 안 냄).
PKT_BUS_CANDIDATES = 0xB4
BUS_ARR_UNKNOWN = 0xFFFF
# ESP32 voice.bin에 클립이 있는 노선번호(voice_clips.h) - ESP32는 이 노선들만 소리 내서 안내한다.
BUS_VOICE_KEYS = {"937", "503", "410", "410-1", "북구2"}

OBJECT_NAMES = {0x01: "자전거", 0x02: "킥보드", 0x03: "볼라드", 0x04: "사람", 0x05: "미상"}
OBJECT_CODES = {name: code for code, name in OBJECT_NAMES.items()}  # CMD_OBSTACLE_CONFIRM 페이로드용 역매핑
DIRECTION_NAMES = {0x01: "왼쪽", 0x02: "중앙", 0x03: "오른쪽"}
# 점자블록(0xB2) dir 바이트도 같은 코드를 쓴다(0=없음/미상 - main.cpp send_braille_result, 2026-10-16).
# 근접확정 시 ESP32가 재생하는 클립 - left/right 외에는 전부 "정면"(main.cpp와 동일).
BRAILLE_NEAR_VOICE_KEYS = {0x01: "왼쪽에점자블록", 0x03: "오른쪽에점자블록"}
BRAILLE_NEAR_VOICE_DEFAULT = "정면에점자블록"

# voice_cue로 내보내는 음성 클립 키 - ircamera main.cpp의 voice_clips.h 키와 같고, sew/data/<키>.wav와도
# 같다. ESP32가 자기 스피커로 재생하는 시점에 RPi(이어폰)도 같이 재생하기 위함(earphone_audio.py).
GUIDE_VOICE_KEYS = {
    GUIDE_VOICE_ENTER: "길안내모드전환",
    GUIDE_VOICE_SCHOOL: "학교",
    GUIDE_VOICE_BUS_STOP: "버스정류장",
    GUIDE_VOICE_CONFIRM: "버스정류장으로길안내를시작합니다",
}
# 페이로드 없는 단독 1바이트 명령 중 ESP32가 받는 즉시 클립을 재생하는 것들.
# 횡단보도(0x08)는 wav가 없어서 ESP32도 재생 안 함 -> 여기서도 제외. 낙상 알림(0x0A/0x0B)도 클립 아님.
SIMPLE_CMD_VOICE_KEYS = {
    CMD_GUIDE_ARRIVED: "목적지에도착했습니다",
    CMD_EMERGENCY_ALERT: "긴급상황발생119신고요청",
    CMD_GUIDE_TURN_LEFT: "좌회전",
    CMD_GUIDE_TURN_RIGHT: "우회전",
    CMD_GUIDE_STAIRS: "계단",
    CMD_RETRY_PROMPT: "다시말씀해주세요",
}
# ESP32가 모드에 실제로 들어가면서 재생하는 클립(main.cpp enter_mode()).
MODE_ENTER_VOICE_KEYS = {
    "obstacle": "장애물모드전환",
    "bus_ocr": "버스탐지모드전환",
    "braille": "점자블록모드전환",
}

# ESP32가 버스/점자블록 모드에서 장애물모드로 복귀할 때가 됐는지는 전적으로 ESP32가 판단한다
# (몇 번 매칭됐는지, 정류장 게이팅에서 몇 대를 더 기다려야 하는지, 타임아웃 등은 RPi가 알 수 없는
# ESP32쪽 상태임 - smartcane 저장소 SEW 브랜치 main.cpp의 s_bus_target_count/matched_count,
# kBusModeTimeoutUs 참고). 그래서 RPi는 자체 타이머 없이 ESP32 패킷만 보고 화면을 되돌린다
# (2026-10-16 - 예전 30초 안전망 타이머는 ESP32 쪽 타임아웃이 60초로 늘면서 화면을 먼저 되돌려버리는
# 문제가 생겨서 제거):
#   - PKT_REVERT(0xB3): ESP32가 장애물모드에 진입함 -> 즉시 복귀
#   - PKT_OBSTACLE(0xB0): ESP32는 장애물모드에서만 이걸 보낸다. 버스/점자블록 명령을 보냈는데도
#     이게 계속 오면 ESP32가 모드에 안 들어간 것(버스: 정류장 게이팅에서 도착 버스 없음 / 명령 유실)
#     -> 복귀. 단 명령 직후엔 ESP32가 명령을 처리하기 전에 이미 보낸 0xB0이 올 수 있어서
#     MODE_CMD_GRACE_S 동안은 무시한다.
#   - 시리얼 끊김: 복귀
MODE_CMD_GRACE_S = 2.0

# 영상 스트림(길이 헤더 + JPEG) 관련 상수.
# ESP32가 모드 전환 중 프레임 전송을 멈추거나 프레이밍이 깨진 데이터를 보내면
# recv()가 무한정 블록되거나 length 헤더를 잘못 해석해 스트림이 영구히 어긋날 수 있다.
# 그래서 (1) 응답이 없으면 타임아웃으로 재접속을 유도하고, (2) 프레임이 손상됐으면
# 다음 유효한 프레임 경계를 스트림에서 찾아 재동기화한다.
RECV_TIMEOUT_S = 3.0
MAX_FRAME_BYTES = 400_000  # 정상 프레임 크기보다 넉넉한 상한(이보다 크면 헤더 손상으로 간주)
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
RESYNC_SCAN_LIMIT = MAX_FRAME_BYTES * 2  # 이만큼 스캔해도 못 찾으면 포기하고 재접속

# 화면 표시 갱신 주기. TCP 수신 스레드는 프레임을 받는 족족 디코드해서 "최신 프레임" 한 장만
# 덮어쓰고, 이 타이머(메인 스레드)가 그 주기로 화면에 반영한다. 이렇게 분리하지 않고 프레임마다
# 바로 emit하면, GUI 스레드가 잠깐이라도 밀렸을 때 emit이 큐에 쌓여서 "오래된 프레임들을 순서대로
# 다 보여주며 따라잡는" 형태로 지연이 계속 누적된다. 최신 프레임만 남기면 한 번 밀려도 다음 틱에
# 바로 실시간으로 복귀한다.
DISPLAY_INTERVAL_MS = 50  # ~20fps 상한 (ESP32 전송 속도 ~5fps보다 넉넉히 높게)


class IrCameraLink(QObject):
    frame_received = Signal(QImage)
    obstacle_detected = Signal(str, str, int)   # object_name, direction_name, confidence(0~100)
    bus_result = Signal(str, int, bool)          # text, confidence(0~100), matched(사전 후보 확정 여부)
    bus_candidates = Signal(list)                # [(노선번호, 도착초 또는 None)], 빈 리스트 = 도착 예정 버스 없음
    braille_result = Signal(bool, int, bool, str)  # present, confidence(0~100), near, direction_name("왼쪽"/"중앙"/"오른쪽"/"-")
    mode_changed = Signal(str)                   # "obstacle" | "bus_ocr" | "braille"
    # ESP32가 지금 자기 스피커로 재생하는(했을) 음성 클립 키. 백그라운드 스레드에서 emit됨.
    voice_cue = Signal(str)
    serial_connected = Signal(bool)
    video_client_connected = Signal(bool)

    def __init__(self, serial_port=DEFAULT_SERIAL_PORT, serial_baud=DEFAULT_SERIAL_BAUD,
                 tcp_host=DEFAULT_TCP_HOST, tcp_port=DEFAULT_TCP_PORT, parent=None):
        super().__init__(parent)
        self.serial_port = serial_port
        self.serial_baud = serial_baud
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port

        self._ser = None
        self._ser_lock = threading.Lock()
        self._active_mode = None   # RPi가 명령해서 화면에 띄운 모드("bus_ocr"/"braille"), 장애물이면 None
        self._mode_cmd_at = 0.0    # 마지막 버스/점자블록 명령 시각(time.monotonic) - MODE_CMD_GRACE_S 판정용
        self._stop = threading.Event()
        self._last_gps = None  # (lat, lon) or None — set_gps_fix()로 갱신, 버스모드 게이팅용
        # ESP32가 받은 패킷으로 추정한 ESP32의 실제 모드. _active_mode(RPi가 명령한 모드, 화면용)와 별개다 -
        # 명령을 보내도 ESP32가 안 들어갈 수 있어서(버스: 정류장 게이팅에서 도착 버스 없음 /
        # 이미 그 모드였음 / 전송 실패) 모드 전환 음성은 이 값이 실제로 바뀔 때만 낸다.
        # None = 아직 모름(연결 직후) - 첫 패킷으로 조용히 채우고 소리는 안 낸다.
        self._esp_mode = None

        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._frame_dirty = False
        self._display_timer = None

    # ---------------- 시작/종료 ----------------
    def start(self):
        threading.Thread(target=self._serial_loop, daemon=True).start()
        threading.Thread(target=self._tcp_server_loop, daemon=True).start()

        # 메인 스레드(이 인스턴스가 생성된 스레드)에서 도는 타이머라야 frame_received가
        # direct connection으로 즉시 처리된다. 백그라운드 스레드에서 만들면 다시 큐잉된다.
        self._display_timer = QTimer(self)
        self._display_timer.timeout.connect(self._emit_latest_frame)
        self._display_timer.start(DISPLAY_INTERVAL_MS)

    def _emit_latest_frame(self):
        with self._frame_lock:
            if not self._frame_dirty:
                return
            image = self._latest_frame
            self._frame_dirty = False
        self.frame_received.emit(image)

    def stop(self):
        self._stop.set()
        if self._display_timer:
            self._display_timer.stop()

    def set_gps_fix(self, lat: float, lon: float):
        """dashboard.py가 gps_link의 fix_changed에 연결해서 호출한다. 버스모드 진입 시
        정류장 게이팅용으로 ESP32에 같이 실어 보낼 최신 좌표를 들고 있기 위함."""
        self._last_gps = (lat, lon)

    # ---------------- 모드 전환 명령 (키워드스파팅 릴레이 / 수동 버튼 둘 다 이걸 호출) ----------------
    def request_bus_mode(self):
        # UI 갱신(칩 색상/타이머)은 즉시 반영하고, 시리얼 전송은 백그라운드 스레드로 보낸다.
        # ser.write()가 (ESP32가 UART를 못 받아주는 동안) 블로킹되더라도 이 메서드는
        # 버튼 클릭 슬롯 등 GUI 메인 스레드에서 호출되므로, 여기서 막히면 창 전체가 먹통이 된다.
        self._active_mode = "bus_ocr"
        self._mode_cmd_at = time.monotonic()
        self.mode_changed.emit("bus_ocr")
        # main.cpp(2026-08-23)가 MODE_CMD_BUS 뒤에 lat/lon(float32 LE 4B씩, 총 8B)이
        # 이어붙어 오는 걸 기대하도록 바뀌어서(정류장 게이팅용), 커맨드 바이트와 GPS
        # 페이로드를 한 번의 write()로 같이 보낸다. fix가 아직 없으면 0x01만 보내고,
        # ESP32 쪽이 300ms 타임아웃 후 게이팅 스킵하고 바로 진입하도록 둔다.
        threading.Thread(target=self._send_bus_mode_cmd, args=(self._last_gps,), daemon=True).start()

    def request_braille_mode(self):
        self._active_mode = "braille"
        self._mode_cmd_at = time.monotonic()
        self.mode_changed.emit("braille")
        threading.Thread(target=self._send_mode_byte, args=(MODE_BRAILLE,), daemon=True).start()

    # ---------------- 장애물 확인 트리거 (TOF+AI모드 방향 일치 시 dashboard.py가 호출) ----------------
    def send_obstacle_confirm(self, object_code: int):
        threading.Thread(target=self._send_obstacle_confirm_cmd, args=(object_code,), daemon=True).start()

    def _send_obstacle_confirm_cmd(self, object_code: int):
        payload = bytes([CMD_OBSTACLE_CONFIRM, object_code])
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(payload)
                    if object_code in OBJECT_NAMES and object_code != OBJECT_CODES["미상"]:
                        self.voice_cue.emit(OBJECT_NAMES[object_code])
                except Exception as e:
                    print(f"[ircamera] 장애물 확인 트리거 전송 실패: {e}")
            else:
                print("[ircamera] 시리얼 연결 안 됨, 장애물 확인 트리거 스킵")

    # ---------------- 길안내 음성 안내 트리거 (dashboard.py의 guide 진입/순환/확정 지점에서 호출) ----------------
    def send_guide_voice(self, subcode: int):
        threading.Thread(target=self._send_guide_voice_cmd, args=(subcode,), daemon=True).start()

    def _send_guide_voice_cmd(self, subcode: int):
        payload = bytes([CMD_GUIDE_VOICE, subcode])
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(payload)
                    if subcode in GUIDE_VOICE_KEYS:
                        self.voice_cue.emit(GUIDE_VOICE_KEYS[subcode])
                except Exception as e:
                    print(f"[ircamera] 길안내 음성 트리거 전송 실패: {e}")
            else:
                print("[ircamera] 시리얼 연결 안 됨, 길안내 음성 트리거 스킵")

    # ---------------- 목적지 도착 / 긴급상황 음성 트리거 (둘 다 페이로드 없는 단독 1바이트) ----------------
    def send_guide_arrived(self):
        threading.Thread(target=self._send_mode_byte, args=(CMD_GUIDE_ARRIVED,), daemon=True).start()

    def send_emergency_alert(self):
        threading.Thread(target=self._send_mode_byte, args=(CMD_EMERGENCY_ALERT,), daemon=True).start()

    # ---------------- 길안내 경로 이벤트 음성 트리거 (넷 다 페이로드 없는 단독 1바이트) ----------------
    def send_guide_turn_left(self):
        threading.Thread(target=self._send_mode_byte, args=(CMD_GUIDE_TURN_LEFT,), daemon=True).start()

    def send_guide_turn_right(self):
        threading.Thread(target=self._send_mode_byte, args=(CMD_GUIDE_TURN_RIGHT,), daemon=True).start()

    def send_guide_crosswalk(self):
        threading.Thread(target=self._send_mode_byte, args=(CMD_GUIDE_CROSSWALK,), daemon=True).start()

    def send_guide_stairs(self):
        threading.Thread(target=self._send_mode_byte, args=(CMD_GUIDE_STAIRS,), daemon=True).start()

    # ---------------- 낙상감지 반복 알림 시작/정지 (dashboard.py의 fall_event 핸들러에서 호출) ----------------
    def send_fall_alert_start(self):
        threading.Thread(target=self._send_mode_byte, args=(CMD_FALL_ALERT_START,), daemon=True).start()

    def send_fall_alert_stop(self):
        threading.Thread(target=self._send_mode_byte, args=(CMD_FALL_ALERT_STOP,), daemon=True).start()

    # ---------------- 키워드 인식 실패 안내 (dashboard.py의 on_keyword_recognized에서 호출) ----------------
    def send_retry_prompt(self):
        threading.Thread(target=self._send_mode_byte, args=(CMD_RETRY_PROMPT,), daemon=True).start()

    def _revert_to_obstacle(self):
        self._active_mode = None
        self.mode_changed.emit("obstacle")

    def _send_mode_byte(self, value: int):
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(bytes([value]))
                    if value in SIMPLE_CMD_VOICE_KEYS:
                        self.voice_cue.emit(SIMPLE_CMD_VOICE_KEYS[value])
                except Exception as e:
                    print(f"[ircamera] 모드 전송 실패: {e}")
            else:
                print("[ircamera] 시리얼 연결 안 됨, 모드 전송 스킵")

    def _send_bus_mode_cmd(self, gps):
        payload = bytes([MODE_BUS_OCR])
        if gps is not None:
            lat, lon = gps
            payload += struct.pack("<ff", lat, lon)
        else:
            print("[ircamera] GPS fix 없음 — 정류장 게이팅 없이 0x01만 전송")
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(payload)
                except Exception as e:
                    print(f"[ircamera] 버스모드 명령 전송 실패: {e}")
            else:
                print("[ircamera] 시리얼 연결 안 됨, 모드 전송 스킵")

    # ---------------- 시리얼 수신 루프 ----------------
    def _serial_loop(self):
        while not self._stop.is_set():
            try:
                with self._ser_lock:
                    # write_timeout을 안 주면 ser.write()가 무한 블로킹될 수 있는데, 그러면
                    # 그 write를 감싼 _ser_lock도 계속 잡혀 있어서 여기(재연결 시도)까지 같이 멈춘다.
                    self._ser = serial.Serial(self.serial_port, self.serial_baud, timeout=1, write_timeout=1)
                self._esp_mode = None  # 끊긴 사이 ESP32 모드가 바뀌었을 수 있음 - 다시 첫 패킷으로 채운다
                self.serial_connected.emit(True)
                print(f"[ircamera] 시리얼 연결됨: {self.serial_port}")
                self._read_packets()
            except Exception as e:
                print(f"[ircamera] 시리얼 연결 실패: {e}")
                self.serial_connected.emit(False)
                self._stop.wait(2.0)
            finally:
                # 끊긴 동안 ESP32 모드를 알 수 없고 0xB3도 못 받으니, 버스/점자블록 화면에 갇히지 않게 되돌린다
                if self._active_mode is not None:
                    self._revert_to_obstacle()

    def _read_packets(self):
        ser = self._ser
        while not self._stop.is_set():
            head = ser.read(1)
            if not head:
                continue
            msg_type = head[0]
            try:
                if msg_type == PKT_OBSTACLE:
                    self._handle_obstacle(ser)
                elif msg_type == PKT_BUS:
                    self._handle_bus(ser)
                elif msg_type == PKT_BRAILLE:
                    self._handle_braille(ser)
                elif msg_type == PKT_REVERT:
                    self._handle_revert()
                elif msg_type == PKT_BUS_CANDIDATES:
                    self._handle_bus_candidates(ser)
                else:
                    # 0xB0~0xB2가 아니면 ESP32의 esp_log_set_vprintf 후킹으로 얹혀오는
                    # 텍스트 로그 줄이다("I (12345) ircam: ..." 등). 개행까지 읽어서 출력만 한다 —
                    # 결과 패킷과 같은 USB-CDC 라인을 공유하지만 첫 바이트가 겹치지 않아 구분 가능.
                    self._handle_log_line(ser, head)
            except (TimeoutError, serial.SerialException) as e:
                print(f"[ircamera] 시리얼 끊김/timeout: {e}")
                self.serial_connected.emit(False)
                return  # 바깥 _serial_loop에서 재연결

    def _read_exact(self, ser, n):
        buf = b""
        while len(buf) < n:
            chunk = ser.read(n - len(buf))
            if not chunk:
                raise TimeoutError(f"시리얼 읽기 timeout ({n}바이트 기대, {len(buf)}바이트 받음)")
            buf += chunk
        return buf

    def _handle_obstacle(self, ser):
        payload = self._read_exact(ser, 3)
        object_code, direction_code, confidence = payload[0], payload[1], payload[2]
        obj_name = OBJECT_NAMES.get(object_code, "미상")
        dir_name = DIRECTION_NAMES.get(direction_code, "-")
        self._note_esp_mode("obstacle", announce=False)  # 장애물모드 진입 안내는 0xB3에서만
        # 버스/점자블록을 명령했는데 0xB0이 계속 옴 = ESP32가 그 모드에 안 들어감(위 MODE_CMD_GRACE_S 주석 참고)
        if self._active_mode is not None and time.monotonic() - self._mode_cmd_at > MODE_CMD_GRACE_S:
            print(f"[ircamera] {self._active_mode} 명령 후에도 ESP32가 장애물모드 - 화면 복귀")
            self._revert_to_obstacle()
        self.obstacle_detected.emit(obj_name, dir_name, confidence)

    def _handle_bus(self, ser):
        # [len 1B][텍스트 len바이트][conf 1B][matched 1B(0/1)] — 버스모드 동안 매 사이클(~200ms)
        # 전송됨. matched=0은 아직 사전 후보와 미매칭인 진행중 추정값, 1은 확정. matched=1이
        # 와도 장애물모드로 즉시 돌아가지 않는다 - ESP32가 정류장 게이팅에서 몇 대를 더
        # 기다려야 하는지(s_bus_target_count) RPi는 모르므로, 실제로 장애물모드에 진입했을
        # 때 오는 PKT_REVERT(0xB3)만 보고 되돌린다.
        length = self._read_exact(ser, 1)[0]
        text = self._read_exact(ser, length).decode("utf-8", errors="replace") if length else ""
        confidence = self._read_exact(ser, 1)[0]
        matched = bool(self._read_exact(ser, 1)[0])
        # 0xB1은 ESP32가 버스모드일 때만 보낸다 - 명령 후 첫 0xB1 = 실제로 버스모드에 들어갔다는 뜻.
        # (정류장 게이팅에서 도착 버스가 없으면 ESP32는 진입 안 하고 0xB1도 안 옴 -> 이어폰도 조용.)
        # 한계: 버스 1대 매칭 후 같은 버스모드로 재진입할 때 ESP32가 다시 내는 안내는 여기서 못 잡는다.
        self._note_esp_mode("bus_ocr", announce=True)
        if matched and text in BUS_VOICE_KEYS:
            self.voice_cue.emit(text)  # ESP32가 매칭 순간 노선번호를 재생함(main.cpp queue_voice_clip(s_bus_last_match))
        self.bus_result.emit(text, confidence, matched)

    def _handle_bus_candidates(self, ser):
        count = self._read_exact(ser, 1)[0]
        candidates = []
        for _ in range(count):
            length = self._read_exact(ser, 1)[0]
            route = self._read_exact(ser, length).decode("utf-8", errors="replace")
            arr = struct.unpack("<H", self._read_exact(ser, 2))[0]
            candidates.append((route, None if arr == BUS_ARR_UNKNOWN else arr))

        if not candidates:
            # ESP32는 버스모드에 안 들어가고 장애물모드 그대로 - 0xB0 유예시간(MODE_CMD_GRACE_S)
            # 추정을 기다리지 않고 바로 화면을 되돌린다.
            self.voice_cue.emit("도착버스없음")
            if self._active_mode == "bus_ocr":
                self._revert_to_obstacle()
        else:
            # main.cpp report_api_bus_candidates()와 같은 순서: 클립 있는 노선들 -> "도착예정"
            # (도착초를 모르는 고정 리스트 폴백이면 ESP32도 조용함). 바로 뒤에 ESP32가
            # enter_mode(BUS)로 "버스탐지모드전환"을 내므로 여기서 모드도 같이 표시해둔다.
            if all(arr is not None for _, arr in candidates):
                voiced = [route for route, _ in candidates if route in BUS_VOICE_KEYS]
                for route in voiced:
                    self.voice_cue.emit(route)
                if voiced:
                    self.voice_cue.emit("도착예정")
            self._note_esp_mode("bus_ocr", announce=True)
        self.bus_candidates.emit(candidates)

    def _handle_log_line(self, ser, first_byte):
        # 결과 패킷과 달리 길이가 정해져 있지 않으므로, 개행이 오거나 1초(시리얼 timeout)
        # 동안 다음 바이트가 안 오면 그때까지 받은 걸로 한 줄 취급하고 끊는다 — 여기서
        # 진짜 TimeoutError를 던지면 바깥 _read_packets가 "시리얼 끊김"으로 오인해서
        # 멀쩡한 연결을 재접속시키게 된다. 로그 한 줄 유실은 그냥 무시해도 된다.
        line = bytearray(first_byte)
        while not self._stop.is_set():
            b = ser.read(1)
            if not b:
                break
            line += b
            if b == b"\n":
                break
        text = line.decode("utf-8", errors="replace").rstrip("\r\n")
        if text:
            print(f"[ircamera:fw] {text}")

    def _handle_braille(self, ser):
        # [present 1B][conf 1B][near 1B][dir 1B] (2026-10-16, dir 추가로 4 -> 5바이트).
        # 점자블록모드 동안 매 사이클(~200ms) 전송됨 — near(충분히 근접)가 와도 장애물모드로
        # 즉시 돌아가지 않는다(버스모드와 동일한 이유 - _handle_bus 참고). 실제로 장애물모드에
        # 진입했을 때 오는 PKT_REVERT(0xB3)만 보고 되돌린다.
        payload = self._read_exact(ser, 4)
        present, confidence, near, dir_code = bool(payload[0]), payload[1], bool(payload[2]), payload[3]
        self._note_esp_mode("braille", announce=True)  # 0xB2도 점자블록모드일 때만 옴 - _handle_bus와 같은 원리
        if near:
            # ESP32는 near가 오는 그 사이클에 방향 안내를 재생하고 1초 뒤 장애물모드로 복귀한다 -> 한 번만 옴
            self.voice_cue.emit(BRAILLE_NEAR_VOICE_KEYS.get(dir_code, BRAILLE_NEAR_VOICE_DEFAULT))
        self.braille_result.emit(present, confidence, near, DIRECTION_NAMES.get(dir_code, "-"))

    def _handle_revert(self):
        # 버스/점자블록 모드에서 장애물모드로 실제로 돌아갈 시점은 ESP32만 알 수 있다
        # (매칭 횟수, 정류장 게이팅 등 RPi가 모르는 상태에 달려있음) - 그래서 matched=1/near=1
        # 자체는 무시하고, 이 신호가 왔을 때만 안전망 타이머를 취소하고 되돌린다.
        # 0xB3은 ESP32가 enter_mode(OBSTACLE)할 때마다(부팅 포함) 보내고, 그때 항상 "장애물모드전환"을
        # 재생한다 - 그래서 이전 모드를 모르거나 이미 장애물이었어도 그대로 따라 낸다.
        self._esp_mode = "obstacle"
        self.voice_cue.emit(MODE_ENTER_VOICE_KEYS["obstacle"])
        self._revert_to_obstacle()

    def _note_esp_mode(self, mode, announce):
        prev = self._esp_mode
        self._esp_mode = mode
        if announce and prev is not None and prev != mode:
            self.voice_cue.emit(MODE_ENTER_VOICE_KEYS[mode])

    # ---------------- TCP 영상 서버 ----------------
    def _tcp_server_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.tcp_host, self.tcp_port))
        srv.listen(1)
        print(f"[ircamera] 영상 TCP 서버 대기 중: {self.tcp_host}:{self.tcp_port}")

        while not self._stop.is_set():
            try:
                conn, addr = srv.accept()
            except OSError:
                return  # stop() 등으로 소켓이 닫힌 경우
            print(f"[ircamera] 영상 클라이언트 연결: {addr}")
            self.video_client_connected.emit(True)
            try:
                self._recv_frames(conn)
            except Exception as e:
                print(f"[ircamera] 영상 수신 오류: {e}")
            finally:
                conn.close()
                self.video_client_connected.emit(False)
                print("[ircamera] 영상 클라이언트 연결 끊김, 재접속 대기")

    def _recv_frames(self, conn):
        # 타임아웃 없이 recv()를 걸면 ESP32가 모드 전환 중 프레임 전송을 멈췄을 때
        # 이 스레드가 무한정 블록된다. RECV_TIMEOUT_S 안에 응답이 없으면 socket.timeout이
        # 터져서 바깥 _tcp_server_loop의 except로 빠지고, 그쪽에서 conn을 닫고 재접속을 기다린다.
        conn.settimeout(RECV_TIMEOUT_S)
        while not self._stop.is_set():
            header = self._recv_exact(conn, 4)
            (length,) = struct.unpack("<I", header)

            jpeg_bytes = None
            if 0 < length <= MAX_FRAME_BYTES:
                body = self._recv_exact(conn, length)
                if body[:2] == JPEG_SOI and body[-2:] == JPEG_EOI:
                    jpeg_bytes = body
                else:
                    print(f"[ircamera] 프레임 손상(길이={length}), 재동기화 시도")
            else:
                print(f"[ircamera] 비정상 프레임 길이({length}), 재동기화 시도")

            if jpeg_bytes is None:
                jpeg_bytes = self._resync(conn)
                if jpeg_bytes is None:
                    raise ConnectionError("영상 스트림 재동기화 실패, 재접속 유도")
                print("[ircamera] 재동기화 성공")

            image = QImage.fromData(jpeg_bytes, "JPG")
            if not image.isNull():
                with self._frame_lock:
                    self._latest_frame = image
                    self._frame_dirty = True

    def _resync(self, conn):
        """프레임 프레이밍이 깨졌을 때, 스트림을 1바이트씩 밀어가며 다음으로 유효해
        보이는 (length 헤더 + JPEG SOI) 패턴을 찾아 복구한다.
        RESYNC_SCAN_LIMIT 안에 못 찾으면 None을 돌려주고, 호출부가 연결을 끊어 재접속을 유도한다."""
        window = b""
        scanned = 0
        while scanned < RESYNC_SCAN_LIMIT and not self._stop.is_set():
            b = conn.recv(1)
            if not b:
                return None
            window = (window + b)[-6:]
            scanned += 1
            if len(window) == 6:
                (length,) = struct.unpack("<I", window[:4])
                if 0 < length <= MAX_FRAME_BYTES and window[4:6] == JPEG_SOI:
                    rest = self._recv_exact(conn, length - 2)
                    body = JPEG_SOI + rest
                    if body[-2:] == JPEG_EOI:
                        return body
                    # SOI만 우연히 맞아떨어졌을 수 있으니 여기서 포기하지 않고 계속 스캔한다
        return None

    def _recv_exact(self, conn, n):
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("영상 연결 종료됨")
            buf += chunk
        return buf
