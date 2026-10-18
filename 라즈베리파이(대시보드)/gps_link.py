import math
import threading
import time

import pynmea2
import serial
from PySide6.QtCore import QObject, Signal

DEFAULT_SERIAL_PORT = "/dev/ttyAMA4"
DEFAULT_SERIAL_BAUD = 9600

MIN_NUM_SATS = 4  # 이 미만이면 단독 GPS 오차가 급격히 커짐 - 픽스 자체를 버림
MAX_HDOP = 5.0  # 수평정밀도저하율. 이 값보다 크면 위성 배치가 나빠 오차가 큰 것으로 보고 버림
MAX_SPEED_MPS = 40.0  # 도보/버스 이동을 감안한 여유치(≈144km/h) - 이보다 빠른 "이동"은 GPS 글리치로 보고 이전 값 유지

DEFAULT_HDOP = 2.0  # GGA 문장에 HDOP 필드가 비어있을 때(드묾)만 쓰는 보수적 기본값
BASE_UERE_M = 5.0  # 단독(SPS) GPS의 HDOP=1 기준 대략적 사용자등가거리오차(m) - 흔히 쓰이는 근사치
ACCEL_NOISE_STD_MPS2 = 0.5  # 보행 가속도 불확실성(m/s^2). 0.3~1.0 사이에서 실측으로 조정 - 작을수록 부드럽고 반응이 느림
MAX_SPEED_EST_MPS = 3.0  # 필터가 추정하는 속도의 상한(m/s) - 도보 기준, 글리치로 속도 추정이 튀는 것 방지

METERS_PER_DEG_LAT = 111320.0  # 위도 1도당 거리(m) - 위도에 거의 무관하게 일정


def _haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


class _KalmanAxisCV:
    """1축(동/북 방향, 미터 단위)용 등속(constant velocity) 칼만 필터. 상태 = [위치(m), 속도(m/s)].

    위치만 추정하던 이전 필터는 걸을 때 추정 위치가 실제보다 4~9m 뒤처졌다(속도를 모르기 때문).
    속도를 함께 추정하면 일정한 속도로 걷는 동안 지연이 거의 0이 된다. 측정 노이즈
    (measurement_std_m)는 HDOP에서 추정해 게인에 그대로 반영하므로, 위성 배치가 나쁜 순간의
    픽스는 자동으로 덜 반영된다. 정지 상태에서는 속도 추정이 0으로 수렴하고 위치 분산이 계속 줄어든다.
    """

    def __init__(self, accel_std_mps2: float):
        self._q = accel_std_mps2 ** 2
        self.p = None  # 위치 추정(m)
        self.v = 0.0   # 속도 추정(m/s)
        self.P = None  # 2x2 공분산

    def update(self, z: float, meas_std_m: float, dt: float) -> float:
        R = meas_std_m ** 2
        if self.p is None:
            self.p, self.v = z, 0.0
            self.P = [[R, 0.0], [0.0, 1.0]]
            return self.p

        dt = max(dt, 0.0)
        q = self._q
        (P00, P01), (P10, P11) = self.P
        # 예측 (F = [[1, dt], [0, 1]], 가속도 잡음 q)
        p = self.p + self.v * dt
        v = self.v
        n00 = P00 + dt * (P01 + P10) + dt * dt * P11 + q * dt ** 4 / 4
        n01 = P01 + dt * P11 + q * dt ** 3 / 2
        n10 = P10 + dt * P11 + q * dt ** 3 / 2
        n11 = P11 + q * dt * dt
        # 갱신 (H = [1, 0])
        S = n00 + R
        k0, k1 = n00 / S, n10 / S
        y = z - p
        p += k0 * y
        v = max(-MAX_SPEED_EST_MPS, min(MAX_SPEED_EST_MPS, v + k1 * y))
        self.P = [[(1 - k0) * n00, (1 - k0) * n01],
                  [n10 - k1 * n00, n11 - k1 * n01]]
        self.p, self.v = p, v
        return p


class GPSLink(QObject):
    fix_changed = Signal(bool, float, float)  # connected(유효 픽스 여부), 위도, 경도
    fix_detail = Signal(bool, float, float, float)  # connected, 위도, 경도, HDOP(모르면 0) - 길안내 보드 전송용

    def __init__(self, serial_port=DEFAULT_SERIAL_PORT, serial_baud=DEFAULT_SERIAL_BAUD, parent=None):
        super().__init__(parent)
        self.serial_port = serial_port
        self.serial_baud = serial_baud
        self._stop = threading.Event()
        self._connected = False  # 마지막으로 emit한 connected 상태 - 중복 emit(배지 재렌더) 방지용
        self._last_accepted = None  # 이상치 판정 기준이 되는 마지막 채택 픽스(lat, lon, monotonic time)

        # 칼만 필터는 위도/경도를 직접 다루지 않고, 첫 픽스를 원점으로 하는 로컬 평면
        # (동쪽=east_m, 북쪽=north_m)에 투영해서 미터 단위로 필터링한다 - 위도/경도는
        # 축척이 서로 달라서(경도 1도의 실제 거리가 위도에 따라 변함) 그대로 필터링하면
        # 오차 추정이 왜곡된다.
        self._ref_lat = None
        self._ref_lon = None
        self._east_filter = _KalmanAxisCV(ACCEL_NOISE_STD_MPS2)
        self._north_filter = _KalmanAxisCV(ACCEL_NOISE_STD_MPS2)
        self._last_update_t = None

    def start(self):
        threading.Thread(target=self._serial_loop, daemon=True).start()

    def stop(self):
        self._stop.set()

    def _serial_loop(self):
        while not self._stop.is_set():
            try:
                ser = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
                print(f"[gps] 시리얼 연결됨: {self.serial_port}")
                try:
                    self._read_loop(ser)
                finally:
                    ser.close()
            except Exception as e:
                print(f"[gps] 시리얼 연결 실패/끊김: {e}")
                self._emit_disconnected()
                self._stop.wait(2.0)

    def _read_loop(self, ser):
        while not self._stop.is_set():
            raw = ser.readline()
            if not raw:
                continue
            decoded = raw.decode("ascii", errors="replace").strip()
            if not decoded.startswith(("$GNGGA", "$GPGGA")):
                continue
            try:
                msg = pynmea2.parse(decoded)
            except (pynmea2.ParseError, ValueError):
                continue

            qual = getattr(msg, "gps_qual", 0)
            if not (qual and int(qual) > 0 and msg.latitude and msg.longitude):
                self._emit_disconnected()
                continue

            hdop = self._parse_hdop(msg)
            if not self._passes_accuracy_gate(msg, hdop):
                continue  # 픽스는 있지만 정확도가 나쁨 - 이번 문장은 버리고 이전 표시값 유지

            if not self._passes_outlier_gate(msg.latitude, msg.longitude):
                continue  # 직전 위치에서 물리적으로 말이 안 되는 거리 - GPS 글리치로 보고 버림

            lat, lon = self._filtered(msg.latitude, msg.longitude, hdop)
            self._connected = True
            self.fix_changed.emit(True, lat, lon)
            self.fix_detail.emit(True, lat, lon, hdop if hdop is not None else 0.0)

    def _parse_hdop(self, msg):
        try:
            return float(msg.horizontal_dil)
        except (TypeError, ValueError):
            return None

    def _passes_accuracy_gate(self, msg, hdop) -> bool:
        try:
            num_sats = int(msg.num_sats)
        except (TypeError, ValueError):
            num_sats = None
        if num_sats is not None and num_sats < MIN_NUM_SATS:
            return False

        if hdop is not None and hdop > MAX_HDOP:
            return False

        return True

    def _passes_outlier_gate(self, lat, lon) -> bool:
        now = time.monotonic()
        if self._last_accepted is None:
            self._last_accepted = (lat, lon, now)
            return True

        last_lat, last_lon, last_t = self._last_accepted
        dt = max(now - last_t, 0.001)
        dist = _haversine_m(last_lat, last_lon, lat, lon)
        if dist / dt > MAX_SPEED_MPS:
            print(f"[gps] 이상치 거부: {dist:.0f}m/{dt:.1f}s (>{MAX_SPEED_MPS}m/s) - 이전 위치 유지")
            return False

        self._last_accepted = (lat, lon, now)
        return True

    def _filtered(self, lat, lon, hdop):
        if self._ref_lat is None:
            self._ref_lat, self._ref_lon = lat, lon

        m_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(self._ref_lat))
        east_m = (lon - self._ref_lon) * m_per_deg_lon
        north_m = (lat - self._ref_lat) * METERS_PER_DEG_LAT

        measurement_std_m = max(hdop if hdop is not None else DEFAULT_HDOP, 1.0) * BASE_UERE_M
        now = time.monotonic()
        dt_s = 0.0 if self._last_update_t is None else now - self._last_update_t
        self._last_update_t = now

        east_filt = self._east_filter.update(east_m, measurement_std_m, dt_s)
        north_filt = self._north_filter.update(north_m, measurement_std_m, dt_s)

        filt_lat = self._ref_lat + north_filt / METERS_PER_DEG_LAT
        filt_lon = self._ref_lon + east_filt / m_per_deg_lon
        return filt_lat, filt_lon

    def _emit_disconnected(self):
        # 픽스 없는 상태는 매 문장(초당 ~1회)/재연결 시도(2초 간격)마다 반복되므로,
        # 이미 연결 안 됨으로 전환돼 있으면 다시 emit하지 않는다(배지 불필요한 재렌더 방지).
        if not self._connected:
            return
        self._connected = False
        self.fix_changed.emit(False, 0.0, 0.0)
        self.fix_detail.emit(False, 0.0, 0.0, 0.0)
