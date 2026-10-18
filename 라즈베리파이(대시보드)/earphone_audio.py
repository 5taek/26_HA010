import os
import queue
import subprocess
import threading

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
QUEUE_MAX = 8  # ircamera main.cpp의 s_audio_queue 길이와 동일(2026-10-16 4 -> 8, 버스 후보 안내 때문)
# voice_clips.h 키와 wav 파일명이 다른 것들
CLIP_FILE_ALIASES = {"도착버스없음": "1분이내에도착하는버스가없습니다"}
PLAY_TIMEOUT_S = 15  # 가장 긴 클립이 ~3초라 넉넉하게. paplay가 멈춰도 대기열이 영영 막히지 않게


class EarphoneAudio:
    def __init__(self, data_dir=DATA_DIR):
        self.data_dir = data_dir
        self._queue = queue.Queue(maxsize=QUEUE_MAX)
        threading.Thread(target=self._worker, daemon=True).start()

    def play(self, key: str):
        path = os.path.join(self.data_dir, f"{CLIP_FILE_ALIASES.get(key, key)}.wav")
        if not os.path.isfile(path):
            print(f"[earphone] 클립 '{key}' 없음: {path}")
            return
        try:
            self._queue.put_nowait(path)
        except queue.Full:
            print(f"[earphone] 대기열 꽉참 - '{key}' 재생 건너뜀")

    def _worker(self):
        while True:
            path = self._queue.get()
            sink = self._find_bluez_sink()
            if sink is None:
                continue  # 이어폰 미연결(또는 오디오 프로필 미연결) - ESP32 스피커만 나오면 됨
            try:
                subprocess.run(["paplay", f"--device={sink}", path], timeout=PLAY_TIMEOUT_S,
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            except (subprocess.TimeoutExpired, FileNotFoundError) as e:
                print(f"[earphone] 재생 실패: {e}")

    @staticmethod
    def _find_bluez_sink():
        # 재생 직전에 매번 확인한다(~수 ms) - 이어폰을 중간에 꽂고 빼도 따로 상태를 들고 있을 필요가 없다.
        try:
            out = subprocess.run(["pactl", "list", "short", "sinks"], capture_output=True,
                                 text=True, timeout=2).stdout
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return None
        for line in out.splitlines():
            cols = line.split("\t")
            if len(cols) > 1 and cols[1].startswith("bluez_output."):
                return cols[1]
        return None
