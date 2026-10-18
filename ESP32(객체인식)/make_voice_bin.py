"""data/*.wav -> voice.bin 합치기 + voice_clips.h 오프셋 테이블 검증.

voice_clips.h의 VOICE_CLIPS 순서/오프셋/길이를 기준으로, 각 클립의 wav에서
raw PCM(헤더 제외)만 꺼내 순서대로 이어붙인다. 오프셋/길이가 테이블과 하나라도
다르면 에러로 종료한다 (잘못된 voice.bin을 플래싱하는 사고 방지).

사용:  python make_voice_bin.py
결과:  voice.bin (이 폴더)  →  parttool.py write_partition --partition-name voice --input voice.bin
"""
import re
import sys
import wave
from pathlib import Path

ROOT = Path(__file__).resolve().parent
HEADER = ROOT / "main" / "voice_clips.h"
DATA = ROOT / "data"
OUT = ROOT / "voice.bin"

# 키와 wav 파일 이름이 다른 클립
FILENAME_OVERRIDE = {
    "도착버스없음": "1분이내에도착하는버스가없습니다.wav",
}

ROW = re.compile(r'\{"([^"]+)",\s*(\d+),\s*(\d+)\}')


def main():
    text = HEADER.read_text(encoding="utf-8")
    start = text.index("VOICE_CLIPS[]")
    end = text.index("};", start)
    clips = [(k, int(o), int(n)) for k, o, n in ROW.findall(text[start:end])]
    if not clips:
        sys.exit("voice_clips.h에서 클립을 못 찾음")

    blob = bytearray()
    for key, offset, length in clips:
        wav_path = DATA / FILENAME_OVERRIDE.get(key, f"{key}.wav")
        if not wav_path.exists():
            sys.exit(f"wav 없음: {wav_path}")
        with wave.open(str(wav_path), "rb") as w:
            fmt = (w.getframerate(), w.getnchannels(), w.getsampwidth() * 8)
            if fmt != (24000, 1, 16):
                sys.exit(f"{wav_path.name}: 24kHz/mono/16bit가 아님 {fmt}")
            pcm = w.readframes(w.getnframes())
        if offset != len(blob) or length != len(pcm):
            sys.exit(
                f"테이블 불일치: '{key}' 테이블(offset={offset}, len={length}) "
                f"vs 실제(offset={len(blob)}, len={len(pcm)})"
            )
        blob += pcm

    m = re.search(r"VOICE_CLIPS_COUNT\s*=\s*(\d+)", text)
    if m and int(m.group(1)) != len(clips):
        sys.exit(f"VOICE_CLIPS_COUNT({m.group(1)}) != 클립 수({len(clips)})")
    m = re.search(r"VOICE_BIN_TOTAL_BYTES\s*=\s*(\d+)", text)
    if m and int(m.group(1)) != len(blob):
        sys.exit(f"VOICE_BIN_TOTAL_BYTES({m.group(1)}) != 실제 크기({len(blob)})")

    OUT.write_bytes(blob)
    print(f"OK: {len(clips)}개 클립, {len(blob)}바이트 -> {OUT}")


if __name__ == "__main__":
    main()
