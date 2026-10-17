# Tiny Vision Cane - Fall Detection + Keyword Spotting

`FD_KWS_GPS`는 하나의 XIAO ESP32-S3에서 **낙상 감지(FD)**와 **키워드
인식(KWS)**을 함께 실행하는 통합 펌웨어이다. LSM9DS1 지자기 센서를
이용한 방위각 출력도 포함한다.

낙상 감지는 IMU용 TFLite Micro 모델을, 키워드 인식은 별도의 Edge
Impulse 오디오 모델을 사용한다. 두 모델은 공유 inference lock으로
동시 실행을 방지한다.

## 주요 기능

- 6축 IMU 기반 `normal`/`event` 낙상·낙하 분류
- pickup 감시 기반 `safe_fall`/`danger_fall` 후처리
- Raspberry Pi 5 스피커 제어와 Firebase 보호자 알림
- Raspberry Pi 5에서 받은 16 kHz PCM 오디오의 키워드 분류
- 가속도·자이로·지자기를 결합한 방위각과 이동 상태 출력
- 낙상, KWS, 방위각 프로토콜의 USB Serial/JTAG 공유

## 전체 데이터 흐름

```text
LSM9DS1 IMU -> ESP32-S3 낙상 모델 -> normal / event
                    |-> 0xFF / 0xFE / 0xFD -> Raspberry Pi 5 스피커
                    `-> safe_fall / danger_fall -> Firebase

Raspberry Pi 5 -> 16 kHz, 1초 PCM -> ESP32-S3 KWS 모델
Raspberry Pi 5 <- RESULT:<label> ... <- ESP32-S3

LSM9DS1 가속도·자이로·지자기 -> 방위각 계산 -> HDG 패킷 -> Raspberry Pi 5
```

## 하드웨어와 연결

- **MCU**: Seeed Studio XIAO ESP32-S3
- **IMU**: ADA-4634, Adafruit LSM9DS1 9-DOF Breakout
- **I2C**: SDA `GPIO5`, SCL `GPIO6`, 100 kHz
- **AG 주소 후보**: `0x6A`, `0x6B`
- **지자기 주소 후보**: `0x1E`, `0x1C`
- **호스트 연결**: Raspberry Pi 5와 USB Serial/JTAG 연결

KWS 오디오는 ESP32-S3에 직접 연결된 마이크에서 수집하지 않는다.
Raspberry Pi 5가 오디오를 `int16` PCM으로 구성해 USB로 전달한다.

## 낙상 감지(FD)

현재 낙상 모델은 `models/fall_2sec_event_extended.tflite`다. 25 Hz × 50시점 ×
6축의 float32 입력 `[1,300]`을 사용하며, 출력 순서는 `[event, normal]`이다.
추가 낙상 원본 6개와 구간 이동·노이즈 증강으로 재학습한 모델을
`python tools/convert_tflite_to_cc.py fall_2sec_event_extended.tflite`로 변환했다.
비중첩 2초 윈도우를 사용하며 현재 추가 읽기 조건상 이상적인 수집 루프에서
출력 간격은 약 2.04초다. 평가와 모델 해시는
`../Tiny_Vision_Cane_2sec/EVENT_EXTENSION_RESULTS.md`를 참조한다. 크기는
17,524 bytes, SHA-256은 `3c8d0a8087cc7b38d0e530276a873c77339b4ec7364d53afef263e78586ce4c1`이다.
소스의 C++ 배열과 모델 바이트는 일치한다. 모델 최초 적용 당시에는 빌드·
보드 업로드하지 않았다. 이후 2026-10-13의 빌드 결과는
[변경·검증 기록](../FIRMWARE_ALIGNMENT.md)을 참조한다. 보드 업로드는 별도다.

- 입력: `accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z`
- 수집 주기: 25 Hz
- 현재 소스에 포함된 모델 출력 순서: `[event, normal]`
- 이벤트 임계값: `event >= 0.50`
- 정지 오탐 억제 없음: 2026-10-13에 모든 현재 펌웨어에서 제거했다.
  원시 모델 event 점수 >=0.50을 사용하며 움직임 변화량으로 추가 필터링하지 않는다.
- 이벤트 직후 1.5초 동안 충격 잔여 움직임 무시
- `delta_g >= 0.30 g` 또는 `magnitude_g >= 1.35 g`이면 pickup
- 10초 이내 pickup: `safe_fall`
- 10초 동안 pickup 없음: `danger_fall`

`safe_fall`과 `danger_fall`은 모델 클래스가 아니라 펌웨어의 후처리
상태이다. pickup은 지팡이의 큰 움직임을 감지하는 규칙이며 사용자의 기립,
부상 또는 의식 상태를 직접 확인하지 않는다.

낙상 모델을 교체할 때는 새 모델의 출력 순서가 코드의 `[event, normal]`과
일치하는지 반드시 확인해야 한다.

| USB 바이트 | 의미 |
|---|---|
| `0xFF` | 이벤트 감지, 스피커 안내 시작 |
| `0xFE` | `safe_fall`, 스피커 안내 종료 |
| `0xFD` | `danger_fall`, 위험 안내 |

ESP32-S3가 두 상태를 Cloud Function으로 직접 전송한다. 두 상태 모두
Firestore에 기록되며 `danger_fall`만 FCM 긴급 푸시를 발생시킨다.
이 프로젝트는 NVS 영구 보관과 `eventId`/`eventSequence`를 사용한다.
HTTP 실패는 횟수 제한으로 버리지 않고 1~30초 간격으로 재시도한다.

## 키워드 인식(KWS)

### 오디오 입력

- signed 16-bit PCM (`int16`)
- 16 kHz, 16,000 samples, 1초
- 32,000 bytes/window
- Raspberry Pi 5 -> USB Serial/JTAG -> ESP32-S3

ESP32-S3가 `S3_READY`를 보낸 뒤 호스트가 한 개의 오디오 윈도우를
전송한다. 전송 도중 약 2초 동안 데이터가 없으면 불완전한 윈도우를
폐기하고 다시 준비 상태로 돌아간다.

### KWS 모델과 결과

- Edge Impulse 프로젝트: `KWS1235`
- 모델: int8 양자화 오디오 모델
- 라벨: `one`, `three`, `two`, `zero`
- 분류 임계값: `0.60`
- DSP/모델 입력 feature 수: 650
- TFLite tensor arena: 12,006 bytes

```text
RESULT:<label> (<score>) ACCEPTED
RESULT:<label> (<score>) REJECTED
```

추론 lock을 얻지 못하면 `KWS_BUSY`, 모델 실행에 실패하면 `KWS_ERROR`를
반환한다. 처리 후에는 다음 입력을 위해 `S3_READY`를 다시 전송한다.

## 방위각 출력

낙상 센서 태스크가 I2C를 단독으로 읽고 측정값 스냅샷을 방위각 태스크에
전달한다. `HDG` 진단 출력은 `FALL_DETECTION_TEST_MODE=ON`에서만
전송한다. 실행 모드에서는 출력하지 않는다. 테스트 출력 형식은 다음과 같다.

```text
HDG,<heading_deg>,<valid>,<moving>
```

- `heading_deg`: 0~360도 방위각
- `valid`: 센서 데이터의 최신성과 지자기 값의 유효성
- `moving`: 최근 가속도 분산에 따른 이동 여부

현재 자기 편각은 대구 기준 `-8.5°`다. 실제 설치 방향과 지역에 맞춰
지자기 offset, scale, 전방 축과 편각을 보정해야 한다.

## USB 프로토콜

### Raspberry Pi 5 -> ESP32-S3

- 32,000바이트 KWS 오디오 윈도우

### ESP32-S3 -> Raspberry Pi 5

- 낙상 제어: `0xFF`, `0xFE`, `0xFD` 원시 바이트
- KWS: `S3_READY`, `RESULT:...`, `KWS_BUSY`, `KWS_ERROR`
- 테스트 모드 방위각 진단: `HDG,...` (실행 모드에서는 출력하지 않음)

호스트는 스트림을 텍스트 줄로만 처리하지 말고 원시 낙상 바이트와 텍스트
패킷을 함께 구분해야 한다. 실행 모드에서는 콘솔 로그가 프로토콜을
오염시키지 않도록 `FALL_DETECTION_TEST_MODE=OFF`를 사용한다.

## 동시 실행 구조

- **Core 0 / priority 6**: IMU 수집과 pickup 감시
- **Core 0 / priority 4**: 낙상 추론
- **Core 0 / priority 3**: 방위각 계산
- **Core 1 / priority 6**: KWS 수신과 추론
- **공유 AI lock**: 두 TFLite interpreter의 동시 실행 방지
- **공유 USB TX lock**: 낙상·KWS·방위각 출력 충돌 방지

낙상 추론과 KWS의 AI lock 대기는 각각 최대 8초다. 별도 Firebase 저장
태스크는 Core 0 / priority 2, 전송 태스크는 Core 0 / priority 3이다.

낙상 추론은 double buffer와 queue를 사용해 센서 수집 주기를 차단하지
않는다.

## 프로젝트 구조

- `main/Fall_detection.cc`: 통합 초기화, 낙상 수집·추론·후처리
- `main/accelerometer_handler.cpp`: 가속도·자이로·지자기 수집
- `main/model_input_provider.cc`: 낙상 모델 입력 구성
- `main/keyword_spotting.cc`: USB PCM 수신과 KWS 추론
- `main/ai_inference_lock.cc`: 낙상/KWS 공유 추론 lock
- `main/nav_heading.cc`: 방위각과 이동 상태 계산
- `main/usb_output.cc`: 공유 USB 출력 직렬화
- `main/fall_alert_sender.cc`: Firebase 상태 전송
- `models/fall_model.cc`: 낙상 TFLite 모델 C 배열
- `main/tflite-model/`: KWS TFLite 모델과 resolver
- `main/model-parameters/`: KWS 메타데이터, DSP, 라벨 정의
- `main/edge-impulse-sdk/`: KWS용 Edge Impulse SDK

## 빌드

```powershell
# 통합 실행 모드
idf.py -B build_gps_current -DFALL_DETECTION_EDGE_IMPULSE_DATA_FORWARDER=OFF -DFALL_DETECTION_TEST_MODE=OFF reconfigure build flash

# 개발용 진단 모드
idf.py -B build_gps_current -DFALL_DETECTION_EDGE_IMPULSE_DATA_FORWARDER=OFF -DFALL_DETECTION_TEST_MODE=ON reconfigure build flash monitor

# 낙상 데이터 수집 전용 모드(KWS, 방위각, Firebase, 추론 제외)
idf.py -DFALL_DETECTION_EDGE_IMPULSE_DATA_FORWARDER=ON reconfigure build flash
edge-impulse-data-forwarder --clean --frequency 25

# 지자기 보정 모드
idf.py -B build_gps_current -DFALL_DETECTION_EDGE_IMPULSE_DATA_FORWARDER=OFF -DFALL_DETECTION_TEST_MODE=ON -DNAV_MAG_CALIBRATION=ON reconfigure build flash monitor
```

Firebase 접속 정보는 `idf.py menuconfig`의
**Fall detection Firebase connection**에서 설정한다.

## 모델 관리

낙상 모델은 다음 명령으로 `models/fall_model.cc`를 갱신한다.

```powershell
python tools/convert_tflite_to_cc.py <fall-model.tflite>
```

KWS 모델은 `main/tflite-model/`, `main/model-parameters/`와 필요한 Edge
Impulse SDK 파일을 배포물 단위로 함께 갱신해야 한다. 교체 후 라벨, 입력
길이, 샘플링 주파수, threshold와 tensor arena 크기를 반드시 확인한다.

## 관련 문서

- 단독 낙상 감지: `../Fall_detection/README.md`
- Firebase·보호자 앱: `../Care_app/DIRECT_ESP_SETUP.md`
