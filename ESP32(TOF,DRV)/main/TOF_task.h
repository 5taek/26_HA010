// TOF task header
#ifndef __TOF_TASK_H__
#define __TOF_TASK_H__

#include "common.h"

// 측정 모드: 1이면 USB를 콘솔(로그)에 넘기고 RPi 통신(0xB2/0xB3)을 끈다. 센서 튜닝용.
//            0이면 USB는 RPi 통신 전용(UsbCdcComm). 이때 menuconfig 콘솔은
//            UART0 + "No secondary console"이어야 한다 (USB로 로그가 섞이면 바이너리 패킷이 깨짐).
// 자세한 설명은 smartcane.cc 상단 주석 참고.
#define SMARTCANE_MEASURE_MODE 0

// ⚠️ 임시 스위치 — TOF0 모듈이 내부 손상으로 확정됨(재납땜해도 초기화 실패 + I2C0
// 버스를 오염시켜 DRV0까지 끌고 들어감). 교체 전까지 영상 촬영 등 실사용 테스트를
// 위해 TOF0을 소프트웨어적으로 완전히 잠그고 TOF1 하나만 8x8로 돌린다.
// TOF_task.cc와 smartcane.cc 둘 다에서 봐야 해서 헤더에 둔다.
// TOF0 모듈 교체되면 0으로 되돌리고 8x4/4x4 듀얼 센서 설정으로 복귀할 것
// (TOF_task.cc의 kHeightRowStart/End, kDirColStart/End, usb_cdc_comm.h의
//  SMARTCANE_RAW_GRID_* 전부 같이 되돌려야 함 — 이전 커밋 참고).
#define SMARTCANE_TOF0_DISABLED 0

// (센서 1개 시절 임시 스위치) 단일 센서가 I2C0(SDA=5/SCL=6)에 꽂혀 있을 때 1.
// ⚠️ 듀얼 센서 구성(2026-10-13~)에서는 반드시 0: TOF0=I2C0(상단 4x4), TOF1=I2C1(중하단 8x8).
// TOF1 설정(8x8)을 I2C0에 바인딩한다.
// 센서를 I2C1(SDA=3/SCL=4)로 옮기면 0으로 되돌릴 것. TOF0_DISABLED=1일 때만 의미 있음.
#define SMARTCANE_TOF1_ON_I2C0 0

// 1이면 TOF1을 autonomous 모드 + 긴 적분시간으로 돌려 최대 측정거리를 늘린다.
// 적분시간은 5Hz 주기(200ms)보다 짧아야 한다. 문제가 생기면 0으로 되돌릴 것.
#define SMARTCANE_TOF1_LONG_INTEGRATION 1
#define SMARTCANE_TOF1_INTEGRATION_MS   50

// 반응 지연 단축 (2026-10-13 야외 실측에서 감지가 너무 늦다는 문제):
//  - 측정 주파수 5Hz -> 10Hz : 한 프레임 간격 200ms -> 100ms (8x8 상한 15Hz 이내)
//  - 태스크 폴링 주기 200ms -> 20ms : 예전엔 데이터가 준비돼도 최대 200ms를 더 기다렸다
//  - 적분시간은 주기(100ms)보다 짧아야 해서 100 -> 50ms로 낮춤 (최대 측정거리는 약간 줄 수 있음)
#define SMARTCANE_TOF_FREQ_HZ 10
#define TOF_POLL_MS           20

// ════════════════════════════════════════════════════════════
// 스마트 슬립 플래그 (Readme 5장)
// 사용자가 정지 상태이거나 위험이 해소됐다고 판단하면 true.
// true인 동안은 evaluate_zone_grid_and_act_locked()가 RPi로의 grid 알림 전송을
// 건너뛴다 (햅틱은 이 플래그와 무관하게 계속 동작함).
// 판정 로직은 TOF_task.cc의 update_smart_sleep()에 구현돼 있습니다
// (5.1 진입 / 5.2 해제 조건).
// ════════════════════════════════════════════════════════════
extern volatile bool g_alert_sleep_active;

// ════════════════════════════════════════════════════════════
// 센서 고장 마스크 (0이면 정상)
// 예전엔 bind 실패나 vl53l5cx_init 3회 실패를 전부 무시하고 진행해서, 센서가 죽어도
// 태스크가 1초마다 재시작만 무한 반복하고 사용자는 알 방법이 없었습니다.
// 이제 부팅 실패와 런타임 지속 정지를 여기 기록하고, 진동으로 사용자에게 알립니다.
// ════════════════════════════════════════════════════════════
#define TOF_FAULT_TOF0 0x01
#define TOF_FAULT_TOF1 0x02
uint8_t TOF_GetFaultMask(void);

void TOF_INIT(void);
void TOF_FIRMWARE_INIT(void);
void TOF_CONFIG_SETUP(void);

void TOF0_TASK(void *pvParameters);
void TOF1_TASK(void *pvParameters);
void HAPTIC0_TASK(void *pvParameters);
void HAPTIC1_TASK(void *pvParameters);

// 햅틱이 실제로 발화한 순간(0xB3)을 USB(RPi)로 전송하는 전용 태스크.
// ToF 태스크가 USB write에 블로킹돼서 거리 측정이 멈추는 걸 막기 위해 분리됨.
void ALERT_SENDER_TASK(void *pvParameters);

// 원본 8x8 ToF 그리드(0xB2, TOF1 단독)를 매 측정 회차마다 USB(RPi)로 스트리밍하는 전용 태스크.
void GRID_STREAM_TASK(void *pvParameters);

// 센서 고장이 지속되는 동안 주기적으로 고장 진동을 재생하는 태스크.
// 부팅 때 한 번만 알리면 사용자가 놓칠 수 있고, 주행 중 고장난 경우도 알려야 함.
void TOF_FAULT_NOTIFY_TASK(void *pvParameters);

#endif // __TOF_TASK_H__