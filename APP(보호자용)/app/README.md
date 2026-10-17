# care_guardian_app

Flutter/Dart 보호자 앱이다. Firebase Authentication 로그인, 사용자·기기 연결,
낙상 기록 조회, 지도와 FCM/로컬 알림을 제공한다.

## 현재 동작

- `lib/main.dart`: Firebase 초기화와 앱 시작
- `lib/src/ui/login_page.dart`: 로그인·등록과 보호자/사용자/기기 연결
- `lib/src/ui/home_page.dart`: 보호자 홈, 기기 위치, 이벤트 기록, 위험 팝업
- `lib/src/services/alert_messaging_service.dart`: 알림 권한, FCM 토큰, 로컬 알림
- `users/{uid}/fcmTokens/current`: 토큰 저장, 갱신 및 저장 실패 backoff(최대 5분)
- 전역 `fall_events`: `guardianUids`에 로그인 UID가 포함된 이벤트 구독. 최근 50개, 로그 화면 최대 200개
- `devices/{deviceId}`: 최신 위치를 기본 10초 주기로 조회
- 위치 시각은 `sampledAt`을 우선 사용하고, 2분 초과 또는 시각 불명 시 경고
- `danger_fall`: 푸시와 미확인 위험 팝업; `safe_fall`: 기록, FCM 발송 없음
- Android 전용 알림 채널: `fall_alerts`

앱의 이벤트 구독은 전역 `fall_events`에서 `guardianUids`를 필터링하는 방식이다.
서버 `reportFall`은 전역 이벤트와 보호자별 `users/{uid}/fall_events` 사본을
함께 기록하지만 현재 앱 목록의 구독 경로는 전역 컬렉션이다.
위치는 마지막 저장값이므로 실제 현재 위치나 낙상 당시 위치를 항상 뜻하지 않는다.

## SDK requirements

현재 `pubspec.lock`의 요구 버전은 Dart **>=3.12.0 <4.0.0**, Flutter **>=3.44.0**이다.
기존 문서의 성공한 정적 분석 환경은 Flutter **3.47.2**로 기록돼 있다.
현재 검토에서 분석을 재실행한 결과나 최신 앱 배포 완료를 뜻하지 않는다.

`pubspec.yaml`의 Dart >=3.8.0 하한만으로는 잠긴 의존성을 충족하지 못한다.
lock 파일을 유지하고 호환 SDK를 사용한다. 이번 수정은 SDK 제약이나
의존성 버전을 변경하지 않는다.

```powershell
flutter --version
flutter pub get
flutter analyze
```

Firebase 프로젝트 설정, Android 구성과 알림 권한을 확인해야 실제 기기에서
알림을 받을 수 있다. 실행·설정은 [직접 연결 구성](../DIRECT_ESP_SETUP.md),
펌웨어별 전송 차이는 [이벤트 전달](../EVENT_DELIVERY.md)을 참조한다.
