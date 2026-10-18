#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h" // 라파랑 USB-CDC 통신

#define TAG "TMAP_NAV"
#define MATH_PI 3.14159265358979323846

// 와이파이, TMAP 키
#define WIFI_SSID "sew"
#define WIFI_PASS "shin0509"
#define TMAP_APP_KEY "TJZqUnpN8j2rXMNOIex7u2gWb2GV6ieX9DUln7rP"

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// 라파 <-> ESP32 신호 (로그랑 같은 USB-CDC 선으로 같이 감)
// 라파 -> ESP: 목적지 선택
#define CMD_DEST_1 0x01
#define CMD_DEST_2 0x02
// ESP -> 라파
// 0xFE, 0xFF는 UTF-8에 안 나오는 바이트라 한글 로그랑 안 섞임
#define SIGNAL_NEXT_DIRECTION 0xFE // 방향 안내 나갈 때
#define SIGNAL_ARRIVED 0xFF        // 도착
// 이벤트 신호. 0x00~0x1F 제어문자 영역이라 로그랑 안 겹침
#define SIGNAL_TURN_LEFT 0x03  // 좌회전
#define SIGNAL_TURN_RIGHT 0x04 // 우회전
#define SIGNAL_CROSSWALK 0x05  // 횡단보도
#define SIGNAL_STAIRS 0x06     // 계단
// 이탈/재탐색
#define SIGNAL_OFF_ROUTE 0x07      // 이탈 확정
#define SIGNAL_REROUTING 0x08      // 재탐색 시작
#define SIGNAL_REROUTE_DONE 0x09   // 재탐색 성공
#define SIGNAL_REROUTE_FAILED 0x0A // 재탐색 실패 (잠시 후 재시도)
#define SIGNAL_BACK_ON_ROUTE 0x0B  // 재탐색 전에 경로로 돌아옴

#define MAX_WAYPOINTS 100
#define MAX_ROUTE_LINE_POINTS 300 // 경로선(LineString) 정점 최대
#define HTTP_RESP_BUF_SIZE 20480

// 길안내/이탈 판정 값
#define ANNOUNCE_DISTANCE_M 5.0   // 이 안에 들어오면 안내 1번
#define ARRIVAL_RADIUS_M 5.0      // 도달 판정 반경
#define OFF_ROUTE_DISTANCE_M 20.0 // 경로선에서 이만큼 벗어나면 이탈 후보
#define ON_ROUTE_DISTANCE_M 5.0   // 이 안으로 오면 이탈 해제
#define OFF_ROUTE_CONFIRM_CNT 4   // 4번(4초) 연속이면 이탈 확정
#define REROUTE_COOLDOWN_MS 15000 // 재탐색 직후 바로 또 안 하게
#define REROUTE_RETRY_BASE_MS 10000 // 재탐색 실패 시 재시도 간격 (2배씩 늘림)
#define REROUTE_RETRY_MAX_MS 60000
#define GPS_FRESH_MS 5000      // GPS 5초 넘게 안 오면 이탈 판정 안 함
#define PASS_MARGIN_SEGMENTS 2 // 지점 통과 판정 여유 (정점 개수)
#define PASS_CONFIRM_CNT 2
// 5초마다 상태 로그
// 루프는 1초 그대로 둬야 함. 5초로 바꾸면 이탈 확정이 20초 걸림
#define GUIDANCE_LOG_INTERVAL_MS 5000

// 지팡이 지자기 방위로 GPS 보정. 라파가 "HDG,<방위>,<ok>,<moving>" 으로 넘겨줌
// 방위 없거나 ok=0이면 보정 안 하고 원래대로
#define USE_HEADING_ASSIST 1 // 0이면 보정 끔 (비교 테스트용)
#define HEADING_FRESH_MS 1000 // 1초 넘게 안 오면 방위 없는 걸로
// A. 멈춰 있을 때 GPS 평균내서 고정 (신호 기다릴 때 위치 튀는 거 방지)
#define HOLD_MAX_SAMPLES 30
#define GPS_MOVING_OVERRIDE_MPS 0.7 // 지팡이는 정지인데 GPS 속도가 이거 넘으면 걷는 걸로
#define MOTION_OVERRIDE_CONFIRM 3   // 3번 연속일 때
// B. 걷는 방향 반대로 크게 튄 좌표는 버림
#define JUMP_MIN_M 4.0
#define JUMP_HEADING_DEG 100.0
#define JUMP_MAX_REJECT 3 // 3번 연속이면 진짜 이동으로 봄

typedef struct { double lon; double lat; int turnType; int facilityType; char facilityName[32]; char description[128]; } Waypoint_t;
typedef struct { double lat; double lon; } LatLon_t;

Waypoint_t route_points[MAX_WAYPOINTS];
int total_waypoints = 0;
int current_target_idx = 0;
static TaskHandle_t navigation_task_handle = NULL;

// 목적지 명령 큐 (크기 1). 요청 두 개가 동시에 돌면 경로 배열이 꼬여서 하나씩 처리
static QueueHandle_t route_cmd_q = NULL;

// 경로선 (LineString 좌표 이어붙인 것). 이탈 판정 기준
static LatLon_t route_line[MAX_ROUTE_LINE_POINTS];
static int route_line_count = 0;
// 안내지점별로 경로선에서 제일 가까운 정점 번호
static int waypoint_line_idx[MAX_WAYPOINTS];

// 지금 가는 목적지 (재탐색할 때 씀)
static int current_dest_idx = -1;
// 안내 태스크 멈추는 플래그. HTTP 중에 vTaskDelete 하면 메모리 새서 이걸로 멈춤
static volatile bool nav_stop_requested = false;

// 목적지 목록 (라파가 0x01/0x02로 선택)
typedef struct { double lat; double lon; const char *name; } Destination_t;
static const Destination_t DESTINATIONS[] = {
    { 35.88407372414365, 128.61325021409556, "목적지1(버스정류장)" },
    { 35.88708878857774, 128.6121873917122, "목적지2(학교)" },
};
#define DESTINATION_COUNT (sizeof(DESTINATIONS) / sizeof(DESTINATIONS[0]))

void navigation_task(void *pvParameters);
void fetch_tmap_pedestrian_route(double dest_lat, double dest_lon);
void start_navigation_to_destination(int dest_idx);
static bool try_reroute(void);

// 내 위치 (GPS 오기 전엔 이 기본값 사용)
double current_my_lon = 128.61161779437046;
double current_my_lat = 35.887123468705354;

// GPS 한 번이라도 받았는지
bool has_real_gps_fix = false;

// 마지막 GPS 받은 시간. GPS 끊긴 건지 진짜 이탈인지 구분용
static volatile TickType_t last_gps_tick = 0;

// 위도/경도 따로 읽다가 중간에 값 바뀌면 좌표가 섞여서 락 걸어둠
static portMUX_TYPE gps_mux = portMUX_INITIALIZER_UNLOCKED;

static void gps_update(double lat, double lon) {
    portENTER_CRITICAL(&gps_mux);
    current_my_lat = lat;
    current_my_lon = lon;
    last_gps_tick = xTaskGetTickCount();
    portEXIT_CRITICAL(&gps_mux);
}

// 현재 위치 읽기. fresh = 최근 5초 안에 GPS 들어왔는지
static void gps_snapshot(double *lat, double *lon, bool *fresh) {
    portENTER_CRITICAL(&gps_mux);
    double snap_lat = current_my_lat;
    double snap_lon = current_my_lon;
    TickType_t snap_tick = last_gps_tick;
    bool has_fix = has_real_gps_fix;
    portEXIT_CRITICAL(&gps_mux);

    if (lat) *lat = snap_lat;
    if (lon) *lon = snap_lon;
    if (fresh) {
        *fresh = has_fix &&
                 (xTaskGetTickCount() - snap_tick) < pdMS_TO_TICKS(GPS_FRESH_MS);
    }
}

const char *current_position_source(void) {
    return has_real_gps_fix ? "실시간 GPS" : "기본값(하드코딩)";
}

// 지팡이 방위 (위치랑 같은 이유로 락)
static portMUX_TYPE hdg_mux = portMUX_INITIALIZER_UNLOCKED;
static float hdg_deg = 0.0f;
static bool hdg_ok = false, hdg_moving = false, hdg_ever = false;
static TickType_t hdg_tick = 0;

static void heading_update(float deg, bool ok, bool moving) {
    portENTER_CRITICAL(&hdg_mux);
    hdg_deg = deg;
    hdg_ok = ok;
    hdg_moving = moving;
    hdg_tick = xTaskGetTickCount();
    hdg_ever = true;
    portEXIT_CRITICAL(&hdg_mux);
}

// 방위 최근 거면 true. ok = 자기장 교란 없음, moving = 지팡이 움직이는 중
static bool heading_snapshot(float *deg, bool *ok, bool *moving) {
    portENTER_CRITICAL(&hdg_mux);
    float d = hdg_deg;
    bool o = hdg_ok, m = hdg_moving, ever = hdg_ever;
    TickType_t tick = hdg_tick;
    portEXIT_CRITICAL(&hdg_mux);

    if (!ever || (xTaskGetTickCount() - tick) > pdMS_TO_TICKS(HEADING_FRESH_MS)) return false;
    if (deg) *deg = d;
    if (ok) *ok = o;
    if (moving) *moving = m;
    return true;
}

// 로그 출력용 보정 상태
static volatile bool gps_hold_active = false;
static volatile uint32_t gps_jump_reject_total = 0;

// 라파로 신호 1바이트 보내기
static void usb_send_signal(uint8_t signal) {
    usb_serial_jtag_write_bytes(&signal, 1, pdMS_TO_TICKS(100));
}

/* --------------------------------------------------------------------
 * 이벤트 판정 (TMAP 보행자 경로 코드)
 *   turnType : 12/16/17 좌회전, 13/18/19 우회전, 211~217 횡단보도
 *
 *   facilityType 숫자는 TMAP 문서랑 실제 값이 안 맞아서 안 씀. facilityName 글자로 판정
 *   (facility 정보는 LineString에 붙어 있어서 parse_route_body에서 앞 지점에 옮겨둠)
 * ------------------------------------------------------------------ */
static bool is_turn_left(int turnType) {
    return turnType == 12 || turnType == 16 || turnType == 17;
}
static bool is_turn_right(int turnType) {
    return turnType == 13 || turnType == 18 || turnType == 19;
}
static bool is_crosswalk(const Waypoint_t *wp) {
    return strstr(wp->facilityName, "횡단보도") != NULL ||
           (wp->turnType >= 211 && wp->turnType <= 217);
}
static bool is_stairs(const Waypoint_t *wp) {
    return strstr(wp->facilityName, "계단") != NULL;
}

// 로그용 동작 문구 (예: "좌회전 + 횡단보도")
static void maneuver_label(const Waypoint_t *wp, char *out, size_t out_size) {
    const char *turn = "직진";
    if (is_turn_left(wp->turnType)) turn = "좌회전";
    else if (is_turn_right(wp->turnType)) turn = "우회전";

    snprintf(out, out_size, "%s%s%s", turn,
             is_crosswalk(wp) ? " + 횡단보도" : "",
             is_stairs(wp) ? " + 계단" : "");
}

// 안내지점 이벤트 신호 전송 (회전+횡단보도처럼 여러 개면 다 보냄)
static void send_guidance_event_signals(const Waypoint_t *wp) {
    if (is_turn_left(wp->turnType)) {
        ESP_LOGI(TAG, "📤 [이벤트 신호] 좌회전 (0x03)");
        usb_send_signal(SIGNAL_TURN_LEFT);
    } else if (is_turn_right(wp->turnType)) {
        ESP_LOGI(TAG, "📤 [이벤트 신호] 우회전 (0x04)");
        usb_send_signal(SIGNAL_TURN_RIGHT);
    }

    if (is_crosswalk(wp)) {
        ESP_LOGI(TAG, "📤 [이벤트 신호] 횡단보도 (0x05)");
        usb_send_signal(SIGNAL_CROSSWALK);
    }

    if (is_stairs(wp)) {
        ESP_LOGI(TAG, "📤 [이벤트 신호] 계단 (0x06)");
        usb_send_signal(SIGNAL_STAIRS);
    }
}

// TMAP 응답 로그 찍을 때 ASCII만 남김 (0xFE/0xFF 같은 신호 바이트가 섞여 나갈 수 있어서)
static void sanitize_for_log(const char *src, char *dst, size_t dst_size) {
    size_t i = 0;
    for (; src[i] != '\0' && i < dst_size - 1; i++) {
        unsigned char ch = (unsigned char)src[i];
        dst[i] = (ch >= 0x20 && ch < 0x7F) ? (char)ch : '.';
    }
    dst[i] = '\0';
}

double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double dLat = (lat2 - lat1) * MATH_PI / 180.0;
    double dLon = (lon2 - lon1) * MATH_PI / 180.0;
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * MATH_PI / 180.0) * cos(lat2 * MATH_PI / 180.0) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

/* --------------------------------------------------------------------
 * 점-선분 거리 (m)
 *   거리가 짧아서 haversine 대신 평면 근사로 계산
 *   (A 기준으로 미터 좌표로 바꾸고 선분에 투영)
 * ------------------------------------------------------------------ */
static double point_to_segment_meters(double plat, double plon,
                                      double alat, double alon,
                                      double blat, double blon) {
    const double M_PER_DEG_LAT = 110540.0;
    double m_per_deg_lon = 111320.0 * cos(plat * MATH_PI / 180.0);

    double px = (plon - alon) * m_per_deg_lon, py = (plat - alat) * M_PER_DEG_LAT;
    double bx = (blon - alon) * m_per_deg_lon, by = (blat - alat) * M_PER_DEG_LAT;

    double seg_len2 = bx * bx + by * by;
    double t = (seg_len2 <= 0.0) ? 0.0 : (px * bx + py * by) / seg_len2;
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;

    double dx = px - t * bx, dy = py - t * by;
    return sqrt(dx * dx + dy * dy);
}

// 경로선까지 최단거리(m). out_seg_idx = 제일 가까운 구간 번호
// 경로선 없으면 -1
static double distance_to_route_meters(double lat, double lon, int *out_seg_idx) {
    if (route_line_count < 2) {
        if (out_seg_idx) *out_seg_idx = -1;
        return -1.0;
    }

    double best = -1.0;
    int best_idx = 0;
    for (int i = 0; i < route_line_count - 1; i++) {
        double d = point_to_segment_meters(lat, lon,
                                           route_line[i].lat, route_line[i].lon,
                                           route_line[i + 1].lat, route_line[i + 1].lon);
        if (best < 0.0 || d < best) {
            best = d;
            best_idx = i;
        }
    }
    if (out_seg_idx) *out_seg_idx = best_idx;
    return best;
}

// 안내지점마다 가장 가까운 경로선 정점 찾아두기 (경로 받을 때 1번)
static void build_waypoint_line_index(void) {
    for (int i = 0; i < total_waypoints; i++) {
        waypoint_line_idx[i] = 0;
        if (route_line_count < 1) continue;

        double best = -1.0;
        for (int j = 0; j < route_line_count; j++) {
            double d = calculate_distance(route_points[i].lat, route_points[i].lon,
                                          route_line[j].lat, route_line[j].lon);
            if (best < 0.0 || d < best) {
                best = d;
                waypoint_line_idx[i] = j;
            }
        }
    }
}

/* ====================================================================
 * 1. 라파 GPS 문자열 파싱
 *   형식: "GPS,<위도>,<경도>\n"  (예: "GPS,35.887123,128.611617\n")
 *   라파에서 이미 십진수 위경도로 바꿔서 보내줌
 * ==================================================================== */
bool parse_gps_line(const char *line, double *lat, double *lon) {
    if (strncmp(line, "GPS,", 4) != 0) return false;

    double parsed_lat, parsed_lon;
    if (sscanf(line + 4, "%lf,%lf", &parsed_lat, &parsed_lon) != 2) return false;

    // 범위 밖이면 버림
    if (parsed_lat < -90.0 || parsed_lat > 90.0 || parsed_lon < -180.0 || parsed_lon > 180.0) return false;

    *lat = parsed_lat;
    *lon = parsed_lon;
    return true;
}

// 지팡이 방위: "HDG,<방위>,<ok 0/1>,<moving 0/1>"
static bool parse_heading_line(const char *line, float *deg, bool *ok, bool *moving) {
    if (strncmp(line, "HDG,", 4) != 0) return false;
    float d;
    int o, m;
    if (sscanf(line + 4, "%f,%d,%d", &d, &o, &m) != 3) return false;
    if (!(d >= 0.0f && d <= 360.0f)) return false; // NaN도 여기서 걸림
    *deg = (d >= 360.0f) ? d - 360.0f : d;
    *ok = o != 0;
    *moving = m != 0;
    return true;
}

// 방위각 (북 0°, 시계방향). 평면 근사
static double bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    double m_per_deg_lon = 111320.0 * cos(lat1 * MATH_PI / 180.0);
    double b = atan2((lon2 - lon1) * m_per_deg_lon, (lat2 - lat1) * 110540.0) * 180.0 / MATH_PI;
    return b < 0.0 ? b + 360.0 : b;
}

static double wrap180(double d) {
    while (d > 180.0) d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return d;
}

typedef enum { GPS_TAKEN, GPS_HELD, GPS_REJECTED } GpsAccept_t;

/* ====================================================================
 * 지자기 방위로 GPS 보정 (gps_update 전에 거름)
 *   A. 멈춰 있으면 GPS 평균내서 고정
 *   B. 걷는 방향 반대로 크게 튄 좌표는 버림 (건물 옆에서 자주 튐)
 *   방위 없으면 그냥 받은 좌표 씀
 *   고정/버릴 때도 gps_update는 불러서 수신 시간은 갱신 (안 하면 GPS 끊긴 걸로 됨)
 * ==================================================================== */
static GpsAccept_t gps_accept(double lat, double lon) {
    static bool have_prev = false, have_acc = false;
    static double prev_lat, prev_lon, acc_lat, acc_lon, hold_lat, hold_lon;
    static TickType_t prev_tick;
    static int fast_cnt = 0, hold_n = 0, reject_cnt = 0;

    // GPS 속도. 지팡이 moving 값이 틀려도 계속 고정되는 거 막으려고
    TickType_t now = xTaskGetTickCount();
    double v = 0.0;
    if (have_prev) {
        double sec = (double)(now - prev_tick) * portTICK_PERIOD_MS / 1000.0;
        if (sec >= 0.2) {
            v = calculate_distance(prev_lat, prev_lon, lat, lon) / sec;
            fast_cnt = (v >= GPS_MOVING_OVERRIDE_MPS) ? fast_cnt + 1 : 0;
        }
    }
    prev_lat = lat;
    prev_lon = lon;
    prev_tick = now;
    have_prev = true;

    float hdg = 0.0f;
    bool ok = false, moving = false;
    if (!USE_HEADING_ASSIST || !heading_snapshot(&hdg, &ok, &moving)) {
        hold_n = 0;
        reject_cnt = 0;
        gps_hold_active = false;
        acc_lat = lat;
        acc_lon = lon;
        have_acc = true;
        gps_update(lat, lon);
        return GPS_TAKEN;
    }

    /* A. 정지 중 고정 */
    bool gps_says_moving = fast_cnt >= MOTION_OVERRIDE_CONFIRM;
    if (!moving && !gps_says_moving) {
        if (hold_n == 0) {
            hold_lat = lat;
            hold_lon = lon;
            ESP_LOGI(TAG, "🛑 [정지 평균] 지팡이가 멈춰 있어 GPS 흔들림을 평균으로 고정합니다.");
        }
        if (hold_n < HOLD_MAX_SAMPLES) hold_n++;
        hold_lat += (lat - hold_lat) / hold_n;
        hold_lon += (lon - hold_lon) / hold_n;
        gps_hold_active = true;
        reject_cnt = 0;
        acc_lat = hold_lat;
        acc_lon = hold_lon;
        have_acc = true;
        gps_update(hold_lat, hold_lon);
        return GPS_HELD;
    }
    if (hold_n > 0) {
        if (!moving) {
            ESP_LOGW(TAG, "▶️ [정지 평균 해제] 지팡이는 정지인데 GPS가 %.1fm/s로 움직여 걷는 중으로 봅니다.", v);
        } else {
            ESP_LOGI(TAG, "▶️ [정지 평균 해제] 지팡이가 다시 움직입니다.");
        }
        hold_n = 0;
        gps_hold_active = false;
    }

    /* B. 튄 좌표 버리기 (ok일 때만) */
    if (ok && moving && have_acc) {
        double d = calculate_distance(acc_lat, acc_lon, lat, lon);
        double diff = fabs(wrap180(bearing_deg(acc_lat, acc_lon, lat, lon) - hdg));
        if (d > JUMP_MIN_M && diff > JUMP_HEADING_DEG) {
            if (++reject_cnt < JUMP_MAX_REJECT) {
                gps_jump_reject_total++;
                ESP_LOGW(TAG, "🚫 [GPS 튐 무시] 걷는 방향(%.0f°)과 %.0f° 다른 쪽으로 %.0fm 튀었습니다. (%d/%d)",
                         hdg, diff, d, reject_cnt, JUMP_MAX_REJECT - 1);
                gps_update(acc_lat, acc_lon); // 위치 그대로, 시간만 갱신
                return GPS_REJECTED;
            }
            ESP_LOGW(TAG, "↪️ [GPS 튐 판정 해제] %d번 연속이라 실제 위치 변화로 보고 받아들입니다.", reject_cnt);
        }
    }
    reject_cnt = 0;
    acc_lat = lat;
    acc_lon = lon;
    have_acc = true;
    gps_update(lat, lon);
    return GPS_TAKEN;
}

// 목적지 명령을 큐에서 하나씩 꺼내 처리 (GPS 수신 루프가 HTTP 때문에 안 멈추게)
// 명령마다 태스크 새로 만들었더니 연달아 오면 안내 태스크가 2개 떠서 이렇게 바꿈
void route_request_task(void *pvParameters) {
    int dest_idx;
    while (1) {
        if (xQueueReceive(route_cmd_q, &dest_idx, portMAX_DELAY) == pdTRUE) {
            start_navigation_to_destination(dest_idx);
        }
    }
}

/* ====================================================================
 * 2. USB-CDC로 라파 GPS/명령 받는 태스크
 *   - 콘솔 로그도 같은 포트라 드라이버가 이미 깔려 있을 수 있음. 있으면 그대로 씀
 *   - GPS 줄이랑 명령 바이트(0x01/0x02)가 섞여 들어와서 명령은 먼저 걸러냄
 * ==================================================================== */
void gps_usb_task(void *pvParameters) {
    ESP_LOGI(TAG, "🛰️ 라즈베리파이 GPS/명령 데이터 수신 대기 중... (USB-CDC)");

    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    }

    uint8_t rx_data[128];
    char line_buffer[256];
    int line_pos = 0;

    while (1) {
        int len = usb_serial_jtag_read_bytes(rx_data, sizeof(rx_data) - 1, pdMS_TO_TICKS(20));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t raw = rx_data[i];

                if (raw == CMD_DEST_1 || raw == CMD_DEST_2) {
                    int dest_idx = (raw == CMD_DEST_1) ? 0 : 1;
                    // 안 기다리고 덮어씀 (마지막 명령 우선)
                    xQueueOverwrite(route_cmd_q, &dest_idx);
                    continue;
                }

                char c = (char)raw;
                if (c == '\n' || c == '\r') {
                    if (line_pos > 0) {
                        line_buffer[line_pos] = '\0';

                        double real_lat, real_lon;
                        float hdg_val;
                        bool hdg_ok_val, hdg_moving_val;
                        if (parse_gps_line(line_buffer, &real_lat, &real_lon)) {
                            if (!has_real_gps_fix) {
                                has_real_gps_fix = true;
                                ESP_LOGI(TAG, "✅ [GPS 최초 수신] 기본값 위치 -> 실제 위치로 전환합니다.");
                            }
                            GpsAccept_t res = gps_accept(real_lat, real_lon);
                            ESP_LOGI(TAG, "🌍 [실시간 GPS] 위도: %f, 경도: %f%s", real_lat, real_lon,
                                     res == GPS_HELD ? " (정지 평균)" : (res == GPS_REJECTED ? " (튐 무시)" : ""));
                        } else if (parse_heading_line(line_buffer, &hdg_val, &hdg_ok_val, &hdg_moving_val)) {
                            heading_update(hdg_val, hdg_ok_val, hdg_moving_val);
                            // 5Hz라 처음 한 번만 찍음 (나머진 5초 로그에 나옴)
                            static bool first_heading = true;
                            if (first_heading) {
                                first_heading = false;
                                ESP_LOGI(TAG, "🧭 [방위 수신 시작] 지팡이 방위 %.0f° (품질 %s)",
                                         hdg_val, hdg_ok_val ? "정상" : "교란");
                            }
                        }
                        line_pos = 0;
                    }
                } else {
                    if (line_pos < sizeof(line_buffer) - 1) {
                        line_buffer[line_pos++] = c;
                    } else {
                        // 줄바꿈 없이 버퍼 꽉 참 -> 버림
                        ESP_LOGW(TAG, "⚠️ GPS 라인 버퍼 오버플로우, 리셋합니다.");
                        line_pos = 0;
                    }
                }
            }
        }
    }
}

/* ====================================================================
 * 3. 길안내 태스크 (1초마다)
 * ==================================================================== */
void navigation_task(void *pvParameters) {
    ESP_LOGI(TAG, "🚀 [실시간 길 안내 태스크] 시작됨!");

    if (!has_real_gps_fix) {
        ESP_LOGW(TAG, "⚠️ 아직 GPS 실측값을 못 받아서 기본값 위치(%.6f, %.6f) 기준으로 안내를 시작합니다.",
                 current_my_lat, current_my_lon);
    }

    if (total_waypoints > 0) {
        ESP_LOGI(TAG, "=============================================");
        ESP_LOGI(TAG, "📣 길 안내를 시작합니다.");
        ESP_LOGI(TAG, "👉 [초기 방향] %s", route_points[0].description);
        ESP_LOGI(TAG, "=============================================\n");
    }

    int off_route_count = 0, pass_count = 0;
    bool off_route = false;
    TickType_t next_reroute_allowed = 0;
    uint32_t retry_ms = REROUTE_RETRY_BASE_MS;
    TickType_t next_guidance_log = 0; // 처음엔 바로 로그
    int last_logged_idx = -1;         // 목표 바뀌면 바로 로그
    int announced_idx = -1;           // 안내 보낸 지점 (지점당 1번)

    while (!nav_stop_requested && current_target_idx < total_waypoints) {
        double my_lat, my_lon;
        bool fresh;
        gps_snapshot(&my_lat, &my_lon, &fresh);

        // GPS 살아있고 경로선 있을 때만 이탈/통과 판정
        int seg_idx = -1;
        double off_dist = fresh ? distance_to_route_meters(my_lat, my_lon, &seg_idx) : -1.0;
        bool can_judge = (off_dist >= 0.0);

        /* (A) 이탈 판정 (20m 밖 4번 연속이면 이탈, 5m 안으로 오면 해제) */
        if (can_judge) {
            if (!off_route) {
                if (off_dist > OFF_ROUTE_DISTANCE_M) {
                    if (++off_route_count >= OFF_ROUTE_CONFIRM_CNT) {
                        off_route = true;
                        ESP_LOGW(TAG, "=============================================");
                        ESP_LOGW(TAG, "⚠️ [경로 이탈] 경로에서 %.1fm 벗어났습니다!", off_dist);
                        ESP_LOGW(TAG, "=============================================\n");
                        usb_send_signal(SIGNAL_OFF_ROUTE);
                    }
                } else {
                    off_route_count = 0;
                }
            } else if (off_dist < ON_ROUTE_DISTANCE_M) {
                // 재탐색 전에 경로로 돌아옴
                off_route = false;
                off_route_count = 0;
                retry_ms = REROUTE_RETRY_BASE_MS;
                ESP_LOGI(TAG, "✅ [경로 복귀] 다시 경로 위로 돌아왔습니다. 기존 안내를 계속합니다.");
                usb_send_signal(SIGNAL_BACK_ON_ROUTE);
            }
        } else {
            off_route_count = 0;
        }

        /* (B) 재탐색 */
        if (off_route && xTaskGetTickCount() >= next_reroute_allowed) {
            if (try_reroute()) {
                off_route = false;
                off_route_count = 0;
                pass_count = 0;
                // 새 경로라 인덱스 0부터. 이것도 초기화 안 하면 첫 지점 안내가 안 나감
                announced_idx = -1;
                last_logged_idx = -1;
                retry_ms = REROUTE_RETRY_BASE_MS;
                next_reroute_allowed = xTaskGetTickCount() + pdMS_TO_TICKS(REROUTE_COOLDOWN_MS);
                continue; // 새 경로로 다시
            }
            next_reroute_allowed = xTaskGetTickCount() + pdMS_TO_TICKS(retry_ms);
            retry_ms = (retry_ms * 2 > REROUTE_RETRY_MAX_MS) ? REROUTE_RETRY_MAX_MS : retry_ms * 2;
        }

        /* (C) 지점 통과 감지
         *   반경 밖으로 지나가면 인덱스가 안 넘어가서 안내가 멈췄음
         *   이미 지나간 지점은 안내 없이 그냥 넘김 */
        if (!off_route && can_judge &&
            seg_idx > waypoint_line_idx[current_target_idx] + PASS_MARGIN_SEGMENTS) {
            if (++pass_count >= PASS_CONFIRM_CNT) {
                ESP_LOGW(TAG, "⏭️ [안내지점 통과] '%s'를 이미 지나쳐 다음 지점으로 넘어갑니다.",
                         route_points[current_target_idx].description);
                current_target_idx++;
                pass_count = 0;
                continue;
            }
        } else {
            pass_count = 0;
        }

        Waypoint_t target = route_points[current_target_idx];
        double distance = calculate_distance(my_lat, my_lon, target.lat, target.lon);

        /* (D) 5초마다 상태 로그 + 목표 바뀌면 바로
         *   GPS 오차 때문에 지점 근처를 못 밟으면 로그가 아예 안 나와서 계속 찍게 함
         *   (라파로 가는 신호는 (E)에서 지점당 1번) */
        if (xTaskGetTickCount() >= next_guidance_log || current_target_idx != last_logged_idx) {
            next_guidance_log = xTaskGetTickCount() + pdMS_TO_TICKS(GUIDANCE_LOG_INTERVAL_MS);
            last_logged_idx = current_target_idx;

            if (off_route) {
                // 이탈 중엔 방향 안내 안 함
                ESP_LOGW(TAG, "🧭 [안내] 경로를 이탈했습니다. 경로선에서 %.0fm 이격 | 재탐색 대기 중", off_dist);
            } else if (!fresh) {
                ESP_LOGW(TAG, "🧭 [안내] GPS 신호를 기다리는 중 (%s 기준) | 다음: %s",
                         current_position_source(), target.description);
            } else {
                char label[48];
                maneuver_label(&target, label, sizeof(label));
                // 지자기 상태 (확인용)
                char assist[96];
                float hdg_now;
                bool hdg_ok_now, hdg_moving_now;
                if (heading_snapshot(&hdg_now, &hdg_ok_now, &hdg_moving_now)) {
                    snprintf(assist, sizeof(assist), " | 방위 %.0f°%s | %s | 튐 무시 누적 %lu회",
                             hdg_now, hdg_ok_now ? "" : "(교란)",
                             gps_hold_active ? "정지 평균 중" : (hdg_moving_now ? "이동" : "정지"),
                             (unsigned long)gps_jump_reject_total);
                } else {
                    snprintf(assist, sizeof(assist), " | 방위 없음");
                }
                if (can_judge) {
                    ESP_LOGI(TAG, "🧭 [안내 %d/%d] %.0fm 앞 %s | %s | 경로선 이격 %.0fm%s",
                             current_target_idx + 1, total_waypoints,
                             distance, label, target.description, off_dist, assist);
                } else {
                    // 경로선 없으면 이격 거리 빼고
                    ESP_LOGI(TAG, "🧭 [안내 %d/%d] %.0fm 앞 %s | %s%s",
                             current_target_idx + 1, total_waypoints,
                             distance, label, target.description, assist);
                }
            }
        }

        /* (E) 접근 안내 - ANNOUNCE_DISTANCE_M 안에 들어오면 1번 보냄
         *   이탈 중이거나 GPS 끊겼으면 안 보냄 */
        if (!off_route && fresh && distance <= ANNOUNCE_DISTANCE_M &&
            announced_idx != current_target_idx) {
            announced_idx = current_target_idx;
            ESP_LOGI(TAG, "=============================================");
            ESP_LOGI(TAG, "👉 [방향 안내 | %.0fm 앞] %s", distance, target.description);
            if (is_crosswalk(&target)) ESP_LOGW(TAG, "🚨 [부저 ON 띠띠띠!] 전방에 횡단보도가 있습니다!");
            else if (is_stairs(&target)) ESP_LOGW(TAG, "🚨 [부저 ON 띠띠띠!] 전방에 계단이 있습니다! 조심하세요!");
            ESP_LOGI(TAG, "=============================================\n");
            usb_send_signal(SIGNAL_NEXT_DIRECTION); // 0xFE
            send_guidance_event_signals(&target);   // 0x03~0x06
        }

        /* (F) 도달하면 다음 지점으로 */
        if (distance <= ARRIVAL_RADIUS_M) {
            current_target_idx++;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (nav_stop_requested) {
        // 새 목적지 때문에 멈춘 거라 도착 신호 X
        ESP_LOGI(TAG, "🛑 기존 안내를 중단합니다.");
    } else {
        ESP_LOGI(TAG, "🏁 목적지 부근에 도착했습니다. 안내를 종료합니다.");
        usb_send_signal(SIGNAL_ARRIVED); // 0xFF
    }
    navigation_task_handle = NULL;
    vTaskDelete(NULL);
}


/* ====================================================================
 * 와이파이 / TMAP API
 * ==================================================================== */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_init(); esp_event_loop_create_default(); esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS, }, };
    esp_wifi_set_mode(WIFI_MODE_STA); esp_wifi_set_config(WIFI_IF_STA, &wifi_config); esp_wifi_start();
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

typedef struct { char *buffer; int len; bool truncated; } response_data_t;
static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    response_data_t *resp = (response_data_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && resp->buffer) {
        if (resp->len + evt->data_len < HTTP_RESP_BUF_SIZE) {
            memcpy(resp->buffer + resp->len, evt->data, evt->data_len);
            resp->len += evt->data_len;
            resp->buffer[resp->len] = '\0';
        } else {
            // 버퍼 넘침. 표시해두고 나중에 에러 로그
            resp->truncated = true;
        }
    }
    return ESP_OK;
}

/* --------------------------------------------------------------------
 * TMAP 요청 / 파싱 따로 분리
 *   재탐색 실패해도 기존 경로 날아가지 않게, 응답이 괜찮을 때만 경로 교체
 * ------------------------------------------------------------------ */

// HTTP 요청. 성공하면 응답 반환 (free 해줘야 함), 실패하면 NULL
static char *tmap_request_route_body(double dest_lat, double dest_lon) {
    char *local_response_buffer = (char *)malloc(HTTP_RESP_BUF_SIZE);
    if (!local_response_buffer) return NULL;
    memset(local_response_buffer, 0, HTTP_RESP_BUF_SIZE);
    response_data_t resp = { .buffer = local_response_buffer, .len = 0, .truncated = false };

    esp_http_client_config_t config = {
        .url = "https://apis.openapi.sk.com/tmap/routes/pedestrian?version=1&format=json",
        .method = HTTP_METHOD_POST,
        .event_handler = http_event_handler,
        .user_data = &resp,
        .buffer_size = 2048,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "appKey", TMAP_APP_KEY);

    double start_lat, start_lon;
    gps_snapshot(&start_lat, &start_lon, NULL);
    ESP_LOGI(TAG, "📍 출발지 위치 기준: %s (위도: %f, 경도: %f)",
             current_position_source(), start_lat, start_lon);

    char post_data[256];
    snprintf(post_data, sizeof(post_data),
        "{\"startX\":\"%f\",\"startY\":\"%f\",\"endX\":\"%f\",\"endY\":\"%f\",\"startName\":\"출발지\",\"endName\":\"목적지\",\"reqCoordType\":\"WGS84GEO\",\"resCoordType\":\"WGS84GEO\"}",
        start_lon, start_lat, dest_lon, dest_lat);

    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_err_t err = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status_code != 200) {
        char safe_body[256];
        sanitize_for_log(local_response_buffer, safe_body, sizeof(safe_body));
        ESP_LOGE(TAG, "❌ TMAP 요청 실패! err=%s, status_code=%d, 응답: %s",
                 esp_err_to_name(err), status_code, safe_body);
        free(local_response_buffer);
        return NULL;
    }

    if (resp.truncated) {
        ESP_LOGE(TAG, "❌ TMAP 응답이 수신 버퍼(%d바이트)를 넘겨 잘렸습니다. 경로가 너무 길거나 HTTP_RESP_BUF_SIZE를 키워야 합니다.",
                 HTTP_RESP_BUF_SIZE);
        free(local_response_buffer);
        return NULL;
    }

    return local_response_buffer;
}

// 응답 파싱해서 경로 저장. 안내지점 개수 반환 (0이면 기존 경로 그대로)
static int parse_route_body(const char *body) {
    cJSON *root = cJSON_Parse(body);
    if (!root) {
        char safe_body[256];
        sanitize_for_log(body, safe_body, sizeof(safe_body));
        ESP_LOGE(TAG, "❌ TMAP 응답 JSON 파싱 실패! 원본 응답: %s", safe_body);
        return 0;
    }

    cJSON *features = cJSON_GetObjectItem(root, "features");
    int array_size = cJSON_IsArray(features) ? cJSON_GetArraySize(features) : 0;

    // 안내지점 있는지 먼저 확인 (빈 응답이면 기존 경로 유지)
    int usable_points = 0;
    for (int i = 0; i < array_size; i++) {
        cJSON *geometry = cJSON_GetObjectItem(cJSON_GetArrayItem(features, i), "geometry");
        cJSON *type = geometry ? cJSON_GetObjectItem(geometry, "type") : NULL;
        if (type && cJSON_IsString(type) && strcmp(type->valuestring, "Point") == 0) usable_points++;
    }
    if (usable_points == 0) {
        ESP_LOGE(TAG, "❌ TMAP 응답에 안내 지점이 없습니다. (features %d개)", array_size);
        cJSON_Delete(root);
        return 0;
    }

    // 기존 경로 교체
    total_waypoints = 0;
    route_line_count = 0;
    current_target_idx = 0;
    bool point_overflow = false, line_overflow = false;

    for (int i = 0; i < array_size; i++) {
        cJSON *feature = cJSON_GetArrayItem(features, i);
        cJSON *geometry = cJSON_GetObjectItem(feature, "geometry");
        cJSON *properties = cJSON_GetObjectItem(feature, "properties");
        if (!geometry) continue;

        cJSON *type = cJSON_GetObjectItem(geometry, "type");
        if (!type || !cJSON_IsString(type)) continue;
        const char *geo_type = type->valuestring;
        cJSON *coords = cJSON_GetObjectItem(geometry, "coordinates");
        if (!coords) continue;

        if (properties && strcmp(geo_type, "Point") == 0) {
            if (cJSON_GetArraySize(coords) != 2) continue;
            if (total_waypoints >= MAX_WAYPOINTS) { point_overflow = true; continue; }

            // facility 정보는 다음 LineString에 있어서 일단 비워두고 아래에서 채움
            Waypoint_t *wp = &route_points[total_waypoints];
            wp->lon = cJSON_GetArrayItem(coords, 0)->valuedouble;
            wp->lat = cJSON_GetArrayItem(coords, 1)->valuedouble;
            cJSON *turn = cJSON_GetObjectItem(properties, "turnType");
            wp->turnType = turn ? turn->valueint : 0;
            wp->facilityType = 0;
            wp->facilityName[0] = '\0';
            wp->description[0] = '\0';
            cJSON *desc = cJSON_GetObjectItem(properties, "description");
            if (desc && desc->valuestring) {
                strncpy(wp->description, desc->valuestring, sizeof(wp->description) - 1);
                wp->description[sizeof(wp->description) - 1] = '\0';
            }
            total_waypoints++;
        } else if (strcmp(geo_type, "LineString") == 0) {
            // 경로선 좌표 저장 (이탈 판정용)
            int n = cJSON_GetArraySize(coords);
            for (int j = 0; j < n; j++) {
                cJSON *pair = cJSON_GetArrayItem(coords, j);
                if (cJSON_GetArraySize(pair) != 2) continue;
                if (route_line_count >= MAX_ROUTE_LINE_POINTS) { line_overflow = true; break; }
                route_line[route_line_count].lon = cJSON_GetArrayItem(pair, 0)->valuedouble;
                route_line[route_line_count].lat = cJSON_GetArrayItem(pair, 1)->valuedouble;
                route_line_count++;
            }

            // 응답 순서가 Point -> LineString -> Point ... 라서
            // 이 구간의 시설물(계단, 횡단보도)을 앞 지점에 붙여둠 (그 지점에서 미리 알려주려고)
            if (properties && total_waypoints > 0) {
                Waypoint_t *prev = &route_points[total_waypoints - 1];
                cJSON *ftype = cJSON_GetObjectItem(properties, "facilityType");
                cJSON *fname = cJSON_GetObjectItem(properties, "facilityName");
                if (ftype) prev->facilityType = ftype->valueint;
                if (fname && fname->valuestring) {
                    strncpy(prev->facilityName, fname->valuestring, sizeof(prev->facilityName) - 1);
                    prev->facilityName[sizeof(prev->facilityName) - 1] = '\0';
                }
            }
        }
    }

    cJSON_Delete(root);

    if (point_overflow) ESP_LOGW(TAG, "⚠️ 안내 지점이 MAX_WAYPOINTS(%d)를 넘어 잘렸습니다.", MAX_WAYPOINTS);
    if (line_overflow) ESP_LOGW(TAG, "⚠️ 경로선 정점이 MAX_ROUTE_LINE_POINTS(%d)를 넘어 잘렸습니다. 경로 뒷부분의 이탈 판정이 부정확해집니다.", MAX_ROUTE_LINE_POINTS);

    build_waypoint_line_index();

    ESP_LOGI(TAG, "✅ 경로 저장 완료! (안내 지점 %d개, 경로선 정점 %d개)", total_waypoints, route_line_count);
    if (route_line_count < 2) {
        ESP_LOGW(TAG, "⚠️ 경로선을 얻지 못해 경로 이탈 감지/재탐색이 비활성화됩니다.");
    }
    // 시설물 구간 로그 (facilityName 실제로 뭐 오는지 확인용)
    for (int i = 0; i < total_waypoints; i++) {
        if (route_points[i].facilityName[0] != '\0') {
            ESP_LOGI(TAG, "🏗️ [시설물] 지점 %d '%s' 다음 구간: %s (facilityType=%d)%s%s",
                     i, route_points[i].description, route_points[i].facilityName,
                     route_points[i].facilityType,
                     is_stairs(&route_points[i]) ? " -> 계단 판정" : "",
                     is_crosswalk(&route_points[i]) ? " -> 횡단보도 판정" : "");
        }
    }
    return total_waypoints;
}

// 요청 + 파싱 한 번에
void fetch_tmap_pedestrian_route(double dest_lat, double dest_lon) {
    char *body = tmap_request_route_body(dest_lat, dest_lon);
    if (!body) return;
    parse_route_body(body);
    free(body);
}

/* ====================================================================
 * 재탐색: 지금 위치에서 같은 목적지로 경로 다시 받기
 * 실패하면 기존 경로 유지하고 false
 * ==================================================================== */
static bool try_reroute(void) {
    if (current_dest_idx < 0 || current_dest_idx >= (int)DESTINATION_COUNT) return false;
    Destination_t dest = DESTINATIONS[current_dest_idx];

    ESP_LOGW(TAG, "🔄 [재탐색] 현재 위치에서 %s 까지 경로를 다시 계산합니다...", dest.name);
    usb_send_signal(SIGNAL_REROUTING);

    char *body = tmap_request_route_body(dest.lat, dest.lon);
    if (!body) {
        ESP_LOGE(TAG, "❌ [재탐색 실패] 기존 경로를 유지한 채 잠시 후 다시 시도합니다.");
        usb_send_signal(SIGNAL_REROUTE_FAILED);
        return false;
    }

    int n = parse_route_body(body); // 0이면 기존 경로 그대로
    free(body);
    if (n == 0) {
        ESP_LOGE(TAG, "❌ [재탐색 실패] 기존 경로를 유지한 채 잠시 후 다시 시도합니다.");
        usb_send_signal(SIGNAL_REROUTE_FAILED);
        return false;
    }

    ESP_LOGI(TAG, "✅ [재탐색 완료] 새 경로 %d개 지점으로 안내를 이어갑니다.", n);
    ESP_LOGI(TAG, "👉 [초기 방향] %s", route_points[0].description);
    usb_send_signal(SIGNAL_REROUTE_DONE);
    return true;
}

/* ====================================================================
 * 4. 선택한 목적지로 안내 시작
 * ==================================================================== */
void start_navigation_to_destination(int dest_idx) {
    if (dest_idx < 0 || dest_idx >= (int)DESTINATION_COUNT) return;

    Destination_t dest = DESTINATIONS[dest_idx];
    if (dest.lat == 0.0 && dest.lon == 0.0) {
        ESP_LOGE(TAG, "❌ %s 좌표가 아직 설정되지 않았습니다. main.c의 DESTINATIONS 배열을 채워주세요.", dest.name);
        return;
    }

    ESP_LOGI(TAG, "🎯 [목적지 선택] %s (으)로 경로를 요청합니다.", dest.name);

    // 안내 중이면 멈추고 다시 시작
    // HTTP 중에 강제 종료하면 메모리 새서 플래그 세우고 끝날 때까지 기다림
    if (navigation_task_handle != NULL) {
        ESP_LOGI(TAG, "🔁 기존 안내를 중단하고 새 목적지로 다시 안내를 시작합니다.");
        nav_stop_requested = true;
        for (int i = 0; i < 400 && navigation_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(50)); // 최대 20초 (HTTP 타임아웃 15초)
        }
        if (navigation_task_handle != NULL) { // 그래도 안 끝나면 강제 종료
            ESP_LOGW(TAG, "⚠️ 기존 안내 태스크가 응답하지 않아 강제 종료합니다.");
            vTaskDelete(navigation_task_handle);
            navigation_task_handle = NULL;
        }
        nav_stop_requested = false;
    }

    total_waypoints = 0;
    route_line_count = 0;
    current_target_idx = 0;
    current_dest_idx = dest_idx; // 재탐색용

    fetch_tmap_pedestrian_route(dest.lat, dest.lon);

    if (total_waypoints > 0) {
        // 재탐색 때 HTTP+TLS 돌려서 스택 8192
        xTaskCreate(&navigation_task, "navi_task", 8192, NULL, 5, &navigation_task_handle);
    } else {
        ESP_LOGE(TAG, "❌ 경로를 가져오지 못해 안내를 시작할 수 없습니다.");
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase()); ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    // 큐 먼저 만들기 (gps_task보다 먼저)
    route_cmd_q = xQueueCreate(1, sizeof(int));
    if (route_cmd_q == NULL) {
        ESP_LOGE(TAG, "❌ 목적지 명령 큐 생성 실패");
        return;
    }
    // 경로 요청 태스크 (HTTP+TLS라 스택 8192)
    xTaskCreate(&route_request_task, "route_req_task", 8192, NULL, 5, NULL);

    // GPS/명령 수신. 0x01/0x02 오면 경로 요청 -> 안내 시작
    xTaskCreate(&gps_usb_task, "gps_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "📡 키워드스파팅(라파)의 목적지 선택 신호(0x01=버스정류장, 0x02=학교)를 기다립니다...");
}