#include "fall_alert_sender.h"
#include "fall_alert_outbox.h"
#include "cJSON.h"
#include "esp_random.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <cstdio>
#include <cstring>

#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace {
constexpr char kTag[] = "fall_alert_sender";
constexpr EventBits_t kWifiConnected = BIT0;

// WIFI_EVENT_STA_DISCONNECTED used to call esp_wifi_connect() again
// immediately and unconditionally, with no delay of any kind. If the AP is
// out of range or unstable (WIFI_REASON_NO_AP_FOUND et al.), that turns
// into a near-continuous scan/connect RF duty cycle with no idle gap -- a
// real, sustained heat/power cost, not just log spam. Back off
// exponentially instead, capped, and reset once a connection succeeds.
constexpr uint32_t kReconnectBaseDelayMs = 1000;
constexpr uint32_t kReconnectMaxDelayMs = 30000;
constexpr uint32_t kReconnectMaxShift = 5;  // 1000ms << 5 = 32000ms, already past the cap

using AlertMessage = FallAlertRecord;
EventGroupHandle_t wifi_events = nullptr;
FallAlertOutbox outbox;
SemaphoreHandle_t outbox_mutex = nullptr;
nvs_handle_t outbox_nvs = 0;
uint64_t next_sequence = 1;
// Producer touches RAM only; flash and HTTPS belong to background tasks.
FallAlertOutbox ingress;
portMUX_TYPE ingress_mux = portMUX_INITIALIZER_UNLOCKED;
bool storage_ready = false;
uint64_t persisted_sequence = 0;

void SlotKey(int slot, char* key, size_t length) {
  std::snprintf(key, length, "event_%02d", slot);
}

bool InitializeOutbox() {
  // Erasing NVS on an initialization error would erase undelivered falls.
  const esp_err_t init = nvs_flash_init();
  if (init != ESP_OK) {
    ESP_LOGE(kTag, "NVS init failed; preserving pending events: %s",
             esp_err_to_name(init));
    return false;
  }
  outbox_mutex = xSemaphoreCreateMutex();
  if (outbox_mutex == nullptr ||
      nvs_open("fall_outbox", NVS_READWRITE, &outbox_nvs) != ESP_OK) return false;
  uint64_t stored_sequence = 0;
  const esp_err_t sequence_result = nvs_get_u64(outbox_nvs, "sequence", &stored_sequence);
  if (sequence_result != ESP_OK && sequence_result != ESP_ERR_NVS_NOT_FOUND) return false;
  persisted_sequence = stored_sequence;
  next_sequence = stored_sequence + 1;
  for (int i = 0; i < FallAlertOutbox::kCapacity; ++i) {
    char key[16];
    SlotKey(i, key, sizeof(key));
    size_t length = sizeof(AlertMessage);
    AlertMessage record = {};
    const esp_err_t result = nvs_get_blob(outbox_nvs, key, &record, &length);
    if (result == ESP_ERR_NVS_NOT_FOUND) continue;
    if (result != ESP_OK || length != sizeof(record) || record.version != 1 ||
        record.event_id[32] != '\0' || record.state[19] != '\0' ||
        record.reason[63] != '\0') {
      ESP_LOGE(kTag, "Cannot read pending event slot %d", i);
      return false;
    }
    outbox.records[i] = record;
    outbox.occupied[i] = true;
    if (record.sequence >= next_sequence) next_sequence = record.sequence + 1;
  }
  return true;
}

esp_timer_handle_t reconnect_timer = nullptr;
uint32_t reconnect_attempt = 0;

void ReconnectTimerCallback(void*) {
  esp_wifi_connect();
}

void ScheduleReconnect() {
  const uint32_t shift = reconnect_attempt < kReconnectMaxShift
                              ? reconnect_attempt
                              : kReconnectMaxShift;
  uint32_t delay_ms = kReconnectBaseDelayMs << shift;
  if (delay_ms > kReconnectMaxDelayMs) {
    delay_ms = kReconnectMaxDelayMs;
  }
  ++reconnect_attempt;
  ESP_LOGW(kTag, "reconnecting in %u ms (attempt %u)",
           static_cast<unsigned>(delay_ms),
           static_cast<unsigned>(reconnect_attempt));
  esp_timer_stop(reconnect_timer);  // no-op if not currently running
  esp_timer_start_once(reconnect_timer,
                        static_cast<uint64_t>(delay_ms) * 1000);
}

void WifiEvent(void*, esp_event_base_t base, int32_t id, void* event_data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
    ESP_LOGI(kTag, "STA started; connecting to SSID=\"%s\"",
             CONFIG_FALL_WIFI_SSID);
    esp_wifi_connect();
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    xEventGroupClearBits(wifi_events, kWifiConnected);
    const auto* disconnected =
        static_cast<wifi_event_sta_disconnected_t*>(event_data);
    ESP_LOGW(kTag, "disconnected, reason=%d",
             disconnected != nullptr ? disconnected->reason : -1);
    ScheduleReconnect();
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    const auto* got_ip = static_cast<ip_event_got_ip_t*>(event_data);
    ESP_LOGI(kTag, "connected, ip=" IPSTR,
             IP2STR(&got_ip->ip_info.ip));
    reconnect_attempt = 0;
    xEventGroupSetBits(wifi_events, kWifiConnected);
  }
}

bool PostAlert(const AlertMessage& alert) {
  const EventBits_t bits = xEventGroupWaitBits(
      wifi_events, kWifiConnected, pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
  if ((bits & kWifiConnected) == 0) {
    ESP_LOGE(kTag, "Wi-Fi unavailable");
    return false;
  }

  cJSON* payload = cJSON_CreateObject();
  if (payload == nullptr) return false;
  const bool complete =
      cJSON_AddStringToObject(payload, "deviceId", CONFIG_FALL_FIREBASE_DEVICE_ID) &&
      cJSON_AddStringToObject(payload, "eventId", alert.event_id) &&
      cJSON_AddStringToObject(payload, "state", alert.state) &&
      cJSON_AddStringToObject(payload, "reason", alert.reason) &&
      cJSON_AddNumberToObject(payload, "score", alert.score) &&
      cJSON_AddNumberToObject(payload, "eventSequence", alert.sequence) &&
      cJSON_AddNumberToObject(payload, "deviceTimeMs", alert.device_time_ms);
  char* json = complete ? cJSON_PrintUnformatted(payload) : nullptr;
  cJSON_Delete(payload);
  if (json == nullptr) return false;
  const int length = std::strlen(json);

  esp_http_client_config_t config = {};
  config.url = CONFIG_FALL_FIREBASE_FUNCTION_URL;
  config.method = HTTP_METHOD_POST;
  // esp-tls's connect timeout covers DNS + TCP + the full TLS handshake as
  // one lump sum (esp_tls.c's esp_tls_conn_new_sync loop), not a per-step
  // budget. On a phone personal hotspot the cellular backhaul alone can eat
  // several seconds, and 160 MHz (reverted from 240 MHz for heat) plus
  // PSRAM's extra access latency make the handshake's crypto/parsing slower
  // too -- 10s was getting exhausted by genuinely-still-progressing
  // connections, not stuck ones. This only raises the worst-case latency of
  // one attempt. Pending events remain in NVS until the server accepts them.
  config.timeout_ms = 20000;
  config.crt_bundle_attach = esp_crt_bundle_attach;
  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) { cJSON_free(json); return false; }
  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_header(client, "X-Device-Token",
                             CONFIG_FALL_FIREBASE_DEVICE_TOKEN);
  esp_http_client_set_post_field(client, json, length);
  const esp_err_t result = esp_http_client_perform(client);
  const int status = esp_http_client_get_status_code(client);
  esp_http_client_cleanup(client);
  cJSON_free(json);
  if (result != ESP_OK || status < 200 || status >= 300) {
    ESP_LOGE(kTag, "Firebase failed: error=%s HTTP=%d",
             esp_err_to_name(result), status);
    return false;
  }
  ESP_LOGI(kTag, "Firebase accepted state=%s HTTP=%d", alert.state, status);
  return true;
}

void StorageTask(void*) {
  while (true) {
    AlertMessage alert = {};
    portENTER_CRITICAL(&ingress_mux);
    const int pending = ingress.SelectNext();
    if (pending >= 0) alert = ingress.records[pending];
    portEXIT_CRITICAL(&ingress_mux);
    if (pending < 0) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    xSemaphoreTake(outbox_mutex, portMAX_DELAY);
    const int slot = outbox.FreeSlot(alert);
    const bool replacing = slot >= 0 && outbox.occupied[slot];
    const uint64_t high_sequence = alert.sequence > persisted_sequence
                                     ? alert.sequence : persisted_sequence;
    const bool saved = outbox.Add(alert, [&](int index, const AlertMessage& record) {
      char key[16];
      SlotKey(index, key, sizeof(key));
      const char* stage = "sequence";
      esp_err_t error = nvs_set_u64(outbox_nvs, "sequence", high_sequence);
      if (error == ESP_OK) {
        stage = "event_blob";
        error = nvs_set_blob(outbox_nvs, key, &record, sizeof(record));
      }
      if (error == ESP_OK) {
        stage = "commit";
        error = nvs_commit(outbox_nvs);
      }
      if (error != ESP_OK) {
        ESP_LOGE(kTag, "Outbox NVS failure: stage=%s error=%s slot=%d bytes=%u",
                 stage, esp_err_to_name(error), index,
                 static_cast<unsigned>(sizeof(record)));
        nvs_stats_t stats = {};
        if (nvs_get_stats(nullptr, &stats) == ESP_OK) {
          ESP_LOGE(kTag, "NVS entries: used=%u available=%u total=%u",
                   static_cast<unsigned>(stats.used_entries),
                   static_cast<unsigned>(stats.available_entries),
                   static_cast<unsigned>(stats.total_entries));
        }
      }
      return error == ESP_OK;
    });
    if (saved) persisted_sequence = high_sequence;
    xSemaphoreGive(outbox_mutex);
    if (saved) {
      if (replacing) ESP_LOGW(kTag, "Persistent outbox full; replaced an older event");
      portENTER_CRITICAL(&ingress_mux);
      ingress.Acknowledge(alert.event_id, [](int) { return true; });
      portEXIT_CRITICAL(&ingress_mux);
    } else {
      if (slot < 0) {
        ESP_LOGE(kTag, "Outbox slot selection failed; event remains in RAM");
      } else {
        ESP_LOGE(kTag, "Event remains in RAM; storage retry pending");
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void SenderTask(void*) {
  uint32_t retry_delay_ms = 1000;
  while (true) {
    AlertMessage alert = {};
    xSemaphoreTake(outbox_mutex, portMAX_DELAY);
    const int slot = outbox.SelectNext();
    if (slot >= 0) alert = outbox.records[slot];
    xSemaphoreGive(outbox_mutex);
    if (slot < 0) {
      retry_delay_ms = 1000;
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    if (PostAlert(alert)) {
      xSemaphoreTake(outbox_mutex, portMAX_DELAY);
      const bool removed = outbox.Acknowledge(alert.event_id, [](int index) {
        char key[16];
        SlotKey(index, key, sizeof(key));
        const esp_err_t erased = nvs_erase_key(outbox_nvs, key);
        return (erased == ESP_OK || erased == ESP_ERR_NVS_NOT_FOUND) &&
               nvs_commit(outbox_nvs) == ESP_OK;
      });
      xSemaphoreGive(outbox_mutex);
      if (removed) { retry_delay_ms = 1000; continue; }
      ESP_LOGE(kTag, "Accepted event retained because NVS erase failed");
    }
    // Re-select after each failed attempt so a newly arrived danger event
    // takes priority over older safe reports. No retry-count discard.
    vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
    retry_delay_ms = retry_delay_ms < 30000 / 2 ? retry_delay_ms * 2 : 30000;
  }
}
}  // namespace

void InitializeFallAlertSender() {
  if (!InitializeOutbox()) {
    ESP_LOGE(kTag, "Persistent outbox unavailable");
    outbox_mutex = nullptr;
    return;
  }
  if (xTaskCreatePinnedToCore(StorageTask, "fall_alert_storage", 4096, nullptr,
                              2, nullptr, 0) != pdPASS) {
    ESP_LOGE(kTag, "Storage task unavailable");
    return;
  }
  storage_ready = true;
  if (std::strlen(CONFIG_FALL_WIFI_SSID) == 0 ||
      std::strlen(CONFIG_FALL_FIREBASE_FUNCTION_URL) == 0 ||
      std::strlen(CONFIG_FALL_FIREBASE_DEVICE_TOKEN) == 0) {
    ESP_LOGE(kTag, "Wi-Fi/Firebase configuration incomplete");
    return;
  }
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  wifi_events = xEventGroupCreate();
  if (wifi_events == nullptr) {
    ESP_LOGE(kTag, "Wi-Fi event group allocation failed; events stay in NVS");
    return;
  }

  esp_timer_create_args_t reconnect_timer_args = {};
  reconnect_timer_args.callback = &ReconnectTimerCallback;
  reconnect_timer_args.dispatch_method = ESP_TIMER_TASK;
  reconnect_timer_args.name = "wifi_reconnect";
  ESP_ERROR_CHECK(esp_timer_create(&reconnect_timer_args, &reconnect_timer));

  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init));
  ESP_ERROR_CHECK(esp_event_handler_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiEvent, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &WifiEvent, nullptr));
  wifi_config_t station = {};
  strlcpy(reinterpret_cast<char*>(station.sta.ssid), CONFIG_FALL_WIFI_SSID,
          sizeof(station.sta.ssid));
  strlcpy(reinterpret_cast<char*>(station.sta.password),
          CONFIG_FALL_WIFI_PASSWORD, sizeof(station.sta.password));
  station.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &station));
  ESP_ERROR_CHECK(esp_wifi_start());
  // Kept on core 0 next to the Wi-Fi stack, and below the keyword-spotting
  // task's priority: alert delivery already retries, so it must not preempt
  // latency-sensitive inference with its CPU-heavy TLS handshakes.
  if (xTaskCreatePinnedToCore(SenderTask, "fall_alert_sender", 8192, nullptr, 3,
                              nullptr, 0) != pdPASS) {
    ESP_LOGE(kTag, "Sender task unavailable; events stay in NVS");
  }
}

bool QueueFallState(const char* state, const char* reason, float score) {
  if (!storage_ready || state == nullptr || reason == nullptr) return false;
  AlertMessage alert = {};
  alert.version = 1;
  alert.device_time_ms = esp_timer_get_time() / 1000;
  strlcpy(alert.state, state, sizeof(alert.state));
  strlcpy(alert.reason, reason, sizeof(alert.reason));
  alert.score = score;
  uint8_t random_id[16];
  esp_fill_random(random_id, sizeof(random_id));
  for (size_t i = 0; i < sizeof(random_id); ++i)
    std::snprintf(alert.event_id + i * 2, 3, "%02x", random_id[i]);
  portENTER_CRITICAL(&ingress_mux);
  alert.sequence = next_sequence++;
  const int slot = ingress.FreeSlot(alert);
  const bool replacing = slot >= 0 && ingress.occupied[slot];
  const bool accepted = ingress.Add(alert, [](int, const AlertMessage&) { return true; });
  portEXIT_CRITICAL(&ingress_mux);
  if (replacing) ESP_LOGW(kTag, "RAM outbox full; replaced an older event");
  if (!accepted) ESP_LOGE(kTag, "RAM outbox full of danger events; safe event rejected");
  return accepted;
}
