#include "nav_heading.h"

#include <cmath>
#include <cstdio>
#include <limits>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "usb_output.h"

namespace {
constexpr char kTag[] = "nav_heading";
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kAccelLowPassAlpha = 0.1f;
constexpr float kMagCorrectionGain = 0.05f;
constexpr float kNominalDtSeconds = 0.04f;
constexpr uint32_t kAccelStaleMs = 200;
constexpr int kMovingWindowSamples = 25;
constexpr int kOutputEvery = 5;
constexpr int kQueueLength = 4;
constexpr TickType_t kUsbTimeout = pdMS_TO_TICKS(5);

// Mounting/calibration constants. These deliberately match the request-file
// starting values; replace them with measurements from the assembled cane.
constexpr float kForwardAxis[3] = {1.0f, 0.0f, 0.0f};
constexpr float kForwardFlipDeg = 0.0f;
constexpr float kDeclinationDeg = -8.5f;  // Daegu baseline; periodically verify.
constexpr float kGyroSign = 1.0f;
constexpr float kMagOffset[3] = {0.0f, 0.0f, 0.0f};
constexpr float kMagScale[3] = {1.0f, 1.0f, 1.0f};
constexpr float kMagNormGauss = 0.50f;
constexpr float kMovingVarThreshold = 0.0004f;

QueueHandle_t sample_queue = nullptr;
StaticQueue_t sample_queue_storage = {};
uint8_t sample_queue_data[kQueueLength * sizeof(NavSensorSample)] = {};

float accel_lp[3] = {0.0f, 0.0f, 1.0f};
float heading_deg = 0.0f;
bool heading_initialized = false;
float accel_magnitude[kMovingWindowSamples] = {};
int accel_magnitude_count = 0;
int accel_magnitude_position = 0;
uint32_t last_sample_count = 0;
uint32_t last_fresh_ms = 0;
uint32_t previous_timestamp_ms = 0;
int output_counter = 0;

#if NAV_MAG_CALIBRATION
float calibration_min[3] = {
    std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max()};
float calibration_max[3] = {
    std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::lowest()};
uint32_t calibration_started_ms = 0;
uint32_t calibration_last_report_ms = 0;
#endif

float Dot(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Cross(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

bool Normalize(float value[3]) {
  const float norm = std::sqrt(Dot(value, value));
  if (norm < 1e-6f) return false;
  for (int i = 0; i < 3; ++i) value[i] /= norm;
  return true;
}

float Wrap180(float value) {
  while (value > 180.0f) value -= 360.0f;
  while (value <= -180.0f) value += 360.0f;
  return value;
}

void Wrap360(float* value) {
  while (*value < 0.0f) *value += 360.0f;
  while (*value >= 360.0f) *value -= 360.0f;
}

bool UpdateMoving(const float accel[3]) {
  accel_magnitude[accel_magnitude_position] = std::sqrt(Dot(accel, accel));
  accel_magnitude_position =
      (accel_magnitude_position + 1) % kMovingWindowSamples;
  if (accel_magnitude_count < kMovingWindowSamples) ++accel_magnitude_count;

  float mean = 0.0f;
  for (int i = 0; i < accel_magnitude_count; ++i) {
    mean += accel_magnitude[i];
  }
  mean /= accel_magnitude_count;

  float variance = 0.0f;
  for (int i = 0; i < accel_magnitude_count; ++i) {
    const float delta = accel_magnitude[i] - mean;
    variance += delta * delta;
  }
  variance /= accel_magnitude_count;
  return accel_magnitude_count == kMovingWindowSamples &&
         variance > kMovingVarThreshold;
}

#if NAV_MAG_CALIBRATION
void UpdateCalibration(const NavSensorSample& sample) {
  if (!sample.mag_ok) return;
  if (calibration_started_ms == 0) {
    calibration_started_ms = sample.timestamp_ms;
    calibration_last_report_ms = sample.timestamp_ms;
  }
  if (sample.timestamp_ms - calibration_started_ms > 30000) return;

  for (int i = 0; i < 3; ++i) {
    if (sample.mag[i] < calibration_min[i]) calibration_min[i] = sample.mag[i];
    if (sample.mag[i] > calibration_max[i]) calibration_max[i] = sample.mag[i];
  }
  if (sample.timestamp_ms - calibration_last_report_ms < 5000) return;
  calibration_last_report_ms = sample.timestamp_ms;

  float offset[3] = {};
  float radius[3] = {};
  float mean_radius = 0.0f;
  for (int i = 0; i < 3; ++i) {
    offset[i] = (calibration_max[i] + calibration_min[i]) * 0.5f;
    radius[i] = (calibration_max[i] - calibration_min[i]) * 0.5f;
    mean_radius += radius[i];
  }
  mean_radius /= 3.0f;
  float scale[3] = {};
  for (int i = 0; i < 3; ++i) {
    scale[i] = radius[i] > 1e-6f ? mean_radius / radius[i] : 0.0f;
  }

  char line[160];
  const int length = std::snprintf(
      line, sizeof(line),
      "MAG_CAL,offset=%.5f,%.5f,%.5f,scale=%.5f,%.5f,%.5f,norm=%.5f\n",
      offset[0], offset[1], offset[2], scale[0], scale[1], scale[2],
      mean_radius);
  if (length > 0) {
#if FALL_DETECTION_TEST_MODE
    UsbWriteLine(line, static_cast<size_t>(length), kUsbTimeout);
#endif
  }
}
#endif

void ProcessSample(const NavSensorSample& sample) {
#if NAV_MAG_CALIBRATION
  UpdateCalibration(sample);
  return;
#else
  const bool fresh = sample.sample_count != last_sample_count;
  if (fresh) {
    last_sample_count = sample.sample_count;
    last_fresh_ms = sample.timestamp_ms;
  }

  for (int i = 0; i < 3; ++i) {
    accel_lp[i] +=
        kAccelLowPassAlpha * (sample.accel[i] - accel_lp[i]);
  }
  const bool moving = UpdateMoving(sample.accel);

  const bool imu_fresh = last_sample_count != 0 &&
      (sample.timestamp_ms - last_fresh_ms <= kAccelStaleMs);
  float down[3] = {-accel_lp[0], -accel_lp[1], -accel_lp[2]};
  float mag[3] = {};
  for (int i = 0; i < 3; ++i) {
    mag[i] = (sample.mag[i] - kMagOffset[i]) * kMagScale[i];
  }
  const float mag_norm = std::sqrt(Dot(mag, mag));
  const bool mag_range_ok = std::isfinite(mag_norm) &&
      std::fabs(mag_norm - kMagNormGauss) <= 0.25f * kMagNormGauss;
  const bool down_ok = Normalize(down);
  const float forward_vertical = std::fabs(Dot(kForwardAxis, down));
  const bool forward_ok = down_ok && forward_vertical <= 0.95f;

  float east[3] = {};
  float north[3] = {};
  Cross(down, mag, east);
  const bool east_ok = Normalize(east);
  Cross(east, down, north);
  const bool ok = imu_fresh && sample.mag_ok && mag_range_ok &&
      down_ok && forward_ok && east_ok;

  float dt = kNominalDtSeconds;
  if (previous_timestamp_ms != 0 && sample.timestamp_ms > previous_timestamp_ms) {
    dt = static_cast<float>(sample.timestamp_ms - previous_timestamp_ms) /
         1000.0f;
    if (dt > 0.2f) dt = kNominalDtSeconds;
  }
  previous_timestamp_ms = sample.timestamp_ms;

  if (ok) {
    float magnetic_heading =
        std::atan2(Dot(kForwardAxis, east), Dot(kForwardAxis, north)) *
        kRadToDeg;
    magnetic_heading += kDeclinationDeg + kForwardFlipDeg;
    if (!heading_initialized) {
      heading_deg = magnetic_heading;
      heading_initialized = true;
    }
    heading_deg += kGyroSign * Dot(sample.gyro, down) * dt;
    heading_deg +=
        kMagCorrectionGain * Wrap180(magnetic_heading - heading_deg);
    Wrap360(&heading_deg);
  }

#if FALL_DETECTION_TEST_MODE
  if (++output_counter % kOutputEvery == 0) {
    char line[40];
    const int length = std::snprintf(
        line, sizeof(line), "HDG,%.1f,%d,%d\n", heading_deg, ok ? 1 : 0,
        moving ? 1 : 0);
    if (length > 0 &&
        !UsbWriteLine(line, static_cast<size_t>(length), kUsbTimeout)) {
      ESP_LOGD(kTag, "HDG line dropped because USB TX was busy");
    }
  }
  if (output_counter % 25 == 0) {
    ESP_LOGI(kTag,
        "NAV_DIAG imu_fresh=%d mag_read=%d mag_gauss=%.4f,%.4f,%.4f "
        "mag_norm=%.4f range_ok=%d down_ok=%d forward_vertical=%.3f "
        "forward_ok=%d east_ok=%d",
        imu_fresh ? 1 : 0, sample.mag_ok ? 1 : 0,
        sample.mag[0], sample.mag[1], sample.mag[2], mag_norm,
        mag_range_ok ? 1 : 0, down_ok ? 1 : 0, forward_vertical,
        forward_ok ? 1 : 0, east_ok ? 1 : 0);
  }
#endif
#endif
}

void NavHeadingTask(void*) {
  while (true) {
    NavSensorSample sample = {};
    if (xQueueReceive(sample_queue, &sample, portMAX_DELAY) == pdTRUE) {
      ProcessSample(sample);
    }
  }
}
}  // namespace

bool InitializeNavHeading() {
  sample_queue = xQueueCreateStatic(kQueueLength, sizeof(NavSensorSample),
                                    sample_queue_data,
                                    &sample_queue_storage);
  if (sample_queue == nullptr) return false;
  return xTaskCreatePinnedToCore(NavHeadingTask, "nav_heading", 4096, nullptr,
                                 3, nullptr, 0) == pdPASS;
}

void PublishNavSensorSample(const NavSensorSample& sample) {
  if (sample_queue == nullptr) return;
  if (xQueueSend(sample_queue, &sample, 0) == pdTRUE) return;

  NavSensorSample discarded = {};
  xQueueReceive(sample_queue, &discarded, 0);
  xQueueSend(sample_queue, &sample, 0);
}
