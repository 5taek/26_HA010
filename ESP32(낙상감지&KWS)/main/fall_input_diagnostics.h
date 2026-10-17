#pragma once

#include <cmath>
#include "esp_log.h"

// Summarize the exact raw feature window, before embedded normalization.
// Diagnostic only: never changes model input or the event decision.
inline void LogFallInputDiagnostics(const float* values, int length) {
#if FALL_DETECTION_TEST_MODE
  if (values == nullptr || length <= 0 || length % 6 != 0) return;
  const int count = length / 6;
  float mean[6] = {}, sd[6] = {};
  float acc_peak = 0.0f, gyro_peak = 0.0f, acc_step_peak = 0.0f;
  bool finite = true;
  for (int n = 0; n < count; ++n) {
    float a2 = 0.0f, g2 = 0.0f, step2 = 0.0f;
    for (int axis = 0; axis < 6; ++axis) {
      const float v = values[n * 6 + axis];
      finite = finite && std::isfinite(v);
      mean[axis] += v / count;
      if (axis < 3) {
        a2 += v * v;
        if (n > 0) {
          const float d = v - values[(n - 1) * 6 + axis];
          step2 += d * d;
        }
      } else {
        g2 += v * v;
      }
    }
    const float a = std::sqrt(a2), g = std::sqrt(g2);
    const float step = std::sqrt(step2);
    if (a > acc_peak) acc_peak = a;
    if (g > gyro_peak) gyro_peak = g;
    if (step > acc_step_peak) acc_step_peak = step;
  }
  for (int n = 0; n < count; ++n) {
    for (int axis = 0; axis < 6; ++axis) {
      const float d = values[n * 6 + axis] - mean[axis];
      sd[axis] += d * d / count;
    }
  }
  for (float& v : sd) v = std::sqrt(v);
  ESP_LOGI("fall_input",
      "FD_INPUT n=%d finite=%d acc_mean_g=%.4f,%.4f,%.4f "
      "acc_sd_g=%.4f,%.4f,%.4f gyro_mean_dps=%.3f,%.3f,%.3f "
      "gyro_sd_dps=%.3f,%.3f,%.3f acc_peak_g=%.3f "
      "gyro_peak_dps=%.3f acc_step_peak_g=%.4f",
      count, finite ? 1 : 0, mean[0], mean[1], mean[2],
      sd[0], sd[1], sd[2], mean[3], mean[4], mean[5],
      sd[3], sd[4], sd[5], acc_peak, gyro_peak, acc_step_peak);
#else
  (void)values;
  (void)length;
#endif
}
