#ifndef FALL_DETECTION_MAIN_NAV_HEADING_H_
#define FALL_DETECTION_MAIN_NAV_HEADING_H_

#include <cstdint>

struct NavSensorSample {
  float accel[3];
  float gyro[3];
  float mag[3];
  uint32_t sample_count;
  uint32_t timestamp_ms;
  bool mag_ok;
};

// Creates the heading input queue and its low-priority processing task.
bool InitializeNavHeading();

// Non-blocking handoff from the 25 Hz sensor owner. If heading processing is
// behind, the oldest queued navigation sample is discarded, never an FD sample.
void PublishNavSensorSample(const NavSensorSample& sample);

#endif  // FALL_DETECTION_MAIN_NAV_HEADING_H_
