#ifndef FALL_DETECTION_MAIN_MODEL_INPUT_PROVIDER_H_
#define FALL_DETECTION_MAIN_MODEL_INPUT_PROVIDER_H_

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/core/api/error_reporter.h"

TfLiteStatus SetupModelInputProvider(tflite::ErrorReporter* error_reporter);

// Return the number of scalar features expected by the fall model.
int ModelInputFeatureCount(const TfLiteTensor* input);

// Convert one already-complete 6-axis IMU window into the model tensor.
// Sensor I/O intentionally happens outside this function so inference can
// wait for KWS without stopping the 25 Hz acquisition task.
bool FillModelInputFromFeatures(TfLiteTensor* input, const float* features,
                                int feature_length,
                                tflite::ErrorReporter* error_reporter);

#endif  // FALL_DETECTION_MAIN_MODEL_INPUT_PROVIDER_H_
