#ifndef FALL_DETECTION_MAIN_FALL_ALERT_SENDER_H_
#define FALL_DETECTION_MAIN_FALL_ALERT_SENDER_H_

void InitializeFallAlertSender();
// Nonblocking RAM acceptance; background storage persists and retries.
// True does not mean the record is already durable or delivered.
bool QueueFallState(const char* state, const char* reason, float score);

#endif
