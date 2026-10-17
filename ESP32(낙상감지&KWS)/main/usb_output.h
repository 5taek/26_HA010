#ifndef FALL_DETECTION_MAIN_USB_OUTPUT_H_
#define FALL_DETECTION_MAIN_USB_OUTPUT_H_

#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"

// Initializes the common USB TX lock. Call after installing the
// USB Serial/JTAG driver and before starting producer tasks.
bool InitializeUsbOutput();

// Writes a complete protocol line with a single driver call while holding the
// shared TX lock. A future HDG producer can use a short timeout and drop only
// that line if the port is busy.
bool UsbWriteLine(const char* data, size_t length, TickType_t timeout_ticks);

// Sends one of the existing binary fall-state bytes without changing its wire
// format. It uses the same lock as text protocol lines.
bool UsbWriteSignal(uint8_t signal, TickType_t timeout_ticks);

#endif  // FALL_DETECTION_MAIN_USB_OUTPUT_H_
