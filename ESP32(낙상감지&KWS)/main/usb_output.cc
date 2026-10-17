#include "usb_output.h"

#include "driver/usb_serial_jtag.h"
#include "freertos/semphr.h"

namespace {
StaticSemaphore_t tx_mutex_storage = {};
SemaphoreHandle_t tx_mutex = nullptr;

bool WriteLocked(const void* data, size_t length, TickType_t timeout_ticks) {
  if (tx_mutex == nullptr || data == nullptr || length == 0) return false;
  if (xSemaphoreTake(tx_mutex, timeout_ticks) != pdTRUE) return false;

  // Keep this as exactly one call: RESULT/S3_READY/HDG must never be split by
  // another application-level producer after it acquires the shared lock.
  const int written = usb_serial_jtag_write_bytes(
      data, length, timeout_ticks);
  xSemaphoreGive(tx_mutex);
  return written == static_cast<int>(length);
}
}  // namespace

bool InitializeUsbOutput() {
  if (tx_mutex != nullptr) return true;
  tx_mutex = xSemaphoreCreateMutexStatic(&tx_mutex_storage);
  return tx_mutex != nullptr;
}

bool UsbWriteLine(const char* data, size_t length, TickType_t timeout_ticks) {
  return WriteLocked(data, length, timeout_ticks);
}

bool UsbWriteSignal(uint8_t signal, TickType_t timeout_ticks) {
  return WriteLocked(&signal, sizeof(signal), timeout_ticks);
}
