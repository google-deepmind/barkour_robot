// Copyright 2024 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ==============================================================================

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_ASYNC_UART_LOGGER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_ASYNC_UART_LOGGER_H_

#include "pw_bytes/span.h"
#include "pw_result/result.h"
#include "pw_ring_buffer/prefixed_entry_ring_buffer.h"
#include "pw_span/span.h"
#include "stm32h7xx_hal.h"  // NOLINT

namespace barkour {

// Buffer sizes are in bytes.
constexpr size_t kAsyncUartLoggerDmaTxBufferSize = 512;
constexpr size_t kAsyncUartBufferRingBufferSize = 4000;

// An Asynchronous UART logging interface.
class AsyncUartLogger {
 public:
  // Singleton access to AsyncUartLogger.
  static AsyncUartLogger& Get();

  // Clears the ring-buffer.
  // Clearing the ring-buffer may be useful when calling Log() after setting
  // disable_async to true.
  void ClearRingBuffer();

  // Writes a log entry to the UART asynchronously.
  // Each log entry is pushed to an internal ring-buffer, which emptied by
  // calling the Update().
  //
  // If the internal ring-buffer is full, pw::Status::ResourceExhausted() will
  // be returned.
  //
  // If the log entry is too large for the ring-buffer, pw::Status::Internal()
  // will be returned.
  //
  // Note: If DisableAsynchronousLogging() has been called, all calls to Log()
  //   will be block while the peripheral is transmitting. Additionally, the
  //   first call to Log() after calling DisableAsynchronousLogging() will
  //   block until all other entries presently in the ring-buffer have been
  //   written. This is done to ensure that no log entries are skipped.
  pw::Status Log(const pw::span<char> entry);

  // Responsible for consuming data from the internal ring-buffer and
  // starting the next DMA transfer.
  // This function should be continually called, and should probably be
  // called from a relatively low-priority task.
  // Returns pw::Status::Internal() in the case of an error.
  pw::Status Update();

  // Disables asynchronous logging.
  // See Log() for more detail.
  void DisableAsynchronousLogging();

  // Disables asynchronous logging.
  // See Log() for more detail.
  void EnableAsynchronousLogging();

 private:
  bool asynchronous_logging_disabled_{false};
  pw::ring_buffer::PrefixedEntryRingBuffer ring_buffer_{false};
  std::byte raw_buffer_[kAsyncUartBufferRingBufferSize]{};
  uint8_t dma_tx_buffer_[kAsyncUartLoggerDmaTxBufferSize]{0};

  // Constructs an AsyncUartLogger.
  AsyncUartLogger();
  // Destructs an AsyncUartLogger.
  ~AsyncUartLogger();

  // Attempts to make a blocking UART transmission if an internal error has
  // been encountered.
  void InternalError(const std::string_view& message);

  // Performs a Synchronous Log operation.
  pw::Status LogSynchronous(const pw::span<char> entry);
};

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_ASYNC_UART_LOGGER_H_
