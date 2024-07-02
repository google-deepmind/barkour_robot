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

#include "actuator/firmware/barkour/common/async_uart_logger.h"

#include <algorithm>
#include <cstring>

#include "pw_assert/check.h"
#include "pw_string/string_builder.h"

namespace barkour {

constexpr size_t kUartBlockingTimeoutMilliseconds = 1000;

UART_HandleTypeDef uart5_handle = {};

void UartMspInit(UART_HandleTypeDef* huart) {
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART5;
  RCC_PeriphClkInit.Usart234578ClockSelection = RCC_UART5CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  __HAL_RCC_UART5_CLK_ENABLE();

  __HAL_RCC_DMA1_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_UART5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // UART RX GPIO pin configuration
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Alternate = GPIO_AF14_UART5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hdma_tx.Instance = DMA1_Stream6;
  hdma_tx.Init.Request = DMA_REQUEST_UART5_TX;
  hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode = DMA_NORMAL;
  hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_tx.Init.MemBurst = DMA_MBURST_INC4;
  hdma_tx.Init.PeriphBurst = DMA_PBURST_INC4;
  HAL_DMA_Init(&hdma_tx);
  __HAL_LINKDMA(huart, hdmatx, hdma_tx);

  hdma_rx.Instance = DMA1_Stream5;
  hdma_rx.Init.Request = DMA_REQUEST_UART5_RX;
  hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode = DMA_NORMAL;
  hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst = DMA_MBURST_INC4;
  hdma_rx.Init.PeriphBurst = DMA_PBURST_INC4;
  HAL_DMA_Init(&hdma_rx);
  __HAL_LINKDMA(huart, hdmarx, hdma_rx);

  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  HAL_NVIC_SetPriority(UART5_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
}

void UartMspDeInit(UART_HandleTypeDef* huart) {
  __HAL_RCC_UART5_FORCE_RESET();
  __HAL_RCC_UART5_RELEASE_RESET();

  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5);

  HAL_DMA_DeInit(huart->hdmatx);
  HAL_DMA_DeInit(huart->hdmarx);

  HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
}

AsyncUartLogger::AsyncUartLogger() {
  uart5_handle.Instance = UART5;

  uart5_handle.Init.BaudRate = 460800;
  uart5_handle.Init.WordLength = UART_WORDLENGTH_8B;
  uart5_handle.Init.StopBits = UART_STOPBITS_1;
  uart5_handle.Init.Parity = UART_PARITY_NONE;
  uart5_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart5_handle.Init.Mode = UART_MODE_TX_RX;
  uart5_handle.Init.OverSampling = UART_OVERSAMPLING_16;

  HAL_StatusTypeDef result;

  result = HAL_UART_RegisterCallback(
      &uart5_handle, HAL_UART_MSPINIT_CB_ID, UartMspInit);
  PW_CHECK(HAL_OK == result);

  result = HAL_UART_RegisterCallback(
      &uart5_handle, HAL_UART_MSPDEINIT_CB_ID, UartMspDeInit);
  PW_CHECK(HAL_OK == result);

  result = HAL_UART_Init(&uart5_handle);
  PW_CHECK(HAL_OK == result);

  ring_buffer_.SetBuffer(raw_buffer_);
}

AsyncUartLogger::~AsyncUartLogger() { HAL_UART_DeInit(&uart5_handle); }

AsyncUartLogger& AsyncUartLogger::Get() {
  static AsyncUartLogger async_uart_logger;
  return async_uart_logger;
}

void AsyncUartLogger::ClearRingBuffer() { ring_buffer_.Clear(); }

void AsyncUartLogger::DisableAsynchronousLogging() {
  asynchronous_logging_disabled_ = true;
}

void AsyncUartLogger::EnableAsynchronousLogging() {
  asynchronous_logging_disabled_ = false;
}

// Test, entry greater than DMA buffer + rin_buffer size should fail.
pw::Status AsyncUartLogger::Log(const pw::span<char> entry) {
  if (asynchronous_logging_disabled_) {
    return LogSynchronous(entry);
  }
  if (entry.size_bytes() > kAsyncUartBufferRingBufferSize) {
    return pw::Status::Internal();
  }
  size_t ring_bytes_remaining =
      kAsyncUartBufferRingBufferSize - ring_buffer_.TotalUsedBytes();
  if (ring_bytes_remaining < entry.size_bytes()) {
    return pw::Status::ResourceExhausted();
  }

  // auto subspan = reinterpret_cast< pw::span<std::byte>
  // >(entry.subspan(0,entry.size_bytes()));
  auto subspan = pw::as_bytes(entry);
  while (subspan.size_bytes() != 0) {
    // Because partial log entries cannot be pushed back to the front of the
    // ring-buffer, we must chunk up large entries into smaller entries no
    // larger than the maximum-length of the DMA buffer size.
    size_t bytes_to_push =
        std::min(subspan.size_bytes(), kAsyncUartLoggerDmaTxBufferSize);
    auto result = ring_buffer_.TryPushBack(subspan.subspan(0, bytes_to_push));
    if (result != pw::OkStatus()) {
      return pw::Status::Internal();
    }
    subspan = subspan.subspan(bytes_to_push);
  }

  return pw::OkStatus();
}

pw::Status AsyncUartLogger::LogSynchronous(const pw::span<char> entry) {
  // Transmit entries from the DMA buffer first.
  while (ring_buffer_.EntryCount() > 0) {
    size_t entry_size = ring_buffer_.FrontEntryDataSizeBytes();
    std::byte front_entry[entry_size];
    size_t bytes_read;
    if (!ring_buffer_.PeekFront({front_entry, entry_size}, &bytes_read).ok() ||
        !ring_buffer_.PopFront().ok()) {
      return pw::Status::Internal();
    }
    auto result = HAL_UART_Transmit(&uart5_handle,
                                    (uint8_t*)front_entry,
                                    entry_size,
                                    kUartBlockingTimeoutMilliseconds);
    if (HAL_OK != result) {
      return pw::Status::Internal();
    }
  }
  auto result = HAL_UART_Transmit(&uart5_handle,
                                  reinterpret_cast<uint8_t*>(entry.data()),
                                  entry.size_bytes(),
                                  kUartBlockingTimeoutMilliseconds);
  if (HAL_OK != result) {
    return pw::Status::Internal();
  }
  return pw::OkStatus();
}

pw::Status AsyncUartLogger::Update() {
  if (asynchronous_logging_disabled_) {
    return pw::OkStatus();
  }

  HAL_UART_StateTypeDef state = HAL_UART_GetState(&uart5_handle);
  switch (state) {
    case HAL_UART_STATE_READY: {
      break;
    }
    case HAL_UART_STATE_BUSY:
    case HAL_UART_STATE_BUSY_RX:
    case HAL_UART_STATE_BUSY_TX: {
      // DMA transfer still in progress.
      return pw::OkStatus();
      break;
    }
    case HAL_UART_STATE_ERROR:
    case HAL_UART_STATE_RESET:
    case HAL_UART_STATE_TIMEOUT:
    default: {
      // Normally PW_CRASH would be called, but PW_CRASH will rely upon
      // AsyncUartLogger.
      while (1) {
      };
      break;
    }
  }

  pw::span<std::byte> dma_tx_buffer_span(
      reinterpret_cast<std::byte*>(dma_tx_buffer_),
      kAsyncUartLoggerDmaTxBufferSize);
  size_t bytes_written = 0;
  while (ring_buffer_.EntryCount()) {
    if (ring_buffer_.FrontEntryDataSizeBytes() >
        kAsyncUartLoggerDmaTxBufferSize) {
      // If we had a double-ended queue, we could consume a portion of an entry
      // and then push the remainder to the front. Pigweed does not yet have
      // one though. We cannot use std::dequeue because it depends on dynamic
      // allocation. So, the only recourse is to fail if a log entry larger
      // than the DMA buffer is encounted. This should not happen though, as
      // Log() will break up entries into chunks no larger than
      // kAsyncUartLoggerDmaTxBufferSize.
      return pw::Status::Internal();
    } else if (ring_buffer_.FrontEntryDataSizeBytes() >
               dma_tx_buffer_span.size()) {
      // The dma_tx_buffer is effectively full for now, continue when the
      // upcoming DMA transfer is complete.
      break;
    } else {
      // Write the front ring-buffer entry to the dma_tx_buffer.
      size_t bytes_read;
      auto status = ring_buffer_.PeekFront(dma_tx_buffer_span, &bytes_read);
      bytes_written += bytes_read;
      if (status != pw::OkStatus()) {
        return pw::Status::Internal();
      }
      status = ring_buffer_.PopFront();
      if (status != pw::OkStatus()) {
        return pw::Status::Internal();
      }
      dma_tx_buffer_span = dma_tx_buffer_span.subspan(bytes_read);
    }
  }

  if (bytes_written > 0) {
    auto result =
        HAL_UART_Transmit_DMA(&uart5_handle, dma_tx_buffer_, bytes_written);
    if (HAL_OK != result) {
      return pw::Status::Internal();
    }
  }
  return pw::OkStatus();
}

}  // namespace barkour

extern "C" {

// RX
void DMA1_Stream5_IRQHandler(void) {
  HAL_DMA_IRQHandler(barkour::uart5_handle.hdmarx);
}

// TX
void DMA1_Stream6_IRQHandler(void) {
  HAL_DMA_IRQHandler(barkour::uart5_handle.hdmatx);
}

void UART5_IRQHandler(void) { HAL_UART_IRQHandler(&barkour::uart5_handle); }
}
