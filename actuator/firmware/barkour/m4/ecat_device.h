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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_ECAT_DEVICE_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_ECAT_DEVICE_H_

#include <cstddef>
#include <cstdint>

#include "actuator/firmware/barkour/common/bootloader/bootloader.h"
#include "actuator/firmware/barkour/m4/board_setup_m4.h"
#include "pw_status/status.h"
#include "pw_sync/mutex.h"
#include "FreeRTOS.h" // NOLINT
#include "queue.h" // NOLINT
#include "semphr.h" // NOLINT

namespace barkour {

// EtherCAT Application Layer states. See ETG.1000.6, section 6.4.1. These
// states relate purely to the EtherCAT communication protocols, and are
// independent of DS402 states.
enum class EthercatApplicationLayerState : uint8_t {
  kUnknown = 0,
  kInit,
  kBoot,
  kPreop,
  kSafeop,
  kOp,
};

// Provides access to EtherCAT device.
class EthercatDevice {
 public:
  // Singleton access to EtherCAT.
  static EthercatDevice& Get();

  // Returns a mutex which should be used read/write the object dictionary.
  //
  // e.g:
  //  ecat_device.GetObjectDictionaryLock().acquire();
  //  Obj.tx.position = control_state.position;
  //  ecat_device.GetObjectDictionaryLock().release();
  pw::sync::Mutex& GetObjectDictionaryMutex();

  // Go into ethercat processing and run until a signal is received to restart.
  void Run();

  // Requests that the EtherCAT device restarts. Should be called from a
  // different thread to `Run()`.
  //
  // This will have no effect if `Run()` has not yet been called.
  void Restart();

  // End of frame interrupt; tells ethercat a new frame has arrived
  void EofInterrupt();

  // Interrupt from Sync0 line from ESC when using distributed clocks
  void Sync0Interrupt();

  // Interrupt from Sync1 line from ESC when using distributed clocks
  void Sync1Interrupt();

  // Sets the cyclic callback, and associated context pointer.
  void SetCyclicCallback(void (*callback)(void*), void* context);

  // Calls the cyclic callback, passing in the context pointer, or a no-op if
  // the cyclic callback has not been set yet.
  //
  // This is required to be public so it can be called in the global SOES
  // post-packet-receive callback.
  void CallCyclicCallback() const;

  // Send and receive data from ESC
  pw::Status exchange(uint8_t* tx, uint8_t* rx, uint16_t len);

  // Returns the current state of the EtherCAT device.
  //
  // This uses a critical section to stop interrupts while fetching the current
  // state, so should be called from within a FreeRTOS task.
  EthercatApplicationLayerState GetCurrentState() const;

 private:
  EthercatDevice();
  EthercatDevice(const EthercatDevice&) = delete;
  EthercatDevice(EthercatDevice&&) = delete;

  // Runs until a restart signal is received. `first_run` indicates whether this
  // is the first call to RunOnce or not.
  void RunOnce(bool first_run);

  // Cyclic callback, and context pointer.
  void (*cyclic_callback_)(void*);
  void* cyclic_callback_context_;

  // Semaphore for the runloop to wait on the EtherCAT EoF interrupt.
  StaticSemaphore_t runloop_semaphore_buffer_;
  SemaphoreHandle_t runloop_semaphore_;

  // Queue for signalling when to restart the process.
  StaticQueue_t static_stop_queue_;
  QueueHandle_t stop_queue_;

  pw::sync::Mutex object_dictionary_mutex_;
};

// Thin C++ wrapper around flash_firmware to make it testable without odd C
// compilation issues. Do not use this.
uint32_t flash_firmware_test_only(uint32_t address_offset, uint32_t total_size,
                                  uint8_t* data, size_t length);

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_ECAT_DEVICE_H_
