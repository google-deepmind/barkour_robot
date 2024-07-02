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

#include "actuator/firmware/barkour/m4/ecat_device.h"

#include <cstdint>

#include "actuator/firmware/barkour/common/board_config.h"
#include "actuator/firmware/barkour/common/code_version.h"
#include "actuator/firmware/barkour/common/serial_number.h"
#include "FreeRTOS.h" // NOLINT
#include "task.h" // NOLINT
#include "stm32h7xx_hal.h" // NOLINT
#include "pw_assert/check.h"
#include "pw_bytes/endian.h"
#include "pw_log/log.h"
#include "pw_status/status.h"

// Small timer class.
class Timer {
 public:
  // Represents half of "infinity", used to represent negative time
  // in unsigned integer arithmetic.
  constexpr static TickType_t halftime = ((TickType_t)~0UL) / 2;

  // Timer constructor
  // in:
  //    timeout_ms: duration of timer in milliseconds.
  Timer(int timeout_ms) : duration{pdMS_TO_TICKS(timeout_ms)} {
    timeout = xTaskGetTickCount() + duration;
  }
  // Test whether timer has triggered.
  // Automatically updates next timeout.
  inline bool timedout() {
    if (xTaskGetTickCount() - timeout > halftime) {
      return false;
    }
    timeout += duration;
    return true;
  }

 private:
  TickType_t timeout;
  const TickType_t duration;
};

extern "C" {

#include "soes/ecat_slv.h"
#include "soes/esc.h"
#include "soes/esc_foe.h"
#include "actuator/firmware/barkour/m4/ecat/utypes.h"

// This is the global data exchange structure between the local application
// and EtherCAT, for data which should be exposed over the EtherCAT bus.
//
// _Objects is defined in ecat/utypes.h
//
// Devel note: This structure is just an example and not cast in stone.
// _Objects.rx, _Objects.tx, _Objects.parameter may be changed, relocated to
// point to something else, even placed somewhere else... as long as the
// addresses and data types as defined in ecat/objectlist.c are updated.
//
// Obj is only required to store variables that are only being used for the
// operation of EtherCAT itself, such as _Objects.SDO1008.
pw::sync::Mutex ObjMutex;
_Objects Obj;

// A global flags object, similar to Obj, for data which needs to be
// communicated between the local application and the EtherCAT stack, but
// doesn't need to be exposed over the EtherCAT bus.
pw::sync::Mutex EthercatFlagsMutex;
_EthercatFlags EthercatFlags = {
    .calibration_loaded_from_eeprom = false,
    .write_electrical_angle_calibration_value = false,
    .write_joint_angle_calibration_value = false,
};

static barkour::Bootloader& bootloader_ = barkour::Bootloader::Get();

// Called from SOES during transition INIT->PREOP
// See SOES/soes/esc.c:ESC_state(), state transition INIT_TO_PREOP
//
// Used to initialize the application from a boot state
//
// Devel note: All SDO's are automatically initialized to their default values
// as defined in objectlist.c inside SOES, before this function is called
void set_defaults() {
  PW_LOG_INFO("ECAT SetDefaults");
  // Sets the software version
  strncpy(Obj.SDO100A.git_commit, barkour::kCodeVersion,
          std::size(barkour::kCodeVersion));

  // Sets the serial numbers.
  // Short serial.
  Obj.parameter.serial_number = barkour::Get32BitSerialNumber();
  Obj.parameter.extended_serial_number =
      barkour::Get64BitSerialNumber();  // Long serial.
#if MOTOR_TYPE == MOTOR_AK80_9
  Obj.parameter.num_pole_pairs = 21;
  Obj.parameter.torque_constant = 91;   // mNm/A
  Obj.parameter.rated_torque = 1000;    // mNm
  Obj.parameter.rated_current = 12000;  // mA
  Obj.parameter.current_limit = 30000;  //  max current in A / 1000
  Obj.parameter.torque_limit = 20000;   // max torque in Nm / 1000

  Obj.parameter.gear_motor_revolutions = 9;
  Obj.parameter.gear_shaft_revolutions = 1;
  Obj.parameter.phase_order = kAcbPhaseOrder;

  // Note: Measured resistance on the motor is higher than the datasheet
  // value, see b/230861111.
  Obj.parameter.phase_to_phase_resistance = 220;  // mOhm
#else
#error "Unknown motor type. Update ecat_device.cc."
#endif

#if ENCODER_TYPE == ENCODER_RLS_AKSIM2
  Obj.parameter.encoder_increments = 524288;
  Obj.parameter.encoder_motor_revolutions = 1;
#elif ENCODER_TYPE == ENCODER_MA732_ADA
  Obj.parameter.encoder_increments = 65536;
  Obj.parameter.encoder_motor_revolutions = 1;
#else
#error "Unknown encoder type. Update ecat_device.cc."
#endif
}

// Called from SOES when application is stopped (leaves OPERATIONAL)
// See SOES/soes/esc.c:ESC_stopoutput()
//
// Used to set all outputs to a safe state (stop motor driver, etc).
void safeoutput() {
  PW_LOG_INFO("ECAT SafeOutput");
  // MotorControl::Get().DriversOff()
}

// Called from inside SOES just after receiving a RxPDO packet (host to
// device). Its signature is fixed.
// See SOES/soes/ecat_slv.c:DIG_process()
//
// Due to the nature of EtherCAT, PDO's are only exchanged in OP state, so
// this function only gets called during OP.
//
// Devel note: typically, here you'd copy variables from Obj.rx and taking care
// that the data is served in little endian.
void cb_set_outputs() {
  static barkour::EthercatDevice& ecat_device = barkour::EthercatDevice::Get();
  ecat_device.CallCyclicCallback();
}

// Called from inside SOES just before preparing a TxPDO packet (device to
// host). Its signature is fixed.
// See SOES/soes/ecat_slv.c:DIG_process()
//
// Devel note: typically, here you'd copy variables into Obj.tx and taking care
// that the data is in little endian. This callback is rarely used.
void cb_get_inputs() {}

// Post-SDO-download callback.
uint32_t cb_post_sdo_download(uint16_t index, uint8_t subindex,
                              uint16_t flags) {
  // Trigger bank swap if the magic value is written to this entry.
  if (index == 0x3030 && Obj.foe.bank_swap_reset == 0xDEADC0DE) {
    PW_LOG_INFO("Booting new firmware");
    if (pw::Status status = bootloader_.BootNewFirmware(); !status.ok()) {
      PW_LOG_ERROR("Failed to boot new firmware");
    }
  } else if (index == 0x3030 && Obj.foe.bank_swap_reset == 0x01234567) {
    // Trigger reset (no bankswap) if magic value has been written.
    barkour::Reboot();
  }
  return 0;
}

// Print debug state change
#ifdef PW_LOG_DEBUG
void print_state(const char* func, uint8_t* transition_code,
                 uint8_t* state_code) {
  const char *transition, *state;

  switch (*transition_code) {
    // From INIT
    case INIT_TO_INIT: {
      transition = "INIT->INIT";
      break;
    }
    case INIT_TO_PREOP: {
      transition = "INIT->PREOP";
      break;
    }
    case INIT_TO_BOOT: {
      transition = "INIT->BOOT";
      break;
    }
    // From BOOT
    case BOOT_TO_BOOT: {
      transition = "BOOT->BOOT";
      break;
    }
    case BOOT_TO_PREOP: {
      transition = "BOOT->PREOP";
      break;
    }
    // From PREOP
    case PREOP_TO_PREOP: {
      transition = "PREOP->PREOP";
      break;
    }
    case PREOP_TO_INIT: {
      transition = "PREOP->INIT";
      break;
    }
    case PREOP_TO_SAFEOP: {
      transition = "PREOP->SAFEOP";
      break;
    }
    // From SAFEOP
    case SAFEOP_TO_SAFEOP: {
      transition = "SAFEOP->SAFEOP";
      break;
    }
    case SAFEOP_TO_PREOP: {
      transition = "SAFEOP->PREOP";
      break;
    }
    case SAFEOP_TO_INIT: {
      transition = "SAFEOP->INIT";
      break;
    }
    case SAFEOP_TO_OP: {
      transition = "SAFEOP->OP";
      break;
    }
    // From OP
    case OP_TO_OP: {
      transition = "OP->OP";
      break;
    }
    case OP_TO_SAFEOP: {
      transition = "OP->SAFEOP";
      break;
    }
    case OP_TO_PREOP: {
      transition = "OP->PREOP";
      break;
    }
    case OP_TO_INIT: {
      transition = "OP->INIT";
      break;
    }
    default: {
      transition = "INVALID TRANSITION";
      break;
    }
  }

  switch (*state_code & 0x0F) {
    case ESCinit: {
      state = "INIT";
      break;
    }
    case ESCpreop: {
      state = "PREOP";
      break;
    }
    case ESCboot: {
      state = "BOOT";
      break;
    }
    case ESCsafeop: {
      state = "SAFEOP";
      break;
    }
    case ESCop: {
      state = "OP";
      break;
    }
    default: {
      state = "UNKNOWN";
      break;
    }
  }

  PW_LOG_DEBUG("ECat %s: %s %s", func, transition, state);
}
#else
#define print_state(...)
#endif
}

////////////////////////////////////////////////////////////////////////////////
namespace {

// EEPROM address register, unfortunately not defined in SOES/soes/esc.h
constexpr uint32_t ESCREG_EEADDR = 0x504;

// Custom SII Locations.
constexpr uint32_t kElectricalAngleOffsetSiiAddress = 0x800;
constexpr uint32_t kElectricalAngleCalibrationCompletedSiiAddress = 0x802;
constexpr uint32_t kJointAngleOffsetSiiAddress = 0x803;
constexpr uint32_t kJointAngleCalibrationCompletedSiiAddress = 0x805;

// task id, required to wake the thread from interrupt context.
TaskHandle_t htask;

// SPI communications setup and handle.
SPI_HandleTypeDef spi_handle{
    .Instance{SPI5},
    .Init{
        .Mode{SPI_MODE_MASTER},
        .Direction{SPI_DIRECTION_2LINES},
        .DataSize{SPI_DATASIZE_8BIT},
        .CLKPolarity{SPI_POLARITY_HIGH},
        .CLKPhase{SPI_PHASE_2EDGE},
        .NSS{SPI_NSS_HARD_OUTPUT},
        .BaudRatePrescaler{SPI_BAUDRATEPRESCALER_4},
        .FirstBit{SPI_FIRSTBIT_MSB},
        .TIMode{SPI_TIMODE_DISABLE},
        .CRCCalculation{SPI_CRCCALCULATION_DISABLE},
        .NSSPMode{SPI_NSS_PULSE_DISABLE},
        .NSSPolarity{SPI_NSS_POLARITY_LOW},
        .FifoThreshold{SPI_FIFO_THRESHOLD_01DATA},
        .MasterSSIdleness{SPI_MASTER_SS_IDLENESS_00CYCLE},
        .MasterInterDataIdleness{SPI_MASTER_INTERDATA_IDLENESS_00CYCLE},
        .MasterKeepIOState{SPI_MASTER_KEEP_IO_STATE_ENABLE},
        .IOSwap{SPI_IO_SWAP_DISABLE},
    },
};

// Read an EtherCAT Device Controller register.
template <typename T>
inline T ESC_read(uint16_t address) {
  T val;
  ::ESC_read(address, &val, sizeof(T));
  return pw::bytes::ConvertOrderFrom(pw::endian::little, val);
}

// Write an EtherCAT Device Controller register.
template <typename T>
inline void ESC_write(uint16_t address, T val) {
  val = pw::bytes::ConvertOrderTo(pw::endian::little, val);
  ::ESC_write(address, &val, sizeof(T));
}

// SII functions. Details on the registers and commands used are not given in
// the EtherCAT spec, but rather in the documentation for the Beckhoff ESC chips
// (whose interface is copied by the Trinamic chip).
//
// Specifically, see the Beckhoff EtherCAT Slave Controller Hardware Data Sheet,
// Section II - Register Description, 2.45 SII EEPROM Interface.
//
// Link to document in Barkour drive:
// https://drive.google.com/file/d/1a2nH7LUmRiSCXOpddmGn_qxxQA8roA9A/view?usp=sharing&resourcekey=0-1nJkRyGAc4x9HUZjflcbXw

// A guard to make sure 0x501 is released when leaving SII functions.
struct PDIAccessGuard {
  PDIAccessGuard() { write(1); }
  ~PDIAccessGuard() { write(0); }
  inline void write(uint8_t value) { ESC_write(0x501, value); }
};

// Read 4 bytes from SII / EEPROM.
// Note: Only allowed during transition INIT->PREOP.
uint32_t SII_read(uint32_t addr) {
  PDIAccessGuard guard;

  ESC_write(ESCREG_EEADDR, addr);
  ESC_write(ESCREG_EECONTSTAT, uint16_t{0x100});  // Command SII_READ

  // Wait for read to complete
  Timer timer(50);
  while ((ESC_read<uint16_t>(ESCREG_EECONTSTAT) & 0x300) != 0) {
    if (timer.timedout()) {
      return 0;
    }
  }

  // Fetch value
  return ESC_read<uint32_t>(ESCREG_EEDATA);
}

// Writes 2 bytes to SII / EEPROM.
// Note: Only allowed during transition INIT->PREOP.
pw::Status SII_write(uint32_t addr, uint16_t value) {
  PDIAccessGuard guard;

  // Check if the EEPROM is currently busy.
  if (ESC_read<uint16_t>(ESCREG_EECONTSTAT) & 0x8000) {
    PW_LOG_DEBUG("Cannot write to SII - EEPROM busy.");
    return pw::Status::Unavailable();
  }

  // Check if there are any error bits left over from previous SII operations.
  uint16_t sii_error_bits = ESC_read<uint16_t>(ESCREG_EECONTSTAT) & 0x6000;

  if (sii_error_bits) {
    // Clear error bits.
    PW_LOG_DEBUG(
        "Found EEPROM control/status error bits: %#x. Attempting to clear.",
        sii_error_bits);

    // Write a zero command to clear the error bits.
    ESC_write(ESCREG_EECONTSTAT, uint16_t{0x0000});

    // Wait on the EEPROM busy flag, for clearing error bits to succeed.
    Timer error_clear_timer(50);
    while (ESC_read<uint16_t>(ESCREG_EECONTSTAT) & 0x8000) {
      if (error_clear_timer.timedout()) {
        PW_LOG_DEBUG(
            "Clearing EEPROM error bits - EEPROM busy timer timed out.");
        return pw::Status::DeadlineExceeded();
      }
    }
  }

  // Now issue the write command.
  ESC_write(ESCREG_EEADDR, addr);
  ESC_write(ESCREG_EEDATA, value);
  ESC_write(ESCREG_EECONTSTAT, uint16_t{0x200});  // Command SII_WRITE

  // Wait for write to complete.
  Timer write_complete_timer(50);
  while (ESC_read<uint16_t>(ESCREG_EECONTSTAT) & 0x8000) {
    if (write_complete_timer.timedout()) {
      PW_LOG_DEBUG("Writing to SII - EEPROM busy timer timed out.");
      return pw::Status::DeadlineExceeded();
    }
  }

  // Check for error conditions.
  sii_error_bits = ESC_read<uint16_t>(ESCREG_EECONTSTAT) & 0x6000;

  if (sii_error_bits) {
    PW_LOG_DEBUG("Found EEPROM control/status error bits after writing: %#x.",
                 sii_error_bits);
    return pw::Status::Internal();
  }

  // Success. Empirically, we need to wait a few ms after a write to allow the
  // EEPROM to complete the command, even after the EEPROM busy flag has been
  // unset by the ESC.
  Timer eeprom_settling_timer(5);
  while (!eeprom_settling_timer.timedout()) {
  }

  return pw::Status();  // OK
}

// Writes a calibrated angle offset (either joint or electrical angle) to
// SII EEPROM.
template <typename T>
pw::Status WriteCalibrationValue(T offset_angle, uint8_t calibration_completed,
                                 uint32_t offset_sii_address,
                                 uint32_t calibration_completed_sii_address) {
  // Reinterpret the floating point offset value as a pair of 16-bit
  // integers.
  static_assert(sizeof(uint32_t) == sizeof(offset_angle),
                "Angle offset is not the same size as a uint32.");

  // First do a bit_cast from float to uint32_t;
  uint32_t offset_uint32;
  memcpy(static_cast<void*>(&offset_uint32),
         static_cast<const void*>(&(offset_angle)), sizeof(uint32_t));

  uint16_t offset_word_upper = offset_uint32 >> 16;
  uint16_t offset_word_lower = offset_uint32 & 0xffff;

  // Store the words in little-endian order.
  if (pw::Status status = SII_write(offset_sii_address, offset_word_lower);
      !status.ok()) {
    PW_LOG_ERROR(
        "Failed to write lower 16 bits of the calibration offset at address: "
        "%#lx. Status: %s.",
        offset_sii_address, status.str());
    return status;
  }
  if (pw::Status status = SII_write(offset_sii_address + 1, offset_word_upper);
      !status.ok()) {
    PW_LOG_ERROR(
        "Failed to write upper 16 bits of the calibration offset at address: "
        "%#lx. Status: %s.",
        offset_sii_address + 1, status.str());
    return status;
  }

  // Write the flag which says if the calibration has been completed.
  if (pw::Status status =
          SII_write(calibration_completed_sii_address, calibration_completed);
      !status.ok()) {
    PW_LOG_ERROR(
        "Failed to the calibration completed_flag at address: %#lx. Status: "
        "%s.",
        calibration_completed_sii_address, status.str());
    return status;
  }

  return pw::Status();  // OK
}

// Updates the firmware by writing to FLASH_BANK_2.
//
// If file_info->address_offset falls on a sector boundary, the sector is erased
// before writing the new data. This implies that data should be written
// sequentially (or at least sector by sector), because the flash memory of a
// sector has to be erased before writing to it.
//
// To keep things simple, call this function repeatedly with length == 32,
// until the whole binary has been processed.
//
// Note that calling this function does not activate the new binary.
// See swap_flash_banks() or SDO 0x3030 for this.
//
// Args:
// - file_info: FoE file information. Only the address_offset and total_size are
// used.
// - data: Pointer to data to be written.
// - length: Number of bytes to write. This should be a multiple of 32 (bytes)
// and not cross flash sectors.
//
// Returns 0 if successful, >0 if an error occurred.
uint32_t flash_firmware(foe_file_cfg_t* file_info, uint8_t* data,
                        size_t length) {
  pw::Status status;
  // Assign source and destination addresses.
  uint32_t flash_address =
      (uint32_t)file_info->dest_start_address + file_info->address_offset;
  uint32_t data_address = reinterpret_cast<uint32_t>(data);

  status = bootloader_.UpdateFirmware(flash_address, data_address, length);
  return static_cast<uint32_t>(status.code());
}

// Called just before a state change is executed.
// (see SOES/soes/esc.c:ESC_state()
//
// This can be used to execute state exit instructions, or to check whether
// the application is ready for the transition, especially when changing from
// PREOP to SAFEOP
//
// To reject the transition PREOP_TO_SAFEOP for instance, set:
//      *transition = PREOP_TO_PREOP
//      *origin = ESCpreop | ESCerror
//      ESC_ALerror(ALERR_INVALIDSTATECHANGE);       see list in SOES/soes/esc.h
void pre_state_change(uint8_t* transition, uint8_t* origin) {
  print_state(__func__, transition, origin);
}

// Called just after state change completed
// (see SOES/soes/esc.c:ESC_state()
//
// This can be used in the application to execute state entry instructions
void post_state_change(uint8_t* transition,  // low nibble == old state
                                             // high nibble == new state
                       uint8_t* target       // new state
) {
  print_state(__func__, transition, target);

  if (*transition == INIT_TO_PREOP) {
    // SII EEPROM is under PDI control from Init -> PreOP
    // Read out vendor id, product code ane revision number from SII.
    Obj.SDO1018.vendor_id = SII_read(0x0008);
    Obj.SDO1018.product_code = SII_read(0x000a);
    Obj.SDO1018.revision_no = SII_read(0x000c);

    PW_LOG_INFO(
        "Vendor=0x%04lx Product=0x%04lx Revision=0x%04lx Serial=0x%04lx",
        Obj.SDO1018.vendor_id, Obj.SDO1018.product_code,
        Obj.SDO1018.revision_no, Obj.SDO1018.serial_no);

    if (EthercatFlags.write_electrical_angle_calibration_value) {
      PW_LOG_INFO(
          "Writing electrical angle calibration to SII / EEPROM. Calibrated "
          "offset value: %ld encoder counts.",
          Obj.SDO3000.electrical_angle_offset);

      if (!WriteCalibrationValue<int32_t>(
               Obj.SDO3000.electrical_angle_offset,
               Obj.SDO3000.electrical_angle_calibration_completed,
               kElectricalAngleOffsetSiiAddress,
               kElectricalAngleCalibrationCompletedSiiAddress)
               .ok()) {
        PW_LOG_ERROR(
            "Failed to write electrical angle calibration to SII / EEPROM.");
      }

      EthercatFlags.write_electrical_angle_calibration_value = false;
    } else {
      // If we're not writing the calibration entries, read them instead;
      uint32_t offset_uint32 = SII_read(kElectricalAngleOffsetSiiAddress);
      memcpy(static_cast<void*>(&(Obj.SDO3000.electrical_angle_offset)),
             static_cast<const void*>(&offset_uint32), sizeof(uint32_t));

      Obj.SDO3000.electrical_angle_calibration_completed =
          SII_read(kElectricalAngleCalibrationCompletedSiiAddress);

      PW_LOG_INFO(
          "Read electrical angle calibration info from EEPROM: offset value: "
          "%ld, calibration completed: %d",
          Obj.SDO3000.electrical_angle_offset,
          Obj.SDO3000.electrical_angle_calibration_completed);
    }

    if (EthercatFlags.write_joint_angle_calibration_value) {
      PW_LOG_INFO(
          "Writing joint angle calibration to SII / EEPROM. Calibrated "
          "offset value: %ld encoder counts.",
          Obj.SDO3001.joint_angle_offset);

      if (pw::Status status = WriteCalibrationValue<int32_t>(
              Obj.SDO3001.joint_angle_offset,
              Obj.SDO3001.joint_angle_calibration_completed,
              kJointAngleOffsetSiiAddress,
              kJointAngleCalibrationCompletedSiiAddress);
          !status.ok()) {
        PW_LOG_ERROR(
            "Failed to write joint angle calibration to SII / EEPROM.");
      }

      EthercatFlags.write_joint_angle_calibration_value = false;
    } else {
      uint32_t offset_uint32 = SII_read(kJointAngleOffsetSiiAddress);
      memcpy(static_cast<void*>(&(Obj.SDO3001.joint_angle_offset)),
             static_cast<const void*>(&offset_uint32), sizeof(uint32_t));

      Obj.SDO3001.joint_angle_calibration_completed =
          SII_read(kJointAngleCalibrationCompletedSiiAddress);

      PW_LOG_INFO(
          "Read joint angle calibration info from EEPROM: offset value: "
          "%ld, calibration completed: %d",
          Obj.SDO3001.joint_angle_offset,
          Obj.SDO3001.joint_angle_calibration_completed);
    }
  }

  // Setup FoE when in bootstrap mode
  if (*target != ESCboot) {
    // Disable FoE
    static foe_cfg_t config = {.n_files = 0};
    FOE_config(&config);

    if (*transition == BOOT_TO_INIT) {
      PW_LOG_INFO("Leaving Bootstrap");
      // Bank swap here and reboot if writing is successful?
    }
  } else {
    // Enable FoE only when in bootstrap mode
    // See doc/foe_firmware_update.h
    static foe_file_cfg_t files[] = {
        {.name = barkour::Bootloader::kFwNameM7.c_str(),
         .max_data = barkour::Bootloader::kFlashBankSizeInBytes,
         .dest_start_address = barkour::Bootloader::kFwUpdateCortexM7Addr,
         .write_only_in_boot = 1,
         .write_function = flash_firmware},

        {.name = barkour::Bootloader::kFwNameM4.c_str(),
         .max_data = barkour::Bootloader::kFlashBankSizeInBytes,
         .dest_start_address = barkour::Bootloader::kFwUpdateCortexM4Addr,
         .write_only_in_boot = 1,
         .write_function = flash_firmware},
    };

    static uint32_t fbuf[FLASH_NB_32BITWORD_IN_FLASHWORD];
    static foe_cfg_t config = {.fbuffer = (uint8_t*)&fbuf,
                               .buffer_size = sizeof(fbuf),
                               .n_files = PW_ARRAY_SIZE(files),
                               .files = files};

    FOE_config((foe_cfg_t*)&config);
    PW_LOG_INFO("Entered Bootstrap");

    // Memory cleanup.
    bootloader_.PrepareMemory();
  }
}

// Initialize CPU SPI hardware interface, DMA, interrupts
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_SPI5_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  // F9 Output PDI_SPI_MOSI
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOF, &gpio);

  // H5 Output PDI_SPI_CSN
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_5;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOH, &gpio);

  // H6 Output PDI_SPI_CLK
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_6;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOH, &gpio);

  // H7 Input PDI_SPI_MISO
  gpio.Pin = GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOH, &gpio);

  // DMA configuration for receive buffer
  static DMA_HandleTypeDef tx_dma{
      .Instance{DMA1_Stream3},
      .Init{
          .Request{DMA_REQUEST_SPI5_RX},
          .Direction{DMA_PERIPH_TO_MEMORY},
          .PeriphInc{DMA_PINC_DISABLE},
          .MemInc{DMA_MINC_ENABLE},
          .PeriphDataAlignment{DMA_PDATAALIGN_BYTE},
          .MemDataAlignment{DMA_MDATAALIGN_BYTE},
          .Mode{DMA_NORMAL},
          .Priority{DMA_PRIORITY_VERY_HIGH},
      },
  };
  PW_CHECK(HAL_OK == HAL_DMA_Init(&tx_dma));
  __HAL_LINKDMA(hspi, hdmarx, tx_dma);

  // DMA configuration for transmit buffer
  static DMA_HandleTypeDef rx_dma{
      .Instance{DMA1_Stream4},
      .Init{
          .Request{DMA_REQUEST_SPI5_TX},
          .Direction{DMA_MEMORY_TO_PERIPH},
          .PeriphInc{DMA_PINC_DISABLE},
          .MemInc{DMA_MINC_ENABLE},
          .PeriphDataAlignment{DMA_PDATAALIGN_BYTE},
          .MemDataAlignment{DMA_MDATAALIGN_BYTE},
          .Mode{DMA_NORMAL},
          .Priority{DMA_PRIORITY_VERY_HIGH},
      },
  };
  PW_CHECK(HAL_OK == HAL_DMA_Init(&rx_dma));
  __HAL_LINKDMA(hspi, hdmatx, rx_dma);

  // Interrupt
  HAL_NVIC_SetPriority(
      SPI5_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3, 0);
  HAL_NVIC_EnableIRQ(SPI5_IRQn);

  HAL_NVIC_SetPriority(
      DMA1_Stream3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  HAL_NVIC_SetPriority(
      DMA1_Stream4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

// Callback from ISR
void SPI_Complete(SPI_HandleTypeDef*) {
  BaseType_t flag = pdFALSE;
  vTaskNotifyGiveFromISR(htask, &flag);
  portYIELD_FROM_ISR(flag);
}

// Setup CPU hardware to EtherCAT device controller TMC8462 (ESC)
//
// Commented IO pins are connected from ESC to CPU, but unused.
// They're just there for documentation.
void SetupHardware() {
  GPIO_InitTypeDef gpio;

  PW_LOG_INFO("Setting up EtherCAT device.");

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  // __HAL_RCC_GPIOH_CLK_ENABLE();

  // // B7 Output LATCH_IN0
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
  // gpio.Pin = GPIO_PIN_7;
  // gpio.Mode = GPIO_MODE_OUTPUT_PP;
  // gpio.Pull = GPIO_NOPULL;
  // gpio.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(GPIOB, &gpio);

  // B11 Interrupt PDI_EOF
  gpio.Pin = GPIO_PIN_11;
  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gpio);

  // // B10 Input PDI_SOF
  // gpio.Pin = GPIO_PIN_10;
  // gpio.Mode = GPIO_MODE_INPUT;
  // gpio.Pull = GPIO_NOPULL;
  // HAL_GPIO_Init(GPIOB, &gpio);

  // // B12 Input PDI_IRQ_SPI
  // gpio.Pin = GPIO_PIN_12;
  // gpio.Mode = GPIO_MODE_INPUT;
  // gpio.Pull = GPIO_NOPULL;
  // HAL_GPIO_Init(GPIOB, &gpio);

  // // D2 Output MFC_CTRL_SPI_CSN
  // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,  GPIO_PIN_SET);
  // gpio.Pin = GPIO_PIN_2;
  // gpio.Mode = GPIO_MODE_OUTPUT_PP;
  // gpio.Pull = GPIO_NOPULL;
  // gpio.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(GPIOD, &gpio);

  // // D4 Output nRESET
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);

  // // D5 Input RESET_OUT
  // gpio.Pin = GPIO_PIN_5;
  // gpio.Mode = GPIO_MODE_INPUT;
  // gpio.Pull = GPIO_NOPULL;
  // HAL_GPIO_Init(GPIOD, &gpio);

  // // D8 Input PDI_WDSTATE
  // gpio.Pin = GPIO_PIN_8;
  // gpio.Mode = GPIO_MODE_INPUT;
  // gpio.Pull = GPIO_NOPULL;
  // HAL_GPIO_Init(GPIOD, &gpio);

  // // D9 Input PDI_WDTRIGGER
  // gpio.Pin = GPIO_PIN_9;
  // gpio.Mode = GPIO_MODE_INPUT;
  // gpio.Pull = GPIO_NOPULL;
  // HAL_GPIO_Init(GPIOD, &gpio);

  // // D12 Output LATCH_IN1
  // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,  GPIO_PIN_RESET);
  // gpio.Pin = GPIO_PIN_12;
  // gpio.Mode = GPIO_MODE_OUTPUT_PP;
  // gpio.Pull = GPIO_NOPULL;
  // gpio.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(GPIOD, &gpio);

  // D14 Input SYNC_OUT0
  gpio.Pin = GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &gpio);

  // D15 Input SYNC_OUT1
  gpio.Pin = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &gpio);

  // // H11 Input PROM_INIT
  // gpio.Pin = GPIO_PIN_11;
  // gpio.Mode = GPIO_MODE_INPUT;
  // gpio.Pull = GPIO_PULLDOWN;
  // HAL_GPIO_Init(GPIOH, &gpio);

  // Power cycle the EtherCAT chip just in case it didn't boot properly.
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
  vTaskDelay(pdMS_TO_TICKS(50));
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  vTaskDelay(pdMS_TO_TICKS(200));

  // Initialize SPI.
  PW_CHECK(HAL_OK == HAL_SPI_RegisterCallback(
                         &spi_handle, HAL_SPI_MSPINIT_CB_ID, HAL_SPI_MspInit));
  PW_CHECK(HAL_OK == HAL_SPI_Init(&spi_handle));
  PW_CHECK(HAL_OK == HAL_SPI_RegisterCallback(&spi_handle,
                                              HAL_SPI_TX_RX_COMPLETE_CB_ID,
                                              &SPI_Complete));

  // Interrupt
  HAL_NVIC_SetPriority(SPI5_IRQn,
                       configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3, 0);
  HAL_NVIC_EnableIRQ(SPI5_IRQn);
};

}  // anonymous namespace

namespace barkour {

EthercatDevice::EthercatDevice()
    : cyclic_callback_(nullptr), cyclic_callback_context_(nullptr) {
  ::SetupHardware();
  runloop_semaphore_ = xSemaphoreCreateBinaryStatic(&runloop_semaphore_buffer_);

  // Create a queue with at most one element, and an element size of zero.
  stop_queue_ = xQueueCreateStatic(1, 0, NULL, &static_stop_queue_);
}

EthercatDevice& EthercatDevice::Get() {
  static EthercatDevice instance;
  return instance;
}

void EthercatDevice::Sync0Interrupt() {
  // SYNC0 interrupt - "fast" interrupt when using oversampling
  // AssignActivate |= 0x300
}

void EthercatDevice::Sync1Interrupt() {
  // SYNC1 interrupt - "base" interrupt when using oversampling
  // AssignActivate = 0x700
}

void EthercatDevice::EofInterrupt() {
  // End of Frame interrupt
  BaseType_t flag = pdFALSE;
  xSemaphoreGiveFromISR(Get().runloop_semaphore_, &flag);
  portYIELD_FROM_ISR(flag);
}

pw::Status EthercatDevice::exchange(uint8_t* tx, uint8_t* rx, uint16_t len) {
  PW_CHECK(HAL_OK == HAL_SPI_TransmitReceive_DMA(&::spi_handle, tx, rx, len));

  return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50UL))
             ? PW_STATUS_OK
             : PW_STATUS_DEADLINE_EXCEEDED;
}

void EthercatDevice::Run() {
  bool first_run = true;

  while (true) {
    // Before starting, reset the queue.
    xQueueReset(stop_queue_);

    // Run until a restart is requested.
    RunOnce(first_run);
    first_run = false;
  }
}

void EthercatDevice::Restart() {
  PW_LOG_INFO("Requesting a restart of the EtherCAT stack.");

  // Attempt to send a "stop" message, returning immediately if this fails,
  // i.e., if a stop message is already pending.
  xQueueSend(stop_queue_, NULL, 0);
}

void EthercatDevice::RunOnce(bool first_run) {
  PW_LOG_INFO("Initializing the EtherCAT stack.");

  // We only want to initialize the SDOs with default values on the first run.
  bool skip_default_initialization = first_run ? false : true;

  esc_cfg_t config = {
      // This sets the SyncManager watchdog to time out if `DIG_process` is
      // called `watchdog_cnt` consecutive times with the `DIG_PROCESS_WD_FLAG`.
      // We set it to 1 as we manunally manage whether this flag is used based
      // on whether acquiring `runloop_semaphore_` times out after a set time.
      .watchdog_cnt = 1,
      .skip_default_initialization = skip_default_initialization,
      .set_defaults_hook = set_defaults,
      .pre_state_change_hook = pre_state_change,
      .post_state_change_hook = post_state_change,
      .safeoutput_override = safeoutput,
      .post_object_download_hook = cb_post_sdo_download,
  };

  htask = xTaskGetCurrentTaskHandle();

  ecat_slv_init(&config);
  PW_LOG_INFO("Finished initializing the EtherCAT stack.");

  BaseType_t ok;
  while (uxQueueMessagesWaiting(stop_queue_) == 0) {
    // If we need to wait more than 200 ms for the semaphore, that means that no
    // EtherCAT packet has been received in that time, so we pass the
    // `DIG_PROCESS_WD_FLAG` to cause the SyncManager watchdog counter to be
    // decremented. Most likely this is due to excessive jitter on the host
    // side, and will result in the EtherCAT application layer state moving to
    // SAFEOP + ERROR.
    //
    ok = xSemaphoreTake(runloop_semaphore_, pdMS_TO_TICKS(200));
    ecat_slv_poll();
    DIG_process(ok ? DIG_PROCESS_OUTPUTS_FLAG | DIG_PROCESS_INPUTS_FLAG
                   : DIG_PROCESS_WD_FLAG);
  }
}

void EthercatDevice::SetCyclicCallback(void (*callback)(void*), void* context) {
  cyclic_callback_ = callback;
  cyclic_callback_context_ = context;
}

void EthercatDevice::CallCyclicCallback() const {
  if (cyclic_callback_ != nullptr) {
    cyclic_callback_(cyclic_callback_context_);
  }
}

EthercatApplicationLayerState EthercatDevice::GetCurrentState() const {
  taskENTER_CRITICAL();
  uint16_t status = ESCvar.ALstatus;
  taskEXIT_CRITICAL();

  if (status & ESCop) {
    return EthercatApplicationLayerState::kOp;
  } else if (status & ESCinit) {
    return EthercatApplicationLayerState::kInit;
  } else if (status & ESCpreop) {
    return EthercatApplicationLayerState::kPreop;
  } else if (status & ESCsafeop) {
    return EthercatApplicationLayerState::kSafeop;
  } else if (status & ESCboot) {
    return EthercatApplicationLayerState::kBoot;
  } else {
    return EthercatApplicationLayerState::kUnknown;
  }
}

// Do not use. For testing purposes only.
uint32_t flash_firmware_test_only(uint32_t address_offset, uint32_t total_size,
                                  uint8_t* data, size_t length) {
  foe_file_cfg_t file_info;
  file_info.address_offset = address_offset;
  file_info.total_size = total_size;
  file_info.dest_start_address = 0x08100000;
  return flash_firmware(&file_info, data, length);
}

}  // namespace barkour

#ifdef __cplusplus
extern "C" {
#endif
void DMA1_Stream3_IRQHandler(void) { HAL_DMA_IRQHandler(::spi_handle.hdmarx); }
void DMA1_Stream4_IRQHandler(void) { HAL_DMA_IRQHandler(::spi_handle.hdmatx); }
void SPI5_IRQHandler(void) { HAL_SPI_IRQHandler(&::spi_handle); }
#ifdef __cplusplus
}
#endif
