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

#include "actuator/firmware/barkour/drivers/stm32h7_hal/gate_driver.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "actuator/firmware/barkour/common/board_config.h"
#include "actuator/firmware/barkour/drivers/stm32h7_hal/adc.h"
#include "pw_log/log.h"
#include "pw_span/span.h"
#include "pw_status/status.h"
#include "pw_status/try.h"
#include "drv8353.h" // NOLINT
#include "stm32h7xx_hal.h" // NOLINT
#include "stm32h7xx_hal_hrtim.h" // NOLINT

namespace barkour {

// Define these so the value used to set parameter and verify parameter
// is kept the same.

constexpr DRV8353_CTRL03_PeakSourCurHS_e kHSSourceAmps = ISour_HS_0p300_A;
constexpr DRV8353_CTRL03_PeakSinkCurHS_e kHSSinkAmps = ISink_HS_0p300_A;

constexpr DRV8353_CTRL04_PeakSourCurLS_e kLSSourceAmps = ISour_LS_0p300_A;
constexpr DRV8353_CTRL04_PeakSinkCurLS_e kLSSinkAmps = ISink_LS_0p300_A;

constexpr DRV8353_CTRL04_PeakTime_e kTSourceTimeNS = TSour_1000_ns;

constexpr DRV8353_CTRL05_VDSLVL_e kVDSLevelVolts = VDS_Level_1p500_V;
constexpr DRV8353_CTRL05_DeadTime_e kDeadTimeNS = DeadTime_400_ns;

constexpr uint32_t kPWMPeriod = 6000;  // 20Khz = 120Mhz/6000

// Converts a HAL status to pw::Status.
pw::Status HalStatusToPwStatus(const HAL_StatusTypeDef& hal_status,
                               const pw::Status& pw_status) {
  if (hal_status == HAL_OK) {
    return pw::OkStatus();
  } else {
    return pw_status;
  }
}

void Drv8353_HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  (void)hspi;
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_SPI6_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  // Configure GPIO.
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

void Drv8353_HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {
  (void)hspi;
  __HAL_RCC_SPI6_FORCE_RESET();
  __HAL_RCC_SPI6_RELEASE_RESET();
  HAL_GPIO_DeInit(GPIOG, GPIO_PIN_12);
  HAL_GPIO_DeInit(GPIOG, GPIO_PIN_13);
  HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14);
}

GateDriver& GateDriver::Get() {
  static GateDriver gate_driver;
  return gate_driver;
}

GateDriver::GateDriver()
    : state_(GateDriverState::NotInitialized()),
      voltage_on_sto_lines_{false, false},
      target_state_(GateDriverState::PowerOff()),
      sto_line_1_down_counter_(0),
      sto_line_2_down_counter_(0) {
  spi_handle_ = {
      .Instance = SPI6,
      .Init = {.Mode = SPI_MODE_MASTER,
               .Direction = SPI_DIRECTION_2LINES,
               .DataSize = SPI_DATASIZE_16BIT,
               .CLKPolarity = SPI_POLARITY_LOW,
               .CLKPhase = SPI_PHASE_2EDGE,
               .NSS = SPI_NSS_SOFT,
               .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256,
               .FirstBit = SPI_FIRSTBIT_MSB,
               .TIMode = SPI_TIMODE_DISABLE,
               .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
               .CRCPolynomial = 10,
               .CRCLength = SPI_CRC_LENGTH_8BIT,
               .NSSPMode = SPI_NSS_PULSE_DISABLE,
               .MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE}};
  motor_driver_ = {
      .spiHandle = &spi_handle_,
      .enGpioHandle = GPIOG,
      .enGpioNumber = GPIO_PIN_7,
      .nCSGpioHandle = GPIOG,
      .nCSGpioNumber = GPIO_PIN_8,
      .RxTimeOut = false,
      .enableTimeOut = false,
  };
}

GateDriverState GateDriver::CurrentState() const { return state_; }

GateDriverState GateDriver::Update() {
  // Observes hardware state.
  BoardConfig& board = barkour::BoardConfig::Get();
  voltage_on_sto_lines_.first = board.Sto1Status();
  voltage_on_sto_lines_.second = board.Sto2Status();
  const bool motor_fault = MotorFault();

  // Iterates over state transitions and performs action for state transition.
  GateDriverState next_state = state_;
  switch (state_) {
    case GateDriverState::NotInitialized(): {
      // Automatic transition to PowerOff state.
      if (Initialize().ok()) {
        next_state = GateDriverState::PowerOff();
      } else {
        PW_LOG_ERROR("Gate driver initialization failed.");
        next_state = GateDriverState::UnknownError();
      }
      break;
    }
    case GateDriverState::PowerOff(): {
      switch (target_state_) {
        case GateDriverState::PowerOnGateEnabled(): {
          Adc& adc = Adc::Get();
          // Wait to turn on till adc calibration is done.
          if (adc.IsCalibrated()) {
            next_state = GateDriverState::WaitingForPowerOn();
          }
          break;
        }
        default: {
          break;
        }
      }
      break;
    }
    case GateDriverState::WaitingForPowerOn(): {
      switch (target_state_) {
        case GateDriverState::PowerOnGateEnabled(): {
          if (VoltageOnStoLines()) {
            next_state = GateDriverState::PowerOnGateEnabledNotConfigured();
            sto_line_1_down_counter_ = 0;
            sto_line_2_down_counter_ = 0;
          } else {
            next_state = GateDriverState::WaitingForPowerOn();
          }
          break;
        }
        default: {
          next_state = GateDriverState::PowerOff();
          break;
        }
      }
      break;
    }
    case GateDriverState::PowerOnGateEnabledNotConfigured(): {
      switch (target_state_) {
        case GateDriverState::PowerOnGateEnabled(): {
          if (VoltageOnStoLines()) {
            if (ConfigureMotorDriver().ok()) {
              next_state = GateDriverState::PowerOnGateEnabled();
            } else {
              PW_LOG_ERROR("Gate driver configuration failed.");
              next_state = GateDriverState::UnknownError();
            }
          } else {
            next_state = GateDriverState::WaitingForPowerOn();
          }
          break;
        }
        default: {
          next_state = GateDriverState::PowerOff();
          break;
        }
      }
      break;
    }
    case GateDriverState::PowerOnGateEnabled(): {
      switch (target_state_) {
        case GateDriverState::PowerOnGateEnabled(): {
          if (motor_fault) {
            PW_LOG_WARN("Gate driver motor fault. Power cycling driver.");
            next_state = GateDriverState::PowerOff();
            DRV_SPI_8353_Vars_t driver_vars;
            driver_vars.ReadCmd = true;
            DRV8353_readData(&motor_driver_, &driver_vars);
            LogGateDriverState(driver_vars);
            break;
          }

          if (!VoltageOnStoLine1()) {
            if (++sto_line_1_down_counter_ > kMaxConsecutiveStoDownReadings) {
              PW_LOG_WARN(
                  "STO line 1 observed down %d consecutive times. Switching "
                  "off gate driver.",
                  sto_line_1_down_counter_);
              next_state = GateDriverState::WaitingForPowerOn();
            }
          } else {
            sto_line_1_down_counter_ = 0;
          }

          if (!VoltageOnStoLine2()) {
            if (++sto_line_2_down_counter_ > kMaxConsecutiveStoDownReadings) {
              PW_LOG_WARN(
                  "STO line 2 observed down %d consecutive times. Switching "
                  "off gate driver.",
                  sto_line_2_down_counter_);
              next_state = GateDriverState::WaitingForPowerOn();
            }
          } else {
            sto_line_2_down_counter_ = 0;
          }
          break;
        }
        default: {
          next_state = GateDriverState::PowerOff();
          break;
        }
      }
      break;
    }
    case GateDriverState::UnknownError(): {
      // No way to recover as of yet.
      break;
    }
    default: {
      PW_LOG_ERROR("Unhandled state in state transitions: %s.", state_.str());
      break;
    }
  }

  if (next_state != state_) {
    PW_LOG_INFO("Gate driver switching state from %s to %s.", state_.str(),
                next_state.str());
  }
  state_ = next_state;

  // Performs actions associated with state.
  switch (state_) {
    case GateDriverState::NotInitialized():
    case GateDriverState::PowerOff():
    case GateDriverState::UnknownError(): {
      if (!StopPwm().ok()) {
        PW_LOG_WARN("Could not stop PWM in %s mode.", state_.str());
      }
      if (!board.DisableMotorPower().ok()) {
        PW_LOG_WARN("Could not disable motor power in %s mode.", state_.str());
      }
      GateDisable();
      break;
    }
    case GateDriverState::WaitingForPowerOn(): {
      if (!StopPwm().ok()) {
        PW_LOG_WARN("Could not stop PWM in %s mode.", state_.str());
      }
      if (!board.EnableMotorPower().ok()) {
        PW_LOG_WARN("Could not enable motor power in %s mode.", state_.str());
      }
      GateDisable();
      break;
    }
    case GateDriverState::PowerOnGateEnabledNotConfigured(): {
      if (!StopPwm().ok()) {
        PW_LOG_WARN("Could not stop PWM in %s mode.", state_.str());
      }
      if (!board.MotorPowerEnabled()) {
        PW_LOG_WARN("Detected motor power disabled in %s mode.", state_.str());
      }
      GateEnable();
      break;
    }
    case GateDriverState::PowerOnGateEnabled(): {
      if (!board.MotorPowerEnabled()) {
        PW_LOG_WARN("Detected motor power disabled in %s mode.", state_.str());
      }
      GateEnable();
      break;
    }
    default: {
      PW_LOG_ERROR("Unknown state: %s.", state_.str());
      break;
    }
  }

  return state_;
}

pw::Status GateDriver::SetTargetState(const GateDriverState& state) {
  if (Update().error())
    return pw::Status::FailedPrecondition();

  GateDriverState new_target_state = GateDriverState::UnknownError();
  switch (state) {
    case GateDriverState::PowerOff():
    case GateDriverState::PowerOnGateEnabled(): {
      new_target_state = state;
      break;
    }
    case GateDriverState::NotInitialized():
    case GateDriverState::UnknownError():
    case GateDriverState::PowerOnGateEnabledNotConfigured():
    case GateDriverState::WaitingForPowerOn(): {
      PW_LOG_ERROR("Requesting an invalid state transition to %s.",
                   state.str());
      return pw::Status::InvalidArgument();
    }
    default: {
      PW_LOG_ERROR("Transition to unknown state requested: %s.", state.str());
      return pw::Status::InvalidArgument();
    }
  }

  if (target_state_ != new_target_state) {
    PW_LOG_INFO("Updating target state from %s to %s. Current state is: %s.",
                target_state_.str(), new_target_state.str(), state_.str());
  }

  target_state_ = new_target_state;

  return pw::OkStatus();
}

void GateDriver::GateEnable() {
  HAL_GPIO_WritePin(motor_driver_.enGpioHandle, motor_driver_.enGpioNumber,
                    GPIO_PIN_SET);
}

void GateDriver::GateDisable() {
  HAL_GPIO_WritePin(motor_driver_.enGpioHandle, motor_driver_.enGpioNumber,
                    GPIO_PIN_SET);
}

bool GateDriver::MotorFault() {
  return (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) == GPIO_PIN_RESET);
}

pw::Status GateDriver::Initialize() {
  // Register the SPI hardware initialization/deinitialization callbacks for
  // this SPI handle.
  //
  // The provided function handle, Drv8353_HAL_SPI_MspInit(), is responsible for
  // performing hardware initialization associated with the SPI peripheral
  // associated with spi_handle_.
  //
  // If this callback function were not provided, the HAL would instead call
  // HAL_SPI_MspInit(), a global initialization function for all SPI devices,
  // which is weakly defined by the HAL.
  //
  // By explicitly registering initialization callbacks in this fashion, the
  // need to initialize all SPI peripherals within a single function is avoided.
  PW_TRY(HalStatusToPwStatus(
      HAL_SPI_RegisterCallback(&spi_handle_, HAL_SPI_MSPINIT_CB_ID,
                               Drv8353_HAL_SPI_MspInit),
      pw::Status::Internal()));

  PW_TRY(HalStatusToPwStatus(
      HAL_SPI_RegisterCallback(&spi_handle_, HAL_SPI_MSPDEINIT_CB_ID,
                               Drv8353_HAL_SPI_MspDeInit),
      pw::Status::Internal()));

  PW_TRY(
      HalStatusToPwStatus(HAL_SPI_Init(&spi_handle_), pw::Status::Internal()));

  PW_TRY(StartPwm());

  return pw::OkStatus();
}

void GateDriver::LogGateDriverState(DRV_SPI_8353_Vars_t& driver_vars) {
  PW_LOG_DEBUG("DRV8353 Stat 0 VDS_LC: %d", driver_vars.Stat_Reg_00.VDS_LC);
  PW_LOG_DEBUG("DRV8353 Stat 0 VDS_HC: %d", driver_vars.Stat_Reg_00.VDS_HC);
  PW_LOG_DEBUG("DRV8353 Stat 0 VDS_LB: %d", driver_vars.Stat_Reg_00.VDS_LB);
  PW_LOG_DEBUG("DRV8353 Stat 0 VDS_HB: %d", driver_vars.Stat_Reg_00.VDS_HB);
  PW_LOG_DEBUG("DRV8353 Stat 0 VDS_LA: %d", driver_vars.Stat_Reg_00.VDS_LA);
  PW_LOG_DEBUG("DRV8353 Stat 0 VDS_HA: %d", driver_vars.Stat_Reg_00.VDS_HA);
  PW_LOG_DEBUG("DRV8353 Stat 0 OTSD: %d", driver_vars.Stat_Reg_00.OTSD);
  PW_LOG_DEBUG("DRV8353 Stat 0 UVLO: %d", driver_vars.Stat_Reg_00.UVLO);
  PW_LOG_DEBUG("DRV8353 Stat 0 GDF: %d", driver_vars.Stat_Reg_00.GDF);
  PW_LOG_DEBUG("DRV8353 Stat 0 VDS_OCP: %d", driver_vars.Stat_Reg_00.VDS_OCP);
  PW_LOG_DEBUG("DRV8353 Stat 0 FAULT: %d", driver_vars.Stat_Reg_00.FAULT);
  PW_LOG_DEBUG("DRV8353 Stat 1 VGS_LC: %d", driver_vars.Stat_Reg_01.VGS_LC);
  PW_LOG_DEBUG("DRV8353 Stat 1 VGS_HC: %d", driver_vars.Stat_Reg_01.VGS_HC);
  PW_LOG_DEBUG("DRV8353 Stat 1 VGS_LB: %d", driver_vars.Stat_Reg_01.VGS_LB);
  PW_LOG_DEBUG("DRV8353 Stat 1 VGS_HB: %d", driver_vars.Stat_Reg_01.VGS_HB);
  PW_LOG_DEBUG("DRV8353 Stat 1 VGS_LA: %d", driver_vars.Stat_Reg_01.VGS_LA);
  PW_LOG_DEBUG("DRV8353 Stat 1 VGS_HA: %d", driver_vars.Stat_Reg_01.VGS_HA);
  PW_LOG_DEBUG("DRV8353 Stat 1 GDUV: %d", driver_vars.Stat_Reg_01.GDUV);
  PW_LOG_DEBUG("DRV8353 Stat 1 OTW: %d", driver_vars.Stat_Reg_01.OTW);
  PW_LOG_DEBUG("DRV8353 Stat 1 SC_OC: %d", driver_vars.Stat_Reg_01.SC_OC);
  PW_LOG_DEBUG("DRV8353 Stat 1 SB_OC: %d", driver_vars.Stat_Reg_01.SB_OC);
  PW_LOG_DEBUG("DRV8353 Stat 1 SA_OC: %d", driver_vars.Stat_Reg_01.SA_OC);
}

pw::Status GateDriver::ConfigureMotorDriver() {
  DRV_SPI_8353_Vars_t driver_vars;
  DRV8353_setupSpi(&motor_driver_, &driver_vars);
  bool error = false;

  // Check for error conditions.
  if (driver_vars.Ctrl_Reg_02.PWM_MODE != PwmMode_6) {
    PW_LOG_ERROR("Gate driver has incorrect PWM mode.");
    error = true;
  }
  if (driver_vars.Ctrl_Reg_02.COAST || driver_vars.Ctrl_Reg_02.BRAKE) {
    PW_LOG_ERROR("Gate driver should not be in brake or coasting mode.");
    error = true;
  }
  if (driver_vars.Stat_Reg_00.FAULT) {
    PW_LOG_ERROR("Gate driver has fault on startup.");
    LogGateDriverState(driver_vars);
    error = true;
  }

  if (error) {
    return pw::Status::Internal();
  }

  driver_vars.Ctrl_Reg_02.OTW_REP =
      true;  // Report over temperature (OTW) as fault.
  driver_vars.Ctrl_Reg_02.PWM_MODE = PwmMode_6;  // 6xPWM mode.

  // High side gate current config.
  driver_vars.Ctrl_Reg_03.IDRIVEP_HS = kHSSourceAmps;  // peak source current.
  driver_vars.Ctrl_Reg_03.IDRIVEN_HS = kHSSinkAmps;    // peak sink current.
  // Low side gate current config.
  driver_vars.Ctrl_Reg_04.IDRIVEP_LS = kLSSourceAmps;  // peak source current.
  driver_vars.Ctrl_Reg_04.IDRIVEN_LS = kLSSinkAmps;    // peak sink current.
  // Peak gate current drive time.
  driver_vars.Ctrl_Reg_04.TDRIVE = kTSourceTimeNS;

  // Over current protection. Currently disabled to avoid frequent robot
  // crashes, see http://b/265942182.
  driver_vars.Ctrl_Reg_05.OCP_MODE = Disable_OCP;

  // Configure fixed dead-time.
  driver_vars.Ctrl_Reg_05.DEAD_TIME = kDeadTimeNS;

  // Configures current sensing across FETs.
  driver_vars.Ctrl_Reg_05.VDS_LVL = kVDSLevelVolts;
  driver_vars.Ctrl_Reg_06.CSA_FET = true;
  driver_vars.Ctrl_Reg_06.CSA_GAIN = Gain_20VpV;

  driver_vars.WriteCmd = true;
  DRV8353_writeData(&motor_driver_, &driver_vars);
  DRV8353_setupSpi(&motor_driver_, &driver_vars);

  if (!driver_vars.Ctrl_Reg_02.OTW_REP) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }
  if (driver_vars.Ctrl_Reg_02.PWM_MODE != PwmMode_6) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }
  if (driver_vars.Ctrl_Reg_05.OCP_MODE != Disable_OCP) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }
  if (driver_vars.Ctrl_Reg_03.IDRIVEP_HS != kHSSourceAmps) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }
  if (driver_vars.Ctrl_Reg_03.IDRIVEN_HS != kHSSinkAmps) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }
  if (driver_vars.Ctrl_Reg_04.IDRIVEP_LS != kLSSourceAmps) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }
  if (driver_vars.Ctrl_Reg_04.IDRIVEN_LS != kLSSinkAmps) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }
  if (driver_vars.Ctrl_Reg_04.TDRIVE != kTSourceTimeNS) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }
  if (driver_vars.Ctrl_Reg_05.VDS_LVL != kVDSLevelVolts) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }
  if (driver_vars.Ctrl_Reg_05.DEAD_TIME != kDeadTimeNS) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }
  if (driver_vars.Ctrl_Reg_06.CSA_GAIN != Gain_20VpV) {
    PW_LOG_ERROR("Gate driver registers weren't updated successfully.");
    return pw::Status::Internal();
  }

  return pw::OkStatus();
}

// Compare values are set to be center aligned.

// period = PWM Full period time
// duty_cycle = 0-1  or -1-0
// if duty_cycle > 0 compare value is for the turn off time and set > period/2
// if duty_cycle < 0 compare value is for the turn on  time and set < period/2
// duty = 0 compare = period /2
// duty_cycle = +1 compare = period
// duty_cycle = -1 compare = 0

uint32_t GateDriver::DutyCycleToCompareValue(uint32_t period,
                                             float duty_cycle) {
  uint32_t compare_value;

  compare_value = (period + static_cast<int32_t>(duty_cycle * period)) / 2;

  // +-5 - can't actually let compare goto 0 or period; per data sheet.
  compare_value = std::clamp<uint32_t>(compare_value, 5u, period - 5);

  return compare_value;
}

// Controls the duty cycles for the PWM outputs.
//
// Args:
// - duty_cycles: da,db,dc  are 3 floating point numbers between 0 and 1
//   (inclusive) specifying the PWM duty cycle, one value for each phase. 0 ==
//   low side FET on for the phase 100% of the time, 1 == high side FET on
//   100% of the time. Values outside the range [0, 1] will be clipped to this
//   range.
// - ignore_current_state: if true, this method does not verify that the gate
//   driver is currently in PowerOnGateEnabled mode.
//
//
//  NOTE: assumes da,db,dc are in physical order, if phases are swapped the
//  parameters
//        must be swapped before passing to this function.
//
//
// Returns:
// - pw::Ok: If the PWM duty cycles were updated successfully.
// - pw::Status::FailedPrecondition: If ignore_current_state == false and the
//   gate driver is not in the PowerOnGateEnabled state.
// - For pw::Ok the returned value is a value of the PhaseSampleSelection enum
//   indicating which two phase currents should be sampled by the injected
//   ADCs on the next FOC cycle.
pw::Result<PhaseSampleSelection> GateDriver::SetDutyCycles(
    float da, float db, float dc, bool ignore_current_state) {
  if (!ignore_current_state &&
      CurrentState() != GateDriverState::PowerOnGateEnabled()) {
    PW_LOG_ERROR(
        "PWM can only be set when the motor is powered on. Current "
        "state is: %s",
        CurrentState().str());
    return pw::Status::FailedPrecondition();
  }

  da = std::clamp(da, 0.0f, 1.0f);
  db = std::clamp(db, 0.0f, 1.0f);
  dc = std::clamp(dc, 0.0f, 1.0f);

  // All timers share the same period.
  uint32_t period = high_resolution_timer_config_.Instance
                        ->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C]
                        .PERxR;

  // Generate center aligned PWM pulses.
  // Phase A.
  high_resolution_timer_config_.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A]
      .CMP1xR = DutyCycleToCompareValue(period, -da);
  high_resolution_timer_config_.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A]
      .CMP2xR = DutyCycleToCompareValue(period, da);

  // Phase B.
  high_resolution_timer_config_.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B]
      .CMP1xR = DutyCycleToCompareValue(period, -db);
  high_resolution_timer_config_.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B]
      .CMP2xR = DutyCycleToCompareValue(period, db);

  // Phase C.
  high_resolution_timer_config_.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C]
      .CMP1xR = DutyCycleToCompareValue(period, -dc);
  high_resolution_timer_config_.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C]
      .CMP2xR = DutyCycleToCompareValue(period, dc);

  // Calculate which phase has minimum on time,
  // we will ignore the ADC reading from that phase.
  // Rational: The ADC needs time for CSA to settle and for ADC conversion.
  // ADC samples taken in middle of on times.
  // When on time is two short may not get a good reading on that phase,
  // therefore we use the other two phases and reconstruct the missing one.
  // Current hardware only allows simultaneous reading of 2 phases, so
  // we only ever use two phases at most.

  PhaseSampleSelection phase_select;

  if (da > db) {
    if (dc > da) {
      phase_select = PhaseSampleSelection::kAB;  // C max - ignored.
    } else {
      phase_select = PhaseSampleSelection::kBC;  // A max - ignored.
    }
  } else if (dc > db) {
    phase_select = PhaseSampleSelection::kAB;  // C max - ignored.
  } else {
    phase_select = PhaseSampleSelection::kAC;  // B max - ignored.
  }

  return phase_select;
}

pw::Status GateDriver::StartPwm() {
  // +1 for config settings used for TIMER_D (ADC triggers)
  auto timebase_configs = pw::span(timebase_config_, kNumPhases + 1);
  auto timer_configs = pw::span(timer_config_, kNumPhases + 1);
  auto output_configs = pw::span(output_config_, kNumPhases);
  auto compare_configs = pw::span(compare_config_, kNumPhases);
  auto deadtime_configs = pw::span(deadtime_config_, kNumPhases);
  auto fault_configs = pw::span(fault_config_, kNumPhases);
  uint32_t timer_indexes[kNumPhases] = {HRTIM_TIMERINDEX_TIMER_A,
                                        HRTIM_TIMERINDEX_TIMER_B,
                                        HRTIM_TIMERINDEX_TIMER_C};
  uint32_t timer_outputs[kNumPhases][2] = {
      {HRTIM_OUTPUT_TA1, HRTIM_OUTPUT_TA2},
      {HRTIM_OUTPUT_TB1, HRTIM_OUTPUT_TB2},
      {HRTIM_OUTPUT_TC1, HRTIM_OUTPUT_TC2},
  };

  // Initialize HRTIM.
  memset(&high_resolution_timer_config_, 0,
         sizeof(high_resolution_timer_config_));
  high_resolution_timer_config_.Instance = HRTIM1;
  high_resolution_timer_config_.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  high_resolution_timer_config_.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  PW_TRY(HalStatusToPwStatus(HAL_HRTIM_Init(&high_resolution_timer_config_),
                             pw::Status::Internal()));

  // Configure timers for each phase.
  for (uint8_t i = 0; i < kNumPhases; ++i) {
    // Configure the timebase.
    timebase_configs[i].Mode = HRTIM_MODE_CONTINUOUS;
    timebase_configs[i].Period = kPWMPeriod;
    timebase_configs[i].PrescalerRatio = HRTIM_PRESCALERRATIO_DIV2;
    timebase_configs[i].RepetitionCounter = 0;
    PW_TRY(HalStatusToPwStatus(
        HAL_HRTIM_TimeBaseConfig(&high_resolution_timer_config_,
                                 timer_indexes[i], &timebase_configs[i]),
        pw::Status::Internal()));

    // Configure the timer.
    timer_configs[i].DMARequests = HRTIM_TIM_DMA_NONE;
    timer_configs[i].DMASrcAddress = 0x0;
    timer_configs[i].DMADstAddress = 0x0;
    timer_configs[i].DMASize = 0x0;
    timer_configs[i].HalfModeEnable = HRTIM_HALFMODE_DISABLED;
    timer_configs[i].StartOnSync = HRTIM_SYNCSTART_DISABLED;
    timer_configs[i].ResetOnSync = HRTIM_SYNCRESET_DISABLED;
    timer_configs[i].DACSynchro = HRTIM_DACSYNC_NONE;
    timer_configs[i].PreloadEnable = HRTIM_PRELOAD_ENABLED;
    timer_configs[i].UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
    timer_configs[i].BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    timer_configs[i].RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
    timer_configs[i].ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
    timer_configs[i].InterruptRequests = HRTIM_TIM_IT_REP;
    timer_configs[i].PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
    timer_configs[i].FaultEnable = HRTIM_TIMFAULTENABLE_FAULT1;
    timer_configs[i].FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
    timer_configs[i].DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
    timer_configs[i].DelayedProtectionMode =
        HRTIM_TIMDELAYEDPROTECTION_DISABLED;
    timer_configs[i].UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
    timer_configs[i].ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
    PW_TRY(HalStatusToPwStatus(
        HAL_HRTIM_WaveformTimerConfig(&high_resolution_timer_config_,
                                      timer_indexes[i], &timer_configs[i]),
        pw::Status::Internal()));

    // Configure the outputs.
    // Upon reset the output will be logic-low. While the timer count exceeds
    // the CompareValue, the output will be logic-high.
    // The duty cycle can be computed for a given CompareValue:
    //   CompareValue 0:
    //     duty cycle = 0 [%]
    //   CompareValue (1,5999]:
    //     duty cycle = 100*(period - CompareValue)/period [%]
    //   CompareValue 6000:
    //     duty cycle = 100 [%]
    output_configs[i].Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    output_configs[i].SetSource = HRTIM_OUTPUTSET_TIMCMP1;
    output_configs[i].ResetSource = HRTIM_OUTPUTRESET_TIMCMP2;
    output_configs[i].IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
    output_configs[i].IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    output_configs[i].FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
    output_configs[i].ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    output_configs[i].BurstModeEntryDelayed =
        HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
    PW_TRY(
        HalStatusToPwStatus(HAL_HRTIM_WaveformOutputConfig(
                                &high_resolution_timer_config_,
                                timer_indexes[i],  // HRTIM_TIMERINDEX_TIMER_x.
                                timer_outputs[i][0],  // HRTIM_OUTPUT_Tx1.
                                &output_configs[i]),
                            pw::Status::Internal()));

    PW_TRY(
        HalStatusToPwStatus(HAL_HRTIM_WaveformOutputConfig(
                                &high_resolution_timer_config_,
                                timer_indexes[i],  // HRTIM_TIMERINDEX_TIMER_x.
                                timer_outputs[i][1],  // HRTIM_OUTPUT_Tx2.
                                &output_configs[i]),
                            pw::Status::Internal()));

    // Configure the compare units.
    // Default on point is 1/4 period,
    // Default off point is 3/4 period,
    // Therefore on time = 50% or 0 Volts.
    compare_configs[i].CompareValue = kPWMPeriod >> 2;  // 1/4 period
    PW_TRY(HalStatusToPwStatus(
        HAL_HRTIM_WaveformCompareConfig(&high_resolution_timer_config_,
                                        timer_indexes[i], HRTIM_COMPAREUNIT_1,
                                        &compare_configs[i]),
        pw::Status::Internal()));

    compare_configs[i].CompareValue = (kPWMPeriod * 3) >> 2;  // 3/4 period
    PW_TRY(HalStatusToPwStatus(
        HAL_HRTIM_WaveformCompareConfig(&high_resolution_timer_config_,
                                        timer_indexes[i], HRTIM_COMPAREUNIT_2,
                                        &compare_configs[i]),
        pw::Status::Internal()));

    // Configure dead time.
    // Note: The dead-time setting here does not really matter, as it is
    //   automatically added by the gate driver.
    // f_HRTIM = 240Mhz (see SystemClock_Config())
    // f_DTG = f_HRTIM * HRTIM_DeadTimeCfgTypeDef.
    // Prescaler = 240Mhz * 8 = 1920Mhz
    // Thus FallingValue of 40 => 40/1920Mhz ~= 20.8nS.
    deadtime_configs[i].FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
    deadtime_configs[i].FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
    deadtime_configs[i].FallingSignLock =
        HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_READONLY;
    // Add extra dead time.
    deadtime_configs[i].FallingValue = static_cast<uint16_t>(0);
    deadtime_configs[i].Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV2;
    deadtime_configs[i].RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
    deadtime_configs[i].RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
    deadtime_configs[i].RisingSignLock =
        HRTIM_TIMDEADTIME_RISINGSIGNLOCK_READONLY;
    // Add extra dead time.
    deadtime_configs[i].RisingValue = static_cast<uint16_t>(0);
    PW_TRY(HalStatusToPwStatus(
        HAL_HRTIM_DeadTimeConfig(&high_resolution_timer_config_,
                                 timer_indexes[i],
                                 &deadtime_configs[i]),
        pw::Status::Internal()));
  }

  // Configure the timebase for timer D (ADC Trigger).
  // Set timer D period to 2x regular period so it only generates
  // ADC trigger on every 2nd PWM cycle (50us*2 = 100us)
  // this way the control loop has enough time (needs ~30-40us) to do one loop.
  uint32_t timer_d_idx = kNumPhases;

  timebase_configs[timer_d_idx].Mode = HRTIM_MODE_CONTINUOUS;
  // 10khz = 20Khz/2 = 120Mhz/(2*6000)
  timebase_configs[timer_d_idx].Period = 2 * kPWMPeriod;
  // 120Mhz
  timebase_configs[timer_d_idx].PrescalerRatio = HRTIM_PRESCALERRATIO_DIV2;
  timebase_configs[timer_d_idx].RepetitionCounter = 0;
  PW_TRY(HalStatusToPwStatus(
      HAL_HRTIM_TimeBaseConfig(&high_resolution_timer_config_,
                               HRTIM_TIMERINDEX_TIMER_D,
                               &timebase_configs[timer_d_idx]),
      pw::Status::Internal()));

  // Configure the D timer.
  timer_configs[timer_d_idx].DMARequests = HRTIM_TIM_DMA_NONE;
  timer_configs[timer_d_idx].DMASrcAddress = 0x0;
  timer_configs[timer_d_idx].DMADstAddress = 0x0;
  timer_configs[timer_d_idx].DMASize = 0x0;
  timer_configs[timer_d_idx].HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  timer_configs[timer_d_idx].StartOnSync = HRTIM_SYNCSTART_DISABLED;
  timer_configs[timer_d_idx].ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  timer_configs[timer_d_idx].DACSynchro = HRTIM_DACSYNC_NONE;
  timer_configs[timer_d_idx].PreloadEnable = HRTIM_PRELOAD_ENABLED;
  timer_configs[timer_d_idx].UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  timer_configs[timer_d_idx].BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  timer_configs[timer_d_idx].RepetitionUpdate =
      HRTIM_UPDATEONREPETITION_ENABLED;
  timer_configs[timer_d_idx].ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  timer_configs[timer_d_idx].InterruptRequests = HRTIM_TIM_IT_REP;
  timer_configs[timer_d_idx].PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  timer_configs[timer_d_idx].FaultEnable = HRTIM_TIMFAULTENABLE_FAULT1;
  timer_configs[timer_d_idx].FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  timer_configs[timer_d_idx].DeadTimeInsertion =
      HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  timer_configs[timer_d_idx].DelayedProtectionMode =
      HRTIM_TIMDELAYEDPROTECTION_DISABLED;
  timer_configs[timer_d_idx].UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  timer_configs[timer_d_idx].ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  PW_TRY(HalStatusToPwStatus(
      HAL_HRTIM_WaveformTimerConfig(&high_resolution_timer_config_,
                                    HRTIM_TIMERINDEX_TIMER_D,
                                    &timer_configs[timer_d_idx]),
      pw::Status::Internal()));

  // User timer D (No Output) to trigger ADC.
  // Configure compare unit 2 config for the ADC. Used as ADC Trigger.
  // The injected ADC conversion begin trigger point is set here.
  // The Trigger point is adjusted to sample at middle of off period of
  // PWM cycle.
  HRTIM_CompareCfgTypeDef adc_compare_config = {0};
  // Trigger @ middle of 1st and 2nd PWM period (when all phases are low).
  adc_compare_config.CompareValue = kPWMPeriod / 2;
  PW_TRY(HalStatusToPwStatus(
      HAL_HRTIM_WaveformCompareConfig(&high_resolution_timer_config_,
                                      HRTIM_TIMERINDEX_TIMER_D,
                                      HRTIM_COMPAREUNIT_2, &adc_compare_config),
      pw::Status::Internal()));

  // Configure the ADC trigger.
  // When Timer D compare unit 2 is triggered, ADC Trigger 2 will go active.
  // Note: The ADC must be set up to act upon this signal.
  HRTIM_ADCTriggerCfgTypeDef adc_trigger_config = {0};
  adc_trigger_config.Trigger = HRTIM_ADCTRIGGEREVENT24_TIMERD_CMP2;
  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_D;
  PW_TRY(HalStatusToPwStatus(
      HAL_HRTIM_ADCTriggerConfig(&high_resolution_timer_config_,
                                 HRTIM_ADCTRIGGER_2,
                                 &adc_trigger_config),
      pw::Status::Internal()));

  // make sure outputs start in 0 torque state.
  SetDutyCycles(0.5f, 0.5f, 0.5f, true).IgnoreError();

  // Start PWM.
  PW_TRY(HalStatusToPwStatus(
      HAL_HRTIM_WaveformOutputStart(&high_resolution_timer_config_,
                                    HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                                        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
                                        HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2),
      pw::Status::Internal()));

  HAL_HRTIM_WaveformCountStart(&high_resolution_timer_config_,
                               HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B |
                                   HRTIM_TIMERID_TIMER_C |
                                   HRTIM_TIMERID_TIMER_D);

  // Calibrate ADC Offsets
  Adc& adc = Adc::Get();
  adc.StartOffsetCalibration();

  return pw::OkStatus();
}

pw::Status GateDriver::StopPwm() {
  // For now, this just makes the motor coast.
  pw::Result<PhaseSampleSelection> result =
      SetDutyCycles(0.5f, 0.5f, 0.5f, true);

  if (result.ok()) {
    return pw::OkStatus();
  }

  return result.status();
}

GateDriverStatusRegisters GateDriver::GetFaultStatusRegisters() {
  GateDriverStatusRegisters registers = {0, 0};
  registers.fault_register_1 =
      DRV8353_readSpi(&motor_driver_, Address_Status_0);
  registers.fault_register_2 =
      DRV8353_readSpi(&motor_driver_, Address_Status_1);
  return registers;
}

}  // namespace barkour

extern "C" {
void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef* hhrtim) {
  (void)hhrtim;
  GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_HRTIM1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // Phases C High, C Low, B Low.
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_HRTIM1;
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Phases A High, A Low, B High.
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_HRTIM1;
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
}
