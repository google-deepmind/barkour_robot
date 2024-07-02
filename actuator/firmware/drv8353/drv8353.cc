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

//! \file   drivers/drvic/drv8305/src/32b/f28x/f2806x/drv8353.c
//! \brief  Contains the various functions related to the DRV8353 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.
//  MW_INSTALL_DIR
//  ${PROJECT_LOC}\..\..\..\..\..\..\..\..\..\..
//
// **************************************************************************
// the includes
#include "drv8353.h"  // NOLINT

#include <math.h>
#include <stdio.h>

#include <cassert>

#include "FreeRTOS.h" // NOLINT
#include "cycle_counter.h"  // NOLINT
#include "task.h" // NOLINT

// **************************************************************************
// drivers


// **************************************************************************
// modules


// **************************************************************************
// platforms


// **************************************************************************
// the defines
#define SPI_TIMEOUT_DURATION 1000

// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


DRV8353_Handle DRV8353_init(void *pMemory, const size_t numBytes)
{
  DRV8353_Handle handle;

  if (numBytes < sizeof(DRV8353_Obj))
    return((DRV8353_Handle)NULL);

  // assign the handle
  handle = (DRV8353_Handle)pMemory;

  DRV8353_resetRxTimeout(handle);
  DRV8353_resetEnableTimeout(handle);

  return(handle);
}  // end of DRV8353_init() function


void DRV8353_enable(DRV8353_Handle handle)
{
  // Enable the DRV8353
  HAL_GPIO_WritePin(handle->enGpioHandle, handle->enGpioNumber, GPIO_PIN_SET);

  // Wait for driver to come online
  vTaskDelay(10);

  while ((DRV8353_readSpi(handle, Address_Status_0) &
          DRV8353_STATUS00_FAULT_BITS) != 0) {}

  // Wait for the DRV8301 registers to update
  vTaskDelay(1);

  return;
}  // end of DRV8353_enable() function

#ifdef DRV8353_SPI
DRV8353_CTRL03_PeakSourCurHS_e DRV8353_getPeakSourCurHS(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle, Address_Control_3);

  // mask the bits
  data &= DRV8353_CTRL03_IDRIVEP_HS_BITS;

  return((DRV8353_CTRL03_PeakSourCurHS_e)data);
}  // end of DRV8353_getPeakSourCurHS function


DRV8353_CTRL03_PeakSinkCurHS_e DRV8353_getPeakSinkCurHS(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle, Address_Control_3);

  // mask the bits
  data &= DRV8353_CTRL03_IDRIVEN_HS_BITS;

  return((DRV8353_CTRL03_PeakSinkCurHS_e)data);
}  // end of DRV8353_getPeakSinkCurHS function


DRV8353_CTRL04_PeakTime_e DRV8353_getPeakSourTime(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle, Address_Control_4);

  // mask the bits
  data &= DRV8353_CTRL04_TDRIVE_BITS;

  return((DRV8353_CTRL04_PeakTime_e)data);
}  // end of DRV8353_getPeakSourTime function


DRV8353_CTRL04_PeakSourCurLS_e DRV8353_getPeakSourCurLS(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle, Address_Control_4);

  // mask the bits
  data &= DRV8353_CTRL04_IDRIVEP_LS_BITS;

  return((DRV8353_CTRL04_PeakSourCurLS_e)data);
}  // end of DRV8353_getPeakSourCurLS function


DRV8353_CTRL04_PeakSinkCurLS_e DRV8353_getPeakSinkCurLS(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle, Address_Control_4);

  // mask the bits
  data &= DRV8353_CTRL04_IDRIVEN_LS_BITS;

  return((DRV8353_CTRL04_PeakSinkCurLS_e)data);
}  // end of DRV8353_getPeakSinkCurLS function
#endif
/*
DRV8353_CTRL04_PeakSinkTime_e DRV8353_getPeakSinkTime(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle,Address_Control_4);

  // mask the bits
  data &= DRV8353_CTRL04_TDRIVE_BITS;

  return((DRV8353_CTRL04_PeakSinkTime_e)data);
} // end of DRV8353_getPeakSinkTime function
*/
#ifdef DRV8353_SPI
DRV8353_CTRL05_OcpDeg_e DRV8353_getVDSDeglitch(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle, Address_Control_5);

  // mask the bits
  data &= DRV8353_CTRL05_OCP_DEG_BITS;

  return((DRV8353_CTRL05_OcpDeg_e)data);
}  // end of DRV8353_getVDSDeglitch function

DRV8353_CTRL05_DeadTime_e DRV8353_getDeadTime(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle, Address_Control_5);

  // mask the bits
  data &= DRV8353_CTRL05_DEAD_TIME_BITS;

  return((DRV8353_CTRL05_DeadTime_e)data);
}  // end of DRV8353_getDeadTime function


DRV8353_CTRL02_PwmMode_e DRV8353_getPwmMode(DRV8353_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8353_readSpi(handle, Address_Control_2);

  // mask the bits
  data &= DRV8353_CTRL02_PWM_MODE_BITS;

  return((DRV8353_CTRL02_PwmMode_e)data);
}  // end of DRV8353_getPwmMode function


void DRV8353_setSpiHandle(DRV8353_Handle handle, SPI_Handle spiHandle)
{
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;

  // initialize the serial peripheral interface object
  obj->spiHandle = spiHandle;

  return;
}  // end of DRV8353_setSpiHandle() function
#endif

void DRV8353_setEnGpioHandle(DRV8353_Handle handle, GPIO_Handle gpioHandle) {
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;

  // initialize the gpio interface object
  obj->enGpioHandle = gpioHandle;

  return;
}  // end of DRV8353_setEnGpioHandle() function

void DRV8301_setnCSGpioHandle(DRV8353_Handle handle, GPIO_Handle gpioHandle) {
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;

  // initialize the gpio interface object
  obj->nCSGpioHandle = gpioHandle;

  return;
}  // end of DRV8301_setnCSGpioHandle() function

void DRV8301_setnCSGpioNumber(DRV8353_Handle handle, GPIO_Number_e gpioNumber) {
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;

  // initialize the gpio interface object
  obj->nCSGpioNumber = gpioNumber;

  return;
}  // end of DRV8301_setnCSGpioNumber() function

void DRV8353_setEnGpioNumber(DRV8353_Handle handle, GPIO_Number_e gpioNumber) {
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;

  // initialize the gpio interface object
  obj->enGpioNumber = gpioNumber;

  return;
}  // end of DRV8353_setGpioNumber() function

#ifdef DRV8353_SPI
void DRV8353_setupSpi(DRV8353_Handle handle, DRV_SPI_8353_Vars_t *Spi_8353_Vars)
{
  DRV8353_Address_e drvRegAddr;
  uint16_t drvDataNew;

  // Set Default Values
  // Manual Read/Write
  Spi_8353_Vars->ManReadAddr  = 0;
  Spi_8353_Vars->ManReadData  = 0;
  Spi_8353_Vars->ManReadCmd = false;
  Spi_8353_Vars->ManWriteAddr = 0;
  Spi_8353_Vars->ManWriteData = 0;
  Spi_8353_Vars->ManWriteCmd = false;

  // Read/Write
  Spi_8353_Vars->ReadCmd  = false;
  Spi_8353_Vars->WriteCmd = false;

  // Read registers for default values
  // Read Status Register 0
  drvRegAddr = Address_Status_0;
  drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
  Spi_8353_Vars->Stat_Reg_00.VDS_LC =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_HC =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_LB =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LB_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_HB =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HB_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_LA =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LA_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_HA =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HA_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.OTSD =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_OTSD_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.UVLO =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_UVLO_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.GDF =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_GDF_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.VDS_OCP =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_OCP_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_00.FAULT =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_FAULT_BITS)?1:0;

  // Read Status Register 1
  drvRegAddr = Address_Status_1;
  drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
  Spi_8353_Vars->Stat_Reg_01.VGS_LC =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.VGS_HC =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.VGS_LB =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LB_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.VGS_HB =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HB_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.VGS_LA =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LA_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.VGS_HA =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HA_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.GDUV =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_GDUV_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.OTW =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_OTW_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.SC_OC =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SC_OC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.SB_OC =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SB_OC_BITS)?1:0;
  Spi_8353_Vars->Stat_Reg_01.SA_OC =
      (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SA_OC_BITS)?1:0;

    // Read Control Register 2
  drvRegAddr = Address_Control_2;
  drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_02.CLR_FLT =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_CLR_FLT_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.BRAKE =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_BRAKE_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.COAST =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_COAST_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.PWM1_DIR =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_PWM1_DIR_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.PWM1_COM =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_PWM1_COM_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.PWM_MODE =
      (DRV8353_CTRL02_PwmMode_e)(drvDataNew &
                                 (uint16_t)DRV8353_CTRL02_PWM_MODE_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.OTW_REP =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_OTW_REP_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.DIS_GDF =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_DIS_GDF_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.DIS_CPUV =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_DIS_CPUV_BITS);
  Spi_8353_Vars->Ctrl_Reg_02.OCP_ACT =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_OCP_ACT)?1:0;

  // Read Control Register 3
  drvRegAddr = Address_Control_3;
  drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_03.IDRIVEN_HS =
      (DRV8353_CTRL03_PeakSinkCurHS_e)
      (drvDataNew & (uint16_t)DRV8353_CTRL03_IDRIVEN_HS_BITS);
  Spi_8353_Vars->Ctrl_Reg_03.IDRIVEP_HS =
      (DRV8353_CTRL03_PeakSourCurHS_e)
      (drvDataNew & (uint16_t)DRV8353_CTRL03_IDRIVEP_HS_BITS);
  Spi_8353_Vars->Ctrl_Reg_03.LOCK =
      (DRV8353_CTRL03_Lock_e)(drvDataNew & (uint16_t)DRV8353_CTRL03_LOCK_BITS);

  // Read Control Register 4
  drvRegAddr = Address_Control_4;
  drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_04.IDRIVEN_LS =
      (DRV8353_CTRL04_PeakSinkCurLS_e)
      (drvDataNew & (uint16_t)DRV8353_CTRL04_IDRIVEN_LS_BITS);
  Spi_8353_Vars->Ctrl_Reg_04.IDRIVEP_LS =
      (DRV8353_CTRL04_PeakSourCurLS_e)
      (drvDataNew & (uint16_t)DRV8353_CTRL04_IDRIVEP_LS_BITS);
  Spi_8353_Vars->Ctrl_Reg_04.TDRIVE =
      (DRV8353_CTRL04_PeakTime_e)
      (drvDataNew & (uint16_t)DRV8353_CTRL04_TDRIVE_BITS);
  Spi_8353_Vars->Ctrl_Reg_04.CBC =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL04_CBC_BITS)?1:0;

  // Read Control Register 5
  drvRegAddr = Address_Control_5;
  drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_05.VDS_LVL =
      (DRV8353_CTRL05_VDSLVL_e)(drvDataNew &
                                (uint16_t)DRV8353_CTRL05_VDS_LVL_BITS);
  Spi_8353_Vars->Ctrl_Reg_05.OCP_DEG =
      (DRV8353_CTRL05_OcpDeg_e)(drvDataNew &
                                (uint16_t)DRV8353_CTRL05_OCP_DEG_BITS);
  Spi_8353_Vars->Ctrl_Reg_05.OCP_MODE =
      (DRV8353_CTRL05_OcpMode_e)(drvDataNew &
                                 (uint16_t)DRV8353_CTRL05_OCP_MODE_BITS);
  Spi_8353_Vars->Ctrl_Reg_05.DEAD_TIME =
      (DRV8353_CTRL05_DeadTime_e)(drvDataNew &
                                  (uint16_t)DRV8353_CTRL05_DEAD_TIME_BITS);
  Spi_8353_Vars->Ctrl_Reg_05.TRETRY =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL05_TRETRY_BITS);

  // Read Control Register 6
  drvRegAddr = Address_Control_6;
  drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_06.SEN_LVL =
      (DRV8353_CTRL06_SENLevel_e)(drvDataNew &
                                  (uint16_t)DRV8353_CTRL06_SEN_LVL_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_C =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_C_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_B =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_B_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_A =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_A_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.DIS_SEN =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_DIS_SEN_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.CSA_GAIN =
      (DRV8353_CTRL06_CSAGain_e)(drvDataNew &
                                 (uint16_t)DRV8353_CTRL06_CSA_GAIN_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.LS_REF =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_LS_REF_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.VREF_DIV =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_VREF_DIV_BITS);
  Spi_8353_Vars->Ctrl_Reg_06.CSA_FET =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_FET_BITS);

  // Read Control Register 7
  drvRegAddr = Address_Control_7;
  drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
  Spi_8353_Vars->Ctrl_Reg_07.CAL_MODE =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_CAL_MODE_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_1 =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED1_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_2 =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED2_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_3 =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED3_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_4 =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED4_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_5 =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED5_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_6 =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED6_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_7 =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED7_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_8 =
      (bool)(drvDataNew & (uint16_t)DRV8335_CTRL07_RESERVED8_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_9 =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED9_BITS);
  Spi_8353_Vars->Ctrl_Reg_07.RESERVED_10 =
      (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED10_BITS);

  return;
}  // end of DRV8353_setupSpi() function


uint16_t DRV8353_readSpi(DRV8353_Handle handle, const DRV8353_Address_e regAddr)
{
  // Actuates chipselect.
  HAL_GPIO_WritePin(handle->nCSGpioHandle, handle->nCSGpioNumber,
                    GPIO_PIN_RESET);
  barkour::CycleCounter::Get().BlockForMicroseconds(1);

  // Does blocking read.
  uint16_t zerobuff = 0;
  uint16_t controlword =
      (uint16_t)DRV8353_buildCtrlWord(CtrlMode_Read, regAddr, 0);
  uint16_t recbuff = 0xbeef;
  HAL_SPI_TransmitReceive(handle->spiHandle, (uint8_t *)(&controlword),
                          (uint8_t *)(&recbuff), 1, SPI_TIMEOUT_DURATION);

  barkour::CycleCounter::Get().BlockForMicroseconds(1);

  HAL_GPIO_WritePin(handle->nCSGpioHandle, handle->nCSGpioNumber, GPIO_PIN_SET);

  // TODO(b/167457971): Use pw_assert (PW_CHECK_UNIT_NE, specifically).
  assert(recbuff != 0xbeef);

  return (recbuff & DRV8353_DATA_MASK);
}  // end of DRV8353_readSpi() function

void DRV8353_writeSpi(DRV8353_Handle handle, const DRV8353_Address_e regAddr,
                      const uint16_t data)
{
  // Actuates chipselect.
  HAL_GPIO_WritePin(handle->nCSGpioHandle, handle->nCSGpioNumber,
                    GPIO_PIN_RESET);
  barkour::CycleCounter::Get().BlockForMicroseconds(1);

  // Does blocking write.
  uint16_t controlword =
      (uint16_t)DRV8353_buildCtrlWord(CtrlMode_Write, regAddr, data);
  HAL_SPI_Transmit(handle->spiHandle, (uint8_t *)(&controlword), 1,
                   SPI_TIMEOUT_DURATION);
  barkour::CycleCounter::Get().BlockForMicroseconds(1);

  // Actuates chipselect.
  HAL_GPIO_WritePin(handle->nCSGpioHandle, handle->nCSGpioNumber, GPIO_PIN_SET);
  barkour::CycleCounter::Get().BlockForMicroseconds(1);

  return;
}  // end of DRV8353_writeSpi() function

void DRV8353_writeData(DRV8353_Handle handle,
                       DRV_SPI_8353_Vars_t *Spi_8353_Vars)
{
  DRV8353_Address_e drvRegAddr;
  uint16_t drvDataNew = 0;

  if (Spi_8353_Vars->WriteCmd)
  {
      // Write Control Register 2
      drvRegAddr = Address_Control_2;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_02.CLR_FLT << 0)  | \
                   (Spi_8353_Vars->Ctrl_Reg_02.BRAKE << 1)    | \
                   (Spi_8353_Vars->Ctrl_Reg_02.COAST <<2)     | \
                   (Spi_8353_Vars->Ctrl_Reg_02.PWM1_DIR << 3) | \
                   (Spi_8353_Vars->Ctrl_Reg_02.PWM1_COM << 4) | \
                   (Spi_8353_Vars->Ctrl_Reg_02.PWM_MODE)      | \
                   (Spi_8353_Vars->Ctrl_Reg_02.OTW_REP << 7)  | \
                   (Spi_8353_Vars->Ctrl_Reg_02.DIS_GDF << 8)  | \
                   (Spi_8353_Vars->Ctrl_Reg_02.DIS_CPUV <<9)  | \
                   (Spi_8353_Vars->Ctrl_Reg_02.OCP_ACT << 10);
      DRV8353_writeSpi(handle, drvRegAddr, drvDataNew);

      // Write Control Register 3
      drvRegAddr = Address_Control_3;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_03.IDRIVEN_HS) | \
                   (Spi_8353_Vars->Ctrl_Reg_03.IDRIVEP_HS) | \
                   (Spi_8353_Vars->Ctrl_Reg_03.LOCK);
      DRV8353_writeSpi(handle, drvRegAddr, drvDataNew);

      // Write Control Register 4
      drvRegAddr = Address_Control_4;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_04.IDRIVEN_LS) | \
                   (Spi_8353_Vars->Ctrl_Reg_04.IDRIVEP_LS) | \
                   (Spi_8353_Vars->Ctrl_Reg_04.TDRIVE) | \
                   (Spi_8353_Vars->Ctrl_Reg_04.CBC << 10);
      DRV8353_writeSpi(handle, drvRegAddr, drvDataNew);

      // Write Control Register 5
      drvRegAddr = Address_Control_5;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_05.VDS_LVL)      | \
                   (Spi_8353_Vars->Ctrl_Reg_05.OCP_DEG)      | \
                   (Spi_8353_Vars->Ctrl_Reg_05.OCP_MODE)     | \
                   (Spi_8353_Vars->Ctrl_Reg_05.DEAD_TIME)    | \
                   (Spi_8353_Vars->Ctrl_Reg_05.TRETRY << 10);
      DRV8353_writeSpi(handle, drvRegAddr, drvDataNew);

      // Write Control Register 6
      drvRegAddr = Address_Control_6;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_06.SEN_LVL)        | \
                   (Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_C << 2) | \
                   (Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_B << 3) | \
                   (Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_A << 4) | \
                   (Spi_8353_Vars->Ctrl_Reg_06.DIS_SEN << 5)   | \
                   (Spi_8353_Vars->Ctrl_Reg_06.CSA_GAIN)       | \
                   (Spi_8353_Vars->Ctrl_Reg_06.LS_REF << 8)    | \
                   (Spi_8353_Vars->Ctrl_Reg_06.VREF_DIV << 9)  | \
                   (Spi_8353_Vars->Ctrl_Reg_06.CSA_FET << 10);
      DRV8353_writeSpi(handle, drvRegAddr, drvDataNew);

      // Write Control Register 7
      drvRegAddr = Address_Control_7;
      drvDataNew = (Spi_8353_Vars->Ctrl_Reg_07.CAL_MODE) << 0   | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_1 << 1) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_2 << 2) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_3 << 3) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_4 << 4) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_5 << 5) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_6 << 6) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_7 << 7) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_8 << 8) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_9 << 9) | \
                   (Spi_8353_Vars->Ctrl_Reg_07.RESERVED_10 << 10);
    DRV8353_writeSpi(handle, drvRegAddr, drvDataNew);

    Spi_8353_Vars->WriteCmd = false;
  }

  // Manual write to the DRV8353
  if (Spi_8353_Vars->ManWriteCmd) {
    // Custom Write
    drvRegAddr = (DRV8353_Address_e)(Spi_8353_Vars->ManWriteAddr << 11);
    drvDataNew = Spi_8353_Vars->ManWriteData;
    DRV8353_writeSpi(handle, drvRegAddr, drvDataNew);
    drvDataNew = 0;
    Spi_8353_Vars->ManWriteCmd = false;
  }

  return;
}  // end of DRV8353_writeData() function


void DRV8353_readData(DRV8353_Handle handle, DRV_SPI_8353_Vars_t *Spi_8353_Vars)
{
  DRV8353_Address_e drvRegAddr;
  uint16_t drvDataNew;

  if (Spi_8353_Vars->ReadCmd)
  {
    // Read registers for default values
    // Read Status Register 0
    drvRegAddr = Address_Status_0;
    drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
    Spi_8353_Vars->Stat_Reg_00.VDS_LC =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LC_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_00.VDS_HC  =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HC_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_00.VDS_LB  =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LB_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_00.VDS_HB  =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HB_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_00.VDS_LA   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_LA_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_00.VDS_HA  =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_HA_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_00.OTSD   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_OTSD_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_00.UVLO   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_UVLO_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_00.GDF  =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_GDF_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_00.VDS_OCP =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_VDS_OCP_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_00.FAULT =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS00_FAULT_BITS)?1:0;

    // Read Status Register 1
    drvRegAddr = Address_Status_1;
    drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
    Spi_8353_Vars->Stat_Reg_01.VGS_LC   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LC_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_01.VGS_HC   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HC_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_01.VGS_LB   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LB_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_01.VGS_HB =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HB_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_01.VGS_LA =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_LA_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_01.VGS_HA   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_VGS_HA_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_01.GDUV   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_GDUV_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_01.OTW   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_OTW_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_01.SC_OC   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SC_OC_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_01.SB_OC   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SB_OC_BITS)?1:0;
    Spi_8353_Vars->Stat_Reg_01.SA_OC   =
        (bool)(drvDataNew & (uint16_t)DRV8353_STATUS01_SA_OC_BITS)?1:0;

    // Read Control Register 2
    drvRegAddr = Address_Control_2;
    drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
    Spi_8353_Vars->Ctrl_Reg_02.CLR_FLT =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_CLR_FLT_BITS);
    Spi_8353_Vars->Ctrl_Reg_02.BRAKE =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_BRAKE_BITS);
    Spi_8353_Vars->Ctrl_Reg_02.COAST =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_COAST_BITS);
    Spi_8353_Vars->Ctrl_Reg_02.PWM1_DIR =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_PWM1_DIR_BITS);
    Spi_8353_Vars->Ctrl_Reg_02.PWM1_COM =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_PWM1_COM_BITS);
    Spi_8353_Vars->Ctrl_Reg_02.PWM_MODE =
        (DRV8353_CTRL02_PwmMode_e)(drvDataNew &
                                   (uint16_t)DRV8353_CTRL02_PWM_MODE_BITS);
    Spi_8353_Vars->Ctrl_Reg_02.OTW_REP =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_OTW_REP_BITS);
    Spi_8353_Vars->Ctrl_Reg_02.DIS_GDF =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_DIS_GDF_BITS);
    Spi_8353_Vars->Ctrl_Reg_02.DIS_CPUV =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_DIS_CPUV_BITS);
    Spi_8353_Vars->Ctrl_Reg_02.OCP_ACT =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL02_OCP_ACT)?1:0;

    // Read Control Register 3
    drvRegAddr = Address_Control_3;
    drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
    Spi_8353_Vars->Ctrl_Reg_03.IDRIVEN_HS =
        (DRV8353_CTRL03_PeakSinkCurHS_e)
        (drvDataNew & (uint16_t)DRV8353_CTRL03_IDRIVEN_HS_BITS);
    Spi_8353_Vars->Ctrl_Reg_03.IDRIVEP_HS =
        (DRV8353_CTRL03_PeakSourCurHS_e)
        (drvDataNew & (uint16_t)DRV8353_CTRL03_IDRIVEP_HS_BITS);
    Spi_8353_Vars->Ctrl_Reg_03.LOCK =
        (DRV8353_CTRL03_Lock_e)(drvDataNew &
                                (uint16_t)DRV8353_CTRL03_LOCK_BITS);

    // Read Control Register 4
    drvRegAddr = Address_Control_4;
    drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
    Spi_8353_Vars->Ctrl_Reg_04.IDRIVEN_LS =
        (DRV8353_CTRL04_PeakSinkCurLS_e)
        (drvDataNew & (uint16_t)DRV8353_CTRL04_IDRIVEN_LS_BITS);
    Spi_8353_Vars->Ctrl_Reg_04.IDRIVEP_LS =
        (DRV8353_CTRL04_PeakSourCurLS_e)
        (drvDataNew & (uint16_t)DRV8353_CTRL04_IDRIVEP_LS_BITS);
    Spi_8353_Vars->Ctrl_Reg_04.TDRIVE =
        (DRV8353_CTRL04_PeakTime_e)(drvDataNew &
                                    (uint16_t)DRV8353_CTRL04_TDRIVE_BITS);
    Spi_8353_Vars->Ctrl_Reg_04.CBC =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL04_CBC_BITS)?1:0;

    // Read Control Register 5
    drvRegAddr = Address_Control_5;
    drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
    Spi_8353_Vars->Ctrl_Reg_05.VDS_LVL =
        (DRV8353_CTRL05_VDSLVL_e)(drvDataNew &
                                  (uint16_t)DRV8353_CTRL05_VDS_LVL_BITS);
    Spi_8353_Vars->Ctrl_Reg_05.OCP_DEG =
        (DRV8353_CTRL05_OcpDeg_e)(drvDataNew &
                                  (uint16_t)DRV8353_CTRL05_OCP_DEG_BITS);
    Spi_8353_Vars->Ctrl_Reg_05.OCP_MODE =
        (DRV8353_CTRL05_OcpMode_e)(drvDataNew &
                                   (uint16_t)DRV8353_CTRL05_OCP_MODE_BITS);
    Spi_8353_Vars->Ctrl_Reg_05.DEAD_TIME =
        (DRV8353_CTRL05_DeadTime_e)(drvDataNew &
                                    (uint16_t)DRV8353_CTRL05_DEAD_TIME_BITS);
    Spi_8353_Vars->Ctrl_Reg_05.TRETRY =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL05_TRETRY_BITS);

    // Read Control Register 6
    drvRegAddr = Address_Control_6;
    drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
    Spi_8353_Vars->Ctrl_Reg_06.SEN_LVL =
        (DRV8353_CTRL06_SENLevel_e)(drvDataNew &
                                    (uint16_t)DRV8353_CTRL06_SEN_LVL_BITS);
    Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_C =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_C_BITS);
    Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_B =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_B_BITS);
    Spi_8353_Vars->Ctrl_Reg_06.CSA_CAL_A =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_CAL_A_BITS);
    Spi_8353_Vars->Ctrl_Reg_06.DIS_SEN =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_DIS_SEN_BITS);
    Spi_8353_Vars->Ctrl_Reg_06.CSA_GAIN =
        (DRV8353_CTRL06_CSAGain_e)(drvDataNew &
                                   (uint16_t)DRV8353_CTRL06_CSA_GAIN_BITS);
    Spi_8353_Vars->Ctrl_Reg_06.LS_REF =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_LS_REF_BITS);
    Spi_8353_Vars->Ctrl_Reg_06.VREF_DIV =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_VREF_DIV_BITS);
    Spi_8353_Vars->Ctrl_Reg_06.CSA_FET =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL06_CSA_FET_BITS);

    // Read Control Register 7
    drvRegAddr = Address_Control_7;
    drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
    Spi_8353_Vars->Ctrl_Reg_07.CAL_MODE =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_CAL_MODE_BITS);
    Spi_8353_Vars->Ctrl_Reg_07.RESERVED_1 =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED1_BITS);
    Spi_8353_Vars->Ctrl_Reg_07.RESERVED_2 =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED2_BITS);
    Spi_8353_Vars->Ctrl_Reg_07.RESERVED_3 =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED3_BITS);
    Spi_8353_Vars->Ctrl_Reg_07.RESERVED_4 =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED4_BITS);
    Spi_8353_Vars->Ctrl_Reg_07.RESERVED_5 =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED5_BITS);
    Spi_8353_Vars->Ctrl_Reg_07.RESERVED_6 =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED6_BITS);
    Spi_8353_Vars->Ctrl_Reg_07.RESERVED_7 =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED7_BITS);
    Spi_8353_Vars->Ctrl_Reg_07.RESERVED_8 =
        (bool)(drvDataNew & (uint16_t)DRV8335_CTRL07_RESERVED8_BITS);
    Spi_8353_Vars->Ctrl_Reg_07.RESERVED_9 =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED9_BITS);
    Spi_8353_Vars->Ctrl_Reg_07.RESERVED_10 =
        (bool)(drvDataNew & (uint16_t)DRV8353_CTRL07_RESERVED10_BITS);

    Spi_8353_Vars->ReadCmd = false;
  }

  // Manual read from the DRV8353
  if (Spi_8353_Vars->ManReadCmd) {
    // Custom Read
    drvRegAddr = (DRV8353_Address_e)(Spi_8353_Vars->ManReadAddr << 11);
    drvDataNew = DRV8353_readSpi(handle, drvRegAddr);
    Spi_8353_Vars->ManReadData = drvDataNew;
    Spi_8353_Vars->ManReadCmd = false;
  }

    return;
}  // end of DRV8353_readData() function

#endif
// end of file
