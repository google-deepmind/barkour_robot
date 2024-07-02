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

#ifndef _DRV8353_H_
#define _DRV8353_H_

//! \file   drivers/drvic/drv8353/src/32b/f28x/f2806x/drv8353.h
//! \brief  Contains public interface to various functions related
//!         to the DRV8305 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include <stdbool.h>

#include "stm32h7xx_hal.h"  // NOLINT

// Port
typedef SPI_HandleTypeDef *SPI_Handle;
typedef GPIO_TypeDef *GPIO_Handle;
typedef uint16_t GPIO_Number_e;

// **************************************************************************
// modules


// **************************************************************************
// solutions


//!
//! \defgroup DRV8353

//!
//! \ingroup DRV8353
//@{


#ifdef __cplusplus
extern "C" {
#endif
#define DRV8353_SPI
#ifdef DRV8353_SPI
extern uint16_t gDeviceVariant;
// **************************************************************************
// the defines

//! \brief Defines the address mask
//!
#define DRV8353_ADDR_MASK                   (0x7800)


//! \brief Defines the data mask
//!
#define DRV8353_DATA_MASK                   (0x07FF)


//! \brief Defines the R/W mask
//!
#define DRV8353_RW_MASK                     (0x8000)


//! \brief Defines the R/W mask
//!
#define DRV8353_FAULT_TYPE_MASK             (0x07FF)

#define DRV8353_STATUS00_VDS_LC_BITS        (1 << 0)
#define DRV8353_STATUS00_VDS_HC_BITS        (1 << 1)
#define DRV8353_STATUS00_VDS_LB_BITS        (1 << 2)
#define DRV8353_STATUS00_VDS_HB_BITS        (1 << 3)
#define DRV8353_STATUS00_VDS_LA_BITS        (1 << 4)
#define DRV8353_STATUS00_VDS_HA_BITS        (1 << 5)

//! \brief Defines the location of the OTSD (Over temperature shutdown)
// bits in the Status 1 register
//!
#define DRV8353_STATUS00_OTSD_BITS          (1 << 6)
#define DRV8353_STATUS00_UVLO_BITS          (1 << 7)
#define DRV8353_STATUS00_GDF_BITS           (1 << 8)
#define DRV8353_STATUS00_VDS_OCP_BITS       (1 << 9)
#define DRV8353_STATUS00_FAULT_BITS         (1 << 10)


//! \brief Defines the location of the VGS_LC bits in the Status 2 register
//!
#define DRV8353_STATUS01_VGS_LC_BITS     (1 << 0)

//! \brief Defines the location of the VGS_HC bits in the Status 2 register
//!
#define DRV8353_STATUS01_VGS_HC_BITS     (1 << 1)

//! \brief Defines the location of the VGS_LB bits in the Status 2 register
//!
#define DRV8353_STATUS01_VGS_LB_BITS     (1 << 2)

//! \brief Defines the location of the VGS_HB bits in the Status 2 register
//!
#define DRV8353_STATUS01_VGS_HB_BITS     (1 << 3)

//! \brief Defines the location of the VGS_LA bits in the Status 2 register
//!
#define DRV8353_STATUS01_VGS_LA_BITS     (1 << 4)

//! \brief Defines the location of the VGS_HA bits in the Status 2 register
//!
#define DRV8353_STATUS01_VGS_HA_BITS     (1 << 5)

//! \brief Defines the location of the CPUV (charge pump undervoltage)
// bits in the Status 2 register
//!
#define DRV8353_STATUS01_GDUV_BITS     (1 << 6)

//! \brief Defines the location of the OTW bits in the Status 2 register
//!
#define DRV8353_STATUS01_OTW_BITS     (1 << 7)

//! \brief Defines the location of the SC_OC bits in the Status 2 register
//!
#define DRV8353_STATUS01_SC_OC_BITS     (1 << 8)

//! \brief Defines the location of the SB_OC bits in the Status 2 register
//!
#define DRV8353_STATUS01_SB_OC_BITS     (1 << 9)

//! \brief Defines the location of the SA_OC bits in the Status 2 register
//!
#define DRV8353_STATUS01_SA_OC_BITS     (1 << 10)



//! \brief Defines the location of the CLR_FLT bits in the Control 2 register
//!
#define DRV8353_CTRL02_CLR_FLT_BITS       (1 << 0)

//! \brief Defines the location of the BRAKE bits in the Control 2 register
//!
#define DRV8353_CTRL02_BRAKE_BITS         (1 << 1)

//! \brief Defines the location of the COAST bits in the Control 2 register
//!
#define DRV8353_CTRL02_COAST_BITS         (1 << 2)

//! \brief Defines the location of the 1PWM_DIR bits in the Control 2 register
//!
#define DRV8353_CTRL02_PWM1_DIR_BITS       (1 << 3)

//! \brief Defines the location of the 1PWM_COM bits in the Control 2 register
//!
#define DRV8353_CTRL02_PWM1_COM_BITS       (1 << 4)

//! \brief Defines the location of the PWM_MODE bits in the Control 2 register
//!
#define DRV8353_CTRL02_PWM_MODE_BITS       (3 << 5)

//! \brief Defines the location of the OTW_REP bits in the Control 2 register
//!
#define DRV8353_CTRL02_OTW_REP_BITS        (1 << 7)

//! \brief Defines the location of the DIS_GDF bits in the Control 2 register
//!
#define DRV8353_CTRL02_DIS_GDF_BITS        (1 << 8)

//! \brief Defines the location of the DIS_CPUV bits in the Control 2 register
//!
#define DRV8353_CTRL02_DIS_CPUV_BITS       (1 << 9)

//! \brief Defines the location of the RESERVED1 bits in the Control 2 register
//!
#define DRV8353_CTRL02_OCP_ACT       (1 << 10)


//! \brief Defines the location of the IDRIVEN_HS bits in the Control 3 register
//!
#define DRV8353_CTRL03_IDRIVEN_HS_BITS      (15 << 0)

//! \brief Defines the location of the IDRIVEP_HS bits in the Control 3 register
//!
#define DRV8353_CTRL03_IDRIVEP_HS_BITS      (15 << 4)

//! \brief Defines the location of the LOCK bits in the Control 3 register
//!
#define DRV8353_CTRL03_LOCK_BITS            (7 << 8)

//! \brief Defines the location of the IDRIVEN_LS bits in the Control 4 register
//!
#define DRV8353_CTRL04_IDRIVEN_LS_BITS      (15 << 0)

//! \brief Defines the location of the IDRIVEP_LS bits in the Control 4 register
//!
#define DRV8353_CTRL04_IDRIVEP_LS_BITS      (15 << 4)

//! \brief Defines the location of the TDRIVE bits in the Control 4 register
//!
#define DRV8353_CTRL04_TDRIVE_BITS          (3 << 8)

//! \brief Defines the location of the CBC bits in the Control 4 register
//!
#define DRV8353_CTRL04_CBC_BITS             (1 << 10)


//! \brief Defines the location of the VDS_LVL bits in the Control 5 register
//!
#define DRV8353_CTRL05_VDS_LVL_BITS         (15 << 0)

//! \brief Defines the location of the OCP_DEG bits in the Control 5 register
//!
#define DRV8353_CTRL05_OCP_DEG_BITS         (3 << 4)

//! \brief Defines the location of the OCP_MODE bits in the Control 5 register
//!
#define DRV8353_CTRL05_OCP_MODE_BITS        (3 << 6)

//! \brief Defines the location of the DEAD_TIME bits in the Control 5 register
//!
#define DRV8353_CTRL05_DEAD_TIME_BITS      (3 << 8)

//! \brief Defines the location of the TRETRY bits in the Control 5 register
//!
#define DRV8353_CTRL05_TRETRY_BITS         (1 << 10)


//! \brief Defines the location of the SEN_LVL bits in the Control 6 register
//!
#define DRV8353_CTRL06_SEN_LVL_BITS         (3 << 0)

//! \brief Defines the location of the CSA_CAL_C bits in the Control 6 register
//!
#define DRV8353_CTRL06_CSA_CAL_C_BITS       (1 << 2)

//! \brief Defines the location of the CSA_CAL_B bits in the Control 6 register
//!
#define DRV8353_CTRL06_CSA_CAL_B_BITS       (1 << 3)

//! \brief Defines the location of the CSA_CAL_A bits in the Control 6 register
//!
#define DRV8353_CTRL06_CSA_CAL_A_BITS       (1 << 4)

//! \brief Defines the location of the DIS_SEN bits in the Control 6 register
//!
#define DRV8353_CTRL06_DIS_SEN_BITS         (1 << 5)

//! \brief Defines the location of the CSA_GAIN bits in the Control 6 register
//!
#define DRV8353_CTRL06_CSA_GAIN_BITS        (3 << 6)

//! \brief Defines the location of the LS_REF bits in the Control 6 register
//!
#define DRV8353_CTRL06_LS_REF_BITS          (1 << 8)

//! \brief Defines the location of the VREF_DIV bits in the Control 6 register
//!
#define DRV8353_CTRL06_VREF_DIV_BITS        (1 << 9)

//! \brief Defines the location of the CSA_FET bits in the Control 6 register
//!
#define DRV8353_CTRL06_CSA_FET_BITS         (1 << 10)

//! \brief Defines the location of the CAL MODE bits in the Control 7 register
//!
#define DRV8353_CTRL07_CAL_MODE_BITS        (1 << 0)

//! \brief Defines the location of the RESERVED1 bits in the Control 7 register
//!
#define DRV8353_CTRL07_RESERVED1_BITS       (1 << 1)

//! \brief Defines the location of the RESERVED1 bits in the Control 7 register
//!
#define DRV8353_CTRL07_RESERVED2_BITS       (1 << 2)

//! \brief Defines the location of the RESERVED3 bits in the Control 7 register
//!
#define DRV8353_CTRL07_RESERVED3_BITS       (1 << 3)

//! \brief Defines the location of the RESERVED4 bits in the Control 7 register
//!
#define DRV8353_CTRL07_RESERVED4_BITS       (1 << 4)

//! \brief Defines the location of the RESERVED5 bits in the Control 7 register
//!
#define DRV8353_CTRL07_RESERVED5_BITS       (1 << 5)

//! \brief Defines the location of the RESERVED6 bits in the Control 7 register
//!
#define DRV8353_CTRL07_RESERVED6_BITS       (1 << 6)

//! \brief Defines the location of the RESERVED7 bits in the Control 7 register
//!
#define DRV8353_CTRL07_RESERVED7_BITS       (1 << 7)

//! \brief Defines the location of the RESERVED8 bits in the Control 7 register
//!
#define DRV8335_CTRL07_RESERVED8_BITS       (1 << 8)

//! \brief Defines the location of the RESERVED9 bits in the Control 7 register
//!
#define DRV8353_CTRL07_RESERVED9_BITS       (1 << 9)

//! \brief Defines the location of the RESERVED3 bits in the Control 7 register
//!
#define DRV8353_CTRL07_RESERVED10_BITS       (1 << 10)




// **************************************************************************
// the typedefs

//! \brief Enumeration for the R/W modes
//!
typedef enum
{
  CtrlMode_Read = 1 << 15,   //!< Read Mode
  CtrlMode_Write = 0 << 15   //!< Write Mode
} DRV8353_CtrlMode_e;


//! \brief Enumeration for the Status 0 register, faults
//!
typedef enum
{
  VDS_LC      = (1 << 0),    //!< VDS overcurrent fault on C low-side MOSFET
  VDS_HC      = (1 << 1),    //!< VDS overcurrent fault on C high-side MOSFET
  VDS_LB      = (1 << 2),    //!< VDS overcurrent fault on B low-side MOSFET
  VDS_HB      = (1 << 3),    //!< VDS overcurrent fault on B high-side MOSFET
  VDS_LA      = (1 << 4),    //!< VDS overcurrent fault on A low-side MOSFET
  VDS_HA      = (1 << 5),    //!< VDS overcurrent fault on A high-side MOSFET
  OTSD        = (1 << 6),    //!< Overtemperature shutdown
  UVLO        = (1 << 7),    //!< Undervoltage lockout fault condition
  GDF         = (1 << 8),    //!< Gate driver fault condition
  VDS_OCP     = (1 << 9),    //!< VDS monitor overcurrent fault condition
  FAULT       = (1 << 10)    //!< FAULT type, 0-Warning, 1-Latched
} DRV8353_STATUS00_WarningWatchdog_e;


//! \brief Enumeration for the Status 1 register, OV/VDS faults
//!
typedef enum
{
  VGS_LC      = (1 << 0),    //!< VGS gate drive fault on C low-side MOSFET
  VGS_HC      = (1 << 1),    //!< VGS gate drive fault on C high-side MOSFET
  VGS_LB      = (1 << 2),    //!< VGS gate drive fault on B low-side MOSFET
  VGS_HB      = (1 << 3),    //!< VGS gate drive fault on B high-side MOSFET
  VGS_LA      = (1 << 4),    //!< VGS gate drive fault on A low-side MOSFET
  VGS_HA      = (1 << 5),    //!< VGS gate drive fault on A high-side MOSFET
  GDUV        = (1 << 6),    //!< charge pump undervoltage fault
  OTW         = (1 << 7),    //!< overtemperature warning
  SC_OC       = (1 << 8),    //!< overcurrent on phase C
  SB_OC       = (1 << 9),    //!< overcurrent on phase B
  SA_OC       = (1 << 10)    //!< overcurrent on phase A
} DRV8353_STATUS01_OvVdsFaults_e;


//! \brief Enumeration for the driver PWM mode
//!
typedef enum
{
  PwmMode_6 = (0 << 5),     //!< PWM_MODE = 6 inputs
  PwmMode_3 = (1 << 5),     //!< PWM_MODE = 3 inputs
  PwmMode_1 = (2 << 5),      //!< PWM_MODE = 1 input
  PwmMode_Ind = (3 << 5)      //!< PWM_MODE = 1 input
} DRV8353_CTRL02_PwmMode_e;


//! \brief Enumeration for the high side gate drive peak source current;
typedef enum
{
  ISour_HS_0p050_A = (0 << 4),  //!< IDRIVEP_HS = 0.050A
  ISour_HS_0p05_A = (1 << 4),  //!< IDRIVEP_HS = 0.050A
  ISour_HS_0p100_A = (2 << 4),  //!< IDRIVEP_HS = 0.100A
  ISour_HS_0p150_A = (3 << 4),  //!< IDRIVEP_HS = 0.150A
  ISour_HS_0p300_A = (4 << 4),  //!< IDRIVEP_HS = 0.300A
  ISour_HS_0p350_A = (5 << 4),  //!< IDRIVEP_HS = 0.350A
  ISour_HS_0p400_A = (6 << 4),  //!< IDRIVEP_HS = 0.400A
  ISour_HS_0p450_A = (7 << 4),  //!< IDRIVEP_HS = 0.450A
  ISour_HS_0p550_A = (8 << 4),  //!< IDRIVEP_HS = 0.550A
  ISour_HS_0p600_A = (9 << 4),  //!< IDRIVEP_HS = 0.600A
  ISour_HS_0p650_A = (10 << 4),  //!< IDRIVEP_HS = 0.650A
  ISour_HS_1p700_A = (11 << 4),  //!< IDRIVEP_HS = 0.700A
  ISour_HS_0p850_A = (12 << 4),  //!< IDRIVEP_HS = 0.850A
  ISour_HS_0p900_A = (13 << 4),  //!< IDRIVEP_HS = 0.900A
  ISour_HS_0p950_A = (14 << 4),  //!< IDRIVEP_HS = 0.950A
  ISour_HS_1p000_A = (15 << 4)  //!< IDRIVEP_HS = 1.000A
} DRV8353_CTRL03_PeakSourCurHS_e;


//! \brief Enumeration for the high side gate drive peak sink current;
typedef enum
{
  ISink_HS_0p100_A = (0 << 0),  //!< IDRIVEN_HS = 0.100A
  ISink_HS_0p10_A = (1 << 0),  //!< IDRIVEN_HS = 0.100A
  ISink_HS_0p200_A = (2 << 0),  //!< IDRIVEN_HS = 0.200A
  ISink_HS_0p300_A = (3 << 0),  //!< IDRIVEN_HS = 0.300A
  ISink_HS_0p600_A = (4 << 0),  //!< IDRIVEN_HS = 0.600A
  ISink_HS_0p700_A = (5 << 0),  //!< IDRIVEN_HS = 0.700A
  ISink_HS_0p800_A = (6 << 0),  //!< IDRIVEN_HS = 0.800A
  ISink_HS_0p900_A = (7 << 0),  //!< IDRIVEN_HS = 0.900A
  ISink_HS_0p1100_A = (8 << 0),  //!< IDRIVEN_HS = 01.100A
  ISink_HS_0p1200_A = (9 << 0),  //!< IDRIVEN_HS = 01.200A
  ISink_HS_0p1300_A = (10 << 0),  //!< IDRIVEN_HS = 01.300A
  ISink_HS_0p1400_A = (11 << 0),  //!< IDRIVEN_HS = 01.400A
  ISink_HS_0p1700_A = (12 << 0),  //!< IDRIVEN_HS = 01.700A
  ISink_HS_0p1800_A = (13 << 0),  //!< IDRIVEN_HS = 01.800A
  ISink_HS_0p1900_A = (14 << 0),  //!< IDRIVEN_HS = 01.900A
  ISink_HS_0p2000_A = (15 << 0)  //!< IDRIVEN_HS = 02.00A
} DRV8353_CTRL03_PeakSinkCurHS_e;


//! \brief Enumeration for the high side and low side gate drive peak
// source time;
//!
typedef enum
{
  Lock_lock     = (6 << 8),     //!< Lock settings
  Lock_unlock   = (3 << 8)      //!< Unlock settings
} DRV8353_CTRL03_Lock_e;


//! \brief Enumeration for the high side and low side gate drive peak source
// time;
//!
typedef enum
{
  TSour_500_ns  = (0 << 8),     //!< TDRIVEN = 500ns
  TSour_1000_ns  = (1 << 8),     //!< TDRIVEN = 1000ns
  TSour_2000_ns = (2 << 8),     //!< TDRIVEN = 2000ns
  TSour_4000_ns = (3 << 8)      //!< TDRIVEN = 4000ns
} DRV8353_CTRL04_PeakTime_e;


//! \brief Enumeration for the low side gate drive peak source current;
//!
typedef enum {
  ISour_LS_0p050_A = (0 << 4),   //!< IDRIVEP_LS = 0.050A
  ISour_LS_0p05_A = (1 << 4),    //!< IDRIVEP_LS = 0.050A
  ISour_LS_0p100_A = (2 << 4),   //!< IDRIVEP_LS = 0.100A
  ISour_LS_0p150_A = (3 << 4),   //!< IDRIVEP_LS = 0.150A
  ISour_LS_0p300_A = (4 << 4),   //!< IDRIVEP_LS = 0.300A
  ISour_LS_0p350_A = (5 << 4),   //!< IDRIVEP_LS = 0.350A
  ISour_LS_0p400_A = (6 << 4),   //!< IDRIVEP_LS = 0.400A
  ISour_LS_0p450_A = (7 << 4),   //!< IDRIVEP_LS = 0.450A
  ISour_LS_0p550_A = (8 << 4),   //!< IDRIVEP_LS = 0.550A
  ISour_LS_0p600_A = (9 << 4),   //!< IDRIVEP_LS = 0.600A
  ISour_LS_0p650_A = (10 << 4),  //!< IDRIVEP_LS = 0.650A
  ISour_LS_0p700_A = (11 << 4),  //!< IDRIVEP_LS = 0.700A
  ISour_LS_0p850_A = (12 << 4),  //!< IDRIVEP_LS = 0.850A
  ISour_LS_0p900_A = (13 << 4),  //!< IDRIVEP_LS = 0.900A
  ISour_LS_0p950_A = (14 << 4),  //!< IDRIVEP_LS = 0.950A
  ISour_LS_1p000_A = (15 << 4)   //!< IDRIVEP_LS = 1.000A
} DRV8353_CTRL04_PeakSourCurLS_e;

//! \brief Enumeration for the low side gate drive peak sink current;
//!
typedef enum {
  ISink_LS_0p100_A = (0 << 0),   //!< IDRIVEN_LS = 0.100A
  ISink_LS_0p10_A = (1 << 0),    //!< IDRIVEN_LS = 0.100A
  ISink_LS_0p200_A = (2 << 0),   //!< IDRIVEN_LS = 0.200A
  ISink_LS_0p300_A = (3 << 0),   //!< IDRIVEN_LS = 0.300A
  ISink_LS_0p600_A = (4 << 0),   //!< IDRIVEN_LS = 0.600A
  ISink_LS_0p700_A = (5 << 0),   //!< IDRIVEN_LS = 0.700A
  ISink_LS_0p800_A = (6 << 0),   //!< IDRIVEN_LS = 0.800A
  ISink_LS_0p900_A = (7 << 0),   //!< IDRIVEN_LS = 0.900A
  ISink_LS_1p100_A = (8 << 0),   //!< IDRIVEN_LS = 01.100A
  ISink_LS_1p200_A = (9 << 0),   //!< IDRIVEN_LS = 01.200A
  ISink_LS_1p300_A = (10 << 0),  //!< IDRIVEN_LS = 01.300A
  ISink_LS_1p400_A = (11 << 0),  //!< IDRIVEN_LS = 01.400A
  ISink_LS_1p700_A = (12 << 0),  //!< IDRIVEN_LS = 01.700A
  ISink_LS_1p800_A = (13 << 0),  //!< IDRIVEN_LS = 01.800A
  ISink_LS_1p900_A = (14 << 0),  //!< IDRIVEN_LS = 01.900A
  ISink_LS_2p000_A = (15 << 0)   //!< IDRIVEN_LS = 02.00A
} DRV8353_CTRL04_PeakSinkCurLS_e;

//! \brief Enumeration for the VDS comparator threshold
//!
typedef enum {
  VDS_Level_0p060_V = (0 << 0),   //!< VDS_LEVEL = 0.060V
  VDS_Level_0p070_V = (1 << 0),   //!< VDS_LEVEL = 0.070V
  VDS_Level_0p080_V = (2 << 0),   //!< VDS_LEVEL = 0.080V
  VDS_Level_0p090_V = (3 << 0),   //!< VDS_LEVEL = 0.090V
  VDS_Level_0p100_V = (4 << 0),   //!< VDS_LEVEL = 0.100V
  VDS_Level_0p200_V = (5 << 0),   //!< VDS_LEVEL = 0.200V
  VDS_Level_0p300_V = (6 << 0),   //!< VDS_LEVEL = 0.300V
  VDS_Level_0p400_V = (7 << 0),   //!< VDS_LEVEL = 0.400V
  VDS_Level_0p500_V = (8 << 0),   //!< VDS_LEVEL = 0.500V
  VDS_Level_0p600_V = (9 << 0),   //!< VDS_LEVEL = 0.600V
  VDS_Level_0p700_V = (10 << 0),  //!< VDS_LEVEL = 0.700V
  VDS_Level_0p800_V = (11 << 0),  //!< VDS_LEVEL = 0.800V
  VDS_Level_0p900_V = (12 << 0),  //!< VDS_LEVEL = 0.900V
  VDS_Level_1p000_V = (13 << 0),  //!< VDS_LEVEL = 1.0000V
  VDS_Level_1p500_V = (14 << 0),  //!< VDS_LEVEL = 1.500V
  VDS_Level_2p000_V = (15 << 0)   //!< VDS_LEVEL = 2.000V
} DRV8353_CTRL05_VDSLVL_e;

//! \brief Enumeration for the OCP/VDS sense deglitch time;
//!
typedef enum
{
  VDSDeg_0_us = (0 << 4),       //!< TVDS = 0us
  VDSDeg_2_us = (1 << 4),       //!< TVDS = 2us
  VDSDeg_4_us = (2 << 4),       //!< TVDS = 4us
  VDSDeg_8_us = (3 << 4)        //!< TVDS = 8us
} DRV8353_CTRL05_OcpDeg_e;


//! \brief Enumeration for the OCP report mode
//!
typedef enum
{
  Latched_Shutdown = (0 << 6),  //!< OCP_MODE = Latched fault
  Automatic_Retry = (1 << 6),   //!< OCP_MODE = Automatic Retry
  Report_Only  = (2 << 6),      //!< OCP_MODE = Report only
  Disable_OCP = (3 << 6)        //!< OCP_MODE = Disabled
} DRV8353_CTRL05_OcpMode_e;


//! \brief Enumeration for the driver dead time
//!
typedef enum
{
  DeadTime_50_ns = (0 << 8),    //!< DEAD_TIME = 50ns
  DeadTime_100_ns = (1 << 8),   //!< DEAD_TIME = 100ns
  DeadTime_200_ns = (2 << 8),   //!< DEAD_TIME = 200ns
  DeadTime_400_ns = (3 << 8)    //!< DEAD_TIME = 400ns
} DRV8353_CTRL05_DeadTime_e;


//! \brief Enumeration for the Sense OCP level
//!
typedef enum
{
  SEN_Lvl_Ocp_0p25 = (0 << 0),  //!< SEN_LVL = 0.25V
  SEN_Lvl_Ocp_0p50 = (1 << 0),  //!< SEN_LVL = 0.50V
  SEN_Lvl_Ocp_0p75 = (2 << 0),  //!< SEN_LVL = 0.75V
  SEN_Lvl_Ocp_1p00 = (3 << 0)   //!< SEN_LVL = 1.00V
} DRV8353_CTRL06_SENLevel_e;


//! \brief Enumeration for the gain of shunt amplifier
//!
typedef enum
{
  Gain_5VpV =  (0 << 6),   //!< GAIN_CSA = 5V/V
  Gain_10VpV = (1 << 6),   //!< GAIN_CSA = 10V/V
  Gain_20VpV = (2 << 6),   //!< GAIN_CSA = 20V/V
  Gain_40VpV = (3 << 6)    //!< GAIN_CSA = 40V/V
} DRV8353_CTRL06_CSAGain_e;


//! \brief Enumeration for the register addresses
//!
typedef enum
{
  Address_Status_0  = 0 << 11,   //!< Status Register 0
  Address_Status_1  = 1 << 11,   //!< Status Register 1
  Address_Control_2 = 2 << 11,   //!< Control Register 2
  Address_Control_3 = 3 << 11,   //!< Control Register 3
  Address_Control_4 = 4 << 11,   //!< Control Register 4
  Address_Control_5 = 5 << 11,   //!< Control Register 5
  Address_Control_6 = 6 << 11,    //!< Control Register 6
  Address_Control_7 = 7 << 11    //!< Control Register 7
} DRV8353_Address_e;


//! \brief Object for the DRV8353 STATUS00 register
//!
typedef struct _DRV_SPI_8353_Stat00_t_
{
  bool                  VDS_LC;         // Bits 0
  bool                  VDS_HC;         // Bits 1
  bool                  VDS_LB;         // Bits 2
  bool                  VDS_HB;         // Bits 3
  bool                  VDS_LA;         // Bits 4
  bool                  VDS_HA;         // Bits 5
  bool                  OTSD;           // Bits 6
  bool                  UVLO;           // Bits 7
  bool                  GDF;            // Bits 8
  bool                  VDS_OCP;        // Bits 9
  bool                  FAULT;          // Bits 10
}DRV_SPI_8353_Stat00_t_;


//! \brief Object for the DRV8353 STATUS01 register
//!
typedef struct _DRV_SPI_8353_Stat01_t_
{
  bool                  VGS_LC;         // Bits 0
  bool                  VGS_HC;         // Bits 1
  bool                  VGS_LB;         // Bits 2
  bool                  VGS_HB;         // Bits 3
  bool                  VGS_LA;         // Bits 4
  bool                  VGS_HA;         // Bits 5
  bool                  GDUV;           // Bits 6
  bool                  OTW;            // Bits 7
  bool                  SC_OC;          // Bits 8
  bool                  SB_OC;          // Bits 9
  bool                  SA_OC;          // Bits 10
}DRV_SPI_8353_Stat01_t_;


//! \brief Object for the DRV8353 CTRL02 register
//!
typedef struct _DRV_SPI_8353_Ctrl02_t_
{
  bool                          CLR_FLT;        // Bits 0
  bool                          BRAKE;          // Bits 1
  bool                          COAST;          // Bits 2
  bool                          PWM1_DIR;       // Bits 3
  bool                          PWM1_COM;       // Bits 4
  DRV8353_CTRL02_PwmMode_e      PWM_MODE;       // Bits 6-5
  bool                          OTW_REP;        // Bits 7
  bool                          DIS_GDF;        // Bits 8
  bool                          DIS_CPUV;       // Bits 9
  bool                          OCP_ACT;    // Bits 10
}DRV_SPI_8353_Ctrl02_t_;


//! \brief Object for the DRV8353 CTRL03 register
//!
typedef struct _DRV_SPI_8353_Ctrl03_t_
{
  DRV8353_CTRL03_PeakSinkCurHS_e    IDRIVEN_HS;     // Bits 3-0
  DRV8353_CTRL03_PeakSourCurHS_e    IDRIVEP_HS;     // Bits 7-4
  DRV8353_CTRL03_Lock_e             LOCK;           // Bits 10-8
}DRV_SPI_8353_Ctrl03_t_;


//! \brief Object for the DRV8353 CTRL04 register
//!
typedef struct _DRV_SPI_8353_Ctrl04_t_
{
  DRV8353_CTRL04_PeakSinkCurLS_e    IDRIVEN_LS;     // Bits 3-0
  DRV8353_CTRL04_PeakSourCurLS_e    IDRIVEP_LS;     // Bits 7-4
  DRV8353_CTRL04_PeakTime_e         TDRIVE;         // Bits 9-8
  bool                              CBC;            // Bits 10
}DRV_SPI_8353_Ctrl04_t_;


//! \brief Object for the DRV8353 CTRL05 register
//!
typedef struct _DRV_SPI_8353_Ctrl05_t_
{
  DRV8353_CTRL05_VDSLVL_e           VDS_LVL;        // Bits 3-0
  DRV8353_CTRL05_OcpDeg_e           OCP_DEG;        // Bits 5-4
  DRV8353_CTRL05_OcpMode_e          OCP_MODE;       // Bits 7-5
  DRV8353_CTRL05_DeadTime_e         DEAD_TIME;      // Bits 9-8
  bool                              TRETRY;         // Bits 10
}DRV_SPI_8353_Ctrl05_t_;


//! \brief Object for the DRV8353 CTRL06 register
//!
typedef struct _DRV_SPI_8353_Ctrl06_t_
{
  DRV8353_CTRL06_SENLevel_e SEN_LVL;  // Bits 1-0
  bool CSA_CAL_C;  // Bits 2
  bool CSA_CAL_B;  // Bits 3
  bool CSA_CAL_A;  // Bits 4
  bool DIS_SEN;  // Bits 5
  DRV8353_CTRL06_CSAGain_e CSA_GAIN;  // Bits 7-6
  bool LS_REF;  // Bits 8
  bool VREF_DIV;  // Bits 9
  bool CSA_FET;  // Bits 10
}DRV_SPI_8353_Ctrl06_t_;
//! \brief Object for the DRV8353 CTRL06 register
//!
typedef struct _DRV_SPI_8353_Ctrl07_t_
{
  bool CAL_MODE;  // Bits 0
  bool RESERVED_1;  // Bit 1
  bool RESERVED_2;  // Bit 2
  bool RESERVED_3;  // Bit 3
  bool RESERVED_4;  // Bit 4
  bool RESERVED_5;  // Bit 5
  bool RESERVED_6;  // Bit 6
  bool RESERVED_7;  // Bit 7
  bool RESERVED_8;  // Bit 8
  bool RESERVED_9;  // Bit 9
  bool RESERVED_10;  // Bit 10
}DRV_SPI_8353_Ctrl07_t_;




//! \brief Object for the DRV8353 registers and commands
//!
typedef struct _DRV_SPI_8353_Vars_t_
{
  DRV_SPI_8353_Stat00_t_    Stat_Reg_00;
  DRV_SPI_8353_Stat01_t_    Stat_Reg_01;

  DRV_SPI_8353_Ctrl02_t_    Ctrl_Reg_02;
  DRV_SPI_8353_Ctrl03_t_    Ctrl_Reg_03;
  DRV_SPI_8353_Ctrl04_t_    Ctrl_Reg_04;
  DRV_SPI_8353_Ctrl05_t_    Ctrl_Reg_05;
  DRV_SPI_8353_Ctrl06_t_    Ctrl_Reg_06;
  DRV_SPI_8353_Ctrl07_t_    Ctrl_Reg_07;
  bool                      WriteCmd;
  bool                      ReadCmd;

  uint16_t                  ManWriteAddr;
  uint16_t                  ManReadAddr;
  uint16_t                  ManWriteData;
  uint16_t                  ManReadData;
  bool                      ManWriteCmd;
  bool                      ManReadCmd;
}DRV_SPI_8353_Vars_t;

#endif
#define DRV8353RS 1
#define DRV8353RH 0

//! \brief Defines the DRV8353 object
//!
typedef struct _DRV8353_Obj_
{
  SPI_Handle spiHandle;      //!< the handle for the serial peripheral interface
  GPIO_Handle enGpioHandle;  //!< the gpio handle that is connected to the
                             //!< drv8353 enable pin
  GPIO_Number_e enGpioNumber;   //!< the gpio number that is connected to the
                                //!< drv8353 enable pin
  GPIO_Handle nCSGpioHandle;    //!< the gpio handle that is connected to the
                                //!< drv8353 nCS pin
  GPIO_Number_e nCSGpioNumber;  //!< the gpio number that is connected to the
                                //!< drv8353 nCS pin
  bool RxTimeOut;               //!< the timeout flag for the RX fifo
  bool enableTimeOut;           //!< the timeout flag for drv8353 enable
} DRV8353_Obj;


//! \brief Defines the DRV8353 handle
//!
typedef struct _DRV8353_Obj_ *DRV8353_Handle;


//! \brief Defines the DRV8353 Word type
//!
typedef  uint16_t    DRV8353_Word_t;


// **************************************************************************
// the globals



// **************************************************************************
// the function prototypes

//! \brief     Initializes the DRV8353 object
//! \param[in] pMemory A pointer to the memory for the DRV8353 object
//! \param[in] numBytes The number of bytes allocated for the DRV8353 object,
//!                     bytes
//! \return    The DRV8305 object handle
extern DRV8353_Handle DRV8353_init(void *pMemory, const size_t numBytes);
#ifdef DRV8353_SPI

//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV8353_Word_t DRV8353_buildCtrlWord(
    const DRV8353_CtrlMode_e ctrlMode, const DRV8353_Address_e regAddr,
    const uint16_t data)
{
  DRV8353_Word_t ctrlWord = ctrlMode | regAddr | (data & DRV8353_DATA_MASK);

  return(ctrlWord);
}  // end of DRV8353_buildCtrlWord() function

#endif
//! \brief     Enables the DRV8305
//! \param[in] handle     The DRV8305 handle
extern void DRV8353_enable(DRV8353_Handle handle);

#ifdef DRV8353_SPI

//! \brief     Sets the SPI handle in the DRV8353
//! \param[in] handle     The DRV8353 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8353_setSpiHandle(DRV8353_Handle handle, SPI_Handle spiHandle);
#endif

//! \brief     Sets the GPIO handle in the DRV8353
//! \param[in] handle       The DRV8353 handle
//! \param[in] gpioHandle   The GPIO handle to use
void DRV8353_setGpioHandle(DRV8353_Handle handle, GPIO_Handle gpioHandle);


//! \brief     Sets the GPIO number in the DRV8353
//! \param[in] handle       The DRV8353 handle
//! \param[in] gpioHandle   The GPIO number to use
void DRV8353_setGpioNumber(DRV8353_Handle handle, GPIO_Number_e gpioNumber);


//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8353 handle
static inline void DRV8353_resetEnableTimeout(DRV8353_Handle handle)
{
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;

  obj->enableTimeOut = false;

  return;
}  // end of DRV8353_resetEnableTimeout() function


//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8353 handle
static inline void DRV8353_resetRxTimeout(DRV8353_Handle handle)
{
  DRV8353_Obj *obj = (DRV8353_Obj *)handle;

  obj->RxTimeOut = false;

  return;
}  // end of DRV8353_resetRxTimeout() function

#ifdef DRV8353_SPI
//! \brief     Initialize the interface to all 8353 SPI variables
//! \param[in] handle  The DRV8353 handle
extern void DRV8353_setupSpi(DRV8353_Handle handle,
                             DRV_SPI_8353_Vars_t *Spi_8353_Vars);


//! \brief     Reads data from the DRV8353 register
//! \param[in] handle   The DRV8353 handle
//! \param[in] regAddr  The register address
//! \return    The data value
extern uint16_t DRV8353_readSpi(DRV8353_Handle handle,
                                const DRV8353_Address_e regAddr);


//! \brief     Writes data to the DRV8353 register
//! \param[in] handle   The DRV8353 handle
//! \param[in] regAddr  The register name
//! \param[in] data     The data value
extern void DRV8353_writeSpi(DRV8353_Handle handle,
                             const DRV8353_Address_e regAddr,
                             const uint16_t data);


//! \brief     Write to the DRV8353 SPI registers
//! \param[in] handle  The DRV8353 handle
//! \param[in] Spi_8353_Vars  The (DRV_SPI_8353_Vars_t) structure that contains
//!                           all DRV8353 Status/Control register options
extern void DRV8353_writeData(DRV8353_Handle handle,
                             DRV_SPI_8353_Vars_t *Spi_8353_Vars);

//! \brief     Read from the DRV8353 SPI registers
//! \param[in] handle  The DRV8353 handle
//! \param[in] Spi_8353_Vars  The (DRV_SPI_8353_Vars_t) structure that contains
//!                           all DRV8353 Status/Control register options
extern void DRV8353_readData(DRV8353_Handle handle,
DRV_SPI_8353_Vars_t *Spi_8353_Vars);
#endif

#ifdef __cplusplus
}
#endif  // extern "C"

//@}  // ingroup

#endif  // end of _DRV8305_H_ definition
