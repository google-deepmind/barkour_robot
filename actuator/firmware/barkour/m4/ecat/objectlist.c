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

// Data exchange definition file
//
// The data definitions contained in here is the primary souce
// of information between SOES and the application about the
// data that is exchanged between the application and EtherCAT
// master.
//
// Its primary goal is to define
//    const _objectlist SDOobjects[] = {...};
// at the end of the file. This list must be in global "C" namespace
// for SOES to find it.
//
// SDOobjects is a list of EtherCAT objects available on the bus.
//
// Devel note: The variable addresses point to members in Obj (declared
// in utypes.h and defined in ecat_device.cc). This is by no means required
// by SOES, it is just an implementation example. The variables can be
// elsewhere, just update the address information in the individual
//    static const _objd SDOxxxx[] = {.data = &my_variable}
// definitions.
//
// Note however, that this structure organization helps in maintaining data
// consistency between threads using mutexes, but if concurrent access is not
// expected, this organization can be broken.

#include "actuator/firmware/barkour/m4/ecat/utypes.h"

#include <stddef.h>

#include "soes/esc_coe.h"

#ifndef HW_REV
#define HW_REV "1.0"
#endif

static const char acNameMaxSubidx[] = "Max SubIndex";
static const char acNameNULL[] = "";

// See ETG.1000.6 par 5.6.7.4
// M = mandatory (either for EtherCAT or for CiA-402)

static const char acNameSDO1000[] = "Device type";
static const _objd SDO1000[] = {
    // M (ETG.1000.6, ETG.6010)
    // EtherCAT servo drive, no safety, CiA402
    // See ETG.6010 Table 84 (Object 1000)
    // Change to 0x000A0192 for FSoE
    // .15-.0     = 402
    // .23-.16    = 02 (servo), 0A (servo, FSoE)
    {0x00, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameSDO1000, 0x00020192, NULL},
};

static const char acNameSDO1001[] = "Error register";
static const _objd SDO1001[] = {
    // ETG.1000.6 par5.6.7.4.2
    // .0       generic error
    // .1       current error
    // .2       voltage error
    // .3       temperature error
    // .4       communication error
    // .5       device profile specific error
    // .6
    // .7       manufacturer specific error
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameSDO1001, 0, &Obj.tx.error},
};

static const char acNameSDO1002[] = "Manufacturer status register";
static const _objd SDO1002[] = {
    {0x00, DTYPE_UNSIGNED32, 32, ATYPE_RO | ATYPE_TXPDO, acNameSDO1002, 0,
     &Obj.tx.manufacturer_status},
};
static const char acNameSDO1008[] = "Device name";
static const _objd SDO1008[] = {
    {0x00, DTYPE_VISIBLE_STRING, 0, ATYPE_RO, acNameSDO1008, 0,
     Obj.SDO1008.name},
};
static const char acNameSDO1009[] = "Hardware version";
static const _objd SDO1009[] = {
    {0x00, DTYPE_VISIBLE_STRING, 24, ATYPE_RO, acNameSDO1009, 0, HW_REV},
};

// Software version is the git commit.
static const char acNameSDO100A[] = "Software version";
static const _objd SDO100A[] = {
    {0x00, DTYPE_VISIBLE_STRING, 320, ATYPE_RO, acNameSDO100A, 0,
      &Obj.SDO100A.git_commit},
};

static const char acNameSDO1018[] = "Identity object";
static const _objd SDO1018[] = {
    // M (ETG.1000.6, ETG.6010)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 4, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Vendor ID", 0,
     &Obj.SDO1018.vendor_id},
    {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Product code", 0,
     &Obj.SDO1018.product_code},
    {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Revision", 0,
     &Obj.SDO1018.revision_no},
    {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Serial number", 0,
     &Obj.parameter.serial_number},
};
static const char acNameSDO1C00[] = "Sync Manager type";
static const _objd SDO1C00[] = {
    // M (ETG.1000.6 par5.6.7.4.9)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 4, NULL},
    {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, "SM0 (Mbx Rx)", 1, NULL},
    {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, "SM1 (Mbx Tx)", 2, NULL},
    {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, "SM2 (RxPDO)", 3, NULL},
    {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, "SM3 (TxPDO)", 4, NULL},
};
static const char acNameSDO1C12[] = "RxPDO assign";
// IMPORTANT: The total size of the TxPDO or RxPDO should not exceed 256
// bytes (see ecat_options.h)! Exceeding this limit will result in
// subtle and confusing errors (e.g. b/234461322).
// You can double check this by running `ethercat pdos` and adding up the
// entries.
static const _objd SDO1C12[] = {
    // C (ETG.1000.6 par5.6.7.4.10)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RWpre, acNameMaxSubidx, 8,
     &Obj.SM1C12.maxsub},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1600,
     &Obj.SM1C12.value[0]},
    {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1602,
     &Obj.SM1C12.value[1]},
    {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1610,
     &Obj.SM1C12.value[2]},
    {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1620,
     &Obj.SM1C12.value[3]},
    {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SM1C12.value[4]},
    {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SM1C12.value[5]},
    {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SM1C12.value[6]},
    {0x08, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SM1C12.value[7]},
};
static const char acNameSDO1C13[] = "TxPDO assign";
// IMPORTANT: The total size of the TxPDO or RxPDO should not exceed 256
// bytes (see ecat_options.h)! Exceeding this limit will result in
// subtle and confusing errors (e.g. b/234461322).
// You can double check this by running `ethercat pdos` and adding up the
// entries.
static const _objd SDO1C13[] = {
    // C (ETG.1000.6 par5.6.7.4.10)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RWpre, acNameMaxSubidx, 27,
     &Obj.SM1C13.maxsub},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A00,
     &Obj.SM1C13.value[0]},
    {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A01,
     &Obj.SM1C13.value[1]},
    {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A02,
     &Obj.SM1C13.value[2]},
    {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A03,
     &Obj.SM1C13.value[3]},
    {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A10,
     &Obj.SM1C13.value[4]},
    {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A20,
     &Obj.SM1C13.value[5]},
    // kBusVoltageObjectInfo
    {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A21,
     &Obj.SM1C13.value[6]},
    // kPhaseACurrentObjectInfo
    {0x08, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A22,
     &Obj.SM1C13.value[7]},
    // kPhaseBCurrentObjectInfo
    {0x09, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A23,
     &Obj.SM1C13.value[8]},
    // kPhaseCCurrentObjectInfo
    {0x0a, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A24,
     &Obj.SM1C13.value[9]},
    // kThermistor1ObjectInfo
    {0x0b, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A25,
     &Obj.SM1C13.value[10]},
    // kThermistor2ObjectInfo
    {0x0c, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A26,
     &Obj.SM1C13.value[11]},
    // kThermistor3ObjectInfo
    {0x0d, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A27,
     &Obj.SM1C13.value[12]},
    // kThermistor4ObjectInfo
    {0x0e, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A28,
     &Obj.SM1C13.value[13]},
    // kQuadratureCurrentObjectInfo
    {0x0f, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A29,
     &Obj.SM1C13.value[14]},
    // kDirectCurrentObjectInfo
    {0x10, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A2A,
     &Obj.SM1C13.value[15]},
    // kShaftAngleObjectInfo
    {0x11, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A2B,
     &Obj.SM1C13.value[16]},
    // kElectricalAngleObjectInfo
    {0x12, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A2C,
     &Obj.SM1C13.value[17]},
    // kQuadratureVoltageObjectInfo
    {0x13, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A2D,
     &Obj.SM1C13.value[18]},
    // kDirectVoltageObjectInfo
    {0x14, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A2E,
     &Obj.SM1C13.value[19]},
    // kPhaseADutyCycleObjectInfo
    {0x15, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A2F,
     &Obj.SM1C13.value[20]},
    // kPhaseBDutyCycleObjectInfo
    {0x16, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A30,
     &Obj.SM1C13.value[21]},
    // kPhaseCDutyCycleObjectInfo
    {0x17, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A31,
     &Obj.SM1C13.value[22]},
    // kLinearAccelerationXObjectInfo
    {0x18, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A32,
     &Obj.SM1C13.value[23]},
    // kLinearAccelerationYObjectInfo
    {0x19, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A33,
     &Obj.SM1C13.value[24]},
    // kLinearAccelerationZObjectInfo
    {0x1A, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A34,
     &Obj.SM1C13.value[25]},
    // Manufacturer status register
    {0x1B, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acNameNULL, 0x1A35,
     &Obj.SM1C13.value[26]},
};
static const char acNameSDO1C32[] = "SM output (Rx) parameter";
static const _objd SDO1C32[] = {
    // O (ETG.1000.6, ETG.1020 par21.1.2)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 32, NULL},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, "Sync mode", 0,
     &Obj.SyncMgrParam.Sync_mode},
    {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, "Cycle time", 0,
     &Obj.SyncMgrParam.CycleTime},
    {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Shift time", 0,
     &Obj.SyncMgrParam.ShiftTime},
    {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, "Sync modes supported", 0,
     &Obj.SyncMgrParam.Sync_modes_supported},
    {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Minimum cycle time", 125000,
     &Obj.SyncMgrParam.Minimum_Cycle_Time},
    {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Calc and copy time", 0,
     &Obj.SyncMgrParam.Calc_and_Copy_Time},
    {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Minimum delay time", 0,
     &Obj.SyncMgrParam.Minimum_Delay_Time},
    {0x08, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, "Command", 0,
     &Obj.SyncMgrParam.GetCycleTime},
    {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Maximum delay time", 0,
     &Obj.SyncMgrParam.DelayTime},
    {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Sync0CycleTime", 0,
     &Obj.SyncMgrParam.Sync0CycleTime},
    {0x0B, DTYPE_UNSIGNED16, 16, ATYPE_RO, "SM event missed counter", 0,
     &Obj.SyncMgrParam.SMEventMissedCnt},
    {0x0C, DTYPE_UNSIGNED16, 16, ATYPE_RO, "Cycle exceeded counter", 0,
     &Obj.SyncMgrParam.CycleTimeTooSmallCnt},
    {0x0D, DTYPE_UNSIGNED16, 16, ATYPE_RO, "Shift too short counter", 0,
     &Obj.SyncMgrParam.Shift_too_short_counter},
    {0x0E, DTYPE_UNSIGNED16, 16, ATYPE_RO, "RxPDOToggleFailed", 0,
     &Obj.SyncMgrParam.RxPDOToggleFailed},
    {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Minimum Cycle Distance", 0,
     &Obj.SyncMgrParam.Minimum_Cycle_Distance},
    {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Maximum Cycle Distance", 0,
     &Obj.SyncMgrParam.Maximum_Cycle_Distance},
    {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Minimum SM Sync Distance", 0,
     &Obj.SyncMgrParam.Minimum_SM_Sync_Distance},
    {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Maximum SM Sync Distance", 0,
     &Obj.SyncMgrParam.Maximum_SM_Sync_Distance},
    {0x13, 0, 0, 0, acNameNULL, 0, NULL},
    {0x14, 0, 0, 0, acNameNULL, 0, NULL},
    {0x15, 0, 0, 0, acNameNULL, 0, NULL},
    {0x16, 0, 0, 0, acNameNULL, 0, NULL},
    {0x17, 0, 0, 0, acNameNULL, 0, NULL},
    {0x18, 0, 0, 0, acNameNULL, 0, NULL},
    {0x19, 0, 0, 0, acNameNULL, 0, NULL},
    {0x1A, 0, 0, 0, acNameNULL, 0, NULL},
    {0x1B, 0, 0, 0, acNameNULL, 0, NULL},
    {0x1C, 0, 0, 0, acNameNULL, 0, NULL},
    {0x1D, 0, 0, 0, acNameNULL, 0, NULL},
    {0x1E, 0, 0, 0, acNameNULL, 0, NULL},
    {0x1F, 0, 0, 0, acNameNULL, 0, NULL},
    {0x20, DTYPE_BOOLEAN, 1, ATYPE_RO, "SyncError", 0,
     &Obj.SyncMgrParam.SyncError},
};

// Tx/Rx blobs shared with M7
static const char acNameSDO2010[] = "Rx M7 blob";
static const _objd SDO2010[] = {
    // =>#x1620
    {0x00, DTYPE_UNSIGNED8, 8 * sizeof(Obj.rx.blob), ATYPE_RW | ATYPE_RXPDO,
     acNameSDO2010, 0, Obj.rx.blob},
};
static const char acNameSDO2011[] = "Tx M7 blob";
static const _objd SDO2011[] = {
    // =>#x1A20
    {0x00, DTYPE_UNSIGNED8, 8 * sizeof(Obj.tx.blob), ATYPE_RO | ATYPE_TXPDO,
     acNameSDO2011, 0, Obj.tx.blob},
};

// Barkour custom objects.

static const char acNameSDO3000[] = "Electrical angle calibration";
static const _objd SDO3000[] = {
    // M (ETG.1000.6, ETG.6010)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 2, NULL},
    {0x01, DTYPE_INTEGER32, 32, ATYPE_RO, "Electrical angle offset", 0,
     &Obj.SDO3000.electrical_angle_offset},
    {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO,
     "Electrical angle calibration completed", 0,
     &Obj.SDO3000.electrical_angle_calibration_completed},
};

static const char acNameSDO3001[] = "Joint angle calibration";
static const _objd SDO3001[] = {
    // M (ETG.1000.6, ETG.6010)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 2, NULL},
    {0x01, DTYPE_INTEGER32, 32, ATYPE_RO, "Joint angle offset", 0,
     &Obj.SDO3001.joint_angle_offset},
    {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Joint angle calibration completed", 0,
     &Obj.SDO3001.joint_angle_calibration_completed},
};

static const char acNameSDO3010[] = "Extended serial sumber";
static const _objd SDO3010[] = {
    {0x00, DTYPE_UNSIGNED64, 64, ATYPE_RO, acNameSDO3010, 0,
     &Obj.parameter.extended_serial_number},
};

// Torque constant in mNm/A
static const char acNameSDO3020[] = "Torque constant (mNm/A)";
static const _objd SDO3020[] = {
    {0x00, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameSDO3020, 0,
     &Obj.parameter.torque_constant},
};

// Number of pole pairs
static const char acNameSDO3021[] = "Number of pole pairs";
static const _objd SDO3021[] = {
    {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RO, acNameSDO3021, 0,
     &Obj.parameter.num_pole_pairs},
};

// Phase to phase resistance (mOhm)
static const char acNameSDO3022[] = "Phase to phase resistance (mOhm)";
static const _objd SDO3022[] = {
    {0x00, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameSDO3022, 0,
     &Obj.parameter.phase_to_phase_resistance},
};

// Order of phases: 0 == ABC, 1 == ACB.
// Other values currently not supported.
static const char acNameSDO3023[] = "Phase order";
static const _objd SDO3023[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameSDO3023, 0,
     &Obj.parameter.phase_order},
};

// Write 0x01234567 to this register to reset the STM32.
// Write 0xDEADC0DE to this register to trigger a bank swap,
// and reset the STM32. Make sure to write the new flash binary
// via FoE first.
static const char acNameSDO3030[] = "Bank swap / reset";
static const _objd SDO3030[] = {
    {0x00, DTYPE_UNSIGNED32, 32, ATYPE_WO, acNameSDO3030, 0,
     &Obj.foe.bank_swap_reset},
};

static const char acNameSDO3040[] = "Bus voltage [V]";
static const _objd SDO3040[] = {
    {0x00, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acNameSDO3040, 0,
     &Obj.tx.bus_voltage},
};

static const char acNameSDO3041[] = "Phase currents [A]";
static const _objd SDO3041[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 3, NULL},
    {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Phase A current", 0,
     &Obj.tx.phase_a_current},
    {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Phase B current", 0,
     &Obj.tx.phase_b_current},
    {0x03, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Phase C current", 0,
     &Obj.tx.phase_c_current},
};

static const char acNameSDO3042[] = "Thermistors [C]";
static const _objd SDO3042[] = {
    {0x00, DTYPE_REAL32, 32, ATYPE_RO, acNameMaxSubidx, 4, NULL},
    {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Thermistor 1", 0,
     &Obj.tx.thermistor_1},
    {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Thermistor 2", 0,
     &Obj.tx.thermistor_2},
    {0x03, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Thermistor 3", 0,
     &Obj.tx.thermistor_3},
    {0x04, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Thermistor 4", 0,
     &Obj.tx.thermistor_4},
};

static const char acNameSDO3043[] =
    "Rotating frame currents (quadrature & direct) [A]";
static const _objd SDO3043[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 2, NULL},
    {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Quadrature current", 0,
     &Obj.tx.quadrature_current},
    {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Direct current", 0,
     &Obj.tx.direct_current},
};

static const char acNameSDO3044[] =
    "Encoder data (shaft angle & electrical angle) [rad]";
static const _objd SDO3044[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 2, NULL},
    {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Shaft angle", 0,
     &Obj.tx.shaft_angle},
    {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Electrical angle", 0,
     &Obj.tx.electrical_angle},
};

static const char acNameSDO3045[] =
    "Rotating frame voltage commands (quadrature & direct) [V]";
static const _objd SDO3045[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 2, NULL},
    {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Quadrature voltage", 0,
     &Obj.tx.quadrature_voltage},
    {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Direct voltage", 0,
     &Obj.tx.direct_voltage},
};

static const char acNameSDO3046[] = "PWM duty cycles [0-1]";
static const _objd SDO3046[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 3, NULL},
    {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Phase A duty cycle", 0,
     &Obj.tx.phase_a_duty_cycle},
    {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Phase B duty cycle", 0,
     &Obj.tx.phase_b_duty_cycle},
    {0x03, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Phase C duty cycle", 0,
     &Obj.tx.phase_c_duty_cycle},
};

static const char acNameSDO3047[] = "STO Monitoring";
static const _objd SDO3047[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 3, NULL},
    {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, "FB_STO1 status", 0,
     &Obj.tx.fb_sto1_state},
    {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, "FB_STO2 status", 0,
     &Obj.tx.fb_sto2_state},
    {0x03, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "24V_SAFE_ADC voltage", 0,
     &Obj.tx.sto_24v_safe_adc_voltage},
};

static const char acNameSDO3048[] = "Gate driver fault status";
static const _objd SDO3048[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 2, NULL},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO,
     "Gate driver fault status register 1", 0,
     &Obj.tx.gate_driver_fault_status_1},
    {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO,
      "Gate driver fault status register 2", 0,
      &Obj.tx.gate_driver_fault_status_2},
};

static const char acNameSDO3050[] = "Accelerometer readings [m/s^2]";
static const _objd SDO3050[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 3, NULL},
    {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Linear acceleration X", 0,
     &Obj.tx.acceleration_x},
    {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Linear acceleration Y", 0,
     &Obj.tx.acceleration_y},
    {0x03, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, "Linear acceleration Z", 0,
     &Obj.tx.acceleration_z},
};

// End Barkour custom objects.

static const char acNameSDO6040[] = "Control word";
static const _objd SDO6040[] = {
    // =>#x1600; M (ETG.6010)
    {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_RXPDO, acNameSDO6040, 0,
     &Obj.rx.controlword},
};
static const char acNameSDO6041[] = "Status word";
static const _objd SDO6041[] = {
    // =>#x1A00; M (ETG.6010)
    {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acNameSDO6041, 0,
     &Obj.tx.statusword},
};
static const char acNameSDO6060[] = "Modes of operation";
static const _objd SDO6060[] = {
    // =>#x1610; M (ETG.6010)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_RXPDO, acNameSDO6060, 0,
     &Obj.rx.mode},
};
static const char acNameSDO6061[] = "Modes of operation display";
static const _objd SDO6061[] = {
    // =>#x1A10; M (ETG.6010)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acNameSDO6061, 0,
     &Obj.tx.mode},
};
static const char acNameSDO6064[] = "Position";
static const _objd SDO6064[] = {
    // =>#x1A01; C (ETG.6010, csp or csv)
    {0x00, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_TXPDO, acNameSDO6064, 0,
     &Obj.tx.position},
};
static const char acNameSDO606C[] = "Velocity";
static const _objd SDO606C[] = {
    // =>#x1A02; C (ETG.6010 recommended for csv)
    {0x00, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_TXPDO, acNameSDO606C, 0,
     &Obj.tx.velocity},
};
static const char acNameSDO6071[] = "Torque setpoint";
static const _objd SDO6071[] = {
    // =>#x1602; C (ETG.6010 cst)
    {0x00, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acNameSDO6071, 0,
     &Obj.rx.torque},
};
static const char acNameSDO6072[] = "Torque limit";
static const _objd SDO6072[] = {
    // =>#x1611; C (ETG.6010 mandatory for FG Torque Limiting)
    {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_RXPDO, acNameSDO6072, ~0U,
     &Obj.parameter.torque_limit},
};
static const char acNameSDO6073[] = "Current limit";
static const _objd SDO6073[] = {
    // =>#x1611; C (ETG.6010 mandatory for FG Torque Limiting)
    {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_RXPDO, acNameSDO6073, ~0U,
     &Obj.parameter.current_limit},
};
static const char acNameSDO6075[] = "Rated motor current";
static const _objd SDO6075[] = {
    {0x00, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameSDO6075, 1000,
     &Obj.parameter.rated_current},
};
static const char acNameSDO6076[] = "Rated motor torque";
static const _objd SDO6076[] = {
    {0x00, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameSDO6076, 1000,
     &Obj.parameter.rated_torque},
};
static const char acNameSDO6077[] = "Torque";
static const _objd SDO6077[] = {
    // =>#x1A03; C (ETG.6010 mandatory cst, recommended csv)
    {0x00, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acNameSDO6077, 0,
     &Obj.tx.torque},
};
static const char acNameSDO6078[] = "Current actual value";
static const _objd SDO6078[] = {
    // =>#x1A00; M (ETG.6010)
    {0x00, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acNameSDO6078, 0,
     &Obj.tx.motor_current},
};
static const char acNameSDO6080[] = "Speed limit";
static const _objd SDO6080[] = {
    // =>#x1614
    {0x00, DTYPE_UNSIGNED32, 32, ATYPE_RW | ATYPE_RXPDO, acNameSDO6080, ~0U,
     &Obj.rx.speed_limit},
};
static const char acNameSDO608F[] = "Position encoder resolution";
static const _objd SDO608F[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 2, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Encoder increments", 0,
     &Obj.parameter.encoder_increments},
    {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Motor revolutions", 0,
     &Obj.parameter.encoder_motor_revolutions},
};
static const char acNameSDO6091[] = "Gear ratio";
static const _objd SDO6091[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 2, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Motor revolutions", 0,
     &Obj.parameter.gear_motor_revolutions},
    {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Shaft revolutions", 0,
     &Obj.parameter.gear_shaft_revolutions},
};
static const char acNameSDO60B2[] = "Torque offset";
static const _objd SDO60B2[] = {
    // =>#x1615; O (ETG.6010 recommended csv)
    {0x00, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acNameSDO60B2, 0,
     &Obj.rx.torque_offset},
};
static const char acNameSDO60E0[] = "Max torque";
static const _objd SDO60E0[] = {
    // =>#x1612; C (ETG.6010 mandatory for FG Torque Limiting)
    {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_RXPDO, acNameSDO60E0, ~0U,
     &Obj.parameter.torque_limit},
};
static const char acNameSDO60E1[] = "Min torque";
static const _objd SDO60E1[] = {
    // =>#x1613; C (ETG.6010 mandatory for FG Torque Limiting)
    {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_RXPDO, acNameSDO60E1, ~0U,
     &Obj.parameter.torque_limit},
};
static const char acNameSDO60FF[] = "Velocity setpoint";
static const _objd SDO60FF[] = {
    // =>#x1601; C (ETG.6010 mandatory for csv)
    {0x00, DTYPE_INTEGER32, 32, ATYPE_RW | ATYPE_RXPDO, acNameSDO60FF, 0,
     &Obj.rx.velocity},
};
static const char acNameSDO6502[] = "Supported drive modes";
static const _objd SDO6502[] = {
    // =>#x; M (ETG.6010, par6.8.1)
    // .0       = pp
    // .1       = vl
    // .2       = pv
    // .3       = tq
    // .4       = r
    // .5       = hm
    // .6       = ip
    // .7       = csp
    // .8       = csv
    // .9       = cst
    // .10      = cstca
    // .16-.31  = manufacturer specific
    {0x00, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameSDO6502, 0x0300, NULL},
};

static const _objd SDO1600[] = {
    // C (ETG.1000.6 par5.6.7.4.7)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60400010, NULL},
};
static const _objd SDO1601[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60FF0020, NULL},
};
static const _objd SDO1602[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60710010, NULL},
};
static const _objd SDO1610[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60600008, NULL},
};
static const _objd SDO1611[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60720010, NULL},
};
static const _objd SDO1612[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60E00010, NULL},
};
static const _objd SDO1613[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60E10010, NULL},
};
static const _objd SDO1614[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60800020, NULL},
};
static const _objd SDO1615[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60B20010, NULL},
};
static const _objd SDO1620[] = {
    // See ETG.1020 par15.4 for arrays >255 bytes
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RWpre, acNameMaxSubidx, 5,
     &Obj.SDO1620.maxsub},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0x201000F0,
     &Obj.SDO1620.value[0]},
    {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0xF0,
     &Obj.SDO1620.value[1]},
    {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0xF0,
     &Obj.SDO1620.value[2]},
    {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0xF0,
     &Obj.SDO1620.value[3]},
    {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0x40,
     &Obj.SDO1620.value[4]},
    {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[5]},
    {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[6]},
    {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[7]},
    {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[8]},
    {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[9]},
    {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[10]},
    {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[11]},
    {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[12]},
    {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[13]},
    {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[14]},
    {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1620.value[15]},
};
static const _objd SDO1A00[] = {
    // C (ETG.1000.6 par5.6.7.4.7)
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60410010, NULL},
};
static const _objd SDO1A01[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60640020, NULL},
};
static const _objd SDO1A02[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x606C0020, NULL},
};
static const _objd SDO1A03[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60770010, NULL},
};
static const _objd SDO1A10[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x60610008, NULL},
};
static const _objd SDO1A20[] = {
    // See ETG.1020 par15.4
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RWpre, acNameMaxSubidx, 5,
     &Obj.SDO1A20.maxsub},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0x201100F0,
     &Obj.SDO1A20.value[0]},
    {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0xF0,
     &Obj.SDO1A20.value[1]},
    {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0xF0,
     &Obj.SDO1A20.value[2]},
    {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0xF0,
     &Obj.SDO1A20.value[3]},
    {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0x40,
     &Obj.SDO1A20.value[4]},
    {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[5]},
    {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[6]},
    {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[7]},
    {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[8]},
    {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[9]},
    {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[10]},
    {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[11]},
    {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[12]},
    {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[13]},
    {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[14]},
    {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RWpre, acNameNULL, 0,
     &Obj.SDO1A20.value[15]},
};

// kBusVoltageObjectInfo
static const _objd SDO1A21[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30400020, NULL},
};
// kPhaseACurrentObjectInfo
static const _objd SDO1A22[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30410120, NULL},
};
// kPhaseBCurrentObjectInfo
static const _objd SDO1A23[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30410220, NULL},
};
// kPhaseCCurrentObjectInfo
static const _objd SDO1A24[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30410320, NULL},
};
// kThermistor1ObjectInfo
static const _objd SDO1A25[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30420120, NULL},
};
// kThermistor2ObjectInfo
static const _objd SDO1A26[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30420220, NULL},
};
// kThermistor3ObjectInfo
static const _objd SDO1A27[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30420320, NULL},
};
// kThermistor4ObjectInfo
static const _objd SDO1A28[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30420420, NULL},
};
// kQuadratureCurrentObjectInfo
static const _objd SDO1A29[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30430120, NULL},
};
// kDirectCurrentObjectInfo
static const _objd SDO1A2A[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30430220, NULL},
};
// kShaftAngleObjectInfo
static const _objd SDO1A2B[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30440120, NULL},
};
// kElectricalAngleObjectInfo
static const _objd SDO1A2C[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30440220, NULL},
};
// kQuadratureVoltageObjectInfo
static const _objd SDO1A2D[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30450120, NULL},
};
// kDirectVoltageObjectInfo
static const _objd SDO1A2E[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30450220, NULL},
};
// kPhaseADutyCycleObjectInfo
static const _objd SDO1A2F[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30460120, NULL},
};
// kPhaseBDutyCycleObjectInfo
static const _objd SDO1A30[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30460220, NULL},
};
// kPhaseCDutyCycleObjectInfo
static const _objd SDO1A31[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30460320, NULL},
};
// kLinearAccelerationXObjectInfo
static const _objd SDO1A32[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30500120, NULL},
};
// kLinearAccelerationYObjectInfo
static const _objd SDO1A33[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30500220, NULL},
};
// kLinearAccelerationZObjectInfo
static const _objd SDO1A34[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x30500320, NULL},
};
// Manufacturer status register
static const _objd SDO1A35[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameMaxSubidx, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameNULL, 0x10020020, NULL},
};

static const char acNameTxPDO[] = "TxPDO";
static const char acNameRxPDO[] = "RxPDO";

// SDOobjects is a global object for SOES and must have this
// signature. It lists all EtherCAT objects available on the
// bus
const _objectlist SDOobjects[] = {
    {0x1000, OTYPE_VAR, 0, 0, acNameSDO1000, SDO1000},
    {0x1001, OTYPE_VAR, 0, 0, acNameSDO1001, SDO1001},
    {0x1002, OTYPE_VAR, 0, 0, acNameSDO1002, SDO1002},
    {0x1008, OTYPE_VAR, 0, 0, acNameSDO1008, SDO1008},
    {0x1009, OTYPE_VAR, 0, 0, acNameSDO1009, SDO1009},
    {0x100A, OTYPE_VAR, 0, 0, acNameSDO100A, SDO100A},
    {0x1018, OTYPE_RECORD, 4, 0, acNameSDO1018, SDO1018},
    {0x1600, OTYPE_VAR, 1, 0, acNameRxPDO, SDO1600},
    {0x1601, OTYPE_VAR, 1, 0, acNameRxPDO, SDO1601},
    {0x1602, OTYPE_VAR, 1, 0, acNameRxPDO, SDO1602},
    {0x1610, OTYPE_VAR, 1, 0, acNameRxPDO, SDO1610},
    {0x1611, OTYPE_VAR, 1, 0, acNameRxPDO, SDO1611},
    {0x1612, OTYPE_VAR, 1, 0, acNameRxPDO, SDO1612},
    {0x1613, OTYPE_VAR, 1, 0, acNameRxPDO, SDO1613},
    {0x1614, OTYPE_VAR, 1, 0, acNameRxPDO, SDO1614},
    {0x1615, OTYPE_VAR, 1, 0, acNameRxPDO, SDO1615},
    {0x1620, OTYPE_VAR, 16, 0, acNameRxPDO, SDO1620},
    {0x1A00, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A00},
    {0x1A01, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A01},
    {0x1A02, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A02},
    {0x1A03, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A03},
    {0x1A10, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A10},
    {0x1A20, OTYPE_VAR, 16, 0, acNameTxPDO, SDO1A20},
    {0x1A21, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A21},
    {0x1A22, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A22},
    {0x1A23, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A23},
    {0x1A24, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A24},
    {0x1A25, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A25},
    {0x1A26, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A26},
    {0x1A27, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A27},
    {0x1A28, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A28},
    {0x1A29, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A29},
    {0x1A2A, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A2A},
    {0x1A2B, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A2B},
    {0x1A2C, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A2C},
    {0x1A2D, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A2D},
    {0x1A2E, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A2E},
    {0x1A2F, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A2F},
    {0x1A30, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A30},
    {0x1A31, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A31},
    {0x1A32, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A32},
    {0x1A33, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A33},
    {0x1A34, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A34},
    {0x1A35, OTYPE_VAR, 1, 0, acNameTxPDO, SDO1A35},
    {0x1C00, OTYPE_ARRAY, 4, 0, acNameSDO1C00, SDO1C00},
    {0x1C12, OTYPE_VAR, SDO1C12[0].value, 0, acNameSDO1C12, SDO1C12},
    {0x1C13, OTYPE_VAR, SDO1C13[0].value, 0, acNameSDO1C13, SDO1C13},
    {0x1C32, OTYPE_RECORD, 32, 0, acNameSDO1C32, SDO1C32},
    {0x2010, OTYPE_VAR, 0, 0, acNameSDO2010, SDO2010},
    {0x2011, OTYPE_VAR, 0, 0, acNameSDO2011, SDO2011},
    {0x3000, OTYPE_RECORD, 2, 0, acNameSDO3000, SDO3000},
    {0x3001, OTYPE_RECORD, 2, 0, acNameSDO3001, SDO3001},
    {0x3010, OTYPE_VAR, 0, 0, acNameSDO3010, SDO3010},
    {0x3020, OTYPE_VAR, 0, 0, acNameSDO3020, SDO3020},
    {0x3021, OTYPE_VAR, 0, 0, acNameSDO3021, SDO3021},
    {0x3022, OTYPE_VAR, 0, 0, acNameSDO3022, SDO3022},
    {0x3023, OTYPE_VAR, 0, 0, acNameSDO3023, SDO3023},
    {0x3030, OTYPE_VAR, 0, 0, acNameSDO3030, SDO3030},
    {0x3040, OTYPE_VAR, 0, 0, acNameSDO3040, SDO3040},
    {0x3041, OTYPE_RECORD, 3, 0, acNameSDO3041, SDO3041},
    {0x3042, OTYPE_RECORD, 4, 0, acNameSDO3042, SDO3042},
    {0x3043, OTYPE_RECORD, 2, 0, acNameSDO3043, SDO3043},
    {0x3044, OTYPE_RECORD, 2, 0, acNameSDO3044, SDO3044},
    {0x3045, OTYPE_RECORD, 2, 0, acNameSDO3045, SDO3045},
    {0x3046, OTYPE_RECORD, 3, 0, acNameSDO3046, SDO3046},
    {0x3047, OTYPE_RECORD, 3, 0, acNameSDO3047, SDO3047},
    {0x3048, OTYPE_RECORD, 2, 0, acNameSDO3048, SDO3048},
    {0x3050, OTYPE_RECORD, 3, 0, acNameSDO3050, SDO3050},
    {0x6040, OTYPE_VAR, 0, 0, acNameSDO6040, SDO6040},
    {0x6041, OTYPE_VAR, 0, 0, acNameSDO6041, SDO6041},
    {0x6060, OTYPE_VAR, 0, 0, acNameSDO6060, SDO6060},
    {0x6061, OTYPE_VAR, 0, 0, acNameSDO6061, SDO6061},
    {0x6064, OTYPE_VAR, 0, 0, acNameSDO6064, SDO6064},
    {0x606C, OTYPE_VAR, 0, 0, acNameSDO606C, SDO606C},
    {0x6071, OTYPE_VAR, 0, 0, acNameSDO6071, SDO6071},
    {0x6072, OTYPE_VAR, 0, 0, acNameSDO6072, SDO6072},
    {0x6073, OTYPE_VAR, 0, 0, acNameSDO6073, SDO6073},
    {0x6075, OTYPE_VAR, 0, 0, acNameSDO6075, SDO6075},
    {0x6076, OTYPE_VAR, 0, 0, acNameSDO6076, SDO6076},
    {0x6077, OTYPE_VAR, 0, 0, acNameSDO6077, SDO6077},
    {0x6078, OTYPE_VAR, 0, 0, acNameSDO6078, SDO6078},
    {0x608F, OTYPE_RECORD, 2, 0, acNameSDO608F, SDO608F},
    {0x6091, OTYPE_RECORD, 2, 0, acNameSDO6091, SDO6091},
    {0x6080, OTYPE_VAR, 0, 0, acNameSDO6080, SDO6080},
    {0x60B2, OTYPE_VAR, 0, 0, acNameSDO60B2, SDO60B2},
    {0x60E0, OTYPE_VAR, 0, 0, acNameSDO60E0, SDO60E0},
    {0x60E1, OTYPE_VAR, 0, 0, acNameSDO60E1, SDO60E1},
    {0x60FF, OTYPE_VAR, 0, 0, acNameSDO60FF, SDO60FF},
    {0x6502, OTYPE_VAR, 0, 0, acNameSDO6502, SDO6502},
    {0xffff, 0xff, 0xff, 0xff, NULL, NULL}};

// clang-format on
