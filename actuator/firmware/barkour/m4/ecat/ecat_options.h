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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_ECAT_ECAT_OPTIONS_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_ECAT_ECAT_OPTIONS_H_

// SOES configuration file.
//
// This file is read by SOES/soes/options.h
// Consult this file to see a list of possible configuration options.
//
// Values not defined here are assigned defaults in that header and
// it may not be what you expect -- BEWARE!
//
// Note on addresses:
//
// Addresses here are in hexadecimal bytes and refer to SII header (0x00:0x7F)
// contents and its categories.
//
// These must be kept in sync!
//

#define USE_FOE 1  // SII:38 and Cat1E:06
#define USE_EOE 0  // SII Cat1E:07

// Standard mailbox configuration.
// See SII addr 0x30:0x39 and Category SyncManager (0x0029)
#define MBXSIZE 0x80                      // SII:32,SII:36
#define MBX0_sma 0x1000                   // Start addr   SII:30,Cat29:00
#define MBX0_sml MBXSIZE                  // Length       SII:32,Cat29:02
#define MBX0_sme MBX0_sma + MBX0_sml - 1  // Last byte
#define MBX0_smc 0x26                     // Control Byte        Cat29:04
#define MBX1_sma 0x1080                   // Start addr   SII:34,Cat29:08
#define MBX1_sml MBXSIZE                  // Length       SII:36,Cat29:0A
#define MBX1_sme MBX1_sma + MBX1_sml - 1  // Last byte
#define MBX1_smc 0x22                     // Control Byte        Cat29:0C

// Bootstrap mailbox configuration.
// See SII addr 0x28:0x2F.
#define MBXSIZEBOOT 0x200                       // SII:2A,SII:2E
#define MBX0_sma_b 0x1000                       // Start addr    SII:28
#define MBX0_sml_b MBXSIZEBOOT                  // Length        SII:2A
#define MBX0_sme_b MBX0_sma_b + MBX0_sml_b - 1  // Last byte
#define MBX0_smc_b 0x26                         // Control Byte
#define MBX1_sma_b 0x1200                       // Start addr    SII:2C
#define MBX1_sml_b MBXSIZEBOOT                  // Length        SII:2E
#define MBX1_sme_b MBX1_sma_b + MBX1_sml_b - 1  // Last byte
#define MBX1_smc_b 0x22                         // Control Byte

// SyncManager 2 and 3 configuration.
// See SII Category SyncManager 0x0029.
#define SM2_sma 0x1100  // Start addr     Cat29:10
#define SM2_smc 0x24    // Control Byte   Cat29:14
#define SM2_act 1
#define SM3_sma 0x1400  // Start addr     Cat29:18
#define SM3_smc 0x20    // Control Byte   Cat29:1C
#define SM3_act 1

// Maximum size of PDO.
//
// Note that every PDO requires three buffers in memory to exchange
// data with the master. This limits especially the size of RXPDO, assigned
// to SM2. This has to be smaller than (SM3_sma - SM2_sma)/3.
#define MAX_RXPDO_SIZE 256
#define MAX_TXPDO_SIZE 256

// Maximum number of PDO's that can be mapped into a SyncM.
#define MAX_MAPPINGS_SM2 32
#define MAX_MAPPINGS_SM3 32

// Miscellaneous dynamic IO variables.
#define MAX_M7BLOB_OBJECTS 128

#define MBXBUFFERS 5

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_ECAT_ECAT_OPTIONS_H_
