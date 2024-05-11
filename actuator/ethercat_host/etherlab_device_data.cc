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

#include "etherlab_device_data.h"

#include "ds301_objects.h"
#include "ds402_objects.h"
#include "custom_objects.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "etherlab/include/ecrt.h"

namespace barkour {

namespace {

// List of PDOs which are mapped into memory by the host. This is currently set
// up for torque control only.
//
// The Rx and Tx terminology is from the point of view of the devices, so an Rx
// PDO sends data from the host to the device, and a Tx PDO sends data from the
// device to the host.
ec_pdo_entry_info_t mutable_device_pdo_entries[] = {
    // Rx PDOs
    {
        kControlwordObjectInfo.address.index,
        kControlwordObjectInfo.address.subindex,
        8 * kControlwordObjectInfo.size(),
    },
    {
        kTorqueSetpointObjectInfo.address.index,
        kTorqueSetpointObjectInfo.address.subindex,
        8 * kTorqueObjectInfo.size(),
    },
    {
        kModesObjectInfo.address.index,
        kModesObjectInfo.address.subindex,
        8 * kModesObjectInfo.size(),
    },
    // The Rx and Tx PDOs are too large (128 bytes each ) to fit in a single PDO
    // entry, so they are split over multiple consecutive entries instead.
    {kRxPdoBlobAddress.index, kRxPdoBlobAddress.subindex, 240},
    {0x0000, 0x00, 240}, /* Gap */
    {0x0000, 0x00, 240}, /* Gap */
    {0x0000, 0x00, 240}, /* Gap */
    {0x0000, 0x00, 64},  /* Gap */

    // Tx PDOs
    {
        kStatuswordObjectInfo.address.index,
        kStatuswordObjectInfo.address.subindex,
        8 * kStatuswordObjectInfo.size(),
    },
    {
        kPositionObjectInfo.address.index,
        kPositionObjectInfo.address.subindex,
        8 * kPositionObjectInfo.size(),
    },
    {
        kVelocityObjectInfo.address.index,
        kVelocityObjectInfo.address.subindex,
        8 * kVelocityObjectInfo.size(),
    },
    {
        kTorqueObjectInfo.address.index,
        kTorqueObjectInfo.address.subindex,
        8 * kTorqueObjectInfo.size(),
    },
    {
        kModesDisplayObjectInfo.address.index,
        kModesDisplayObjectInfo.address.subindex,
        8 * kModesDisplayObjectInfo.size(),
    },

    {kTxPdoBlobAddress.index, kTxPdoBlobAddress.subindex, 240},
    {0x0000, 0x00, 240}, /* Gap */
    {0x0000, 0x00, 240}, /* Gap */
    {0x0000, 0x00, 240}, /* Gap */
    {0x0000, 0x00, 64},  /* Gap */

    {
        kBusVoltageObjectInfo.address.index,
        kBusVoltageObjectInfo.address.subindex,
        8 * kBusVoltageObjectInfo.size(),
    },
    {
        kPhaseACurrentObjectInfo.address.index,
        kPhaseACurrentObjectInfo.address.subindex,
        8 * kPhaseACurrentObjectInfo.size(),
    },
    {
        kPhaseBCurrentObjectInfo.address.index,
        kPhaseBCurrentObjectInfo.address.subindex,
        8 * kPhaseBCurrentObjectInfo.size(),
    },
    {
        kPhaseCCurrentObjectInfo.address.index,
        kPhaseCCurrentObjectInfo.address.subindex,
        8 * kPhaseCCurrentObjectInfo.size(),
    },
    {
        kThermistor1ObjectInfo.address.index,
        kThermistor1ObjectInfo.address.subindex,
        8 * kThermistor1ObjectInfo.size(),
    },
    {
        kThermistor2ObjectInfo.address.index,
        kThermistor2ObjectInfo.address.subindex,
        8 * kThermistor2ObjectInfo.size(),
    },
    {
        kThermistor3ObjectInfo.address.index,
        kThermistor3ObjectInfo.address.subindex,
        8 * kThermistor3ObjectInfo.size(),
    },
    {
        kThermistor4ObjectInfo.address.index,
        kThermistor4ObjectInfo.address.subindex,
        8 * kThermistor4ObjectInfo.size(),
    },
    {
        kQuadratureCurrentObjectInfo.address.index,
        kQuadratureCurrentObjectInfo.address.subindex,
        8 * kQuadratureCurrentObjectInfo.size(),
    },
    {
        kDirectCurrentObjectInfo.address.index,
        kDirectCurrentObjectInfo.address.subindex,
        8 * kDirectCurrentObjectInfo.size(),
    },
    {
        kShaftAngleObjectInfo.address.index,
        kShaftAngleObjectInfo.address.subindex,
        8 * kShaftAngleObjectInfo.size(),
    },
    {
        kElectricalAngleObjectInfo.address.index,
        kElectricalAngleObjectInfo.address.subindex,
        8 * kElectricalAngleObjectInfo.size(),
    },
    {
        kQuadratureVoltageObjectInfo.address.index,
        kQuadratureVoltageObjectInfo.address.subindex,
        8 * kQuadratureVoltageObjectInfo.size(),
    },
    {
        kDirectVoltageObjectInfo.address.index,
        kDirectVoltageObjectInfo.address.subindex,
        8 * kDirectVoltageObjectInfo.size(),
    },
    {
        kPhaseADutyCycleObjectInfo.address.index,
        kPhaseADutyCycleObjectInfo.address.subindex,
        8 * kPhaseADutyCycleObjectInfo.size(),
    },
    {
        kPhaseBDutyCycleObjectInfo.address.index,
        kPhaseBDutyCycleObjectInfo.address.subindex,
        8 * kPhaseBDutyCycleObjectInfo.size(),
    },
    {
        kPhaseCDutyCycleObjectInfo.address.index,
        kPhaseCDutyCycleObjectInfo.address.subindex,
        8 * kPhaseCDutyCycleObjectInfo.size(),
    },
    {
        kLinearAccelerationXObjectInfo.address.index,
        kLinearAccelerationXObjectInfo.address.subindex,
        8 * kLinearAccelerationXObjectInfo.size(),
    },
    {
        kLinearAccelerationYObjectInfo.address.index,
        kLinearAccelerationYObjectInfo.address.subindex,
        8 * kLinearAccelerationYObjectInfo.size(),
    },
    {
        kLinearAccelerationZObjectInfo.address.index,
        kLinearAccelerationZObjectInfo.address.subindex,
        8 * kLinearAccelerationZObjectInfo.size(),
    },
    {
        kManufacturerStatusRegisterObjectInfo.address.index,
        kManufacturerStatusRegisterObjectInfo.address.subindex,
        8 * kManufacturerStatusRegisterObjectInfo.size(),
    },
};

ec_pdo_info_t mutable_device_pdos[] = {
    // The first element in each struct is the PDO index, which is the index in
    // the object dictionary which defines the PDO configuration. The CANopen
    // standard defines that RxPDOs are listed starting from 0x1600, and the
    // TxPDOs from 0x1a00.
    //
    // It would be possible to group together some of these PDOs into a single
    // one, although this would require each *combination* to be defined and
    // configured in the device code, so we trade a small performance overhead
    // for the flexibility of selecting our own combinations of PDOs here.
    //
    // IMPORTANT: The total size of the TxPDO or RxPDO should not exceed 256
    // bytes (limit set in firmware/SII)! Exceeding this limit will result in
    // subtle and confusing errors.
    // You can double check this by running `ethercat pdos` and adding up the
    // entries.
    {0x1600, 1, &mutable_device_pdo_entries[0]},  /* RxPDO */
    {0x1602, 1, &mutable_device_pdo_entries[1]},  /* RxPDO */
    {0x1610, 1, &mutable_device_pdo_entries[2]},  /* RxPDO */
    {0x1620, 5, &mutable_device_pdo_entries[3]},  /* RxPDO */
    {0x1a00, 1, &mutable_device_pdo_entries[8]},  /* TxPDO */
    {0x1a01, 1, &mutable_device_pdo_entries[9]},  /* TxPDO */
    {0x1a02, 1, &mutable_device_pdo_entries[10]}, /* TxPDO */
    {0x1a03, 1, &mutable_device_pdo_entries[11]}, /* TxPDO */
    {0x1a10, 1, &mutable_device_pdo_entries[12]}, /* TxPDO */
    {0x1a20, 5, &mutable_device_pdo_entries[13]}, /* TxPDO */
    {0x1a21, 1, &mutable_device_pdo_entries[18]}, /* TxPDO */
    {0x1a22, 1, &mutable_device_pdo_entries[19]}, /* TxPDO */
    {0x1a23, 1, &mutable_device_pdo_entries[20]}, /* TxPDO */
    {0x1a24, 1, &mutable_device_pdo_entries[21]}, /* TxPDO */
    {0x1a25, 1, &mutable_device_pdo_entries[22]}, /* TxPDO */
    {0x1a26, 1, &mutable_device_pdo_entries[23]}, /* TxPDO */
    {0x1a27, 1, &mutable_device_pdo_entries[24]}, /* TxPDO */
    {0x1a28, 1, &mutable_device_pdo_entries[25]}, /* TxPDO */
    {0x1a29, 1, &mutable_device_pdo_entries[26]}, /* TxPDO */
    {0x1a2a, 1, &mutable_device_pdo_entries[27]}, /* TxPDO */
    {0x1a2b, 1, &mutable_device_pdo_entries[28]}, /* TxPDO */
    {0x1a2c, 1, &mutable_device_pdo_entries[29]}, /* TxPDO */
    {0x1a2d, 1, &mutable_device_pdo_entries[30]}, /* TxPDO */
    {0x1a2e, 1, &mutable_device_pdo_entries[31]}, /* TxPDO */
    {0x1a2f, 1, &mutable_device_pdo_entries[32]}, /* TxPDO */
    {0x1a30, 1, &mutable_device_pdo_entries[33]}, /* TxPDO */
    {0x1a31, 1, &mutable_device_pdo_entries[34]}, /* TxPDO */
    {0x1a32, 1, &mutable_device_pdo_entries[35]}, /* TxPDO */
    {0x1a33, 1, &mutable_device_pdo_entries[36]}, /* TxPDO */
    {0x1a34, 1, &mutable_device_pdo_entries[37]}, /* TxPDO */
    {0x1a35, 1, &mutable_device_pdo_entries[38]}, /* TxPDO */
};

const ec_sync_info_t device_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 4, &mutable_device_pdos[0], EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 27, &mutable_device_pdos[4], EC_WD_DISABLE},
    {0xff}};

}  // namespace

absl::Span<const ec_pdo_entry_info_t> kEtherlabDevicePdoEntries(
    mutable_device_pdo_entries);

absl::Span<const ec_pdo_info_t> kEtherlabDevicePdos(mutable_device_pdos);

absl::Span<const ec_sync_info_t> kEtherlabDeviceSyncs(device_syncs);

absl::StatusOr<int> GetEtherlabPdoByteLength(
    const CanObjectAddress& object_address) {
  if (object_address.index == 0 && object_address.subindex == 0) {
    return absl::InvalidArgumentError(
        "Cannot get EtherLab PDO byte length for the zero address.");
  }

  if (object_address == kRxPdoBlobAddress ||
      object_address == kTxPdoBlobAddress) {
    // The binary blob PDOs are made up of multiple entries due to their size,
    // so we manually return their sizes.
    return kBinaryBlobNumBytes;
  }

  for (ec_pdo_entry_info_t pdo_entry : kEtherlabDevicePdoEntries) {
    if (pdo_entry.index != object_address.index ||
        pdo_entry.subindex != object_address.subindex) {
      continue;
    }

    if (pdo_entry.bit_length % 8 != 0) {
      return absl::InternalError(absl::StrFormat(
          "Got a PDO bit length which is not divisible by 8 (bit "
          "length: %d) for object at address [%s].",
          pdo_entry.bit_length, object_address.ToString()));
    }

    return pdo_entry.bit_length / 8;
  }

  return absl::NotFoundError(
      absl::StrFormat("No PDO entry found with object address: [%s].",
                      object_address.ToString()));
}

}  // namespace barkour
