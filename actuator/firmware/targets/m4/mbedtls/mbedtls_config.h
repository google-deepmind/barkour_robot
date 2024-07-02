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

#ifndef BARKOUR_ROBOT_FIRMWARE_TARGETS_M4_MBEDTLS_CONFIG_H_
#define BARKOUR_ROBOT_FIRMWARE_TARGETS_M4_MBEDTLS_CONFIG_H_

#include <mbedtls/version.h>

#if MBEDTLS_VERSION_MAJOR >= 3
#include <mbedtls/build_info.h>
#include <mbedtls/mbedtls_config.h>
#else
#include <mbedtls/config.h>
#endif

// override some flags needed by pigweed
// No file system support.
#undef MBEDTLS_FS_IO
// No posix socket support
#undef MBEDTLS_NET_C
// This feature requires file system support.
#undef MBEDTLS_PSA_ITS_FILE_C
// The following two require MBEDTLS_PSA_ITS_FILE_C
#undef MBEDTLS_PSA_CRYPTO_C
#undef MBEDTLS_PSA_CRYPTO_STORAGE_C
// This feature only works on Unix/Windows
#undef MBEDTLS_TIMING_C
// Use a custom entropy generator
#define MBEDTLS_NO_PLATFORM_ENTROPY
// Error string support for debugging
#define MBEDTLS_ERROR_C
// This feature requires MBEDTLS_PSA_CRYPTO_C.
#undef MBEDTLS_LMS_C
// Enable SHA256 algorithms
#define MBEDTLS_SHA256_C

// Disable a number of TLS related features as we are not maintaining the TLS
// support.
#undef MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA_ENABLED
#undef MBEDTLS_KEY_EXCHANGE_ECDH_RSA_ENABLED
#undef MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED
#undef MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED
#undef MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED
#undef MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_EPHEMERAL_ENABLED
#undef MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_EPHEMERAL_ENABLED
#undef MBEDTLS_HAVE_TIME
#undef MBEDTLS_HAVE_TIME_DATE

#include "mbedtls/check_config.h"

#endif  // BARKOUR_ROBOT_FIRMWARE_TARGETS_M4_MBEDTLS_CONFIG_H_
