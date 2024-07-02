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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_CC_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_CC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <assert.h>
#include <stdint.h>
#include <stddef.h>

#ifndef MIN
#define MIN(a, b) (((a) < (b))?(a):(b))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b))?(a):(b))
#endif

#define CC_PACKED_BEGIN
#define CC_PACKED_END
#define CC_PACKED       __attribute__((packed))
#define CC_ALIGNED(n)   __attribute__((aligned (n)))

#define CC_ASSERT(exp) assert (exp)
#define CC_STATIC_ASSERT(exp) _Static_assert (exp, "")

#define CC_DEPRECATED   __attribute__((deprecated))

#define CC_SWAP32(x) __builtin_bswap32 (x)
#define CC_SWAP16(x) __builtin_bswap16 (x)

#define CC_ATOMIC_SET(var, val) __atomic_store_n(&var, val, __ATOMIC_SEQ_CST)
#define CC_ATOMIC_GET(var) __atomic_load_n(&var, __ATOMIC_SEQ_CST)
#define CC_ATOMIC_ADD(var, val) \
    __atomic_add_fetch(&var, val, __ATOMIC_SEQ_CST)
#define CC_ATOMIC_SUB(var, val) \
__atomic_sub_fetch(&var, val, __ATOMIC_SEQ_CST)
#define CC_ATOMIC_AND(var, val) \
__atomic_and_fetch(&var, val, __ATOMIC_SEQ_CST)
#define CC_ATOMIC_OR(var, val) \
__atomic_or_fetch(&var, val, __ATOMIC_SEQ_CST)

// little-endian functions only
// will not work with big-endian architectures
#define htoes(x) (x)
#define htoel(x) (x)
#define etohs(x) htoes (x)
#define etohl(x) htoel (x)
#define EC_LITTLE_ENDIAN


#define DPRINT(...)

#ifdef __cplusplus
}
#endif

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_M4_CC_H_
