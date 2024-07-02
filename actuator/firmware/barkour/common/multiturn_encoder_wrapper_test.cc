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

#include "actuator/firmware/barkour/common/multiturn_encoder_wrapper.h"

#include <cstdint>

#include "actuator/firmware/barkour/drivers/testing/fake_rotary_encoder.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

TEST(TestMultiturnEncoderWrapper, GetCountsPerTurnPassedThrough) {
  uint32_t counts_per_turn = 1000;
  FakeRotaryEncoder fake_base_encoder(counts_per_turn);
  MultiturnEncoderWrapper wrapper(fake_base_encoder);

  EXPECT_EQ(wrapper.GetCountsPerTurn(), counts_per_turn);
}

TEST(TestMultiturnEncoderWrapper, ValuesInSameTurnDoNotUnwrap) {
  uint32_t counts_per_turn = 1000;

  FakeRotaryEncoder fake_base_encoder(counts_per_turn);
  MultiturnEncoderWrapper wrapper(fake_base_encoder);

  fake_base_encoder.SetRawEncoderCounts(400);
  pw::Result<RotaryEncoderState> encoder_state = wrapper.Update();
  ASSERT_TRUE(encoder_state.ok());

  encoder_state = wrapper.GetEncoderState();
  ASSERT_TRUE(encoder_state.ok());
  ASSERT_TRUE(encoder_state->multiturn_encoder_counts.has_value());
  EXPECT_EQ(encoder_state->multiturn_encoder_counts.value(), 400);

  fake_base_encoder.SetRawEncoderCounts(500);
  encoder_state = wrapper.Update();
  ASSERT_TRUE(encoder_state.ok());

  encoder_state = wrapper.GetEncoderState();
  ASSERT_TRUE(encoder_state.ok());
  ASSERT_TRUE(encoder_state->multiturn_encoder_counts.has_value());
  EXPECT_EQ(encoder_state->multiturn_encoder_counts.value(), 500);

  fake_base_encoder.SetRawEncoderCounts(450);
  encoder_state = wrapper.Update();
  ASSERT_TRUE(encoder_state.ok());

  encoder_state = wrapper.GetEncoderState();
  ASSERT_TRUE(encoder_state.ok());
  ASSERT_TRUE(encoder_state->multiturn_encoder_counts.has_value());
  EXPECT_EQ(encoder_state->multiturn_encoder_counts.value(), 450);
}

TEST(TestMultiturnEncoderWrapper, UnwrappingWorksAsExpected) {
  uint32_t counts_per_turn = 1000;

  FakeRotaryEncoder fake_base_encoder(counts_per_turn);
  MultiturnEncoderWrapper wrapper(fake_base_encoder);

  fake_base_encoder.SetRawEncoderCounts(999);
  pw::Result<RotaryEncoderState> encoder_state = wrapper.Update();
  ASSERT_TRUE(encoder_state.ok());

  encoder_state = wrapper.GetEncoderState();
  ASSERT_TRUE(encoder_state.ok());
  ASSERT_TRUE(encoder_state->multiturn_encoder_counts.has_value());
  EXPECT_EQ(encoder_state->multiturn_encoder_counts.value(), 999);

  fake_base_encoder.SetRawEncoderCounts(1);
  encoder_state = wrapper.Update();
  ASSERT_TRUE(encoder_state.ok());

  encoder_state = wrapper.GetEncoderState();
  ASSERT_TRUE(encoder_state.ok());
  ASSERT_TRUE(encoder_state->multiturn_encoder_counts.has_value());
  EXPECT_EQ(encoder_state->multiturn_encoder_counts.value(), 1001);
}

TEST(TestMultiturnEncoderWrapper, GetEncoderStatusPassesThroughErrors) {
  uint32_t counts_per_turn = 1000;

  FakeRotaryEncoder fake_base_encoder(counts_per_turn);
  MultiturnEncoderWrapper wrapper(fake_base_encoder);

  fake_base_encoder.SetEncoderStateReturnStatus(pw::Status::InvalidArgument());

  ASSERT_EQ(wrapper.GetEncoderState().status(), pw::Status::InvalidArgument());
}

}  // namespace
}  // namespace barkour
