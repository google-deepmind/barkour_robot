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

#include "actuator/firmware/barkour/m4/shared_state.h"

#include "actuator/firmware/barkour/common/board_config.h"
#include "pw_status/status.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep

namespace barkour {
namespace {

TEST(SharedState, CanGetMultipleViews) {
  MotorDriverState& state = MotorDriverState::Get();
  state.GetReader();
  state.GetReader();
}

TEST(SharedState, WriterValidatesInputSize) {
  MotorDriverState& state = MotorDriverState::Get();
  auto writer_result =
      state.GetWriter(barkour::MotorDriverStateWriters::kMainThread);
  EXPECT_EQ(writer_result.status(), pw::OkStatus());
  auto* writer = writer_result.value();

  EXPECT_EQ(writer->SetVariable(MotorDriverVariable::kTargetTorque, 1),
            pw::OkStatus());
  EXPECT_EQ(writer->SetVariable(MotorDriverVariable::kTargetTorque, (double)3),
            pw::Status::InvalidArgument());
}

// Two views and two writers, verify consistency after call to Update().
TEST(SharedState, ViewsMatchesWritersAfterUpdate) {
  MotorDriverState& state = MotorDriverState::Get();
  MotorDriverStateView view_1 = state.GetReader();
  MotorDriverStateView view_2 = state.GetReader();
  auto writer_1_result =
      state.GetWriter(barkour::MotorDriverStateWriters::kMainThread);
  ASSERT_EQ(writer_1_result.status(), pw::OkStatus());
  auto* writer_1 = writer_1_result.value();
  auto writer_2_result =
      state.GetWriter(barkour::MotorDriverStateWriters::kEtherCat);
  ASSERT_EQ(writer_2_result.status(), pw::OkStatus());
  auto* writer_2 = writer_2_result.value();

  EXPECT_EQ(writer_1->SetVariable(MotorDriverVariable::kPosition, -100),
            pw::OkStatus());
  writer_2->State().velocity_ = 123;
  writer_2->State().target_velocity_ = 0xBEEF;  // This should be ignored.
  writer_2->SetDirtyFlag(MotorDriverVariable::kVelocity);

  EXPECT_EQ(state.Update(), pw::OkStatus());
  int32_t velocity_read;
  EXPECT_EQ(view_1.GetVariable(MotorDriverVariable::kVelocity, velocity_read),
            pw::OkStatus());
  EXPECT_EQ(velocity_read, 123);
  EXPECT_EQ(view_2.State().velocity_, 123);
  EXPECT_NE(view_2.State().target_velocity_, 0xBEEF);
  EXPECT_EQ(writer_2->SetVariable(MotorDriverVariable::kPosition, 9999),
            pw::OkStatus());
  EXPECT_EQ(view_2.State().position_, -100);
}

TEST(SharedState, ByteArraysAreFullyCopied) {
  MotorDriverState& state = MotorDriverState::Get();
  MotorDriverStateView view = state.GetReader();
  auto writer_result = state.GetWriter(barkour::MotorDriverStateWriters::kAdc);
  ASSERT_EQ(writer_result.status(), pw::OkStatus());
  auto* writer = writer_result.value();

  writer->State().blob_[99] = std::byte(5);
  writer->State().blob_[42] = std::byte(4);
  writer->State().blob_[2] = std::byte(99);
  writer->SetDirtyFlag(MotorDriverVariable::kBlob);

  EXPECT_EQ(state.Update(), pw::OkStatus());

  EXPECT_EQ(view.State().blob_[99], std::byte(5));
  EXPECT_EQ(view.State().blob_[42], std::byte(4));
  EXPECT_EQ(view.State().blob_[2], std::byte(99));

  std::byte blob[150];
  EXPECT_EQ(view.GetVariable(MotorDriverVariable::kBlob, blob), pw::OkStatus());

  EXPECT_EQ(blob[99], std::byte(5));
  EXPECT_EQ(blob[42], std::byte(4));
  EXPECT_EQ(blob[2], std::byte(99));
}

TEST(SharedState, ConcurrentWritesAreDetected) {
  MotorDriverState& state = MotorDriverState::Get();
  state.GetReader();
  state.GetReader();
  auto writer_1_result =
      state.GetWriter(barkour::MotorDriverStateWriters::kMainThread);
  ASSERT_EQ(writer_1_result.status(), pw::OkStatus());
  auto* writer_1 = writer_1_result.value();
  auto writer_2_result =
      state.GetWriter(barkour::MotorDriverStateWriters::kEtherCat);
  ASSERT_EQ(writer_2_result.status(), pw::OkStatus());
  auto* writer_2 = writer_2_result.value();

  EXPECT_EQ(writer_1->SetVariable(MotorDriverVariable::kPosition, -100),
            pw::OkStatus());
  EXPECT_EQ(writer_2->SetVariable(MotorDriverVariable::kPosition, -99),
            pw::OkStatus());

  EXPECT_EQ(state.Update(true), pw::Status::DataLoss());

  EXPECT_EQ(writer_1->SetVariable(MotorDriverVariable::kPosition, -100),
            pw::OkStatus());
  EXPECT_EQ(writer_2->SetVariable(MotorDriverVariable::kPosition, -99),
            pw::OkStatus());
  EXPECT_EQ(state.Update(false), pw::OkStatus());

  EXPECT_EQ(writer_1->SetVariable(MotorDriverVariable::kPosition, -100),
            pw::OkStatus());
  writer_2->State().position_ = -99;

  EXPECT_EQ(state.Update(true), pw::OkStatus());
}

// Verifies that concurrent writes to the same memory spaces result in a
// consistent result.
TEST(SharedState, WriteConsistency) {
  MotorDriverState& state = MotorDriverState::Get();
  MotorDriverStateView view = state.GetReader();
  auto writer_1_result =
      state.GetWriter(barkour::MotorDriverStateWriters::kMainThread);
  ASSERT_EQ(writer_1_result.status(), pw::OkStatus());
  auto* writer_1 = writer_1_result.value();
  auto writer_2_result =
      state.GetWriter(barkour::MotorDriverStateWriters::kEtherCat);
  ASSERT_EQ(writer_2_result.status(), pw::OkStatus());
  auto* writer_2 = writer_2_result.value();

  writer_1->State().blob_[42] = std::byte(14);
  writer_1->State().blob_[0] = std::byte(3);
  writer_1->State().blob_[100] = std::byte(23);
  writer_1->State().blob_[5] = std::byte(7);
  writer_2->SetDirtyFlag(MotorDriverVariable::kBlob);
  writer_2->State().blob_[100] = std::byte(70);
  writer_1->State().blob_[0] = std::byte(30);
  writer_2->State().blob_[42] = std::byte(13);
  writer_2->State().blob_[5] = std::byte(1);
  writer_1->SetDirtyFlag(MotorDriverVariable::kBlob);

  EXPECT_EQ(state.Update(false), pw::OkStatus());

  // Result must be either fully writer_1 or writer_2.
  if (view.State().blob_[42] == std::byte(14)) {
    // writer 1.
    EXPECT_EQ(view.State().blob_[0], std::byte(30));
    EXPECT_EQ(view.State().blob_[100], std::byte(23));
    EXPECT_EQ(view.State().blob_[5], std::byte(7));
  } else {
    // writer 2.
    EXPECT_EQ(view.State().blob_[42], std::byte(13));
    EXPECT_EQ(view.State().blob_[100], std::byte(70));
    EXPECT_EQ(view.State().blob_[5], std::byte(1));
  }
}

}  // namespace
}  // namespace barkour
