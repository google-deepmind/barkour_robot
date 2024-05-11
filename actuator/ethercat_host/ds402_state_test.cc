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

#include "ds402_state.h"

#include <cstdint>

#include "gtest/gtest.h"

namespace barkour {
namespace {

TEST(TestDs402, CheckDs402StateAsStringValues) {
  EXPECT_EQ(Ds402StateAsString(Ds402State::kNotReady), "NOT_READY");
  EXPECT_EQ(Ds402StateAsString(Ds402State::kSwitchOnDisabled),
            "SWITCH_ON_DISABLED");
  EXPECT_EQ(Ds402StateAsString(Ds402State::kReadyToSwitchOn),
            "READY_TO_SWITCH_ON");
  EXPECT_EQ(Ds402StateAsString(Ds402State::kSwitchedOn), "SWITCHED_ON");
  EXPECT_EQ(Ds402StateAsString(Ds402State::kOperationEnabled),
            "OPERATION_ENABLED");
  EXPECT_EQ(Ds402StateAsString(Ds402State::kQuickStop), "QUICK_STOP");
  EXPECT_EQ(Ds402StateAsString(Ds402State::kFaultReaction), "FAULT_REACTION");
  EXPECT_EQ(Ds402StateAsString(Ds402State::kFault), "FAULT");
  EXPECT_EQ(Ds402StateAsString(Ds402State(4643)), "UNKNOWN_STATE");
}

TEST(TestDs402, CheckDs402StateFromStatusWordValues) {
  Ds402State state;
  EXPECT_TRUE(Ds402StateFromStatusWord(0b0000'0000, state));
  EXPECT_EQ(state, Ds402State::kNotReady);

  EXPECT_TRUE(Ds402StateFromStatusWord(0b0100'0000, state));
  EXPECT_EQ(state, Ds402State::kSwitchOnDisabled);

  EXPECT_TRUE(Ds402StateFromStatusWord(0b0010'0001, state));
  EXPECT_EQ(state, Ds402State::kReadyToSwitchOn);

  EXPECT_TRUE(Ds402StateFromStatusWord(0b0010'0011, state));
  EXPECT_EQ(state, Ds402State::kSwitchedOn);

  EXPECT_TRUE(Ds402StateFromStatusWord(0b0010'0111, state));
  EXPECT_EQ(state, Ds402State::kOperationEnabled);

  EXPECT_TRUE(Ds402StateFromStatusWord(0b0000'0111, state));
  EXPECT_EQ(state, Ds402State::kQuickStop);

  EXPECT_TRUE(Ds402StateFromStatusWord(0b0000'1111, state));
  EXPECT_EQ(state, Ds402State::kFaultReaction);

  EXPECT_TRUE(Ds402StateFromStatusWord(0b0000'1000, state));
  EXPECT_EQ(state, Ds402State::kFault);
}

TEST(TestDs402, CheckDs402StateFromStatusWordFailure) {
  Ds402State state = Ds402State::kSwitchedOn;
  EXPECT_FALSE(Ds402StateFromStatusWord(0b1111'1111, state));
  // Check that state isn't modified.
  EXPECT_EQ(state, Ds402State::kSwitchedOn);
}

TEST(TestDs402, CheckDs402StatusWordFromStateValues) {
  EXPECT_EQ(Ds402StatusWordFromState(Ds402State::kNotReady), 0b0000'0000);
  EXPECT_EQ(Ds402StatusWordFromState(Ds402State::kSwitchOnDisabled),
            0b0100'0000);
  EXPECT_EQ(Ds402StatusWordFromState(Ds402State::kReadyToSwitchOn),
            0b0010'0001);
  EXPECT_EQ(Ds402StatusWordFromState(Ds402State::kSwitchedOn), 0b0010'0011);
  EXPECT_EQ(Ds402StatusWordFromState(Ds402State::kOperationEnabled),
            0b0010'0111);
  EXPECT_EQ(Ds402StatusWordFromState(Ds402State::kQuickStop), 0b0000'0111);
  EXPECT_EQ(Ds402StatusWordFromState(Ds402State::kFaultReaction),
            0b0000'1111);
  EXPECT_EQ(Ds402StatusWordFromState(Ds402State::kFault), 0b0000'1000);
  // Invalid state
  EXPECT_EQ(Ds402StatusWordFromState(Ds402State(5432)), 0b0000'0000);
}

TEST(TestDs402, Ds402SControlWordForTargetStateCheckAllTransitions) {
  uint16_t control_word;
  // Transition 1 is automatic
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kSwitchOnDisabled,
                                              Ds402State::kNotReady,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0000);
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kOperationEnabled,
                                              Ds402State::kNotReady,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0000);

  // Transition 2
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kReadyToSwitchOn,
                                              Ds402State::kSwitchOnDisabled,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0110);
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kSwitchedOn,
                                              Ds402State::kSwitchOnDisabled,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0110);
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kQuickStop,
                                              Ds402State::kSwitchOnDisabled,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0110);

  // Transition 3
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kSwitchedOn,
                                              Ds402State::kReadyToSwitchOn,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0111);

  // Transition 3 + 4
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kOperationEnabled,
                                              Ds402State::kReadyToSwitchOn,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b1111);
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kQuickStop,
                                              Ds402State::kReadyToSwitchOn,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b1111);

  // Transition 4
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kOperationEnabled,
                                              Ds402State::kSwitchedOn,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b1111);
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kQuickStop,
                                              Ds402State::kSwitchedOn,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b1111);

  // Transition 5
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kSwitchedOn,
                                              Ds402State::kOperationEnabled,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0111);

  // Transition 6
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kReadyToSwitchOn,
                                              Ds402State::kSwitchedOn,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0110);

  // Transition 7
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kSwitchOnDisabled,
                                              Ds402State::kReadyToSwitchOn,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0000);

  // Transition 8
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kReadyToSwitchOn,
                                              Ds402State::kOperationEnabled,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0110);

  // Transition 9
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kSwitchOnDisabled,
                                              Ds402State::kOperationEnabled,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0000);

  // Transition 10
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kSwitchOnDisabled,
                                              Ds402State::kSwitchedOn,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0000);

  // Transition 11
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kQuickStop,
                                              Ds402State::kOperationEnabled,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0010);

  // Transition 12
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kSwitchOnDisabled,
                                              Ds402State::kQuickStop,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0000);

  // Transition 13 is triggered by drive when a fault is encountered.

  // Transition 14 is automatic
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kSwitchedOn,
                                              Ds402State::kFaultReaction,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0000);
  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(Ds402State::kReadyToSwitchOn,
                                              Ds402State::kFaultReaction,
                                              control_word, true));
  EXPECT_EQ(control_word, 0b0000);

  // Transition 15 is fault reset, target state shouldn't matter as long as it
  // isn't fault.
  control_word = 0;
  EXPECT_FALSE(Ds402ControlWordForTargetState(
      Ds402State::kSwitchedOn, Ds402State::kFault, control_word, false));
  EXPECT_EQ(control_word, 0b0000);

  control_word = 0;
  EXPECT_FALSE(Ds402ControlWordForTargetState(
      Ds402State::kFault, Ds402State::kFault, control_word, true));
  EXPECT_EQ(control_word, 0b0000);

  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(
      Ds402State::kSwitchOnDisabled, Ds402State::kFault, control_word, true));
  EXPECT_EQ(control_word, 0b1000'0000);

  control_word = 0;
  EXPECT_TRUE(Ds402ControlWordForTargetState(
      Ds402State::kOperationEnabled, Ds402State::kFault, control_word, true));
  EXPECT_EQ(control_word, 0b1000'0000);
}

TEST(TestDs402, CheckDs402SControlWordForTargetStateTargetStateAchieved) {
  // Initialize to an unusual value to check control_word is not set.
  uint16_t control_word = 0b0011'1101;
  uint16_t passed_control_word = control_word;
  EXPECT_FALSE(Ds402ControlWordForTargetState(Ds402State::kSwitchedOn,
                                               Ds402State::kSwitchedOn,
                                               passed_control_word, false));
  EXPECT_EQ(passed_control_word, control_word);

  EXPECT_FALSE(Ds402ControlWordForTargetState(Ds402State::kFaultReaction,
                                               Ds402State::kFaultReaction,
                                               passed_control_word, true));
  EXPECT_EQ(passed_control_word, control_word);
}

TEST(TestDs402, CheckDs402SControlWordForTargetStateNoValidPath) {
  // When there is no clear path from current_state to target_state, false
  // should be returned and control_word unmodified.

  uint16_t control_word = 0b0011'1101;
  uint16_t passed_control_word = control_word;
  EXPECT_FALSE(Ds402ControlWordForTargetState(Ds402State::kNotReady,
                                               Ds402State::kOperationEnabled,
                                               passed_control_word, true));
  EXPECT_EQ(passed_control_word, control_word);
}

TEST(TestDs402, CheckDs402SControlWordForTargetStateFaultBitSetToZero) {
  uint16_t control_word = 0b1000'0000;
  uint16_t control_word_no_fault_bit = 0b0000'0000;
  EXPECT_FALSE(Ds402ControlWordForTargetState(Ds402State::kNotReady,
                                               Ds402State::kOperationEnabled,
                                               control_word, true));
  EXPECT_EQ(control_word, control_word_no_fault_bit);

  control_word = 0b1000'0000;
  EXPECT_FALSE(Ds402ControlWordForTargetState(
      Ds402State::kSwitchedOn, Ds402State::kSwitchedOn, control_word, true));
  EXPECT_EQ(control_word, control_word_no_fault_bit);
}

}  // namespace
}  // namespace barkour
