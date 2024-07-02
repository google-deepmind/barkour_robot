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

#include "actuator/firmware/barkour/common/test_runner.h"

#include <utility>

#include "pw_function/function.h"
#include "pw_log/log.h"
#include "pw_unit_test/event_handler.h"
#include "pw_unit_test/framework.h"  // IWYU pragma: keep
#include "pw_unit_test/logging_event_handler.h"

namespace barkour {

namespace {

// Subclass the LoggingEventHandler, to insert an action (i.e. a sleep) after
// every test. This avoids overloading the log buffer.
class LoggingEventHandlerWithPostTestAction
    : public pw::unit_test::LoggingEventHandler {
 public:
  explicit LoggingEventHandlerWithPostTestAction(
      pw::Function<void(void)> post_test_action)
      : post_test_action_(std::move(post_test_action)) {}

  void TestCaseEnd(const pw::unit_test::TestCase& test_case,
                   pw::unit_test::TestResult result) override {
    pw::unit_test::LoggingEventHandler::TestCaseEnd(test_case, result);
    post_test_action_();
  }

 private:
  pw::Function<void(void)> post_test_action_;
};

}  // namespace

void run_tests(pw::Function<void(void)> post_test_action) {
  LoggingEventHandlerWithPostTestAction handler(std::move(post_test_action));
  pw::unit_test::RegisterEventHandler(&handler);
  RUN_ALL_TESTS();
}

}  // namespace barkour
