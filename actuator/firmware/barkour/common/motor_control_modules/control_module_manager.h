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

#ifndef BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_CONTROL_MODULE_MANAGER_H_
#define BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_CONTROL_MODULE_MANAGER_H_

#include <cstddef>

#include "actuator/firmware/barkour/common/motor_control_modules/motor_control_module.h"
#include "actuator/firmware/barkour/common/sensor_reading_types.h"
#include "pw_assert/check.h"  // NOLINT
#include "pw_assert/internal/check_impl.h"
#include "pw_containers/flat_map.h"
#include "pw_log/log.h"
#include "pw_result/result.h"
#include "pw_status/status.h"
#include "pw_status/try.h"

namespace barkour {

// Manages a set of `NumControlModules` motor control modules, each of which has
// an associated label of type `LabelType`. The ControlModuleManager manages the
// process of switching between control modules - when a new control module is
// selected it will request that the current control module stop, run it until
// it finishes stopping, and then start the newly selected control module.
//
// At any given time, a ControlModuleManager instance will have a selected
// control module, and if `Start` has been called successfully (without an
// associated call to `Stop`), then there will also be a currently active
// control module, whose outputs are returned from the `Step` method.
//
// As an example, consider the following setup:
//
// ```
// // Some pointers to control modules
// MotorControlModule* first_controller;
// MotorControlModule* second_controller;
// MotorControlModule* third_controller;
//
// // Control output to set
// RotatingFrameVoltages control_output;
//
// enum MyControllerLabels {
//   kFirstController,
//   kSecondController,
//   kThirdController,
// };
//
// // Build a control module manager.
// pw::containers::FlatMap<MyControllerLabels, MotorControlModule*, 3>
//   controller_map({{{kFirstController, first_controller},
//                    {kSecondController, second_controller},
//                    {kThirdController, third_controller}}});
//
// PW_TRY_ASSIGN(
//     ControlModuleManager control_module_manager,
//     ControlModuleManager<MyControllerLabels, 3>::Build(controller_map);
//
// // Upon construction, the selected controller will be the first in the map,
// // i.e. `first_controller` with label `kFirstController`, and there will be
// // no currently active control module.
//
// // Start the control module manager.
// PW_TRY(control_module_manager.Start(sensor_readings, derived_quantities));
//
// // After starting the selected and currently active controllers will both be
// // `first_controller`.
//
// // Run until the currently active control module has started.
// while (true) {
//   PW_TRY_ASSIGN(MotorControlModule::State state,
//                 control_module_manager.GetActiveControlModuleState());
//   if (state == MotorControlModule::State::kStarted) {
//     break;
//   }
//
//   PW_TRY_ASSIGN(control_output,
//                 control_module_manager.Step(sensor_readings,
//                                             derived_quantities,
//                                             references));
// }
//
// PW_CHECK(control_module_manager.IsSelectedControlModuleReady());
//
// // Now let's say we want to switch to using `second_controller` to control
// // the motor, so we select it using its label.
// PW_TRY(control_module_manager.SetSelectedControlModule(kSecondController));
//
// // At this point, the selected control module is `second_controller`, but the
// // currently active controller is still `first_controller`, so the selected
// // control module is not ready.
// PW_CHECK(!control_module_manager.IsSelectedControlModuleReady());
//
// // Now step the manager a few times.
//
// // In the first call to `Step`, the manager will observe that the selected
// // control module has changed, and it will call `Stop` on `first_controller`,
// // which is currently active. Let's say `first_controller` does not stop
// // immediately, instead transitioning to the `kStopping` state. `Step` will
// // then be called on `first_controller` to update it, and let's assume it
// // stays in the `kStopping` state after this update.
// PW_TRY_ASSIGN(control_output,
//               control_module_manager.Step(sensor_readings,
//                                           derived_quantities,
//                                           references));
//
// // At this point, the selected control module still is `second_controller`,
// // but the currently active controller is still `first_controller`, so the
// // selected control module is not ready.
// PW_CHECK(!control_module_manager.IsSelectedControlModuleReady());
//
// // In the next call to `Step`, the manager will again call `Step` on
// // `first_controller`. This time, let's assume it stops, transitioning to
// // the `kStopped` state after this update. The manager will observe that it
// // is now safe to stop using `first_controller` for control, and so will set
// // `second_controller` as the active controller, and call `Start` on it.
// PW_TRY_ASSIGN(control_output,
//               control_module_manager.Step(sensor_readings,
//                                           derived_quantities,
//                                           references));
//
// // At this point, `second_controller` is the selected and active control
// // module, but it is not in the `kStarted` state yet, so it is not ready.
// PW_CHECK(!control_module_manager.IsSelectedControlModuleReady());
//
// while (true) {
//   PW_TRY_ASSIGN(MotorControlModule::State state,
//                 control_module_manager.GetActiveControlModuleState());
//   if (state == MotorControlModule::State::kStarted) {
//     break;
//   }
//
//   PW_TRY_ASSIGN(control_output,
//                 control_module_manager.Step(sensor_readings,
//                                             derived_quantities,
//                                             references));
// }
//
// // At this point, `second_controller` is the selected and active control
// // module, and it is in the `kStarted` state yet, so it is ready.
// PW_CHECK(control_module_manager.IsSelectedControlModuleReady());
//
// // If we want to now transition to using `third_controller`, note how we can
// // use the `IsSelectedControlModuleReady` method to easily check whether the
// // control module switch has completed and simplify the above process.
// PW_TRY(control_module_manager.SetSelectedControlModule(kThirdController));
//
// while (!control_module_manager.IsSelectedControlModeReady()) {
//   PW_TRY_ASSIGN(control_output,
//                 control_module_manager.Step(sensor_readings,
//                                             derived_quantities,
//                                             references));
// }
// ```
//
// LabelType will be passed by value, so should be cheap to copy. The
// ControlModuleManager class was designed to be used with simple types such as
// integers or enums for LabelType.
//
// This class is not designed to be thread safe.
template <typename LabelType, std::size_t NumControlModules>
class ControlModuleManager {
  static_assert(
      NumControlModules > 0,
      "ControlModuleManager must manage at least one control module!");

 public:
  // Factory function.
  //
  // The initially selected control module will be set to the element pointed to
  // by `module_by_label.begin()`.
  //
  // IMPORTANT: After control modules have been passed into this factory
  // function, they will be managed entirely by the ControlModuleManager
  // instance and should no longer be modified directly. The result of doing so
  // is undefined behaiviour.
  //
  // Args:
  // - module_by_label: Map with pointers to the motor control modules to
  //   manage and their associated labels. This map and the referenced control
  //   modules must live at least as long as this manager instance is used.
  //
  // Returns:
  // - Invalid argument error: If any of the pointers to motor control modules
  //   are null.
  // - ControlModuleManager instance: If constructing the control module manager
  //   succeeded.
  static pw::Result<ControlModuleManager> Build(
      pw::containers::FlatMap<LabelType, MotorControlModule*,
                              NumControlModules>& module_by_label);

  // Returns the state of the currently active control module, or a failed
  // precondition error if there is no currently active control module, i.e. the
  // control module manager is not currently running.
  pw::Result<MotorControlModule::State> GetActiveControlModuleState() const;

  // Returns the label of the currently active control module, or a failed
  // precondition error if there is no currently active control module, i.e. the
  // control module manager is not currently running.
  pw::Result<LabelType> GetActiveControlModuleLabel() const;

  // Sets the selected control module.
  //
  // Note this is non-blocking and so does not wait until the selected control
  // module is active. Users should use `GetActiveControlModuleLabel` to see if
  // the selected control module is currently active or not.
  //
  // Will return a not found error if the new label is not found in the control
  // module map.
  pw::Status SetSelectedControlModule(LabelType new_control_module_label);

  // Returns the label of the currently selected control module.
  LabelType GetSelectedControlModuleLabel() const;

  // Returns true if and only if the control module manager is currently
  // running.
  bool IsRunning() const;

  // Returns true if and only if:
  // - There is an active control module.
  // - The active control module is the same as the selected control module.
  // - The active control module is in the `kStarted` state.
  bool IsSelectedControlModuleReady() const;

  // Starts the control module manager.
  //
  // This sets the selected control module as active, and calls `Start` on it.
  //
  // Returns:
  // - Failed precondition error: the control module manager is already running.
  pw::Status Start(const SensorReadings& sensor_readings,
                   const SensorDerivedQuantities& sensor_derived_quantities);

  // Requests that currently active control module, and the overall control
  // module manager, stop.
  //
  // Note that the currently active control module may not stop immediately and
  // further calls to `Step` may be required to actually stop. The return value
  // of `IsRunning` should be checked if confirmation of the stop is required.
  //
  // Returns:
  // - Failed precondition error: the control module manager not currently
  //   running.
  pw::Status Stop(const SensorReadings& sensor_readings,
                  const SensorDerivedQuantities& sensor_derived_quantities);

  // Force-stops the active control module and stops the control module manager.
  void ForceStop();

  // Steps the currently active control module, and returns the output.
  //
  // If the selected control module is different to the currently active control
  // module, this will first attempt to stop the currently active control
  // module, before switching to the selected control module.
  //
  // Returns a failed precondition error if called while the control module
  // manager is not running.
  pw::Result<RotatingFrameVoltages> Step(
      const SensorReadings& sensor_readings,
      const SensorDerivedQuantities& sensor_derived_quantities,
      const ControlReferences& references);

 private:
  using ModuleMapType = pw::containers::FlatMap<LabelType, MotorControlModule*,
                                                NumControlModules>;

  // Constructor.
  //
  // See Build() for argument info.
  explicit ControlModuleManager(ModuleMapType& module_by_label);

  ModuleMapType& module_by_label_;

  // Points to the currently active control module, or `module_by_label_.end()`
  // when the control module manager is not running.
  typename ModuleMapType::const_iterator active_control_module_;

  // Points to the selected control module, or `module_by_label_.end()`
  // when the control module manager is not running.
  typename ModuleMapType::const_iterator selected_control_module_;
};

// --- Implementations ---

template <typename LabelType, std::size_t NumControlModules>
pw::Result<ControlModuleManager<LabelType, NumControlModules>>
ControlModuleManager<LabelType, NumControlModules>::Build(
    pw::containers::FlatMap<LabelType, MotorControlModule*, NumControlModules>&
        module_by_label) {
  for (const auto& [unused_label, control_module_ptr] : module_by_label) {
    if (control_module_ptr == nullptr) {
      PW_LOG_ERROR(
          "Found null pointer to a motor control module when constructing a "
          "ControlModuleManager");
      return pw::Status::InvalidArgument();
    }
  }

  return ControlModuleManager(module_by_label);
}

template <typename LabelType, std::size_t NumControlModules>
pw::Result<MotorControlModule::State> ControlModuleManager
    <LabelType, NumControlModules>::GetActiveControlModuleState() const {
  return active_control_module_ == module_by_label_.end()
             ? pw::Result<MotorControlModule::State>(
                   pw::Status::FailedPrecondition())
             : pw::Result<MotorControlModule::State>(
                   active_control_module_->second->GetState());
}

template <typename LabelType, std::size_t NumControlModules>
pw::Result<LabelType> ControlModuleManager
    <LabelType, NumControlModules>::GetActiveControlModuleLabel() const {
  return active_control_module_ == module_by_label_.end()
             ? pw::Result<LabelType>(pw::Status::FailedPrecondition())
             : pw::Result<LabelType>(active_control_module_->first);
}

template <typename LabelType, std::size_t NumControlModules>
LabelType ControlModuleManager
    <LabelType, NumControlModules>::GetSelectedControlModuleLabel() const {
  // Internal invariant
  PW_CHECK(selected_control_module_ != module_by_label_.end());
  return selected_control_module_->first;
}

template <typename LabelType, std::size_t NumControlModules>
bool ControlModuleManager<LabelType, NumControlModules>::IsRunning() const {
  return active_control_module_ != module_by_label_.end();
}

template <typename LabelType, std::size_t NumControlModules>
bool ControlModuleManager
    <LabelType, NumControlModules>::IsSelectedControlModuleReady() const {
  return (active_control_module_ == selected_control_module_ &&
          active_control_module_->second->GetState() ==
              MotorControlModule::State::kStarted);
}

template <typename LabelType, std::size_t NumControlModules>
pw::Status ControlModuleManager
    <LabelType, NumControlModules>::SetSelectedControlModule(
    LabelType new_control_module_label) {
  typename ModuleMapType::const_iterator maybe_new_module =
      module_by_label_.find(new_control_module_label);
  if (maybe_new_module == module_by_label_.end()) {
    return pw::Status::NotFound();
  }

  selected_control_module_ = maybe_new_module;
  return pw::OkStatus();
}

template <typename LabelType, std::size_t NumControlModules>
pw::Status ControlModuleManager<LabelType, NumControlModules>::Start(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  if (IsRunning()) {
    return pw::Status::FailedPrecondition();
  }

  // Internal invariant
  PW_CHECK(selected_control_module_ != module_by_label_.end());

  active_control_module_ = selected_control_module_;
  return active_control_module_->second->Start(sensor_readings,
                                               sensor_derived_quantities);
}

template <typename LabelType, std::size_t NumControlModules>
pw::Status ControlModuleManager<LabelType, NumControlModules>::Stop(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities) {
  if (!IsRunning()) {
    return pw::Status::FailedPrecondition();
  }

  MotorControlModule& control_module = *active_control_module_->second;

  PW_TRY(control_module.Stop(sensor_readings, sensor_derived_quantities));

  if (control_module.GetState() == MotorControlModule::State::kStopped) {
    // Set no active control module, stopping the controller manager.
    active_control_module_ = module_by_label_.end();
  }
  return pw::OkStatus();
}

template <typename LabelType, std::size_t NumControlModules>
void ControlModuleManager<LabelType, NumControlModules>::ForceStop() {
  if (IsRunning()) {
    active_control_module_->second->ForceStop();
    active_control_module_ = module_by_label_.end();
  }
}

template <typename LabelType, std::size_t NumControlModules>
pw::Result<RotatingFrameVoltages>
ControlModuleManager<LabelType, NumControlModules>::Step(
    const SensorReadings& sensor_readings,
    const SensorDerivedQuantities& sensor_derived_quantities,
    const ControlReferences& references) {
  if (!IsRunning()) {
    return pw::Status::FailedPrecondition();
  }

  MotorControlModule* control_module = active_control_module_->second;

  // Internal invariant that the control module is not stopped if IsRunning
  // returns true.
  MotorControlModule::State current_state = control_module->GetState();
  PW_CHECK(current_state != MotorControlModule::State::kStopped);

  if (selected_control_module_ != active_control_module_) {
    // If we have entered this `if` block, the currently active control module
    // does not match the selected one. We want to make the currently active
    // control module stop so that we can switch it out for the selected one.
    if (current_state == MotorControlModule::State::kStarting ||
        current_state == MotorControlModule::State::kStarted) {
      // In this case, the currently active control module hasn't had `Stop`
      // called on it, so do that.
      control_module->Stop(sensor_readings, sensor_derived_quantities);

      // This may have caused it to stop instantaneously, if so then set
      // the selected control module as currently active.
      if (control_module->GetState() == MotorControlModule::State::kStopped) {
        PW_LOG_DEBUG(
            "Previous control module stopped, switching control module.");
        active_control_module_ = selected_control_module_;
        control_module = active_control_module_->second;
        control_module->Start(sensor_readings, sensor_derived_quantities);
      }
    }
  }

  // In any case, we should step the currently active control module.
  PW_TRY_ASSIGN(RotatingFrameVoltages control_output, control_module->Step(
                    sensor_readings, sensor_derived_quantities, references));

  if (control_module->GetState() == MotorControlModule::State::kStopped) {
    // If we need to change control module after the step, do that here.
    if (selected_control_module_ != active_control_module_) {
      PW_LOG_DEBUG(
          "Previous control module stopped, switching control module.");
      active_control_module_ = selected_control_module_;
      control_module = active_control_module_->second;
      control_module->Start(sensor_readings, sensor_derived_quantities);
    } else {
      // Control module stopped for another reason, set no active control
      // module, stopping the controller manager.
      PW_LOG_DEBUG(
          "Selected control module stopped, stopping controller manager.");
      active_control_module_ = module_by_label_.end();
    }
  }

  return control_output;
}

template <typename LabelType, std::size_t NumControlModules>
ControlModuleManager<LabelType, NumControlModules>::ControlModuleManager(
    ModuleMapType& module_by_label)
    : module_by_label_(module_by_label),
      active_control_module_(module_by_label.end()),
      selected_control_module_(module_by_label.begin()) {}

}  // namespace barkour

#endif  // BARKOUR_ROBOT_FIRMWARE_BARKOUR_COMMON_MOTOR_CONTROL_MODULES_CONTROL_MODULE_MANAGER_H_
