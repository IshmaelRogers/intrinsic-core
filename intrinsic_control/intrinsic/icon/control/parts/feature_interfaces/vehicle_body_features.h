// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Opt-in ICON features for a free-body vehicle.
//
// ReadStatus is the cycle prepare step. ApplyCommand is the cycle apply
// step. Neither allocates. HalArmPart does not construct these classes.
//
// Fault precedence when ApplyCommand writes the HAL, highest first:
//   safety emergency > invalid body state > command watchdog
//     > stale body state > a rejected action command.
// A rejected action command is returned from SetBodyWrench and is not
// stored. ApplyCommand writes the neutral body wrench (zero force and
// torque, frame id "body") for every higher fault. A command watchdog
// stays latched until Reset(). Stale or invalid state is recomputed each
// prepare step. Actuator health is a later HAL contract, so this leaf has
// no fatal-hardware input.

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_VEHICLE_BODY_FEATURES_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_VEHICLE_BODY_FEATURES_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal.fbs.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

struct VehicleFeatureCycleConfig {
  // Monotonic command age. Wall-clock stamps do not feed this limit.
  Duration max_command_age = Milliseconds(50);
  // Monotonic age of the latched body-state producer update.
  Duration max_state_age = Milliseconds(50);
};

class BodyStateFeature : public HalFeatureInterfaceBase, public BodyState {
 public:
  static absl::StatusOr<BodyStateFeature> Create(
      HardwareInterfaceHandle<intrinsic_fbs::BodyState> body_state);

  BodyStateFeature(const BodyStateFeature&) = delete;
  BodyStateFeature& operator=(const BodyStateFeature&) = delete;
  BodyStateFeature(BodyStateFeature&&) = default;
  BodyStateFeature& operator=(BodyStateFeature&&) = default;
  ~BodyStateFeature() override = default;

  // Copies the hardware body state into a fixed-size sample.
  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  const BodyStateSample& LatchedBodyState() const override { return latched_; }

 private:
  explicit BodyStateFeature(
      HardwareInterfaceHandle<intrinsic_fbs::BodyState> body_state);

  HardwareInterfaceHandle<intrinsic_fbs::BodyState> body_state_;
  BodyStateSample latched_;
};

class VehicleLimitsFeature : public HalFeatureInterfaceBase,
                             public VehicleLimitsInterface {
 public:
  // `application` must lie inside `system` wherever both samples configure
  // the same limit. An unconfigured limit is not zero and is not infinity.
  static absl::StatusOr<VehicleLimitsFeature> Create(
      VehicleLimitsSample application, VehicleLimitsSample system);

  VehicleLimitsFeature(const VehicleLimitsFeature&) = delete;
  VehicleLimitsFeature& operator=(const VehicleLimitsFeature&) = delete;
  VehicleLimitsFeature(VehicleLimitsFeature&&) = default;
  VehicleLimitsFeature& operator=(VehicleLimitsFeature&&) = default;
  ~VehicleLimitsFeature() override = default;

  const VehicleLimitsSample& GetApplicationLimits() const override {
    return application_;
  }
  const VehicleLimitsSample& GetSystemLimits() const override {
    return system_;
  }

 private:
  VehicleLimitsFeature(VehicleLimitsSample application,
                       VehicleLimitsSample system);

  VehicleLimitsSample application_;
  VehicleLimitsSample system_;
};

class BodyWrenchFeature : public HalFeatureInterfaceBase,
                          public BodyWrenchCommand {
 public:
  // `body_state` and `limits` are not owned. They must outlive this
  // feature. A null capability returns FailedPrecondition.
  static absl::StatusOr<BodyWrenchFeature> Create(
      MutableHardwareInterfaceHandle<intrinsic_fbs::BodyWrench> wrench,
      const BodyState* body_state, const VehicleLimitsInterface* limits,
      VehicleFeatureCycleConfig config);

  BodyWrenchFeature(const BodyWrenchFeature&) = delete;
  BodyWrenchFeature& operator=(const BodyWrenchFeature&) = delete;
  BodyWrenchFeature(BodyWrenchFeature&&) = default;
  BodyWrenchFeature& operator=(BodyWrenchFeature&&) = default;
  ~BodyWrenchFeature() override = default;

  // Prepare. Reads the already-latched body state and the cycle safety
  // status. The part calls BodyStateFeature::ReadStatus first.
  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  // Apply. Writes the accepted wrench, holds a fresh prior wrench, or
  // writes the neutral body wrench and returns the highest fault.
  RealtimeStatus ApplyCommand(
      RealtimePartInterface::ApplyCommandParameters params) override;

  // Clears a latched command watchdog so a later command can be accepted.
  RealtimeStatus Reset() override;

  RealtimeStatus SetBodyWrench(const BodyWrenchSample& command) override;
  const BodyWrenchSample& PreviousBodyWrench() const override {
    return previous_;
  }

 private:
  enum class FaultRank {
    kNone = 0,
    kStateStale = 1,
    kCommandWatchdog = 2,
    kInvalidState = 3,
    kSafety = 4,
  };

  struct CycleFault {
    FaultRank rank = FaultRank::kNone;
    RealtimeStatus status = OkStatus();
  };

  BodyWrenchFeature(
      MutableHardwareInterfaceHandle<intrinsic_fbs::BodyWrench> wrench,
      const BodyState* body_state, const VehicleLimitsInterface* limits,
      VehicleFeatureCycleConfig config);

  void WriteSample(const BodyWrenchSample& sample);
  BodyWrenchSample NeutralSample() const;
  CycleFault ActiveFault() const;

  MutableHardwareInterfaceHandle<intrinsic_fbs::BodyWrench> wrench_;
  const BodyState* body_state_ = nullptr;
  const VehicleLimitsInterface* limits_ = nullptr;
  VehicleFeatureCycleConfig config_;

  CycleFault cycle_fault_;
  bool watchdog_latched_ = false;
  bool command_stream_started_ = false;
  bool pending_ = false;
  bool have_sequence_ = false;
  uint64_t last_sequence_ = 0;
  Time accepted_at_ = Time(Nanoseconds(0));
  Time last_applied_at_ = Time(Nanoseconds(0));
  BodyWrenchSample pending_command_;
  BodyWrenchSample previous_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_VEHICLE_BODY_FEATURES_H_
