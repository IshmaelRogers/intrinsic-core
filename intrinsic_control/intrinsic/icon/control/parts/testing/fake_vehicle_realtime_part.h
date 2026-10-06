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

// Test-only RealtimePartInterface for a free-body vehicle.
//
// Registers the #19 BodyState, VehicleLimits, and BodyWrench features.
// In-memory HAL segments back intrinsic_fbs::BodyState, BodyWrench, and
// VehicleLimits. Actuator health stays on this fake. It is not a feature
// interface. This target does not command hardware, open sockets, or link
// Gazebo.

#ifndef INTRINSIC_ICON_CONTROL_PARTS_TESTING_FAKE_VEHICLE_REALTIME_PART_H_
#define INTRINSIC_ICON_CONTROL_PARTS_TESTING_FAKE_VEHICLE_REALTIME_PART_H_

#include <array>
#include <cstdint>
#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/vehicle_body_features.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

class HardwareInterfaceRegistry;
class SharedMemoryManager;

// Integers match #28 ThrusterHealthState (and the marine thruster-array
// wire values): nominal 0, disabled 1, derated 2, stuck-off 3, failed 4.
// ThrusterHealthState lives in intrinsic_vehicle and ThrusterHealthKind
// lives in intrinsic_hardware. Neither package is already a dependency of
// intrinsic_control, so this fake keeps a test-only copy instead of adding
// a cross-package dep.
enum class FakeThrusterHealth : int {
  kNominal = 0,
  kDisabled = 1,
  kDerated = 2,
  kStuckOff = 3,
  kFailed = 4,
};

// One thruster slot. `has_derate` false means the injector did not set a
// derate. This fake stores that pair as given. It does not apply the #28
// derate-range rules, because #19 features do not read actuator health.
struct FakeThrusterActuatorHealth {
  FakeThrusterHealth health = FakeThrusterHealth::kNominal;
  bool has_derate = false;
  double derate = 1.0;
};

// Slot count of the #28 six-thruster fixture. Compile-time bound for this
// fake. It is not a discovered hardware size.
inline constexpr int kFakeVehicleThrusterSlots = 6;

// Vehicle part double. ReadStatus writes the injected body state into the
// HAL and then calls BodyStateFeature::ReadStatus before
// BodyWrenchFeature::ReadStatus. ApplyCommand calls BodyWrenchFeature and
// then copies the HAL body wrench.
//
// Vehicle limits are fixed at Create. VehicleLimitsFeature copies the two
// samples, and BodyWrenchFeature keeps a non-owning pointer to that
// feature. Replacing them would rebuild the wrench feature and drop its
// cycle state. #19 exposes no setter, so this fake has no replace path.
class FakeVehicleRealtimePart : public RealtimePartInterface {
 public:
  static absl::StatusOr<std::unique_ptr<FakeVehicleRealtimePart>> Create(
      VehicleLimitsSample application_limits, VehicleLimitsSample system_limits,
      VehicleFeatureCycleConfig cycle_config = {});

  ~FakeVehicleRealtimePart() override;

  FakeVehicleRealtimePart(const FakeVehicleRealtimePart&) = delete;
  FakeVehicleRealtimePart& operator=(const FakeVehicleRealtimePart&) = delete;
  FakeVehicleRealtimePart(FakeVehicleRealtimePart&&) = delete;
  FakeVehicleRealtimePart& operator=(FakeVehicleRealtimePart&&) = delete;

  // Next body-state sample. ReadStatus copies it into the HAL and stamps
  // the segment at `latched_monotonic_ns`. BodyStateFeature reports
  // `latched == true` and that stamp. Pass `latched == true`, and keep
  // bytes past each FixedId64 length at zero, when comparing
  // LatchedBodyState() to this sample.
  RealtimeStatus SetBodyState(const BodyStateSample& sample);

  void SetOperationalStatus(RealtimeOperationalStatus status);

  // Stages health for `slot`. ReadStatus copies the staged slots into the
  // latch. Out-of-range slots and values outside FakeThrusterHealth return
  // a status and leave the slot unchanged.
  RealtimeStatus SetActuatorHealth(int slot, FakeThrusterActuatorHealth health);
  RealtimeStatusOr<FakeThrusterActuatorHealth> ActuatorHealth(int slot) const;
  RealtimeStatusOr<FakeThrusterActuatorHealth> LatchedActuatorHealth(
      int slot) const;

  // HAL body wrench from the latest ApplyCommand. `applied` is not a HAL
  // field and stays false. Before the first ApplyCommand this is empty.
  const BodyWrenchSample& RecordedBodyWrench() const {
    return recorded_wrench_;
  }

  const intrinsic_fbs::VehicleLimits& ApplicationLimitsHal() const {
    return **application_limits_hal_;
  }
  const intrinsic_fbs::VehicleLimits& SystemLimitsHal() const {
    return **system_limits_hal_;
  }

  // Empty part properties. Tests that need a safety status pass it here.
  RealtimeStatus ReadStatusForTest(SafetyStatus safety = {});
  RealtimeStatus ApplyCommandForTest(SafetyStatus safety = {});

  RealtimeStatusOr<RealtimeOperationalStatus> GetOperationalStatus()
      const override;
  HardwareGroupSet GetHardwareDependencies() const override;
  RealtimeStatus ReadStatus(ReadStatusParameters params) override;
  RealtimeStatus ApplyCommand(ApplyCommandParameters params) override;
  FeatureInterfaceRegistry& GetFeatureInterfaces() override;
  const FeatureInterfaceRegistry& GetFeatureInterfaces() const override;

 private:
  FakeVehicleRealtimePart() = default;

  absl::Status Init(VehicleLimitsSample application_limits,
                    VehicleLimitsSample system_limits,
                    VehicleFeatureCycleConfig cycle_config);

  std::unique_ptr<SharedMemoryManager> shm_;
  std::unique_ptr<HardwareInterfaceRegistry> hw_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::BodyState> body_state_editor_;
  HardwareInterfaceHandle<intrinsic_fbs::BodyWrench> body_wrench_reader_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::VehicleLimits>
      application_limits_hal_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::VehicleLimits>
      system_limits_hal_;
  FeatureInterfaceRegistry registry_;
  std::optional<BodyStateFeature> body_state_;
  std::optional<VehicleLimitsFeature> limits_;
  std::optional<BodyWrenchFeature> wrench_;

  BodyStateSample injected_body_state_;
  BodyWrenchSample recorded_wrench_;
  RealtimeOperationalStatus operational_status_{
      .state = RealtimeOperationalState::kEnabled};
  HardwareGroupSet hardware_dependencies_{.operational_hardware = true};
  std::array<FakeThrusterActuatorHealth, kFakeVehicleThrusterSlots> health_{};
  std::array<FakeThrusterActuatorHealth, kFakeVehicleThrusterSlots>
      latched_health_{};
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_TESTING_FAKE_VEHICLE_REALTIME_PART_H_
