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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACE_REGISTRY_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACE_REGISTRY_H_

#include <array>
#include <functional>
#include <type_traits>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/util/demangle.h"

namespace intrinsic::icon {

// Holds (but does not own) pointers to a number of FeatureInterfaces.
// The templated interface allows switching the internals to a more generic,
// type-erased implementation if ICON decides to support user-created
// FeatureInterfaces.
class FeatureInterfaceRegistry {
 public:
  // Creates a new FeatureInterfaceRegistry, and attempts to register
  // all of `interfaces` with it. Each parameter in the `interfaces` pack must
  // implement one or more of the interfaces in
  // intrinsic/icon/control/parts/feature_interfaces.h
  //
  // The resulting FeatureInterfaceRegistry holds pointers to `interfaces`, so
  // they must outlive it.
  //
  // Returns an error if any of `interfaces` could not be added to the registry.
  template <typename... InterfaceTs>
  static absl::StatusOr<FeatureInterfaceRegistry> FromInterfaces(
      InterfaceTs&... interfaces) {
    static_assert(sizeof...(InterfaceTs) > 0,
                  "FromInterfaces requires at least one interface.");
    FeatureInterfaceRegistry feature_interfaces;
    for (const RealtimeStatus& status :
         {feature_interfaces.RegisterAsCompatibleInterfaces(&interfaces)...}) {
      if (!status.ok()) return status;
    }
    return feature_interfaces;
  }

  // Constructs an initially empty registry.
  FeatureInterfaceRegistry() = default;

  absl::flat_hash_set<intrinsic_proto::icon::v1::FeatureInterfaceTypes>
  SupportedFeatureInterfaceTypes() const;

  // Registers a T pointer as an available FeatureInterface, once for each of
  // its base classes that is a valid FeatureInterface. This means
  // `maybe_interface` can end up being registered as more than one interface.
  //
  // Returns OkStatus if `maybe_interface` was successfully registered as one or
  // more FeatureInterfaces.
  // Returns the first error in case registering `maybe_interface` fails for any
  // of its supported base classes.
  // Raises a static_assert if `maybe_interface` does not inherit from any
  // supported FeatureInterface types.
  template <class T>
  RealtimeStatus RegisterAsCompatibleInterfaces(T* maybe_interface);

  // Registers a T pointer as an available FeatureInterface. Only the
  // specializations below are valid, any code using this generic definition
  // will fail to compile.
  //
  // Returns AlreadyExistsError if a pointer of that interface is already
  // registered.
  template <class T>
  RealtimeStatus RegisterInterface(T* v);

  // Specializations for supported interface types.
  template <>
  RealtimeStatus RegisterInterface(JointPosition* v);
  template <>
  RealtimeStatus RegisterInterface(JointVelocity* v);
  template <>
  RealtimeStatus RegisterInterface(JointAcceleration* v);
  template <>
  RealtimeStatus RegisterInterface(JointPositionSensor* v);
  template <>
  RealtimeStatus RegisterInterface(JointVelocityEstimator* v);
  template <>
  RealtimeStatus RegisterInterface(JointAccelerationEstimator* v);
  template <>
  RealtimeStatus RegisterInterface(JointLimitsInterface* v);
  template <>
  RealtimeStatus RegisterInterface(CartesianLimitsInterface* v);
  template <>
  RealtimeStatus RegisterInterface(SimpleGripper* v);
  template <>
  RealtimeStatus RegisterInterface(LinearGripper* v);
  template <>
  RealtimeStatus RegisterInterface(ADIO* v);
  template <>
  RealtimeStatus RegisterInterface(RangeFinder* v);
  template <>
  RealtimeStatus RegisterInterface(ManipulatorKinematics* v);
  template <>
  RealtimeStatus RegisterInterface(JointTorque* v);
  template <>
  RealtimeStatus RegisterInterface(JointTorqueSensor* v);
  template <>
  RealtimeStatus RegisterInterface(Dynamics* v);
  template <>
  RealtimeStatus RegisterInterface(ForceTorqueSensor* v);
  template <>
  RealtimeStatus RegisterInterface(StandaloneForceTorqueSensor* v);
  template <>
  RealtimeStatus RegisterInterface(HandGuiding* v);
  template <>
  RealtimeStatus RegisterInterface(Homing* v);
  template <>
  RealtimeStatus RegisterInterface(ControlModeExporter* v);
  template <>
  RealtimeStatus RegisterInterface(MoveOk* v);
  template <>
  RealtimeStatus RegisterInterface(InertialMeasurementUnit* v);
  template <>
  RealtimeStatus RegisterInterface(ProcessWrenchAtEndeffector* v);
  template <>
  RealtimeStatus RegisterInterface(Payload* v);
  template <>
  RealtimeStatus RegisterInterface(PayloadState* v);
  template <>
  RealtimeStatus RegisterInterface(CartesianPositionState* v);
  template <>
  RealtimeStatus RegisterInterface(BodyState* v);
  template <>
  RealtimeStatus RegisterInterface(BodyWrenchCommand* v);
  template <>
  RealtimeStatus RegisterInterface(VehicleLimitsInterface* v);
  // Returns a pointer to the given interface. Note that the return value
  // can be null if the requested interface has not been registered. As with
  // RegisterInterface, only the explicit specializations are valid.
  template <class T>
  T* GetInterface();

  // Returns the requested interface, or NotFoundError if no interface of type T
  // is registered with this registry.
  // This makes accessing interfaces in (non-realtime) setup functions more
  // seamless.
  template <class T>
  absl::StatusOr<std::reference_wrapper<T>> StatusOrInterface();

  // Specializations for the supported interface types.
  template <>
  JointPosition* GetInterface();
  template <>
  JointVelocity* GetInterface();
  template <>
  JointAcceleration* GetInterface();
  template <>
  JointPositionSensor* GetInterface();
  template <>
  JointVelocityEstimator* GetInterface();
  template <>
  JointAccelerationEstimator* GetInterface();
  template <>
  JointLimitsInterface* GetInterface();
  template <>
  CartesianLimitsInterface* GetInterface();
  template <>
  SimpleGripper* GetInterface();
  template <>
  LinearGripper* GetInterface();
  template <>
  ADIO* GetInterface();
  template <>
  RangeFinder* GetInterface();
  template <>
  ManipulatorKinematics* GetInterface();
  template <>
  JointTorque* GetInterface();
  template <>
  JointTorqueSensor* GetInterface();
  template <>
  Dynamics* GetInterface();
  template <>
  ForceTorqueSensor* GetInterface();
  template <>
  StandaloneForceTorqueSensor* GetInterface();
  template <>
  HandGuiding* GetInterface();
  template <>
  Homing* GetInterface();
  template <>
  ControlModeExporter* GetInterface();
  template <>
  MoveOk* GetInterface();
  template <>
  InertialMeasurementUnit* GetInterface();
  template <>
  ProcessWrenchAtEndeffector* GetInterface();
  template <>
  Payload* GetInterface();
  template <>
  PayloadState* GetInterface();
  template <>
  CartesianPositionState* GetInterface();
  template <>
  BodyState* GetInterface();
  template <>
  BodyWrenchCommand* GetInterface();
  template <>
  VehicleLimitsInterface* GetInterface();

  template <class T>
  const T* GetInterface() const;

  // Specializations for the supported interface types.
  template <>
  const JointPosition* GetInterface() const;
  template <>
  const JointVelocity* GetInterface() const;
  template <>
  const JointAcceleration* GetInterface() const;
  template <>
  const JointPositionSensor* GetInterface() const;
  template <>
  const JointVelocityEstimator* GetInterface() const;
  template <>
  const JointAccelerationEstimator* GetInterface() const;
  template <>
  const JointLimitsInterface* GetInterface() const;
  template <>
  const CartesianLimitsInterface* GetInterface() const;
  template <>
  const SimpleGripper* GetInterface() const;
  template <>
  const LinearGripper* GetInterface() const;
  template <>
  const ADIO* GetInterface() const;
  template <>
  const RangeFinder* GetInterface() const;
  template <>
  const ManipulatorKinematics* GetInterface() const;
  template <>
  const JointTorque* GetInterface() const;
  template <>
  const JointTorqueSensor* GetInterface() const;
  template <>
  const Dynamics* GetInterface() const;
  template <>
  const ForceTorqueSensor* GetInterface() const;
  template <>
  const StandaloneForceTorqueSensor* GetInterface() const;
  template <>
  const HandGuiding* GetInterface() const;
  template <>
  const Homing* GetInterface() const;
  template <>
  const ControlModeExporter* GetInterface() const;
  template <>
  const MoveOk* GetInterface() const;
  template <>
  const InertialMeasurementUnit* GetInterface() const;
  template <>
  const ProcessWrenchAtEndeffector* GetInterface() const;
  template <>
  const Payload* GetInterface() const;
  template <>
  const PayloadState* GetInterface() const;
  template <>
  const CartesianPositionState* GetInterface() const;
  template <>
  const BodyState* GetInterface() const;
  template <>
  const BodyWrenchCommand* GetInterface() const;
  template <>
  const VehicleLimitsInterface* GetInterface() const;

 private:
  template <class T, class... AllowedInterfaceTs>
  RealtimeStatus RegisterAsCompatibleInterfaceImpl(T* v) {
    static_assert(
        std::disjunction_v<std::is_base_of<AllowedInterfaceTs, T>...>,
        "Supplied pointer does not inherit from any FeatureInterfaces.");
    auto combine_status = [](const RealtimeStatus& accumulated_status,
                             const RealtimeStatus& new_status) {
      if (new_status.ok()) {
        return accumulated_status;
      } else if (accumulated_status.ok()) {
        return new_status;
      } else {
        return UnknownError(
            "More than one FeatureInterface registration failed.");
      }
    };

    return absl::c_accumulate(
        std::array<RealtimeStatus, sizeof...(AllowedInterfaceTs)>{([this, v]() {
          // Wrapped in a lambda because we need the constexpr if, since the
          // RegisterInterface() call only compiles if T inherits from the
          // respective FeatureInterface.
          if constexpr (std::is_base_of_v<AllowedInterfaceTs, T>) {
            return RegisterInterface<AllowedInterfaceTs>(v);
          } else {
            // Cast captures to void to avoid compiler errors about unused
            // lambda captures when the constexpr branch is discarded.
            (void)this;
            (void)v;
            return OkStatus();
          }
        }())...},
        OkStatus(), combine_status);
  }

  JointPosition* position_ = nullptr;
  JointVelocity* velocity_ = nullptr;
  JointAcceleration* acceleration_ = nullptr;
  JointPositionSensor* position_sensor_ = nullptr;
  JointVelocityEstimator* velocity_estimator_ = nullptr;
  JointAccelerationEstimator* acceleration_estimator_ = nullptr;
  JointLimitsInterface* joint_limits_ = nullptr;
  CartesianLimitsInterface* cartesian_limits_ = nullptr;
  SimpleGripper* gripper_ = nullptr;
  LinearGripper* linear_gripper_ = nullptr;
  ADIO* adio_ = nullptr;
  RangeFinder* rangefinder_ = nullptr;
  ManipulatorKinematics* manipulator_kinematics_ = nullptr;
  JointTorque* torque_ = nullptr;
  JointTorqueSensor* torque_sensor_ = nullptr;
  Dynamics* dynamics_ = nullptr;
  ForceTorqueSensor* force_torque_sensor_ = nullptr;
  StandaloneForceTorqueSensor* standalone_force_torque_sensor_ = nullptr;
  HandGuiding* native_hand_guiding_ = nullptr;
  Homing* homing_ = nullptr;
  ControlModeExporter* control_mode_exporter_ = nullptr;
  MoveOk* move_ok_ = nullptr;
  InertialMeasurementUnit* inertial_measurement_unit_ = nullptr;
  ProcessWrenchAtEndeffector* process_wrench_at_endeffector_ = nullptr;
  Payload* payload_ = nullptr;
  PayloadState* payload_state_ = nullptr;
  CartesianPositionState* cartesian_position_state_ = nullptr;
  BodyState* body_state_ = nullptr;
  BodyWrenchCommand* body_wrench_command_ = nullptr;
  VehicleLimitsInterface* vehicle_limits_ = nullptr;
};

template <class T>
RealtimeStatus FeatureInterfaceRegistry::RegisterAsCompatibleInterfaces(
    T* maybe_interface) {
  return RegisterAsCompatibleInterfaceImpl<
      T, JointPosition, JointVelocity, JointAcceleration, JointPositionSensor,
      JointVelocityEstimator, JointAccelerationEstimator, JointLimitsInterface,
      CartesianLimitsInterface, SimpleGripper, LinearGripper, ADIO, RangeFinder,
      ManipulatorKinematics, JointTorque, JointTorqueSensor, Dynamics,
      ForceTorqueSensor, StandaloneForceTorqueSensor, HandGuiding, Homing,
      ControlModeExporter, MoveOk, InertialMeasurementUnit,
      ProcessWrenchAtEndeffector, Payload, PayloadState, CartesianPositionState,
      BodyState, BodyWrenchCommand, VehicleLimitsInterface>(maybe_interface);
}

template <class T>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(T* v) {
  // sizeof(T) != sizeof(T) is false for all T, but crucially, can only be
  // evaluated once a concrete T is known, i.e. RegisterInterface<T> is
  // actually invoked.
  // Using this pattern ensures that calls to RegisterInterface with a type
  // that does not have a specialization below cause compile errors.
  // Importantly, std::false_type::value (or just false) would cause compile
  // errors *even if there are no such calls*.
  static_assert(sizeof(T) != sizeof(T), "Unsupported interface type.");
}

template <class T>
T* FeatureInterfaceRegistry::GetInterface() {
  static_assert(sizeof(T) != sizeof(T), "Unsupported interface type.");
}

template <class T>
const T* FeatureInterfaceRegistry::GetInterface() const {
  static_assert(sizeof(T) != sizeof(T), "Unsupported interface type.");
}

template <class T>
absl::StatusOr<std::reference_wrapper<T>>
FeatureInterfaceRegistry::StatusOrInterface() {
  T* interface = GetInterface<T>();
  if (interface == nullptr) {
    return absl::NotFoundError(absl::StrCat(
        "Registry does not have requested interface ", Demangle<T>()));
  }
  return *interface;
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACE_REGISTRY_H_
