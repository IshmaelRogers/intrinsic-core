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

#include "intrinsic/icon/control/parts/feature_interface_registry.h"

#include "absl/container/flat_hash_set.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

using ::intrinsic_proto::icon::v1::FeatureInterfaceTypes;

absl::flat_hash_set<FeatureInterfaceTypes>
FeatureInterfaceRegistry::SupportedFeatureInterfaceTypes() const {
  absl::flat_hash_set<FeatureInterfaceTypes> interfaces;
  if (position_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_POSITION);
  }
  if (velocity_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_VELOCITY);
  }
  if (acceleration_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_ACCELERATION);
  }
  if (position_sensor_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_POSITION_SENSOR);
  }
  if (velocity_estimator_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_VELOCITY_ESTIMATOR);
  }
  if (acceleration_estimator_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_ACCELERATION_ESTIMATOR);
  }
  if (joint_limits_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_LIMITS);
  }
  if (cartesian_limits_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_CARTESIAN_LIMITS);
  }
  if (gripper_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_SIMPLE_GRIPPER);
  }
  if (linear_gripper_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_LINEAR_GRIPPER);
  }
  if (adio_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_ADIO);
  }
  if (rangefinder_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_RANGE_FINDER);
  }
  if (manipulator_kinematics_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_MANIPULATOR_KINEMATICS);
  }
  if (torque_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_TORQUE);
  }
  if (torque_sensor_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_TORQUE_SENSOR);
  }
  if (dynamics_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_DYNAMICS);
  }
  if (force_torque_sensor_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_FORCE_TORQUE_SENSOR);
  }
  if (standalone_force_torque_sensor_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::
                          FEATURE_INTERFACE_STANDALONE_FORCE_TORQUE_SENSOR);
  }
  if (native_hand_guiding_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_HAND_GUIDING);
  }
  if (homing_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_HOMING);
  }
  if (control_mode_exporter_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_CONTROL_MODE_EXPORTER);
  }
  if (move_ok_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_MOVE_OK);
  }
  if (inertial_measurement_unit_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_IMU);
  }
  if (process_wrench_at_endeffector_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_PROCESS_WRENCH_AT_ENDEFFECTOR);
  }
  if (payload_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_PAYLOAD);
  }
  if (payload_state_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_PAYLOAD_STATE);
  }
  if (cartesian_position_state_ != nullptr) {
    interfaces.insert(
        FeatureInterfaceTypes::FEATURE_INTERFACE_CARTESIAN_POSITION_STATE);
  }
  if (body_state_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE);
  }
  if (body_wrench_command_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH);
  }
  if (vehicle_limits_ != nullptr) {
    interfaces.insert(FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS);
  }
  return interfaces;
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(JointPosition* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as JointPosition.");
  }
  if (position_ != nullptr) {
    return AlreadyExistsError("JointPosition is already registered.");
  }
  position_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(JointVelocity* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as JointVelocity.");
  }
  if (velocity_ != nullptr) {
    return AlreadyExistsError("JointVelocity is already registered.");
  }
  velocity_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    JointAcceleration* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as JointAcceleration.");
  }
  if (acceleration_ != nullptr) {
    return AlreadyExistsError("JointAcceleration is already registered.");
  }
  acceleration_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    JointPositionSensor* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as JointPositionSensor.");
  }
  if (position_sensor_ != nullptr) {
    return AlreadyExistsError("JointPositionSensor is already registered.");
  }
  position_sensor_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    JointVelocityEstimator* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as JointVelocityEstimator.");
  }
  if (velocity_estimator_ != nullptr) {
    return AlreadyExistsError("JointVelocityEstimator is already registered.");
  }
  velocity_estimator_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    JointAccelerationEstimator* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as JointAccelerationEstimator.");
  }
  if (acceleration_estimator_ != nullptr) {
    return AlreadyExistsError(
        "JointAccelerationEstimator is already registered.");
  }
  acceleration_estimator_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    JointLimitsInterface* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as JointLimitsInterface.");
  }
  if (joint_limits_ != nullptr) {
    return AlreadyExistsError("JointLimitsInterface is already registered.");
  }
  joint_limits_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    CartesianLimitsInterface* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as CartesianLimitsInterface.");
  }
  if (cartesian_limits_ != nullptr) {
    return AlreadyExistsError(
        "CartesianLimitsInterface is already registered.");
  }
  cartesian_limits_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(SimpleGripper* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as SimpleGripper.");
  }
  if (gripper_ != nullptr) {
    return AlreadyExistsError("SimpleGripper is already registered.");
  }
  gripper_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(LinearGripper* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as LinearGripper.");
  }
  if (linear_gripper_ != nullptr) {
    return AlreadyExistsError("LinearGripper is already registered.");
  }
  linear_gripper_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(ADIO* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as ADIO.");
  }
  if (adio_ != nullptr) {
    return AlreadyExistsError("ADIO is already registered.");
  }
  adio_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(RangeFinder* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as RangeFinder.");
  }
  if (rangefinder_ != nullptr) {
    return AlreadyExistsError("RangeFinder is already registered.");
  }
  rangefinder_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    ManipulatorKinematics* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as ManipulatorKinematics.");
  }
  if (manipulator_kinematics_ != nullptr) {
    return AlreadyExistsError("ManipulatorKinematics is already registered.");
  }
  manipulator_kinematics_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(JointTorque* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as JointTorque.");
  }
  if (torque_ != nullptr) {
    return AlreadyExistsError("JointTorque is already registered.");
  }
  torque_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    JointTorqueSensor* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as JointTorqueSensor.");
  }
  if (torque_sensor_ != nullptr) {
    return AlreadyExistsError("JointTorqueSensor is already registered.");
  }
  torque_sensor_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(Dynamics* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as Dynamics.");
  }
  if (dynamics_ != nullptr) {
    return AlreadyExistsError("Dynamics is already registered.");
  }
  dynamics_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    ForceTorqueSensor* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as ForceTorqueSensor.");
  }
  if (force_torque_sensor_ != nullptr) {
    return AlreadyExistsError("ForceTorqueSensor is already registered.");
  }
  force_torque_sensor_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    StandaloneForceTorqueSensor* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as StandaloneForceTorqueSensor.");
  }
  if (standalone_force_torque_sensor_ != nullptr) {
    return AlreadyExistsError(
        "StandaloneForceTorqueSensor is already registered.");
  }
  standalone_force_torque_sensor_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(HandGuiding* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as HandGuiding.");
  }
  if (native_hand_guiding_ != nullptr) {
    return AlreadyExistsError("HandGuiding is already registered.");
  }
  native_hand_guiding_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(Homing* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as Homing.");
  }
  if (homing_ != nullptr) {
    return AlreadyExistsError("Homing is already registered.");
  }
  homing_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    ControlModeExporter* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as ControlModeExporter.");
  }
  if (control_mode_exporter_ != nullptr) {
    return AlreadyExistsError("ControlModeExporter is already registered.");
  }
  control_mode_exporter_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(MoveOk* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as MoveOk.");
  }
  if (move_ok_ != nullptr) {
    return AlreadyExistsError("MoveOk is already registered.");
  }
  move_ok_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    InertialMeasurementUnit* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as IMU.");
  }
  if (inertial_measurement_unit_ != nullptr) {
    return AlreadyExistsError("IMU is already registered.");
  }
  inertial_measurement_unit_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    ProcessWrenchAtEndeffector* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as ProcessWrenchAtEndeffector.");
  }
  if (process_wrench_at_endeffector_ != nullptr) {
    return AlreadyExistsError(
        "ProcessWrenchAtEndeffector is already registered.");
  }
  process_wrench_at_endeffector_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(Payload* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as Payload.");
  }
  if (payload_ != nullptr) {
    return AlreadyExistsError("Payload is already registered.");
  }
  payload_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(PayloadState* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as PayloadState.");
  }
  if (payload_state_ != nullptr) {
    return AlreadyExistsError("PayloadState is already registered.");
  }
  payload_state_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    CartesianPositionState* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as CartesianPositionState.");
  }
  if (cartesian_position_state_ != nullptr) {
    return AlreadyExistsError("CartesianPositionState is already registered.");
  }
  cartesian_position_state_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(BodyState* v) {
  if (v == nullptr) {
    return InvalidArgumentError("Cannot register nullptr as BodyState.");
  }
  if (body_state_ != nullptr) {
    return AlreadyExistsError("BodyState is already registered.");
  }
  body_state_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    BodyWrenchCommand* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as BodyWrenchCommand.");
  }
  if (body_wrench_command_ != nullptr) {
    return AlreadyExistsError("BodyWrenchCommand is already registered.");
  }
  body_wrench_command_ = v;
  return OkStatus();
}

template <>
RealtimeStatus FeatureInterfaceRegistry::RegisterInterface(
    VehicleLimitsInterface* v) {
  if (v == nullptr) {
    return InvalidArgumentError(
        "Cannot register nullptr as VehicleLimitsInterface.");
  }
  if (vehicle_limits_ != nullptr) {
    return AlreadyExistsError("VehicleLimitsInterface is already registered.");
  }
  vehicle_limits_ = v;
  return OkStatus();
}

template <>
JointPosition* FeatureInterfaceRegistry::GetInterface() {
  return position_;
}

template <>
JointVelocity* FeatureInterfaceRegistry::GetInterface() {
  return velocity_;
}

template <>
JointAcceleration* FeatureInterfaceRegistry::GetInterface() {
  return acceleration_;
}

template <>
JointPositionSensor* FeatureInterfaceRegistry::GetInterface() {
  return position_sensor_;
}

template <>
JointVelocityEstimator* FeatureInterfaceRegistry::GetInterface() {
  return velocity_estimator_;
}

template <>
JointAccelerationEstimator* FeatureInterfaceRegistry::GetInterface() {
  return acceleration_estimator_;
}

template <>
JointLimitsInterface* FeatureInterfaceRegistry::GetInterface() {
  return joint_limits_;
}

template <>
CartesianLimitsInterface* FeatureInterfaceRegistry::GetInterface() {
  return cartesian_limits_;
}

template <>
SimpleGripper* FeatureInterfaceRegistry::GetInterface() {
  return gripper_;
}

template <>
LinearGripper* FeatureInterfaceRegistry::GetInterface() {
  return linear_gripper_;
}

template <>
ADIO* FeatureInterfaceRegistry::GetInterface() {
  return adio_;
}

template <>
RangeFinder* FeatureInterfaceRegistry::GetInterface() {
  return rangefinder_;
}

template <>
ManipulatorKinematics* FeatureInterfaceRegistry::GetInterface() {
  return manipulator_kinematics_;
}

template <>
JointTorque* FeatureInterfaceRegistry::GetInterface() {
  return torque_;
}

template <>
JointTorqueSensor* FeatureInterfaceRegistry::GetInterface() {
  return torque_sensor_;
}

template <>
Dynamics* FeatureInterfaceRegistry::GetInterface() {
  return dynamics_;
}

template <>
ForceTorqueSensor* FeatureInterfaceRegistry::GetInterface() {
  return force_torque_sensor_;
}

template <>
StandaloneForceTorqueSensor* FeatureInterfaceRegistry::GetInterface() {
  return standalone_force_torque_sensor_;
}

template <>
HandGuiding* FeatureInterfaceRegistry::GetInterface() {
  return native_hand_guiding_;
}

template <>
Homing* FeatureInterfaceRegistry::GetInterface() {
  return homing_;
}

template <>
ControlModeExporter* FeatureInterfaceRegistry::GetInterface() {
  return control_mode_exporter_;
}

template <>
MoveOk* FeatureInterfaceRegistry::GetInterface() {
  return move_ok_;
}

template <>
InertialMeasurementUnit* FeatureInterfaceRegistry::GetInterface() {
  return inertial_measurement_unit_;
}

template <>
ProcessWrenchAtEndeffector* FeatureInterfaceRegistry::GetInterface() {
  return process_wrench_at_endeffector_;
}

template <>
Payload* FeatureInterfaceRegistry::GetInterface() {
  return payload_;
}

template <>
PayloadState* FeatureInterfaceRegistry::GetInterface() {
  return payload_state_;
}

template <>
CartesianPositionState* FeatureInterfaceRegistry::GetInterface() {
  return cartesian_position_state_;
}

template <>
BodyState* FeatureInterfaceRegistry::GetInterface() {
  return body_state_;
}

template <>
BodyWrenchCommand* FeatureInterfaceRegistry::GetInterface() {
  return body_wrench_command_;
}

template <>
VehicleLimitsInterface* FeatureInterfaceRegistry::GetInterface() {
  return vehicle_limits_;
}

template <>
const JointPosition* FeatureInterfaceRegistry::GetInterface() const {
  return position_;
}

template <>
const JointVelocity* FeatureInterfaceRegistry::GetInterface() const {
  return velocity_;
}

template <>
const JointAcceleration* FeatureInterfaceRegistry::GetInterface() const {
  return acceleration_;
}

template <>
const JointPositionSensor* FeatureInterfaceRegistry::GetInterface() const {
  return position_sensor_;
}

template <>
const JointVelocityEstimator* FeatureInterfaceRegistry::GetInterface() const {
  return velocity_estimator_;
}

template <>
const JointAccelerationEstimator* FeatureInterfaceRegistry::GetInterface()
    const {
  return acceleration_estimator_;
}

template <>
const JointLimitsInterface* FeatureInterfaceRegistry::GetInterface() const {
  return joint_limits_;
}

template <>
const CartesianLimitsInterface* FeatureInterfaceRegistry::GetInterface() const {
  return cartesian_limits_;
}

template <>
const SimpleGripper* FeatureInterfaceRegistry::GetInterface() const {
  return gripper_;
}

template <>
const LinearGripper* FeatureInterfaceRegistry::GetInterface() const {
  return linear_gripper_;
}

template <>
const ADIO* FeatureInterfaceRegistry::GetInterface() const {
  return adio_;
}

template <>
const RangeFinder* FeatureInterfaceRegistry::GetInterface() const {
  return rangefinder_;
}

template <>
const ManipulatorKinematics* FeatureInterfaceRegistry::GetInterface() const {
  return manipulator_kinematics_;
}

template <>
const JointTorque* FeatureInterfaceRegistry::GetInterface() const {
  return torque_;
}

template <>
const JointTorqueSensor* FeatureInterfaceRegistry::GetInterface() const {
  return torque_sensor_;
}

template <>
const Dynamics* FeatureInterfaceRegistry::GetInterface() const {
  return dynamics_;
}

template <>
const ForceTorqueSensor* FeatureInterfaceRegistry::GetInterface() const {
  return force_torque_sensor_;
}

template <>
const StandaloneForceTorqueSensor* FeatureInterfaceRegistry::GetInterface()
    const {
  return standalone_force_torque_sensor_;
}

template <>
const HandGuiding* FeatureInterfaceRegistry::GetInterface() const {
  return native_hand_guiding_;
}

template <>
const Homing* FeatureInterfaceRegistry::GetInterface() const {
  return homing_;
}

template <>
const ControlModeExporter* FeatureInterfaceRegistry::GetInterface() const {
  return control_mode_exporter_;
}

template <>
const MoveOk* FeatureInterfaceRegistry::GetInterface() const {
  return move_ok_;
}

template <>
const InertialMeasurementUnit* FeatureInterfaceRegistry::GetInterface() const {
  return inertial_measurement_unit_;
}

template <>
const ProcessWrenchAtEndeffector* FeatureInterfaceRegistry::GetInterface()
    const {
  return process_wrench_at_endeffector_;
}

template <>
const Payload* FeatureInterfaceRegistry::GetInterface() const {
  return payload_;
}

template <>
const PayloadState* FeatureInterfaceRegistry::GetInterface() const {
  return payload_state_;
}

template <>
const CartesianPositionState* FeatureInterfaceRegistry::GetInterface() const {
  return cartesian_position_state_;
}

template <>
const BodyState* FeatureInterfaceRegistry::GetInterface() const {
  return body_state_;
}

template <>
const BodyWrenchCommand* FeatureInterfaceRegistry::GetInterface() const {
  return body_wrench_command_;
}

template <>
const VehicleLimitsInterface* FeatureInterfaceRegistry::GetInterface() const {
  return vehicle_limits_;
}

}  // namespace intrinsic::icon
