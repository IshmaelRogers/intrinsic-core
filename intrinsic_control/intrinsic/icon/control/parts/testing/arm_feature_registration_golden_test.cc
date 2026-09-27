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

// Golden for current arm feature-interface registration.
//
// Records FeatureInterfaceTypes and the status values returned when those
// interfaces are registered on FeatureInterfaceRegistry. Production
// registration code is unchanged. HalArmPart reaches that registry through
// RegisterAsCompatibleInterfaces; this fixture records that entry point.
//
// The checked-in fixture is never rewritten unless one of these is used:
//
//   bazel run
//   //intrinsic_control/intrinsic/icon/control/parts/testing:arm_feature_registration_golden_test
//   -- --update_golden
//
//   UPDATE_ARM_FEATURE_REGISTRATION_GOLDEN=1 bazel run
//   //intrinsic_control/intrinsic/icon/control/parts/testing:arm_feature_registration_golden_test
//
// Added, removed, or renumbered feature types fail the byte compare until
// that command is run. A senior owner reviews the resulting golden.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/descriptor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/fake_feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

// Set from main when argv contains --update_golden. Not in the anonymous
// namespace so the test process entry point can assign it.
bool g_update_golden = false;

namespace {

using FeatureInterfaceTypes = intrinsic_proto::icon::v1::FeatureInterfaceTypes;

constexpr absl::string_view kRelativeGolden =
    "intrinsic_control/intrinsic/icon/control/parts/testing/testdata/"
    "arm_feature_registration.golden";

constexpr absl::string_view kUpdateCommand =
    "bazel run "
    "//intrinsic_control/intrinsic/icon/control/parts/testing:"
    "arm_feature_registration_golden_test -- --update_golden";

constexpr absl::string_view kForbiddenTokens[] = {
    "BODY_WRENCH", "BODYWRENCH", "THRUSTER", "UUV", "VEHICLE",
};

bool UpdateRequested() {
  if (g_update_golden) {
    return true;
  }
  const char* env = std::getenv("UPDATE_ARM_FEATURE_REGISTRATION_GOLDEN");
  return env != nullptr && std::strcmp(env, "1") == 0;
}

// Payload has no production fake. Registration only stores the pointer.
class EmptyPayload final : public Payload {};

std::string FormatCodeAndMessage(absl::StatusCode code,
                                 absl::string_view message) {
  if (message.empty()) {
    return std::string(absl::StatusCodeToString(code));
  }
  return absl::StrCat(absl::StatusCodeToString(code), " ", message);
}

std::string FormatStatus(const RealtimeStatus& status) {
  return FormatCodeAndMessage(status.code(), status.message());
}

std::string FormatStatus(const absl::Status& status) {
  return FormatCodeAndMessage(status.code(), status.message());
}

std::string EnumName(int number) {
  const google::protobuf::EnumDescriptor* descriptor =
      intrinsic_proto::icon::v1::FeatureInterfaceTypes_descriptor();
  const google::protobuf::EnumValueDescriptor* value =
      descriptor->FindValueByNumber(number);
  if (value == nullptr) {
    return absl::StrCat("UNKNOWN_", number);
  }
  return std::string(value->name());
}

void AppendSupported(std::string* out,
                     const absl::flat_hash_set<FeatureInterfaceTypes>& types) {
  std::vector<int> numbers;
  numbers.reserve(types.size());
  for (FeatureInterfaceTypes type : types) {
    numbers.push_back(static_cast<int>(type));
  }
  std::sort(numbers.begin(), numbers.end());
  absl::StrAppend(out, "supported_count ", numbers.size(), "\n");
  for (int number : numbers) {
    absl::StrAppend(out, "supported ", EnumName(number), " ", number, "\n");
  }
}

void NoteCode(std::set<std::string>* codes, absl::StatusCode code) {
  codes->insert(std::string(absl::StatusCodeToString(code)));
}

template <typename InterfaceT>
void AppendInterfaceRegistration(std::string* out, InterfaceT* interface,
                                 std::set<std::string>* codes) {
  FeatureInterfaceRegistry registry;
  const RealtimeStatus null_status =
      registry.RegisterInterface<InterfaceT>(nullptr);
  const RealtimeStatus first =
      registry.RegisterInterface<InterfaceT>(interface);
  const RealtimeStatus duplicate =
      registry.RegisterInterface<InterfaceT>(interface);
  const bool present = registry.GetInterface<InterfaceT>() != nullptr;
  const FeatureInterfaceRegistry& registry_const = registry;
  const bool present_const =
      registry_const.GetInterface<InterfaceT>() != nullptr;

  FeatureInterfaceRegistry empty;
  const absl::Status missing = empty.StatusOrInterface<InterfaceT>().status();

  NoteCode(codes, null_status.code());
  NoteCode(codes, first.code());
  NoteCode(codes, duplicate.code());
  NoteCode(codes, missing.code());

  const absl::flat_hash_set<FeatureInterfaceTypes> supported =
      registry.SupportedFeatureInterfaceTypes();
  if (supported.size() == 1) {
    const int number = static_cast<int>(*supported.begin());
    absl::StrAppend(out, "registration ", EnumName(number), " ", number, "\n");
  } else {
    absl::StrAppend(out, "registration UNEXPECTED ", supported.size(), "\n");
  }
  absl::StrAppend(out, "nullptr ", FormatStatus(null_status), "\n");
  absl::StrAppend(out, "first ", FormatStatus(first), "\n");
  absl::StrAppend(out, "duplicate ", FormatStatus(duplicate), "\n");
  absl::StrAppend(out, "get_interface ", present ? "present" : "null", "\n");
  absl::StrAppend(out, "get_interface_const ",
                  present_const ? "present" : "null", "\n");
  AppendSupported(out, supported);
  absl::StrAppend(out, "missing ", FormatStatus(missing), "\n\n");
}

template <typename InterfaceT>
void AppendCompatibleRegistration(std::string* out, InterfaceT* interface,
                                  std::set<std::string>* codes) {
  FeatureInterfaceRegistry registry;
  const RealtimeStatus first =
      registry.RegisterAsCompatibleInterfaces<InterfaceT>(interface);
  const RealtimeStatus duplicate =
      registry.RegisterAsCompatibleInterfaces<InterfaceT>(interface);
  NoteCode(codes, first.code());
  NoteCode(codes, duplicate.code());

  const absl::flat_hash_set<FeatureInterfaceTypes> supported =
      registry.SupportedFeatureInterfaceTypes();
  if (supported.size() == 1) {
    const int number = static_cast<int>(*supported.begin());
    absl::StrAppend(out, "register_as_compatible ", EnumName(number), " ",
                    number, "\n");
  } else {
    absl::StrAppend(out, "register_as_compatible UNEXPECTED ", supported.size(),
                    "\n");
  }
  absl::StrAppend(out, "first ", FormatStatus(first), "\n");
  absl::StrAppend(out, "duplicate ", FormatStatus(duplicate), "\n");
  AppendSupported(out, supported);
  absl::StrAppend(out, "\n");
}

struct ArmFeatureFakes {
  ArmFeatureFakes()
      : joint_position(1),
        joint_velocity(1),
        joint_acceleration(1),
        joint_position_sensor(1),
        joint_velocity_estimator(1),
        joint_acceleration_estimator(1),
        joint_limits(CreateSimpleJointLimits(1, 1.0, 1.0, 1.0, 1.0)),
        cartesian_limits(CartesianLimits()),
        adio(FakeADIO::FakeADIOState{}),
        rangefinder(0.0, Pose3d()),
        manipulator_kinematics(1),
        joint_torque(1),
        joint_torque_sensor(1),
        imu(eigenmath::Vector3d::Zero(), eigenmath::Vector3d::Zero(),
            eigenmath::Quaterniond::Identity(), Pose3d()) {}

  FakeJointPosition joint_position;
  FakeJointVelocity joint_velocity;
  FakeJointAcceleration joint_acceleration;
  FakeJointPositionSensor joint_position_sensor;
  FakeJointVelocityEstimator joint_velocity_estimator;
  FakeJointAccelerationEstimator joint_acceleration_estimator;
  FakeJointLimits joint_limits;
  FakeCartesianLimits cartesian_limits;
  FakeGripper gripper;
  FakeADIO adio;
  FakeRangeFinder rangefinder;
  FakeManipulatorKinematics manipulator_kinematics;
  FakeJointTorque joint_torque;
  FakeJointTorqueSensor joint_torque_sensor;
  MockDynamics dynamics;
  FakeForceTorqueSensor force_torque_sensor;
  FakeLinearGripper linear_gripper;
  FakeHandGuiding hand_guiding;
  FakeControlModeExporter control_mode_exporter;
  FakeMoveOk move_ok;
  FakeInertialMeasurementUnit imu;
  FakeStandaloneForceTorqueSensor standalone_force_torque_sensor;
  FakeProcessWrenchAtEndeffector process_wrench;
  EmptyPayload payload;
  FakePayloadState payload_state;
  FakeCartesianPositionState cartesian_position_state;
  FakeHoming homing;
};

template <typename InterfaceT>
void RecordOne(std::string* out, FeatureInterfaceRegistry* all,
               InterfaceT* interface, std::set<std::string>* codes) {
  AppendInterfaceRegistration<InterfaceT>(out, interface, codes);
  const RealtimeStatus registered =
      all->RegisterInterface<InterfaceT>(interface);
  if (!registered.ok()) {
    absl::StrAppend(out, "combined_register_failed ", FormatStatus(registered),
                    "\n");
  }
}

void RecordAll(std::string* out, FeatureInterfaceRegistry* all,
               ArmFeatureFakes* fakes, std::set<std::string>* codes) {
  RecordOne<JointPosition>(out, all, &fakes->joint_position, codes);
  RecordOne<JointVelocity>(out, all, &fakes->joint_velocity, codes);
  RecordOne<JointPositionSensor>(out, all, &fakes->joint_position_sensor,
                                 codes);
  RecordOne<JointVelocityEstimator>(out, all, &fakes->joint_velocity_estimator,
                                    codes);
  RecordOne<JointAccelerationEstimator>(
      out, all, &fakes->joint_acceleration_estimator, codes);
  RecordOne<JointLimitsInterface>(out, all, &fakes->joint_limits, codes);
  RecordOne<CartesianLimitsInterface>(out, all, &fakes->cartesian_limits,
                                      codes);
  RecordOne<SimpleGripper>(out, all, &fakes->gripper, codes);
  RecordOne<ADIO>(out, all, &fakes->adio, codes);
  RecordOne<RangeFinder>(out, all, &fakes->rangefinder, codes);
  RecordOne<ManipulatorKinematics>(out, all, &fakes->manipulator_kinematics,
                                   codes);
  RecordOne<JointTorque>(out, all, &fakes->joint_torque, codes);
  RecordOne<JointTorqueSensor>(out, all, &fakes->joint_torque_sensor, codes);
  RecordOne<Dynamics>(out, all, &fakes->dynamics, codes);
  RecordOne<ForceTorqueSensor>(out, all, &fakes->force_torque_sensor, codes);
  RecordOne<LinearGripper>(out, all, &fakes->linear_gripper, codes);
  RecordOne<HandGuiding>(out, all, &fakes->hand_guiding, codes);
  RecordOne<ControlModeExporter>(out, all, &fakes->control_mode_exporter,
                                 codes);
  RecordOne<MoveOk>(out, all, &fakes->move_ok, codes);
  RecordOne<InertialMeasurementUnit>(out, all, &fakes->imu, codes);
  RecordOne<StandaloneForceTorqueSensor>(
      out, all, &fakes->standalone_force_torque_sensor, codes);
  RecordOne<ProcessWrenchAtEndeffector>(out, all, &fakes->process_wrench,
                                        codes);
  RecordOne<Payload>(out, all, &fakes->payload, codes);
  RecordOne<PayloadState>(out, all, &fakes->payload_state, codes);
  RecordOne<CartesianPositionState>(out, all, &fakes->cartesian_position_state,
                                    codes);
  RecordOne<Homing>(out, all, &fakes->homing, codes);
  RecordOne<JointAcceleration>(out, all, &fakes->joint_acceleration, codes);
}

std::string RenderArmFeatureRegistration() {
  std::string out;
  absl::StrAppend(
      &out,
      "# arm feature-registration golden\n"
      "# schema: arm-feature-registration-v1\n"
      "# Regenerate only with an explicit flag. This test does not rewrite "
      "the\n"
      "# fixture unless one of these commands is used:\n"
      "#   ",
      kUpdateCommand,
      "\n"
      "#   UPDATE_ARM_FEATURE_REGISTRATION_GOLDEN=1 bazel run "
      "//intrinsic_control/intrinsic/icon/control/parts/testing:"
      "arm_feature_registration_golden_test\n"
      "# Policy: byte-for-byte. Added, removed, or renumbered "
      "FeatureInterfaceTypes\n"
      "# fail until that command is run and a senior owner reviews the "
      "result.\n"
      "# register_as_compatible records the HalArmPart registration entry "
      "point.\n"
      "\n");

  const google::protobuf::EnumDescriptor* descriptor =
      intrinsic_proto::icon::v1::FeatureInterfaceTypes_descriptor();
  std::vector<const google::protobuf::EnumValueDescriptor*> values;
  values.reserve(descriptor->value_count());
  for (int i = 0; i < descriptor->value_count(); ++i) {
    values.push_back(descriptor->value(i));
  }
  std::sort(values.begin(), values.end(),
            [](const google::protobuf::EnumValueDescriptor* lhs,
               const google::protobuf::EnumValueDescriptor* rhs) {
              return lhs->number() < rhs->number();
            });
  absl::StrAppend(&out, "enum_catalog:\n");
  for (const google::protobuf::EnumValueDescriptor* value : values) {
    absl::StrAppend(&out, value->name(), " ", value->number(), "\n");
  }
  absl::StrAppend(&out, "\n");

  std::set<std::string> codes;
  absl::StrAppend(&out, "empty_registry:\n");
  AppendSupported(&out,
                  FeatureInterfaceRegistry().SupportedFeatureInterfaceTypes());
  absl::StrAppend(&out, "\n");

  ArmFeatureFakes fakes;
  FeatureInterfaceRegistry all;
  RecordAll(&out, &all, &fakes, &codes);
  absl::StrAppend(&out, "all_registered:\n");
  AppendSupported(&out, all.SupportedFeatureInterfaceTypes());
  absl::StrAppend(&out, "\n");

  FakeJointPosition from_interfaces_position(1);
  const absl::StatusOr<FeatureInterfaceRegistry> from_interfaces =
      FeatureInterfaceRegistry::FromInterfaces(from_interfaces_position);
  absl::StrAppend(&out, "from_interfaces:\n");
  if (!from_interfaces.ok()) {
    NoteCode(&codes, from_interfaces.status().code());
    absl::StrAppend(&out, "status ", FormatStatus(from_interfaces.status()),
                    "\nsupported_count 0\n\n");
  } else {
    NoteCode(&codes, absl::StatusCode::kOk);
    absl::StrAppend(&out, "status OK\n");
    AppendSupported(&out, from_interfaces->SupportedFeatureInterfaceTypes());
    absl::StrAppend(&out, "\n");
  }

  AppendCompatibleRegistration<ManipulatorKinematics>(
      &out, &fakes.manipulator_kinematics, &codes);
  AppendCompatibleRegistration<Dynamics>(&out, &fakes.dynamics, &codes);

  absl::StrAppend(&out, "status_values:\n");
  for (const std::string& code : codes) {
    absl::StrAppend(&out, code, "\n");
  }
  return out;
}

std::vector<std::string> SplitLines(absl::string_view text) {
  std::vector<std::string> lines;
  size_t start = 0;
  while (start < text.size()) {
    const size_t end = text.find('\n', start);
    if (end == absl::string_view::npos) {
      lines.emplace_back(text.substr(start));
      break;
    }
    lines.emplace_back(text.substr(start, end - start));
    start = end + 1;
  }
  return lines;
}

std::string FirstMismatch(absl::string_view expected,
                          absl::string_view actual) {
  if (expected == actual) {
    return {};
  }
  const std::vector<std::string> expected_lines = SplitLines(expected);
  const std::vector<std::string> actual_lines = SplitLines(actual);
  const size_t shared = std::min(expected_lines.size(), actual_lines.size());
  for (size_t i = 0; i < shared; ++i) {
    if (expected_lines[i] != actual_lines[i]) {
      return absl::StrCat("golden mismatch at line ", i + 1,
                          "\nexpected: ", expected_lines[i],
                          "\nactual: ", actual_lines[i], "\n");
    }
  }
  const std::string expected_tail =
      shared < expected_lines.size() ? expected_lines[shared] : "<eof>";
  const std::string actual_tail =
      shared < actual_lines.size() ? actual_lines[shared] : "<eof>";
  return absl::StrCat("golden mismatch at line ", shared + 1,
                      "\nexpected: ", expected_tail, "\nactual: ", actual_tail,
                      "\n");
}

std::string ReplaceFirst(absl::string_view text, absl::string_view from,
                         absl::string_view to) {
  std::string out(text);
  const size_t pos = out.find(from);
  if (pos == std::string::npos) {
    return out;
  }
  out.replace(pos, from.size(), to.data(), to.size());
  return out;
}

absl::Status RequireLine(const std::vector<std::string>& lines, size_t* index,
                         absl::string_view expected) {
  if (*index >= lines.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("missing line `", expected, "`"));
  }
  if (lines[*index] != expected) {
    return absl::InvalidArgumentError(
        absl::StrCat("line ", *index + 1, " want `", expected, "` got `",
                     lines[*index], "`"));
  }
  ++*index;
  return absl::OkStatus();
}

absl::Status RequirePrefix(const std::vector<std::string>& lines, size_t* index,
                           absl::string_view prefix) {
  if (*index >= lines.size() || !absl::StartsWith(lines[*index], prefix)) {
    const std::string got = *index >= lines.size() ? "<eof>" : lines[*index];
    return absl::InvalidArgumentError(absl::StrCat(
        "line ", *index + 1, " want prefix `", prefix, "` got `", got, "`"));
  }
  ++*index;
  return absl::OkStatus();
}

bool ParseNameNumber(absl::string_view line, absl::string_view* name,
                     int* number) {
  const size_t space = line.rfind(' ');
  if (space == absl::string_view::npos || space == 0) {
    return false;
  }
  *name = line.substr(0, space);
  const absl::string_view number_text = line.substr(space + 1);
  int parsed = 0;
  for (char ch : number_text) {
    if (ch < '0' || ch > '9') {
      return false;
    }
    parsed = parsed * 10 + (ch - '0');
  }
  if (number_text.empty()) {
    return false;
  }
  *number = parsed;
  return true;
}

absl::Status ValidateGolden(absl::string_view text) {
  if (text.empty() || text.back() != '\n') {
    return absl::InvalidArgumentError("golden must end with a newline");
  }
  if (text.find('\r') != absl::string_view::npos) {
    return absl::InvalidArgumentError("golden must use LF newlines");
  }
  if (!absl::StrContains(text, "--update_golden") ||
      !absl::StrContains(text, "UPDATE_ARM_FEATURE_REGISTRATION_GOLDEN=1")) {
    return absl::InvalidArgumentError(
        "golden must document the explicit regenerate commands");
  }
  for (absl::string_view token : kForbiddenTokens) {
    if (absl::StrContains(text, token)) {
      return absl::InvalidArgumentError(
          absl::StrCat("fixture references forbidden token ", token));
    }
  }

  const std::vector<std::string> lines = SplitLines(text);
  size_t index = 0;
  while (index < lines.size() && absl::StartsWith(lines[index], "#")) {
    ++index;
  }
  if (absl::Status status = RequireLine(lines, &index, ""); !status.ok()) {
    return status;
  }
  if (absl::Status status = RequireLine(lines, &index, "enum_catalog:");
      !status.ok()) {
    return status;
  }

  std::vector<std::pair<std::string, int>> catalog;
  int previous_number = -1;
  while (index < lines.size() && !lines[index].empty()) {
    absl::string_view name;
    int number = 0;
    if (!ParseNameNumber(lines[index], &name, &number)) {
      return absl::InvalidArgumentError(
          absl::StrCat("bad catalog line `", lines[index], "`"));
    }
    if (number <= previous_number) {
      return absl::InvalidArgumentError(
          "enum_catalog numbers must be strictly increasing");
    }
    previous_number = number;
    catalog.emplace_back(std::string(name), number);
    ++index;
  }
  if (catalog.empty() || catalog.front().first != "FEATURE_INTERFACE_INVALID" ||
      catalog.front().second != 0) {
    return absl::InvalidArgumentError(
        "enum_catalog must start with FEATURE_INTERFACE_INVALID 0");
  }
  if (absl::Status status = RequireLine(lines, &index, ""); !status.ok()) {
    return status;
  }
  if (absl::Status status = RequireLine(lines, &index, "empty_registry:");
      !status.ok()) {
    return status;
  }
  if (absl::Status status = RequireLine(lines, &index, "supported_count 0");
      !status.ok()) {
    return status;
  }
  if (absl::Status status = RequireLine(lines, &index, ""); !status.ok()) {
    return status;
  }

  std::vector<std::pair<std::string, int>> registered;
  while (index < lines.size() &&
         absl::StartsWith(lines[index], "registration ")) {
    absl::string_view header(lines[index]);
    header.remove_prefix(std::strlen("registration "));
    absl::string_view name;
    int number = 0;
    if (!ParseNameNumber(header, &name, &number)) {
      return absl::InvalidArgumentError(
          absl::StrCat("bad registration header `", lines[index], "`"));
    }
    const std::string name_str(name);
    const bool in_catalog = std::find(catalog.begin(), catalog.end(),
                                      std::pair<std::string, int>(
                                          name_str, number)) != catalog.end();
    if (!in_catalog) {
      return absl::InvalidArgumentError(
          absl::StrCat("registration ", name_str, " is not in enum_catalog"));
    }
    ++index;
    if (absl::Status status =
            RequirePrefix(lines, &index, "nullptr INVALID_ARGUMENT ");
        !status.ok()) {
      return status;
    }
    if (absl::Status status = RequireLine(lines, &index, "first OK");
        !status.ok()) {
      return status;
    }
    if (absl::Status status =
            RequirePrefix(lines, &index, "duplicate ALREADY_EXISTS ");
        !status.ok()) {
      return status;
    }
    if (absl::Status status =
            RequireLine(lines, &index, "get_interface present");
        !status.ok()) {
      return status;
    }
    if (absl::Status status =
            RequireLine(lines, &index, "get_interface_const present");
        !status.ok()) {
      return status;
    }
    if (absl::Status status = RequireLine(lines, &index, "supported_count 1");
        !status.ok()) {
      return status;
    }
    if (absl::Status status = RequireLine(
            lines, &index, absl::StrCat("supported ", name_str, " ", number));
        !status.ok()) {
      return status;
    }
    if (absl::Status status =
            RequirePrefix(lines, &index, "missing NOT_FOUND ");
        !status.ok()) {
      return status;
    }
    if (absl::Status status = RequireLine(lines, &index, ""); !status.ok()) {
      return status;
    }
    registered.emplace_back(name_str, number);
  }
  if (registered.empty()) {
    return absl::InvalidArgumentError("golden has no registration blocks");
  }

  if (absl::Status status = RequireLine(lines, &index, "all_registered:");
      !status.ok()) {
    return status;
  }
  if (absl::Status status = RequireLine(
          lines, &index, absl::StrCat("supported_count ", registered.size()));
      !status.ok()) {
    return status;
  }
  for (const auto& [name, number] : registered) {
    if (absl::Status status = RequireLine(
            lines, &index, absl::StrCat("supported ", name, " ", number));
        !status.ok()) {
      return status;
    }
  }
  if (absl::Status status = RequireLine(lines, &index, ""); !status.ok()) {
    return status;
  }

  if (absl::Status status = RequireLine(lines, &index, "from_interfaces:");
      !status.ok()) {
    return status;
  }
  if (absl::Status status = RequireLine(lines, &index, "status OK");
      !status.ok()) {
    return status;
  }
  if (absl::Status status = RequireLine(lines, &index, "supported_count 1");
      !status.ok()) {
    return status;
  }
  if (absl::Status status =
          RequirePrefix(lines, &index, "supported FEATURE_INTERFACE_");
      !status.ok()) {
    return status;
  }
  if (absl::Status status = RequireLine(lines, &index, ""); !status.ok()) {
    return status;
  }

  int compatible_blocks = 0;
  while (index < lines.size() &&
         absl::StartsWith(lines[index], "register_as_compatible ")) {
    absl::string_view header(lines[index]);
    header.remove_prefix(std::strlen("register_as_compatible "));
    absl::string_view name;
    int number = 0;
    if (!ParseNameNumber(header, &name, &number)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "bad register_as_compatible header `", lines[index], "`"));
    }
    const std::string supported_line =
        absl::StrCat("supported ", name, " ", number);
    ++index;
    if (absl::Status status = RequireLine(lines, &index, "first OK");
        !status.ok()) {
      return status;
    }
    if (absl::Status status =
            RequirePrefix(lines, &index, "duplicate ALREADY_EXISTS ");
        !status.ok()) {
      return status;
    }
    if (absl::Status status = RequireLine(lines, &index, "supported_count 1");
        !status.ok()) {
      return status;
    }
    if (absl::Status status = RequireLine(lines, &index, supported_line);
        !status.ok()) {
      return status;
    }
    if (absl::Status status = RequireLine(lines, &index, ""); !status.ok()) {
      return status;
    }
    ++compatible_blocks;
  }
  if (compatible_blocks < 1) {
    return absl::InvalidArgumentError("missing register_as_compatible blocks");
  }

  if (absl::Status status = RequireLine(lines, &index, "status_values:");
      !status.ok()) {
    return status;
  }
  const std::vector<std::string> expected_codes = {
      "ALREADY_EXISTS",
      "INVALID_ARGUMENT",
      "NOT_FOUND",
      "OK",
  };
  for (const std::string& code : expected_codes) {
    if (absl::Status status = RequireLine(lines, &index, code); !status.ok()) {
      return status;
    }
  }
  if (index != lines.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("unexpected trailing line `", lines[index], "`"));
  }
  return absl::OkStatus();
}

absl::Status WriteFile(absl::string_view path, absl::string_view contents) {
  std::ofstream out(std::string(path), std::ios::binary | std::ios::trunc);
  if (!out) {
    return absl::InternalError(absl::StrCat("cannot write ", path));
  }
  out.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  if (!out) {
    return absl::InternalError(absl::StrCat("failed writing ", path));
  }
  return absl::OkStatus();
}

absl::StatusOr<std::string> ReadFile(absl::string_view path) {
  std::ifstream in(std::string(path), std::ios::binary);
  if (!in) {
    return absl::NotFoundError(absl::StrCat("cannot read ", path));
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

std::string WorkspaceGoldenPath() {
  const char* root = std::getenv("BUILD_WORKSPACE_DIRECTORY");
  if (root == nullptr || root[0] == '\0') {
    return {};
  }
  return absl::StrCat(root, "/", kRelativeGolden);
}

std::vector<std::string> GoldenReadCandidates() {
  std::vector<std::string> paths;
  const char* srcdir = std::getenv("TEST_SRCDIR");
  const char* workspace = std::getenv("TEST_WORKSPACE");
  if (srcdir != nullptr && workspace != nullptr) {
    paths.push_back(absl::StrCat(srcdir, "/", workspace, "/", kRelativeGolden));
  }
  if (srcdir != nullptr) {
    paths.push_back(absl::StrCat(srcdir, "/_main/", kRelativeGolden));
  }
  const char* runfiles = std::getenv("RUNFILES_DIR");
  if (runfiles != nullptr) {
    paths.push_back(absl::StrCat(runfiles, "/_main/", kRelativeGolden));
  }
  const std::string workspace_path = WorkspaceGoldenPath();
  if (!workspace_path.empty()) {
    paths.push_back(workspace_path);
  }
  paths.emplace_back(kRelativeGolden);
  return paths;
}

absl::StatusOr<std::string> ReadCheckedInGolden() {
  if (UpdateRequested()) {
    const std::string path = WorkspaceGoldenPath();
    if (path.empty()) {
      return absl::FailedPreconditionError(
          "refusing to rewrite the fixture: --update_golden and "
          "UPDATE_ARM_FEATURE_REGISTRATION_GOLDEN=1 require `bazel run` so "
          "BUILD_WORKSPACE_DIRECTORY is set");
    }
    return ReadFile(path);
  }
  std::string tried;
  for (const std::string& path : GoldenReadCandidates()) {
    absl::StrAppend(&tried, path, "\n");
    std::ifstream in(path, std::ios::binary);
    if (!in) {
      continue;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
  }
  return absl::NotFoundError(absl::StrCat(
      "arm feature-registration golden not found. Tried:\n", tried));
}

TEST(ArmFeatureRegistrationGoldenTest, RenderSatisfiesRegistrationContract) {
  const absl::Status status = ValidateGolden(RenderArmFeatureRegistration());
  EXPECT_TRUE(status.ok()) << status;
}

TEST(ArmFeatureRegistrationGoldenTest, RenderIsDeterministic) {
  EXPECT_EQ(RenderArmFeatureRegistration(), RenderArmFeatureRegistration());
}

TEST(ArmFeatureRegistrationGoldenTest, RenumberedCatalogLineFailsClosed) {
  const std::string rendered = RenderArmFeatureRegistration();
  const std::string mutated =
      ReplaceFirst(rendered,
                   "enum_catalog:\nFEATURE_INTERFACE_INVALID 0\n"
                   "FEATURE_INTERFACE_JOINT_POSITION 1\n",
                   "enum_catalog:\nFEATURE_INTERFACE_INVALID 0\n"
                   "FEATURE_INTERFACE_JOINT_POSITION 99\n");
  ASSERT_NE(rendered, mutated);
  const std::string diagnostic = FirstMismatch(rendered, mutated);
  EXPECT_TRUE(absl::StrContains(diagnostic, "FEATURE_INTERFACE_JOINT_POSITION"))
      << diagnostic;
  EXPECT_TRUE(absl::StrContains(diagnostic, "99")) << diagnostic;
  EXPECT_FALSE(ValidateGolden(mutated).ok());
}

TEST(ArmFeatureRegistrationGoldenTest, RemovedRegistrationLineFailsClosed) {
  const std::string rendered = RenderArmFeatureRegistration();
  const std::string needle = "registration FEATURE_INTERFACE_HOMING ";
  ASSERT_TRUE(absl::StrContains(rendered, needle));
  const std::string mutated = ReplaceFirst(rendered, needle, "registration ");
  ASSERT_NE(rendered, mutated);
  const std::string diagnostic = FirstMismatch(rendered, mutated);
  EXPECT_TRUE(absl::StrContains(diagnostic, "FEATURE_INTERFACE_HOMING"))
      << diagnostic;
  EXPECT_FALSE(ValidateGolden(mutated).ok());
}

TEST(ArmFeatureRegistrationGoldenTest, AdditiveCatalogLineFailsClosed) {
  const std::string rendered = RenderArmFeatureRegistration();
  const std::string mutated =
      ReplaceFirst(rendered, "FEATURE_INTERFACE_INVALID 0\n",
                   "FEATURE_INTERFACE_INVALID 0\nFEATURE_INTERFACE_EXTRA 28\n");
  ASSERT_NE(rendered, mutated);
  const std::string diagnostic = FirstMismatch(rendered, mutated);
  EXPECT_TRUE(absl::StrContains(diagnostic, "FEATURE_INTERFACE_EXTRA"))
      << diagnostic;
  EXPECT_FALSE(ValidateGolden(mutated).ok());
}

TEST(ArmFeatureRegistrationGoldenTest, MatchesCheckedInGolden) {
  const std::string rendered = RenderArmFeatureRegistration();
  const absl::Status contract = ValidateGolden(rendered);
  ASSERT_TRUE(contract.ok()) << contract;

  if (UpdateRequested()) {
    const std::string path = WorkspaceGoldenPath();
    ASSERT_FALSE(path.empty())
        << "Refusing to rewrite the fixture. Run:\n  " << kUpdateCommand;
    const absl::Status written = WriteFile(path, rendered);
    ASSERT_TRUE(written.ok()) << written;
    std::cerr << "Rewrote " << path
              << " because --update_golden or "
                 "UPDATE_ARM_FEATURE_REGISTRATION_GOLDEN=1 was set.\n";
  }

  const absl::StatusOr<std::string> loaded = ReadCheckedInGolden();
  ASSERT_TRUE(loaded.ok()) << loaded.status();
  EXPECT_EQ(*loaded, rendered)
      << "Refusing to rewrite the fixture without an explicit flag.\n"
      << "Regenerate with:\n  " << kUpdateCommand << "\n"
      << FirstMismatch(*loaded, rendered);
  const absl::Status loaded_contract = ValidateGolden(*loaded);
  EXPECT_TRUE(loaded_contract.ok()) << loaded_contract;
}

}  // namespace
}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  std::vector<char*> args;
  args.reserve(static_cast<size_t>(argc));
  args.push_back(argv[0]);
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--update_golden") == 0) {
      intrinsic::icon::g_update_golden = true;
      continue;
    }
    args.push_back(argv[i]);
  }
  int filtered_argc = static_cast<int>(args.size());
  ::testing::InitGoogleTest(&filtered_argc, args.data());
  return RUN_ALL_TESTS();
}
