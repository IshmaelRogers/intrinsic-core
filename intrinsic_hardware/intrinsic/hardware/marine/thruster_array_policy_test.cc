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

#include "intrinsic/hardware/marine/thruster_array_policy.h"

#include <limits>
#include <vector>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"
#include "gtest/gtest.h"

namespace intrinsic::hardware::marine {
namespace {

using embodiment::ValidityKind;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

ThrusterHeaderView NominalHeader() {
  ThrusterHeaderView header;
  header.header_present = true;
  header.frame_id = vehicle::kBodyFrameId;
  header.source_time_present = true;
  header.source_time = embodiment::ClockReading{1700000000, 250000000};
  header.receive_time_present = true;
  header.receive_time = embodiment::ClockReading{1700000001, 0};
  header.header_validity_present = true;
  header.header_validity_state = 1;
  return header;
}

ThrusterCommandElementView CommandElement(double thrust_n,
                                          const char *name = "surge_port") {
  ThrusterCommandElementView element;
  element.name_present = true;
  element.name = name;
  element.thrust_present = true;
  element.thrust_n = thrust_n;
  return element;
}

ThrusterFeedbackElementView FeedbackElement(double thrust_n, int health = 0,
                                            double derate = 1.0,
                                            double efficiency = 1.0) {
  ThrusterFeedbackElementView element;
  element.name_present = true;
  element.name = "surge_port";
  element.commanded_thrust_present = true;
  element.commanded_thrust_n = thrust_n;
  element.measured_thrust_present = true;
  element.measured_thrust_n = thrust_n;
  element.saturated_present = true;
  element.saturated = false;
  element.health_present = true;
  element.health = health;
  element.health_derate_present = true;
  element.health_derate = derate;
  element.efficiency_present = true;
  element.efficiency = efficiency;
  return element;
}

ThrusterArrayAssessment
AssessCommand(ThrusterHeaderView header,
              const std::vector<ThrusterCommandElementView> &elements) {
  ThrusterArrayCommandView view;
  view.header = header;
  view.thrusters = elements;
  return AssessThrusterArrayCommand(view);
}

ThrusterArrayAssessment
AssessFeedback(ThrusterHeaderView header,
               const std::vector<ThrusterFeedbackElementView> &elements) {
  ThrusterArrayFeedbackView view;
  view.header = header;
  view.thrusters = elements;
  return AssessThrusterArrayFeedback(view);
}

TEST(ThrusterArrayPolicyTest, EmptyMessagesAreAbsentAndNotErrors) {
  const ThrusterArrayAssessment command =
      AssessThrusterArrayCommand(ThrusterArrayCommandView{});
  EXPECT_EQ(command.error, ThrusterArrayError::kNone);
  EXPECT_EQ(command.header_validity, ValidityKind::kAbsent);
  EXPECT_FALSE(command.accepted);

  const ThrusterArrayAssessment feedback =
      AssessThrusterArrayFeedback(ThrusterArrayFeedbackView{});
  EXPECT_EQ(feedback.error, ThrusterArrayError::kNone);
  EXPECT_FALSE(feedback.accepted);
}

TEST(ThrusterArrayPolicyTest, NominalCommandAndZeroThrustAreAccepted) {
  const std::vector<ThrusterCommandElementView> elements = {
      CommandElement(10.0), CommandElement(0.0, "sway_fore")};
  const ThrusterArrayAssessment assessment =
      AssessCommand(NominalHeader(), elements);
  EXPECT_EQ(assessment.error, ThrusterArrayError::kNone);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_TRUE(ThrusterCommandEnabled(elements[0]));
}

TEST(ThrusterArrayPolicyTest, UnsetEnableIsEnabledAndExplicitFalseIsKept) {
  ThrusterCommandElementView unset = CommandElement(4.0);
  EXPECT_FALSE(unset.enable_present);
  EXPECT_TRUE(ThrusterCommandEnabled(unset));

  ThrusterCommandElementView disabled = CommandElement(4.0);
  disabled.enable_present = true;
  disabled.enable = false;
  EXPECT_FALSE(ThrusterCommandEnabled(disabled));
  const std::vector<ThrusterCommandElementView> elements = {disabled};
  EXPECT_TRUE(AssessCommand(NominalHeader(), elements).accepted);
}

TEST(ThrusterArrayPolicyTest, HeaderDefectsAreFirstAndOrdered) {
  std::vector<ThrusterCommandElementView> elements = {CommandElement(kNaN)};
  ThrusterHeaderView missing;
  EXPECT_EQ(AssessCommand(missing, elements).error,
            ThrusterArrayError::kMissingHeader);

  ThrusterHeaderView empty_frame = NominalHeader();
  empty_frame.frame_id = "";
  EXPECT_EQ(AssessCommand(empty_frame, elements).error,
            ThrusterArrayError::kMissingFrame);

  ThrusterHeaderView world = NominalHeader();
  world.frame_id = "world_enu";
  EXPECT_EQ(AssessCommand(world, elements).error,
            ThrusterArrayError::kWrongFrame);

  ThrusterHeaderView reversed = NominalHeader();
  reversed.receive_time = embodiment::ClockReading{1700000000, 1};
  EXPECT_EQ(AssessCommand(reversed, elements).error,
            ThrusterArrayError::kTimeReversal);

  ThrusterHeaderView same_second = NominalHeader();
  same_second.source_time = embodiment::ClockReading{10, 5};
  same_second.receive_time = embodiment::ClockReading{10, 4};
  EXPECT_EQ(AssessCommand(same_second, elements).error,
            ThrusterArrayError::kTimeReversal);
}

TEST(ThrusterArrayPolicyTest, EmptyArrayAndElementDefects) {
  EXPECT_EQ(AssessCommand(NominalHeader(), {}).error,
            ThrusterArrayError::kEmptyArray);

  ThrusterCommandElementView empty_name = CommandElement(1.0);
  empty_name.name = "";
  std::vector<ThrusterCommandElementView> named = {empty_name,
                                                   CommandElement(kNaN)};
  EXPECT_EQ(AssessCommand(NominalHeader(), named).error,
            ThrusterArrayError::kEmptyName);

  ThrusterCommandElementView missing_thrust;
  missing_thrust.name_present = true;
  missing_thrust.name = "surge_port";
  std::vector<ThrusterCommandElementView> no_thrust = {missing_thrust};
  EXPECT_EQ(AssessCommand(NominalHeader(), no_thrust).error,
            ThrusterArrayError::kMissingThrust);

  for (double value : {kNaN, kInf, -kInf}) {
    std::vector<ThrusterCommandElementView> bad = {CommandElement(value)};
    const ThrusterArrayAssessment assessment =
        AssessCommand(NominalHeader(), bad);
    EXPECT_EQ(assessment.error, ThrusterArrayError::kNonFinite);
    EXPECT_FALSE(assessment.accepted);
  }
}

TEST(ThrusterArrayPolicyTest, InvalidHeaderValidityDoesNotReject) {
  ThrusterHeaderView header = NominalHeader();
  header.header_validity_state = 2;
  const std::vector<ThrusterCommandElementView> elements = {CommandElement(1)};
  const ThrusterArrayAssessment assessment = AssessCommand(header, elements);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kInvalid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(ThrusterArrayPolicyTest, UnsetNameIsAbsent) {
  ThrusterCommandElementView element = CommandElement(3.0);
  element.name_present = false;
  element.name = "";
  const std::vector<ThrusterCommandElementView> elements = {element};
  EXPECT_TRUE(AssessCommand(NominalHeader(), elements).accepted);
}

TEST(ThrusterArrayPolicyTest, NominalFeedbackDerateAndEfficiencyBoundaries) {
  const ThrusterHeaderView header = NominalHeader();
  std::vector<ThrusterFeedbackElementView> nominal = {FeedbackElement(0.0)};
  EXPECT_TRUE(AssessFeedback(header, nominal).accepted);

  std::vector<ThrusterFeedbackElementView> derated = {
      FeedbackElement(5.0, /*health=*/2, /*derate=*/0.5, /*efficiency=*/1.0)};
  EXPECT_TRUE(AssessFeedback(header, derated).accepted);

  std::vector<ThrusterFeedbackElementView> efficiency = {
      FeedbackElement(5.0, /*health=*/2, /*derate=*/0.25, /*efficiency=*/0.25)};
  EXPECT_TRUE(AssessFeedback(header, efficiency).accepted);

  for (int health : {1, 3, 4}) {
    std::vector<ThrusterFeedbackElementView> neutral = {
        FeedbackElement(0.0, health, 0.0, 1.0)};
    EXPECT_TRUE(AssessFeedback(header, neutral).accepted) << health;
  }
}

TEST(ThrusterArrayPolicyTest, FeedbackMetadataDefects) {
  const ThrusterHeaderView header = NominalHeader();
  ThrusterFeedbackElementView element = FeedbackElement(1.0);
  element.commanded_thrust_n = kNaN;
  std::vector<ThrusterFeedbackElementView> commanded = {element};
  EXPECT_EQ(AssessFeedback(header, commanded).error,
            ThrusterArrayError::kNonFinite);

  element = FeedbackElement(1.0);
  element.measured_thrust_n = kInf;
  std::vector<ThrusterFeedbackElementView> measured = {element};
  EXPECT_EQ(AssessFeedback(header, measured).error,
            ThrusterArrayError::kNonFinite);

  element = FeedbackElement(1.0);
  element.health_derate = kNaN;
  std::vector<ThrusterFeedbackElementView> derate = {element};
  EXPECT_EQ(AssessFeedback(header, derate).error,
            ThrusterArrayError::kNonFinite);

  element = FeedbackElement(1.0);
  element.efficiency = kNaN;
  std::vector<ThrusterFeedbackElementView> efficiency = {element};
  EXPECT_EQ(AssessFeedback(header, efficiency).error,
            ThrusterArrayError::kNonFinite);

  element = FeedbackElement(1.0, /*health=*/99, /*derate=*/0.0);
  std::vector<ThrusterFeedbackElementView> unknown = {element};
  EXPECT_EQ(AssessFeedback(header, unknown).error, ThrusterArrayError::kHealth);
  EXPECT_EQ(ClassifyThrusterHealth(true, 99),
            ThrusterHealthKind::kUnrecognized);
  EXPECT_EQ(ClassifyThrusterHealth(false, 0), ThrusterHealthKind::kAbsent);
  EXPECT_EQ(ClassifyThrusterHealth(true, 0), ThrusterHealthKind::kNominal);

  element = FeedbackElement(1.0, /*health=*/0, /*derate=*/0.5);
  std::vector<ThrusterFeedbackElementView> mismatch = {element};
  EXPECT_EQ(AssessFeedback(header, mismatch).error,
            ThrusterArrayError::kHealthDerate);

  element = FeedbackElement(1.0, /*health=*/2, /*derate=*/1.0);
  std::vector<ThrusterFeedbackElementView> derated_one = {element};
  EXPECT_EQ(AssessFeedback(header, derated_one).error,
            ThrusterArrayError::kHealthDerate);

  element = FeedbackElement(1.0, /*health=*/2, /*derate=*/0.0);
  std::vector<ThrusterFeedbackElementView> derated_zero = {element};
  EXPECT_EQ(AssessFeedback(header, derated_zero).error,
            ThrusterArrayError::kHealthDerate);

  element = FeedbackElement(1.0, /*health=*/1, /*derate=*/1.0);
  std::vector<ThrusterFeedbackElementView> disabled = {element};
  EXPECT_EQ(AssessFeedback(header, disabled).error,
            ThrusterArrayError::kHealthDerate);

  element = FeedbackElement(1.0);
  element.health_derate_present = false;
  std::vector<ThrusterFeedbackElementView> missing_derate = {element};
  EXPECT_EQ(AssessFeedback(header, missing_derate).error,
            ThrusterArrayError::kHealthDerate);

  element = FeedbackElement(1.0);
  element.efficiency = 0.0;
  std::vector<ThrusterFeedbackElementView> zero_eff = {element};
  EXPECT_EQ(AssessFeedback(header, zero_eff).error,
            ThrusterArrayError::kEfficiency);

  element = FeedbackElement(1.0);
  element.efficiency = 1.1;
  std::vector<ThrusterFeedbackElementView> high_eff = {element};
  EXPECT_EQ(AssessFeedback(header, high_eff).error,
            ThrusterArrayError::kEfficiency);
}

TEST(ThrusterArrayPolicyTest, FeedbackOptionalAbsenceAndFirstDefect) {
  ThrusterFeedbackElementView element;
  element.measured_thrust_present = true;
  element.measured_thrust_n = 0.0;
  const std::vector<ThrusterFeedbackElementView> sparse = {element};
  EXPECT_TRUE(AssessFeedback(NominalHeader(), sparse).accepted);

  ThrusterHeaderView empty_frame = NominalHeader();
  empty_frame.frame_id = "";
  ThrusterFeedbackElementView bad = FeedbackElement(1.0, 99, 0.0);
  bad.efficiency = 4.0;
  const std::vector<ThrusterFeedbackElementView> elements = {bad};
  EXPECT_EQ(AssessFeedback(empty_frame, elements).error,
            ThrusterArrayError::kMissingFrame);
  EXPECT_EQ(AssessFeedback(NominalHeader(), {}).error,
            ThrusterArrayError::kEmptyArray);
}

TEST(ThrusterArrayPolicyTest, HealthKindMatchesWireNumbers) {
  EXPECT_EQ(ClassifyThrusterHealth(true, 0), ThrusterHealthKind::kNominal);
  EXPECT_EQ(ClassifyThrusterHealth(true, 1), ThrusterHealthKind::kDisabled);
  EXPECT_EQ(ClassifyThrusterHealth(true, 2), ThrusterHealthKind::kDerated);
  EXPECT_EQ(ClassifyThrusterHealth(true, 3), ThrusterHealthKind::kStuckOff);
  EXPECT_EQ(ClassifyThrusterHealth(true, 4), ThrusterHealthKind::kFailed);
  EXPECT_TRUE(ThrusterDerateMatchesHealth(ThrusterHealthKind::kNominal, 1.0));
  EXPECT_TRUE(ThrusterDerateMatchesHealth(ThrusterHealthKind::kDerated, 0.5));
  EXPECT_TRUE(ThrusterDerateMatchesHealth(ThrusterHealthKind::kFailed, 0.0));
  EXPECT_TRUE(ThrusterEfficiencyInRange(1.0));
  EXPECT_FALSE(ThrusterEfficiencyInRange(0.0));
}

} // namespace
} // namespace intrinsic::hardware::marine
