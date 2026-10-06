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

#include "intrinsic/scene/sdf/current_field_to_sdf.h"

#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <string_view>

#include "absl/status/status.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/world/current_field_component/current_field_component_policy.h"

namespace intrinsic::sdf {
namespace {

using ::intrinsic::world::CurrentFieldView;
using ::intrinsic::world::MarineComponentValidityView;
using ::intrinsic::world::TimeParts;
using ::testing::HasSubstr;

// Bazel runs tests from the runfiles root of the main repository.
constexpr char kGoldenPath[] =
    "intrinsic/scene/sdf/testdata/"
    "current_field_hydrodynamics_constant.golden.sdf";

MarineComponentValidityView ValidValidity() {
  MarineComponentValidityView view;
  view.present = true;
  view.source_id = "fusion_0";
  view.observation_time_present = true;
  view.observation_time = TimeParts{1700000000, 250000000};
  view.validity_horizon_present = true;
  view.validity_horizon = TimeParts{10, 500000000};
  view.confidence_present = true;
  view.confidence = 0.75;
  view.uncertainty_reference = "cov-ref-7";
  view.validity_present = true;
  view.validity_state = 1;
  return view;
}

CurrentFieldView ConstantCurrent() {
  CurrentFieldView view;
  view.present = true;
  view.validity = ValidValidity();
  view.frame_id = embodiment::kWorldEnuFrameId;
  view.velocity_present = true;
  view.velocity_m_s = embodiment::Vec3{0.2, -0.1, 0.0};
  return view;
}

// 1700000000.250s + 10.500s = 1700000010.750s.
TimeParts Deadline() { return TimeParts{1700000010, 750000000}; }

std::string ReadGolden() {
  std::ifstream file(kGoldenPath, std::ios::binary);
  EXPECT_TRUE(file.is_open()) << kGoldenPath;
  return std::string(std::istreambuf_iterator<char>(file),
                     std::istreambuf_iterator<char>());
}

TEST(CurrentFieldToSdfTest, AbsentComponentIsEmpty) {
  absl::StatusOr<std::string> sdf =
      CurrentFieldToSdf(CurrentFieldView{}, Deadline());
  ASSERT_TRUE(sdf.ok()) << sdf.status();
  EXPECT_TRUE(sdf->empty());
}

TEST(CurrentFieldToSdfTest, AbsentComponentIgnoresOtherFields) {
  CurrentFieldView view = ConstantCurrent();
  view.present = false;
  view.frame_id = "robot";
  absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_TRUE(sdf.ok()) << sdf.status();
  EXPECT_TRUE(sdf->empty());
}

TEST(CurrentFieldToSdfTest, ConstantCurrentMatchesGolden) {
  absl::StatusOr<std::string> sdf =
      CurrentFieldToSdf(ConstantCurrent(), Deadline());
  ASSERT_TRUE(sdf.ok()) << sdf.status();
  EXPECT_EQ(*sdf, ReadGolden());
}

TEST(CurrentFieldToSdfTest, OutputIsDeterministic) {
  absl::StatusOr<std::string> first =
      CurrentFieldToSdf(ConstantCurrent(), Deadline());
  absl::StatusOr<std::string> second =
      CurrentFieldToSdf(ConstantCurrent(), Deadline());
  ASSERT_TRUE(first.ok());
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(*first, *second);
}

TEST(CurrentFieldToSdfTest, DoesNotEmitValidity) {
  absl::StatusOr<std::string> sdf =
      CurrentFieldToSdf(ConstantCurrent(), Deadline());
  ASSERT_TRUE(sdf.ok());
  EXPECT_THAT(*sdf, ::testing::Not(HasSubstr("validity")));
  EXPECT_THAT(*sdf, ::testing::Not(HasSubstr("fusion_0")));
  EXPECT_THAT(*sdf, ::testing::Not(HasSubstr("cov-ref-7")));
}

TEST(CurrentFieldToSdfTest, EachAllowedFrameIsEmittedVerbatim) {
  CurrentFieldView view = ConstantCurrent();
  for (const std::string_view frame_id :
       {embodiment::kWorldEnuFrameId, embodiment::kWorldNedFrameId,
        world::kBodyFrameId}) {
    view.frame_id = frame_id;
    absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
    ASSERT_TRUE(sdf.ok()) << sdf.status();
    EXPECT_THAT(
        *sdf, HasSubstr("<frame_id>" + std::string(frame_id) + "</frame_id>"));
    EXPECT_THAT(*sdf, HasSubstr("<velocity_m_s>0.2 -0.1 0</velocity_m_s>"));
  }
}

TEST(CurrentFieldToSdfTest, NegativeZeroAndZeroRenderTheSame) {
  CurrentFieldView view = ConstantCurrent();
  view.velocity_m_s = embodiment::Vec3{-0.0, 0.0, -0.0};
  absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_TRUE(sdf.ok()) << sdf.status();
  EXPECT_THAT(*sdf, HasSubstr("<velocity_m_s>0 0 0</velocity_m_s>"));
}

TEST(CurrentFieldToSdfTest, FullPrecisionRoundTrips) {
  CurrentFieldView view = ConstantCurrent();
  view.velocity_m_s = embodiment::Vec3{1.0 / 3.0, -2.5, 1e-7};
  absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_TRUE(sdf.ok()) << sdf.status();
  EXPECT_THAT(*sdf, HasSubstr("<velocity_m_s>0.3333333333333333 -2.5 1e-07"
                              "</velocity_m_s>"));
}

TEST(CurrentFieldToSdfTest, InvalidFrameNamesFrameId) {
  CurrentFieldView view = ConstantCurrent();
  for (const std::string_view frame_id :
       {"", "enu", "ned", "world_ENU", "robot", "body "}) {
    view.frame_id = frame_id;
    absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
    ASSERT_FALSE(sdf.ok()) << frame_id;
    EXPECT_EQ(sdf.status().code(), absl::StatusCode::kInvalidArgument);
    EXPECT_THAT(sdf.status().message(), HasSubstr("current_field.frame_id"));
  }
}

TEST(CurrentFieldToSdfTest, MissingVelocityNamesVelocity) {
  CurrentFieldView view = ConstantCurrent();
  view.velocity_present = false;
  absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_FALSE(sdf.ok());
  EXPECT_EQ(sdf.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_THAT(sdf.status().message(),
              HasSubstr("current_field.velocity_m_s: missing"));
}

TEST(CurrentFieldToSdfTest, NonFiniteVelocityNamesVelocity) {
  CurrentFieldView view = ConstantCurrent();
  for (const embodiment::Vec3 velocity :
       {embodiment::Vec3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
        embodiment::Vec3{0.0, std::numeric_limits<double>::infinity(), 0.0},
        embodiment::Vec3{0.0, 0.0, -std::numeric_limits<double>::infinity()}}) {
    view.velocity_m_s = velocity;
    absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
    ASSERT_FALSE(sdf.ok());
    EXPECT_EQ(sdf.status().code(), absl::StatusCode::kInvalidArgument);
    EXPECT_THAT(sdf.status().message(),
                HasSubstr("current_field.velocity_m_s: components must be "
                          "finite"));
  }
}

TEST(CurrentFieldToSdfTest, MissingValidityNamesValidity) {
  CurrentFieldView view = ConstantCurrent();
  view.validity.present = false;
  absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_FALSE(sdf.ok());
  EXPECT_EQ(sdf.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_THAT(sdf.status().message(), HasSubstr("current_field.validity"));
}

TEST(CurrentFieldToSdfTest, StructurallyInvalidValidityNamesValidity) {
  CurrentFieldView view = ConstantCurrent();
  view.validity.source_id = "";
  absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_FALSE(sdf.ok());
  EXPECT_EQ(sdf.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_THAT(sdf.status().message(),
              HasSubstr("current_field.validity: structurally invalid"));
}

TEST(CurrentFieldToSdfTest, FirstDefectWins) {
  CurrentFieldView view = ConstantCurrent();
  view.validity.source_id = "";
  view.frame_id = "robot";
  view.velocity_present = false;
  absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_FALSE(sdf.ok());
  EXPECT_THAT(sdf.status().message(), HasSubstr("current_field.validity"));

  view.validity = ValidValidity();
  sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_FALSE(sdf.ok());
  EXPECT_THAT(sdf.status().message(), HasSubstr("current_field.frame_id"));

  view.frame_id = world::kBodyFrameId;
  sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_FALSE(sdf.ok());
  EXPECT_THAT(sdf.status().message(), HasSubstr("current_field.velocity_m_s"));
}

TEST(CurrentFieldToSdfTest, ExpiredValidityIsRejected) {
  absl::StatusOr<std::string> sdf = CurrentFieldToSdf(
      ConstantCurrent(), TimeParts{Deadline().seconds, Deadline().nanos + 1});
  ASSERT_FALSE(sdf.ok());
  EXPECT_EQ(sdf.status().code(), absl::StatusCode::kFailedPrecondition);
  EXPECT_THAT(sdf.status().message(), HasSubstr("current_field.validity"));
}

TEST(CurrentFieldToSdfTest, UnknownAndInvalidStatesAreRejected) {
  CurrentFieldView view = ConstantCurrent();
  view.validity.validity_present = false;
  absl::StatusOr<std::string> sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_FALSE(sdf.ok());
  EXPECT_EQ(sdf.status().code(), absl::StatusCode::kFailedPrecondition);

  view = ConstantCurrent();
  view.validity.validity_state = 2;
  sdf = CurrentFieldToSdf(view, Deadline());
  ASSERT_FALSE(sdf.ok());
  EXPECT_EQ(sdf.status().code(), absl::StatusCode::kFailedPrecondition);
}

TEST(CurrentFieldToSdfTest, DeadlineIsInclusive) {
  absl::StatusOr<std::string> sdf =
      CurrentFieldToSdf(ConstantCurrent(), Deadline());
  EXPECT_TRUE(sdf.ok()) << sdf.status();
}

}  // namespace
}  // namespace intrinsic::sdf
