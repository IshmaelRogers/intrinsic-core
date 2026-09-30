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

#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"

#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace intrinsic::world {
namespace {

// SHA-256 of "v1\nstate_epoch=42\nbathymetry_reference=3\ncurrent_field=7\n".
// Keep in sync with world_snapshot_policy_test.py.
constexpr char kGoldenSnapshotId[] =
    "b43197bb590584583288de254822664ae70709d0b9103698d296b2bc7ef30f7e";

// SHA-256 of "v1\nstate_epoch=0\n".
constexpr char kEpochOnlyZeroSnapshotId[] =
    "6c3dd0944e942bfa751163e5b542a230c499cf85ecd0f6ba2df501adfae04b8d";

WorldSnapshotView ValidView() {
  WorldSnapshotView view;
  view.present = true;
  view.snapshot_id = kGoldenSnapshotId;
  view.creation_time_present = true;
  view.creation_time = {1700000000, 250000000};
  view.state_epoch = 42;
  view.components = {{std::string(kBathymetryReferenceKind), 3},
                     {std::string(kCurrentFieldKind), 7}};
  return view;
}

TEST(WorldSnapshotPolicyTest, KnownKindStringsAreLocked) {
  EXPECT_EQ(kBathymetryReferenceKind, "bathymetry_reference");
  EXPECT_EQ(kCurrentFieldKind, "current_field");
  EXPECT_EQ(kOccupancyReferenceKind, "occupancy_reference");
  EXPECT_EQ(kSemanticContactsKind, "semantic_contacts");
}

TEST(WorldSnapshotPolicyTest, DigestInputIsExactlyTheLockedText) {
  EXPECT_EQ(SnapshotDigestInput(
                42, {{"current_field", 7}, {"bathymetry_reference", 3}}),
            "v1\nstate_epoch=42\nbathymetry_reference=3\ncurrent_field=7\n");
}

TEST(WorldSnapshotPolicyTest, DigestInputHasNoCreationTimeLine) {
  const std::string input = SnapshotDigestInput(1, {{"current_field", 2}});
  EXPECT_EQ(input.find("creation_time"), std::string::npos);
  EXPECT_EQ(input, "v1\nstate_epoch=1\ncurrent_field=2\n");
}

TEST(WorldSnapshotPolicyTest, GoldenSnapshotId) {
  const std::string id = ComputeSnapshotId(
      42, {{"bathymetry_reference", 3}, {"current_field", 7}});
  EXPECT_EQ(id, kGoldenSnapshotId);
  EXPECT_EQ(id.size(), kSnapshotIdHexLength);
  for (char c : id) {
    EXPECT_TRUE((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')) << c;
  }
}

TEST(WorldSnapshotPolicyTest, SnapshotIdIgnoresInputOrder) {
  const std::string forward = ComputeSnapshotId(
      42, {{"bathymetry_reference", 3}, {"current_field", 7}});
  const std::string reversed = ComputeSnapshotId(
      42, {{"current_field", 7}, {"bathymetry_reference", 3}});
  EXPECT_EQ(forward, reversed);
}

TEST(WorldSnapshotPolicyTest, SortIsByteWiseAscending) {
  // "Z" (0x5a) sorts before "a" (0x61) and "_" (0x5f) before "a".
  EXPECT_EQ(SnapshotDigestInput(0, {{"a", 1}, {"_b", 2}, {"Z", 3}}),
            "v1\nstate_epoch=0\nZ=3\n_b=2\na=1\n");
}

TEST(WorldSnapshotPolicyTest, EpochOnlySnapshotIsHashable) {
  EXPECT_EQ(ComputeSnapshotId(0, {}), kEpochOnlyZeroSnapshotId);
  EXPECT_EQ(SnapshotDigestInput(0, {}), "v1\nstate_epoch=0\n");
}

TEST(WorldSnapshotPolicyTest, EpochAndRevisionsChangeTheId) {
  const std::string base = ComputeSnapshotId(1, {{"current_field", 1}});
  EXPECT_NE(base, ComputeSnapshotId(2, {{"current_field", 1}}));
  EXPECT_NE(base, ComputeSnapshotId(1, {{"current_field", 2}}));
  EXPECT_NE(base, ComputeSnapshotId(1, {{"occupancy_reference", 1}}));
  EXPECT_NE(base, ComputeSnapshotId(1, {}));
}

TEST(WorldSnapshotPolicyTest, RevisionZeroIsDistinctFromAbsent) {
  EXPECT_NE(ComputeSnapshotId(1, {{"current_field", 0}}),
            ComputeSnapshotId(1, {}));
  EXPECT_EQ(SnapshotDigestInput(1, {{"current_field", 0}}),
            "v1\nstate_epoch=1\ncurrent_field=0\n");
}

TEST(WorldSnapshotPolicyTest, LargeUnsignedValuesAreDecimal) {
  EXPECT_EQ(SnapshotDigestInput(UINT64_MAX, {{"k", UINT64_MAX}}),
            "v1\nstate_epoch=18446744073709551615\nk=18446744073709551615\n");
}

TEST(WorldSnapshotPolicyTest, UnknownKindParticipatesLikeKnownKinds) {
  EXPECT_EQ(SnapshotDigestInput(1, {{"vendor_x", 9}, {"current_field", 2}}),
            "v1\nstate_epoch=1\ncurrent_field=2\nvendor_x=9\n");
}

TEST(WorldSnapshotPolicyTest, EmptyViewIsNotPresentAndNotAccepted) {
  const SnapshotAssessment assessment = AssessWorldSnapshot({});
  EXPECT_EQ(assessment.error, SnapshotError::kNone);
  EXPECT_FALSE(assessment.accepted);
}

TEST(WorldSnapshotPolicyTest, ValidViewIsAccepted) {
  const SnapshotAssessment assessment = AssessWorldSnapshot(ValidView());
  EXPECT_EQ(assessment.error, SnapshotError::kNone);
  EXPECT_EQ(assessment.component_index, -1);
  EXPECT_TRUE(assessment.accepted);
}

TEST(WorldSnapshotPolicyTest, EpochOnlyViewIsAccepted) {
  WorldSnapshotView view = ValidView();
  view.components.clear();
  view.state_epoch = 0;
  view.snapshot_id = kEpochOnlyZeroSnapshotId;
  EXPECT_TRUE(AssessWorldSnapshot(view).accepted);
}

TEST(WorldSnapshotPolicyTest, EmptySnapshotIdIsRejected) {
  WorldSnapshotView view = ValidView();
  view.snapshot_id.clear();
  const SnapshotAssessment assessment = AssessWorldSnapshot(view);
  EXPECT_EQ(assessment.error, SnapshotError::kSnapshotId);
  EXPECT_FALSE(assessment.accepted);
}

TEST(WorldSnapshotPolicyTest, MissingCreationTimeIsRejected) {
  WorldSnapshotView view = ValidView();
  view.creation_time_present = false;
  EXPECT_EQ(AssessWorldSnapshot(view).error, SnapshotError::kCreationTime);
}

TEST(WorldSnapshotPolicyTest, BadCreationNanosAreRejected) {
  for (int32_t nanos : {-1, 1000000000, 2000000000}) {
    WorldSnapshotView view = ValidView();
    view.creation_time.nanos = nanos;
    EXPECT_EQ(AssessWorldSnapshot(view).error, SnapshotError::kCreationTime)
        << nanos;
  }
  for (int32_t nanos : {0, 999999999}) {
    WorldSnapshotView view = ValidView();
    view.creation_time.nanos = nanos;
    EXPECT_TRUE(AssessWorldSnapshot(view).accepted) << nanos;
  }
}

TEST(WorldSnapshotPolicyTest, EmptyKindIsRejectedWithIndex) {
  WorldSnapshotView view = ValidView();
  view.components.push_back({"", 1});
  const SnapshotAssessment assessment = AssessWorldSnapshot(view);
  EXPECT_EQ(assessment.error, SnapshotError::kEmptyComponentKind);
  EXPECT_EQ(assessment.component_index, 2);
}

TEST(WorldSnapshotPolicyTest, DuplicateKindIsRejectedAtSecondOccurrence) {
  WorldSnapshotView view = ValidView();
  view.components.push_back({"current_field", 9});
  const SnapshotAssessment assessment = AssessWorldSnapshot(view);
  EXPECT_EQ(assessment.error, SnapshotError::kDuplicateComponentKind);
  EXPECT_EQ(assessment.component_index, 2);
}

TEST(WorldSnapshotPolicyTest, FirstDefectInFieldOrderWins) {
  WorldSnapshotView view = ValidView();
  view.snapshot_id.clear();
  view.creation_time.nanos = -1;
  view.components.push_back({"", 1});
  EXPECT_EQ(AssessWorldSnapshot(view).error, SnapshotError::kSnapshotId);
  view.snapshot_id = kGoldenSnapshotId;
  EXPECT_EQ(AssessWorldSnapshot(view).error, SnapshotError::kCreationTime);
  view.creation_time.nanos = 0;
  EXPECT_EQ(AssessWorldSnapshot(view).error,
            SnapshotError::kEmptyComponentKind);
}

TEST(WorldSnapshotPolicyTest, FirstComponentDefectWinsWhenBothKindsOccur) {
  const SnapshotAssessment duplicate_first =
      AssessComponentKinds({{"a", 1}, {"a", 2}, {"", 3}});
  EXPECT_EQ(duplicate_first.error, SnapshotError::kDuplicateComponentKind);
  EXPECT_EQ(duplicate_first.component_index, 1);
  const SnapshotAssessment empty_first =
      AssessComponentKinds({{"a", 1}, {"", 2}, {"a", 3}});
  EXPECT_EQ(empty_first.error, SnapshotError::kEmptyComponentKind);
  EXPECT_EQ(empty_first.component_index, 1);
}

TEST(WorldSnapshotPolicyTest, ErrorNamesAreStable) {
  EXPECT_STREQ(SnapshotErrorName(SnapshotError::kNone), "none");
  EXPECT_STREQ(SnapshotErrorName(SnapshotError::kSnapshotId), "snapshot_id");
  EXPECT_STREQ(SnapshotErrorName(SnapshotError::kCreationTime),
               "creation_time");
  EXPECT_STREQ(SnapshotErrorName(SnapshotError::kEmptyComponentKind),
               "empty_component_kind");
  EXPECT_STREQ(SnapshotErrorName(SnapshotError::kDuplicateComponentKind),
               "duplicate_component_kind");
}

}  // namespace
}  // namespace intrinsic::world
