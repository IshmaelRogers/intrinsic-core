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

#include <array>
#include <cstdint>
#include <string>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/perception/sonar/observation_bundle_contract_policy.h"

namespace intrinsic::perception::sonar {
namespace {

using embodiment::ValidityKind;
using Modality = ObservationModality;
using Reason = AbsentModalityReason;
using Error = ObservationBundleContractError;

constexpr int64_t kSeconds = 1700000000;
constexpr int32_t kSkewNanos = kFixtureMaxSkewNanos;
const char* kSnapshot =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

int Mod(Modality modality) { return static_cast<int>(modality); }
int Why(Reason reason) { return static_cast<int>(reason); }

ObservationBundleView Base() {
  ObservationBundleView bundle;
  bundle.header_present = true;
  bundle.validity_present = true;
  bundle.validity_state = 1;
  bundle.frame_id = "sync";
  bundle.clock_domain = "monotonic";
  bundle.source_time_present = true;
  bundle.source_time = {kSeconds, 0};
  bundle.receive_time_present = true;
  bundle.receive_time = {kSeconds, 100000000};
  bundle.max_skew_present = true;
  bundle.max_skew_seconds = 0;
  bundle.max_skew_nanos = kSkewNanos;
  return bundle;
}

ObservationSlotView Slot(const char* id, const char* frame, int32_t nanos) {
  ObservationSlotView slot;
  slot.reference_id = id;
  slot.content_type = "application/octet-stream";
  slot.byte_size_present = true;
  slot.byte_size = 16;
  slot.frame_id = frame;
  slot.source_time_present = true;
  slot.source_time = {kSeconds, nanos};
  slot.receive_time_present = true;
  slot.receive_time = {kSeconds, nanos};
  slot.clock_domain = "monotonic";
  return slot;
}

StateReferenceView State(const char* frame, int32_t nanos,
                         const char* snapshot = "") {
  StateReferenceView state;
  state.message_set = true;
  state.frame_id = frame;
  state.source_time_present = true;
  state.source_time = {kSeconds, nanos};
  state.receive_time_present = true;
  state.receive_time = {kSeconds, nanos};
  state.clock_domain = "monotonic";
  state.state_epoch = 0;
  state.world_snapshot_id = snapshot;
  return state;
}

std::array<AbsentModalityEntryView, 4> ExceptFls() {
  return {{
      {Mod(Modality::kSonarSss), Why(Reason::kNotConfigured)},
      {Mod(Modality::kOptical), Why(Reason::kSensorOffline)},
      {Mod(Modality::kPointCloud), Why(Reason::kOutOfRange)},
      {Mod(Modality::kVehicleState), Why(Reason::kIntentionallyOmitted)},
  }};
}

std::array<AbsentModalityEntryView, 4> ExceptOptical() {
  return {{
      {Mod(Modality::kSonarFls), Why(Reason::kNotConfigured)},
      {Mod(Modality::kSonarSss), Why(Reason::kSensorOffline)},
      {Mod(Modality::kPointCloud), Why(Reason::kDroppedForSkew)},
      {Mod(Modality::kVehicleState), Why(Reason::kIntentionallyOmitted)},
  }};
}

std::array<AbsentModalityEntryView, 5> AllAbsent() {
  return {{
      {Mod(Modality::kSonarFls), Why(Reason::kNotConfigured)},
      {Mod(Modality::kSonarSss), Why(Reason::kSensorOffline)},
      {Mod(Modality::kOptical), Why(Reason::kOutOfRange)},
      {Mod(Modality::kPointCloud), Why(Reason::kDroppedForSkew)},
      {Mod(Modality::kVehicleState), Why(Reason::kIntentionallyOmitted)},
  }};
}

ObservationBundleContractAssessment Assess(const ObservationBundleView& view) {
  return AssessMultimodalObservationBundle(view);
}

void ExpectAccepted(const ObservationBundleView& view) {
  const ObservationBundleContractAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

void ExpectError(const ObservationBundleView& view, Error error) {
  const ObservationBundleContractAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.error, error);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_FALSE(assessment.accepted);
}

TEST(ObservationBundleContractTest, ErrorAndEnumValuesAreLocked) {
  EXPECT_EQ(static_cast<int>(Error::kNone), 0);
  EXPECT_EQ(static_cast<int>(Error::kMissingFrame), 1);
  EXPECT_EQ(static_cast<int>(Error::kMaxSkew), 2);
  EXPECT_EQ(static_cast<int>(Error::kSlot), 3);
  EXPECT_EQ(static_cast<int>(Error::kAbsentList), 4);
  EXPECT_EQ(static_cast<int>(Error::kClockDomain), 5);
  EXPECT_EQ(static_cast<int>(Error::kFrameMismatch), 6);
  EXPECT_EQ(static_cast<int>(Error::kExcessiveSkew), 7);
  EXPECT_EQ(static_cast<int>(Error::kTimeReversal), 8);
  EXPECT_EQ(static_cast<int>(Error::kSnapshotId), 9);
  EXPECT_EQ(static_cast<int>(Error::kReference), 10);
  EXPECT_EQ(static_cast<int>(Modality::kUnspecified), 0);
  EXPECT_EQ(static_cast<int>(Modality::kSonarFls), 1);
  EXPECT_EQ(static_cast<int>(Modality::kSonarSss), 2);
  EXPECT_EQ(static_cast<int>(Modality::kOptical), 3);
  EXPECT_EQ(static_cast<int>(Modality::kPointCloud), 4);
  EXPECT_EQ(static_cast<int>(Modality::kVehicleState), 5);
  EXPECT_EQ(static_cast<int>(Reason::kUnspecified), 0);
  EXPECT_EQ(static_cast<int>(Reason::kNotConfigured), 1);
  EXPECT_EQ(static_cast<int>(Reason::kSensorOffline), 2);
  EXPECT_EQ(static_cast<int>(Reason::kOutOfRange), 3);
  EXPECT_EQ(static_cast<int>(Reason::kDroppedForSkew), 4);
  EXPECT_EQ(static_cast<int>(Reason::kIntentionallyOmitted), 5);
  EXPECT_EQ(kFixtureMaxSkewSeconds, 0);
  EXPECT_EQ(kFixtureMaxSkewNanos, 200000000);
}

TEST(ObservationBundleContractTest, EmptyIsNotEngaged) {
  const ObservationBundleView empty;
  EXPECT_FALSE(ObservationBundleEngaged(empty));
  const ObservationBundleContractAssessment assessment = Assess(empty);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_FALSE(assessment.measured_skew_present);
  EXPECT_EQ(assessment.measured_skew.seconds, 0);
  EXPECT_EQ(assessment.measured_skew.nanos, 0);
}

TEST(ObservationBundleContractTest, SonarOnlyCameraOnlyAndAllModalities) {
  const auto sonar_absent = ExceptFls();
  ObservationBundleView sonar = Base();
  sonar.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  sonar.absent = sonar_absent;
  sonar.metadata_present = true;
  ExpectAccepted(sonar);
  const ObservationBundleContractAssessment sonar_assessment = Assess(sonar);
  EXPECT_TRUE(sonar_assessment.measured_skew_present);
  EXPECT_EQ(sonar_assessment.measured_skew.seconds, 0);
  EXPECT_EQ(sonar_assessment.measured_skew.nanos, 0);

  const auto camera_absent = ExceptOptical();
  ObservationBundleView camera = Base();
  camera.optical = Slot("optical-frame-0001", "optical_cam", 50000000);
  camera.absent = camera_absent;
  ExpectAccepted(camera);

  ObservationBundleView all = Base();
  all.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  all.sss = Slot("sss-frame-0001", "sss_sonar", 50000000);
  all.optical = Slot("optical-frame-0001", "optical_cam", 100000000);
  all.point_cloud = Slot("points-0001", "points", 150000000);
  all.vehicle_state = State("body", 180000000, kSnapshot);
  all.vehicle_state.state_epoch = 42;
  ExpectAccepted(all);
  const ObservationBundleContractAssessment all_assessment = Assess(all);
  EXPECT_TRUE(all_assessment.measured_skew_present);
  EXPECT_EQ(all_assessment.measured_skew.seconds, 0);
  EXPECT_EQ(all_assessment.measured_skew.nanos, 180000000);
}

TEST(ObservationBundleContractTest, MissingModalityWithApprovedReason) {
  const auto absent = ExceptFls();
  ObservationBundleView bundle = Base();
  bundle.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  bundle.sss = Slot("sss-frame-0001", "sss_sonar", 1000000);
  // Optical is not present and not listed: a subset, not a defect.
  bundle.absent =
      std::span<const AbsentModalityEntryView>(absent.data() + 2, 2);
  ExpectAccepted(bundle);
}

TEST(ObservationBundleContractTest, AllAbsentWithReasonsIsAccepted) {
  const auto absent = AllAbsent();
  ObservationBundleView bundle = Base();
  bundle.absent = absent;
  ExpectAccepted(bundle);
  EXPECT_FALSE(Assess(bundle).measured_skew_present);
}

TEST(ObservationBundleContractTest, EngagedWithNothingPresentOrAbsentIsSlot) {
  ObservationBundleView bundle = Base();
  ExpectError(bundle, Error::kSlot);
  bundle.metadata_present = true;
  ExpectError(bundle, Error::kSlot);
}

TEST(ObservationBundleContractTest, AbsentListDefects) {
  const auto overlap = ExceptFls();
  ObservationBundleView present_and_absent = Base();
  present_and_absent.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  AbsentModalityEntryView overlapped[] = {
      {Mod(Modality::kSonarFls), Why(Reason::kNotConfigured)},
      overlap[0],
  };
  present_and_absent.absent = overlapped;
  ExpectError(present_and_absent, Error::kAbsentList);

  ObservationBundleView duplicate = Base();
  duplicate.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  AbsentModalityEntryView twice[] = {
      {Mod(Modality::kOptical), Why(Reason::kSensorOffline)},
      {Mod(Modality::kOptical), Why(Reason::kOutOfRange)},
      {Mod(Modality::kSonarSss), Why(Reason::kNotConfigured)},
      {Mod(Modality::kPointCloud), Why(Reason::kOutOfRange)},
      {Mod(Modality::kVehicleState), Why(Reason::kIntentionallyOmitted)},
  };
  duplicate.absent = twice;
  ExpectError(duplicate, Error::kAbsentList);

  ObservationBundleView unspecified = Base();
  unspecified.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  AbsentModalityEntryView bad_reason[] = {
      {Mod(Modality::kOptical), Why(Reason::kUnspecified)},
  };
  unspecified.absent = bad_reason;
  ExpectError(unspecified, Error::kAbsentList);

  AbsentModalityEntryView bad_modality[] = {
      {Mod(Modality::kUnspecified), Why(Reason::kNotConfigured)},
  };
  unspecified.absent = bad_modality;
  ExpectError(unspecified, Error::kAbsentList);

  AbsentModalityEntryView unknown[] = {
      {9, Why(Reason::kNotConfigured)},
  };
  unspecified.absent = unknown;
  ExpectError(unspecified, Error::kAbsentList);
}

TEST(ObservationBundleContractTest, ExactSkewIsOkAndOneNanosecondFails) {
  const auto absent = ExceptFls();
  // Drop SSS from the absent list because SSS is present. Keep the rest.
  std::array<AbsentModalityEntryView, 3> rest = {absent[1], absent[2],
                                                 absent[3]};
  ObservationBundleView exact = Base();
  exact.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  exact.sss = Slot("sss-frame-0001", "sss_sonar", kSkewNanos);
  exact.absent = rest;
  ExpectAccepted(exact);
  EXPECT_EQ(Assess(exact).measured_skew.nanos, kSkewNanos);

  ObservationBundleView inside = exact;
  inside.sss.source_time.nanos = kSkewNanos - 1;
  inside.sss.receive_time.nanos = kSkewNanos - 1;
  ExpectAccepted(inside);

  ObservationBundleView outside = exact;
  outside.sss.source_time.nanos = kSkewNanos + 1;
  outside.sss.receive_time.nanos = kSkewNanos + 1;
  const ObservationBundleContractAssessment assessment = Assess(outside);
  EXPECT_EQ(assessment.error, Error::kExcessiveSkew);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_TRUE(assessment.measured_skew_present);
  EXPECT_EQ(assessment.measured_skew.seconds, 0);
  EXPECT_EQ(assessment.measured_skew.nanos, kSkewNanos + 1);
}

TEST(ObservationBundleContractTest, ZeroMaxSkewAllowsEqualTimesOnly) {
  const auto absent = ExceptFls();
  ObservationBundleView equal = Base();
  equal.max_skew_nanos = 0;
  equal.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  equal.sss = Slot("sss-frame-0001", "sss_sonar", 0);
  std::array<AbsentModalityEntryView, 3> rest = {absent[1], absent[2],
                                                 absent[3]};
  equal.absent = rest;
  ExpectAccepted(equal);

  equal.sss.source_time.nanos = 1;
  equal.sss.receive_time.nanos = 1;
  ExpectError(equal, Error::kExcessiveSkew);
}

TEST(ObservationBundleContractTest, FrameMismatchAndSensorFrames) {
  std::array<AbsentModalityEntryView, 2> rest = {{
      {Mod(Modality::kPointCloud), Why(Reason::kNotConfigured)},
      {Mod(Modality::kVehicleState), Why(Reason::kIntentionallyOmitted)},
  }};
  ObservationBundleView mismatch = Base();
  mismatch.fls = Slot("fls-frame-0001", embodiment::kWorldEnuFrameId.data(), 0);
  mismatch.optical =
      Slot("optical-frame-0001", embodiment::kWorldNedFrameId.data(), 1000);
  mismatch.absent = rest;
  // SSS omitted from both sides is still a subset. The defect is the frames.
  ExpectError(mismatch, Error::kFrameMismatch);

  mismatch.optical.frame_id = embodiment::kWorldEnuFrameId;
  ExpectAccepted(mismatch);

  mismatch.fls.frame_id = "fls_sonar";
  mismatch.optical.frame_id = "optical_cam";
  ExpectAccepted(mismatch);

  mismatch.vehicle_state = State(embodiment::kWorldNedFrameId.data(), 2000);
  mismatch.fls.frame_id = embodiment::kWorldEnuFrameId.data();
  // State is not an observation slot, so its world id does not mismatch.
  std::array<AbsentModalityEntryView, 2> without_state = {{
      {Mod(Modality::kSonarSss), Why(Reason::kNotConfigured)},
      {Mod(Modality::kPointCloud), Why(Reason::kNotConfigured)},
  }};
  mismatch.absent = without_state;
  ExpectAccepted(mismatch);
}

TEST(ObservationBundleContractTest, TimeReversalAndEqualReceive) {
  const auto absent = ExceptFls();
  ObservationBundleView bundle = Base();
  bundle.fls = Slot("fls-frame-0001", "fls_sonar", 100000000);
  bundle.fls.receive_time.nanos = 50000000;
  bundle.absent = absent;
  ExpectError(bundle, Error::kTimeReversal);

  bundle.fls.receive_time.nanos = 100000000;
  ExpectAccepted(bundle);

  bundle.fls.receive_time_present = false;
  ExpectAccepted(bundle);

  bundle.fls.receive_time_present = true;
  bundle.fls.receive_time.nanos = 100000000;
  bundle.receive_time = {kSeconds, 0};
  bundle.source_time = {kSeconds, 1};
  ExpectError(bundle, Error::kTimeReversal);
}

TEST(ObservationBundleContractTest, SnapshotIdAndEpoch) {
  std::array<AbsentModalityEntryView, 4> absent = {{
      {Mod(Modality::kSonarFls), Why(Reason::kNotConfigured)},
      {Mod(Modality::kSonarSss), Why(Reason::kNotConfigured)},
      {Mod(Modality::kOptical), Why(Reason::kSensorOffline)},
      {Mod(Modality::kPointCloud), Why(Reason::kOutOfRange)},
  }};
  ObservationBundleView bundle = Base();
  bundle.vehicle_state = State("body", 0, kSnapshot);
  bundle.vehicle_state.state_epoch = 0;
  bundle.absent = absent;
  ExpectAccepted(bundle);

  bundle.vehicle_state.world_snapshot_id = "";
  ExpectAccepted(bundle);

  const std::string short_id(63, 'a');
  const std::string long_id(65, 'a');
  const std::string upper_id(64, 'A');
  const std::string non_hex(64, 'g');
  bundle.vehicle_state.world_snapshot_id = "not-a-hex";
  ExpectError(bundle, Error::kSnapshotId);
  bundle.vehicle_state.world_snapshot_id = short_id;
  ExpectError(bundle, Error::kSnapshotId);
  bundle.vehicle_state.world_snapshot_id = long_id;
  ExpectError(bundle, Error::kSnapshotId);
  bundle.vehicle_state.world_snapshot_id = upper_id;
  ExpectError(bundle, Error::kSnapshotId);
  bundle.vehicle_state.world_snapshot_id = non_hex;
  ExpectError(bundle, Error::kSnapshotId);

  EXPECT_TRUE(IsLowercaseSha256Hex(kSnapshot));
  EXPECT_TRUE(IsLowercaseSha256Hex(std::string(64, '0')));
  EXPECT_FALSE(IsLowercaseSha256Hex(""));
}

TEST(ObservationBundleContractTest, SlotAndReferenceDefects) {
  const auto absent = ExceptFls();
  ObservationBundleView bundle = Base();
  bundle.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  bundle.fls.frame_id = "";
  bundle.absent = absent;
  ExpectError(bundle, Error::kSlot);

  bundle.fls.frame_id = "fls_sonar";
  bundle.fls.source_time_present = false;
  ExpectError(bundle, Error::kSlot);

  bundle.fls.source_time_present = true;
  bundle.fls.source_time.nanos = 1000000000;
  ExpectError(bundle, Error::kSlot);

  bundle.fls.source_time.nanos = 0;
  bundle.fls.byte_size = 0;
  ExpectError(bundle, Error::kReference);

  bundle.fls.byte_size_present = false;
  bundle.fls.content_type = "";
  ExpectAccepted(bundle);
}

TEST(ObservationBundleContractTest, ClockDomainMustMatch) {
  const auto absent = ExceptFls();
  ObservationBundleView bundle = Base();
  bundle.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  bundle.fls.clock_domain = "utc";
  bundle.absent = absent;
  ExpectError(bundle, Error::kClockDomain);

  bundle.fls.clock_domain = "";
  ExpectError(bundle, Error::kClockDomain);

  bundle.clock_domain = "";
  bundle.fls.clock_domain = "";
  ExpectAccepted(bundle);

  bundle.vehicle_state = State("body", 0);
  bundle.vehicle_state.clock_domain = "monotonic";
  std::array<AbsentModalityEntryView, 3> rest = {absent[0], absent[1],
                                                 absent[2]};
  bundle.absent = rest;
  ExpectError(bundle, Error::kClockDomain);
}

TEST(ObservationBundleContractTest, MaxSkewAndMissingFrameOrder) {
  ObservationBundleView skew_only;
  skew_only.max_skew_present = true;
  skew_only.max_skew_seconds = 1;
  skew_only.validity_present = true;
  skew_only.validity_state = 1;
  ExpectError(skew_only, Error::kMissingFrame);

  ObservationBundleView bundle = Base();
  bundle.max_skew_present = false;
  ExpectError(bundle, Error::kMaxSkew);
  bundle.max_skew_present = true;
  bundle.max_skew_seconds = -1;
  ExpectError(bundle, Error::kMaxSkew);
  bundle.max_skew_seconds = 0;
  bundle.max_skew_nanos = -1;
  ExpectError(bundle, Error::kMaxSkew);
  bundle.max_skew_nanos = 1000000000;
  ExpectError(bundle, Error::kMaxSkew);

  bundle.max_skew_nanos = kSkewNanos;
  bundle.frame_id = "";
  bundle.fls = Slot("fls-frame-0001", "fls_sonar", kSkewNanos + 1);
  ExpectError(bundle, Error::kMissingFrame);
}

TEST(ObservationBundleContractTest, FirstDefectWins) {
  const auto absent = ExceptFls();
  ObservationBundleView bundle = Base();
  bundle.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  bundle.fls.frame_id = "";
  bundle.fls.byte_size = 0;
  bundle.absent = absent;
  ExpectError(bundle, Error::kSlot);

  bundle.fls.frame_id = "fls_sonar";
  bundle.fls.clock_domain = "utc";
  ExpectError(bundle, Error::kReference);

  std::array<AbsentModalityEntryView, 3> without_optical = {
      absent[0], absent[2], absent[3]};
  bundle.absent = without_optical;
  bundle.fls.byte_size = 16;
  bundle.optical = Slot("optical-frame-0001", "world_ned", 0);
  bundle.fls.frame_id = "world_enu";
  bundle.fls.clock_domain = "utc";
  ExpectError(bundle, Error::kClockDomain);

  bundle.fls.clock_domain = "monotonic";
  bundle.optical.clock_domain = "monotonic";
  bundle.fls.receive_time.nanos = 0;
  bundle.fls.source_time.nanos = 1;
  bundle.optical.source_time.nanos = kSkewNanos + 5;
  bundle.optical.receive_time.nanos = kSkewNanos + 5;
  ExpectError(bundle, Error::kFrameMismatch);

  bundle.optical.frame_id = "world_enu";
  ExpectError(bundle, Error::kTimeReversal);
}

TEST(ObservationBundleContractTest, HalfFilledSlotIsNotPresent) {
  ObservationBundleView bundle = Base();
  bundle.fls.frame_id = "fls_sonar";
  bundle.fls.source_time_present = true;
  bundle.fls.clock_domain = "utc";
  const auto absent = ExceptFls();
  bundle.absent = absent;
  // Empty reference_id means the slot is not present, so the clock and frame
  // on it are not assessed. The absent list covers FLS's siblings only, and
  // FLS itself is omitted: that subset is accepted.
  ExpectAccepted(bundle);
  EXPECT_FALSE(SlotPresent(bundle.fls));
}

TEST(ObservationBundleContractTest, StateWithoutFrameIsNotPresent) {
  std::array<AbsentModalityEntryView, 4> absent = ExceptFls();
  ObservationBundleView bundle = Base();
  bundle.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  bundle.vehicle_state.message_set = true;
  bundle.vehicle_state.world_snapshot_id = "not-a-hex";
  bundle.vehicle_state.state_epoch = 9;
  bundle.absent = absent;
  ExpectAccepted(bundle);
  EXPECT_FALSE(StatePresent(bundle.vehicle_state));
}

TEST(ObservationBundleContractTest, ValidityDoesNotRepairOrReplaceStructure) {
  const auto absent = ExceptFls();
  ObservationBundleView bundle = Base();
  bundle.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  bundle.absent = absent;
  bundle.validity_present = false;
  const ObservationBundleContractAssessment absent_validity = Assess(bundle);
  EXPECT_EQ(absent_validity.error, Error::kNone);
  EXPECT_EQ(absent_validity.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(absent_validity.accepted);

  bundle.validity_present = true;
  bundle.validity_state = 2;
  const ObservationBundleContractAssessment invalid = Assess(bundle);
  EXPECT_EQ(invalid.error, Error::kNone);
  EXPECT_EQ(invalid.validity, ValidityKind::kInvalid);
  EXPECT_FALSE(invalid.accepted);

  bundle.validity_state = 99;
  EXPECT_EQ(Assess(bundle).validity, ValidityKind::kUnspecified);
  EXPECT_FALSE(Assess(bundle).accepted);

  bundle.validity_state = 1;
  bundle.fls.byte_size = 0;
  const ObservationBundleContractAssessment structural = Assess(bundle);
  EXPECT_EQ(structural.error, Error::kReference);
  EXPECT_EQ(structural.validity, ValidityKind::kValid);
  EXPECT_FALSE(structural.accepted);
}

TEST(ObservationBundleContractTest, Determinism) {
  const auto absent = ExceptFls();
  ObservationBundleView first = Base();
  first.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  first.absent = absent;
  first.metadata_present = true;
  ObservationBundleView second = first;
  const ObservationBundleContractAssessment a = Assess(first);
  const ObservationBundleContractAssessment b = Assess(second);
  EXPECT_EQ(a.error, b.error);
  EXPECT_EQ(a.validity, b.validity);
  EXPECT_EQ(a.accepted, b.accepted);
  EXPECT_EQ(a.measured_skew_present, b.measured_skew_present);
  EXPECT_EQ(a.measured_skew.seconds, b.measured_skew.seconds);
  EXPECT_EQ(a.measured_skew.nanos, b.measured_skew.nanos);
  EXPECT_TRUE(a.accepted);

  const ObservationBundleContractAssessment again = Assess(first);
  EXPECT_EQ(again.error, a.error);
  EXPECT_EQ(again.measured_skew.nanos, a.measured_skew.nanos);
}

TEST(ObservationBundleContractTest, ZeroMaxSkewAloneDoesNotEngage) {
  ObservationBundleView bundle;
  bundle.max_skew_present = true;
  bundle.max_skew_seconds = 0;
  bundle.max_skew_nanos = 0;
  EXPECT_FALSE(ObservationBundleEngaged(bundle));
  EXPECT_FALSE(Assess(bundle).accepted);
  EXPECT_EQ(Assess(bundle).error, Error::kNone);
}

TEST(ObservationBundleContractTest, PointCloudOnlyAndStateOnly) {
  std::array<AbsentModalityEntryView, 4> except_points = {{
      {Mod(Modality::kSonarFls), Why(Reason::kNotConfigured)},
      {Mod(Modality::kSonarSss), Why(Reason::kNotConfigured)},
      {Mod(Modality::kOptical), Why(Reason::kSensorOffline)},
      {Mod(Modality::kVehicleState), Why(Reason::kIntentionallyOmitted)},
  }};
  ObservationBundleView points = Base();
  points.point_cloud = Slot("points-0001", "points", 0);
  points.point_cloud.byte_size_present = false;
  points.absent = except_points;
  ExpectAccepted(points);

  std::array<AbsentModalityEntryView, 4> except_state = {{
      {Mod(Modality::kSonarFls), Why(Reason::kNotConfigured)},
      {Mod(Modality::kSonarSss), Why(Reason::kSensorOffline)},
      {Mod(Modality::kOptical), Why(Reason::kOutOfRange)},
      {Mod(Modality::kPointCloud), Why(Reason::kDroppedForSkew)},
  }};
  ObservationBundleView state_only = Base();
  state_only.vehicle_state = State("body", 0, kSnapshot);
  state_only.vehicle_state.state_epoch = 7;
  state_only.absent = except_state;
  ExpectAccepted(state_only);
}

TEST(ObservationBundleContractTest, MetadataAloneIsEngaged) {
  ObservationBundleView bundle;
  bundle.metadata_present = true;
  const ObservationBundleContractAssessment assessment = Assess(bundle);
  EXPECT_EQ(assessment.error, Error::kMissingFrame);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(ObservationBundleContractTest, StateTimeParticipatesInSkew) {
  std::array<AbsentModalityEntryView, 3> absent = {{
      {Mod(Modality::kSonarSss), Why(Reason::kNotConfigured)},
      {Mod(Modality::kOptical), Why(Reason::kSensorOffline)},
      {Mod(Modality::kPointCloud), Why(Reason::kOutOfRange)},
  }};
  ObservationBundleView bundle = Base();
  bundle.fls = Slot("fls-frame-0001", "fls_sonar", 0);
  bundle.vehicle_state = State("body", kSkewNanos);
  bundle.absent = absent;
  ExpectAccepted(bundle);
  EXPECT_EQ(Assess(bundle).measured_skew.nanos, kSkewNanos);

  bundle.vehicle_state.source_time.nanos = kSkewNanos + 1;
  bundle.vehicle_state.receive_time.nanos = kSkewNanos + 1;
  ExpectError(bundle, Error::kExcessiveSkew);
}

TEST(ObservationBundleContractTest, AbsentOverlapBeatsABadSlot) {
  ObservationBundleView bundle = Base();
  bundle.fls = Slot("fls-frame-0001", "", 0);
  AbsentModalityEntryView overlap[] = {
      {Mod(Modality::kSonarFls), Why(Reason::kNotConfigured)},
  };
  bundle.absent = overlap;
  ExpectError(bundle, Error::kAbsentList);
}

}  // namespace
}  // namespace intrinsic::perception::sonar
