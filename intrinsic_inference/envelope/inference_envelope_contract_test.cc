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

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"

namespace intrinsic::inference {
namespace {

using embodiment::ValidityKind;
using Error = InferenceEnvelopeContractError;

constexpr std::string_view kSnapshot =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

InferenceCommonView NominalCommon() {
  InferenceCommonView common;
  common.header_present = true;
  common.validity_present = true;
  common.validity_state = 1;
  common.frame_id = "world_enu";
  common.source_time_present = true;
  common.source_time = {1700000000, 0};
  common.state_epoch = 42;
  common.world_snapshot_id = kSnapshot;
  common.deadline_present = true;
  common.deadline = {1700000000, 500000000};
  common.validity_horizon_present = true;
  common.validity_horizon = {2, 0};
  common.confidence_present = true;
  common.confidence = 0.9;
  common.uncertainty_present = true;
  common.uncertainty = 0.05;
  common.input_digest = "sha256:in";
  common.provenance_present = true;
  common.provenance_model_id = "pose_estimator";
  common.metadata_present = true;
  return common;
}

OipIdentifierView NominalOip() {
  OipIdentifierView oip;
  oip.message_set = true;
  oip.model_name = "pose_estimator";
  oip.model_version = "3";
  oip.request_id = "req-0001";
  return oip;
}

InferenceEnvelopeView NominalEnvelope() {
  return InferenceEnvelopeView{NominalCommon(), NominalOip()};
}

InferenceResultView NominalResult() {
  return InferenceResultView{NominalCommon(), NominalOip(), "sha256:out"};
}

// Applies the same edit to the envelope and the result so every shared rule
// is checked on both entry points.
template <typename Edit>
void ExpectBoth(Edit edit, Error expected) {
  InferenceEnvelopeView envelope = NominalEnvelope();
  edit(envelope.common, envelope.oip_request);
  const auto envelope_assessment = AssessInferenceEnvelope(envelope);
  EXPECT_EQ(envelope_assessment.error, expected);
  EXPECT_EQ(envelope_assessment.accepted, expected == Error::kNone);

  InferenceResultView result = NominalResult();
  edit(result.common, result.oip_result);
  const auto result_assessment = AssessInferenceResult(result);
  EXPECT_EQ(result_assessment.error, expected);
  EXPECT_EQ(result_assessment.accepted, expected == Error::kNone);
}

TEST(InferenceEnvelopeContractTest, EmptyEnvelopeIsNotEngaged) {
  const auto assessment = AssessInferenceEnvelope(InferenceEnvelopeView{});
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(InferenceEnvelopeContractTest, EmptyResultIsNotEngaged) {
  const auto assessment = AssessInferenceResult(InferenceResultView{});
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(InferenceEnvelopeContractTest, StateEpochZeroAloneIsNotEngaged) {
  InferenceEnvelopeView view;
  view.common.state_epoch = 0;
  const auto assessment = AssessInferenceEnvelope(view);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_FALSE(assessment.accepted);
}

TEST(InferenceEnvelopeContractTest, AnySingleSignalEngages) {
  const auto engages = [](auto edit) {
    InferenceEnvelopeView view;
    edit(view.common);
    EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kMissingFrame);
  };
  engages([](InferenceCommonView& c) { c.header_present = true; });
  engages([](InferenceCommonView& c) { c.state_epoch = 1; });
  engages([](InferenceCommonView& c) { c.world_snapshot_id = kSnapshot; });
  engages([](InferenceCommonView& c) { c.deadline_present = true; });
  engages([](InferenceCommonView& c) { c.validity_horizon_present = true; });
  engages([](InferenceCommonView& c) { c.confidence_present = true; });
  engages([](InferenceCommonView& c) { c.uncertainty_present = true; });
  engages([](InferenceCommonView& c) { c.input_digest = "d"; });
  engages([](InferenceCommonView& c) { c.provenance_present = true; });
  engages([](InferenceCommonView& c) { c.metadata_present = true; });
}

TEST(InferenceEnvelopeContractTest, EmptyButSetOipMessageEngages) {
  InferenceEnvelopeView view;
  view.oip_request.message_set = true;
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kMissingFrame);
}

TEST(InferenceEnvelopeContractTest, OutputDigestAloneEngagesResult) {
  InferenceResultView view;
  view.output_digest = "sha256:out";
  EXPECT_EQ(AssessInferenceResult(view).error, Error::kMissingFrame);
}

TEST(InferenceEnvelopeContractTest, NominalEnvelopeIsAccepted) {
  const auto assessment = AssessInferenceEnvelope(NominalEnvelope());
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(InferenceEnvelopeContractTest, NominalResultIsAccepted) {
  const auto assessment = AssessInferenceResult(NominalResult());
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(InferenceEnvelopeContractTest, OptionalFieldsMayBeUnset) {
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) {
        c.confidence_present = false;
        c.uncertainty_present = false;
        c.world_snapshot_id = {};
        c.state_epoch = 0;
        c.metadata_present = false;
      },
      Error::kNone);
}

TEST(InferenceEnvelopeContractTest, EmptyModelVersionIsTolerated) {
  ExpectBoth([](InferenceCommonView&,
                OipIdentifierView& oip) { oip.model_version = {}; },
             Error::kNone);
}

TEST(InferenceEnvelopeContractTest, ValidityKindsOtherThanValidAreNotAccepted) {
  InferenceEnvelopeView view = NominalEnvelope();
  view.common.validity_present = false;
  auto assessment = AssessInferenceEnvelope(view);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);

  view = NominalEnvelope();
  view.common.validity_state = 0;
  assessment = AssessInferenceEnvelope(view);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kUnspecified);
  EXPECT_FALSE(assessment.accepted);

  view = NominalEnvelope();
  view.common.validity_state = 2;
  assessment = AssessInferenceEnvelope(view);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
}

TEST(InferenceEnvelopeContractTest, DefectWithValidHeaderIsNotAccepted) {
  InferenceEnvelopeView view = NominalEnvelope();
  view.common.frame_id = {};
  const auto assessment = AssessInferenceEnvelope(view);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_FALSE(assessment.accepted);
}

TEST(InferenceEnvelopeContractTest, MissingFrame) {
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) { c.frame_id = {}; },
      Error::kMissingFrame);
}

TEST(InferenceEnvelopeContractTest, CreationTime) {
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.source_time_present = false; },
             Error::kCreationTime);
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) {
        c.source_time = {1700000000, 1000000000};
      },
      Error::kCreationTime);
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.source_time = {1700000000, -1}; },
             Error::kCreationTime);
}

TEST(InferenceEnvelopeContractTest, DeadlineBeforeCreationIsRejected) {
  for (const embodiment::ClockReading deadline :
       {embodiment::ClockReading{1699999999, 999999999},
        embodiment::ClockReading{1699999999, 0},
        embodiment::ClockReading{0, 0}}) {
    ExpectBoth([&](InferenceCommonView& c,
                   OipIdentifierView&) { c.deadline = deadline; },
               Error::kDeadline);
  }
}

TEST(InferenceEnvelopeContractTest, DeadlineNanosBeforeCreationNanos) {
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) {
        c.source_time = {1700000000, 500};
        c.deadline = {1700000000, 499};
      },
      Error::kDeadline);
}

TEST(InferenceEnvelopeContractTest, DeadlineEqualToCreationIsAccepted) {
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) {
        c.source_time = {1700000000, 7};
        c.deadline = {1700000000, 7};
      },
      Error::kNone);
}

TEST(InferenceEnvelopeContractTest, DeadlineOneNanosecondAfterIsAccepted) {
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) {
        c.source_time = {1700000000, 7};
        c.deadline = {1700000000, 8};
      },
      Error::kNone);
}

TEST(InferenceEnvelopeContractTest, DeadlineMissingOrBadNanosIsRejected) {
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.deadline_present = false; },
             Error::kDeadline);
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.deadline = {1700000001, 1000000000}; },
             Error::kDeadline);
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.deadline = {1700000001, -1}; },
             Error::kDeadline);
}

TEST(InferenceEnvelopeContractTest, ValidityHorizon) {
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.validity_horizon_present = false; },
             Error::kValidityHorizon);
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.validity_horizon = {-1, 0}; },
             Error::kValidityHorizon);
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.validity_horizon = {0, -1}; },
             Error::kValidityHorizon);
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.validity_horizon = {1, 1000000000}; },
             Error::kValidityHorizon);
}

TEST(InferenceEnvelopeContractTest, ZeroValidityHorizonIsAccepted) {
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.validity_horizon = {0, 0}; },
             Error::kNone);
}

TEST(InferenceEnvelopeContractTest, OipIdentifier) {
  ExpectBoth(
      [](InferenceCommonView&, OipIdentifierView& oip) { oip.model_name = {}; },
      Error::kOipIdentifier);
  ExpectBoth(
      [](InferenceCommonView&, OipIdentifierView& oip) { oip.request_id = {}; },
      Error::kOipIdentifier);
  ExpectBoth([](InferenceCommonView&,
                OipIdentifierView& oip) { oip = OipIdentifierView{}; },
             Error::kOipIdentifier);
}

TEST(InferenceEnvelopeContractTest, Provenance) {
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.provenance_present = false; },
             Error::kProvenance);
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.provenance_model_id = {}; },
             Error::kProvenance);
}

TEST(InferenceEnvelopeContractTest, ConfidenceBoundaries) {
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) { c.confidence = 0.0; },
      Error::kNone);
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) { c.confidence = 1.0; },
      Error::kNone);
  for (const double bad : {-0.0000001, 1.0000001, 1.5, -1.0, std::nan(""),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
    ExpectBoth(
        [&](InferenceCommonView& c, OipIdentifierView&) { c.confidence = bad; },
        Error::kConfidence);
  }
}

TEST(InferenceEnvelopeContractTest, UnsetConfidenceIsNotZero) {
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) {
        c.confidence_present = false;
        c.confidence = 7.0;
      },
      Error::kNone);
}

TEST(InferenceEnvelopeContractTest, UncertaintyBoundaries) {
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) { c.uncertainty = 0.0; },
      Error::kNone);
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) { c.uncertainty = 1e9; },
      Error::kNone);
  for (const double bad :
       {-0.0000001, -1.0, std::nan(""), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    ExpectBoth([&](InferenceCommonView& c,
                   OipIdentifierView&) { c.uncertainty = bad; },
               Error::kUncertainty);
  }
}

TEST(InferenceEnvelopeContractTest, UnsetUncertaintyIsNotZero) {
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) {
        c.uncertainty_present = false;
        c.uncertainty = -5.0;
      },
      Error::kNone);
}

TEST(InferenceEnvelopeContractTest, Digests) {
  ExpectBoth(
      [](InferenceCommonView& c, OipIdentifierView&) { c.input_digest = {}; },
      Error::kDigest);

  InferenceResultView result = NominalResult();
  result.output_digest = {};
  EXPECT_EQ(AssessInferenceResult(result).error, Error::kDigest);
}

TEST(InferenceEnvelopeContractTest, EnvelopeDoesNotNeedOutputDigest) {
  EXPECT_TRUE(AssessInferenceEnvelope(NominalEnvelope()).accepted);
}

TEST(InferenceEnvelopeContractTest, SnapshotId) {
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.world_snapshot_id = {}; },
             Error::kNone);
  const std::string too_long = std::string(kSnapshot) + "0";
  const std::string too_short = std::string(kSnapshot.substr(0, 63));
  const std::string upper =
      "0123456789ABCDEF0123456789ABCDEF"
      "0123456789ABCDEF0123456789ABCDEF";
  const std::string non_hex = "g" + std::string(63, '0');
  const std::string spaced = " " + std::string(63, '0');
  for (const std::string& bad : {std::string("not-a-hex"), too_long, too_short,
                                 upper, non_hex, spaced}) {
    ExpectBoth([&](InferenceCommonView& c,
                   OipIdentifierView&) { c.world_snapshot_id = bad; },
               Error::kSnapshotId);
  }
}

TEST(InferenceEnvelopeContractTest, MetadataIsAlwaysTolerated) {
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.metadata_present = true; },
             Error::kNone);
  ExpectBoth([](InferenceCommonView& c,
                OipIdentifierView&) { c.metadata_present = false; },
             Error::kNone);
}

TEST(InferenceEnvelopeContractTest, StateEpochValuesAreTolerated) {
  for (const uint64_t epoch :
       {uint64_t{0}, uint64_t{1}, std::numeric_limits<uint64_t>::max()}) {
    ExpectBoth([&](InferenceCommonView& c,
                   OipIdentifierView&) { c.state_epoch = epoch; },
               Error::kNone);
  }
}

TEST(InferenceEnvelopeContractTest, FirstDefectWins) {
  // Break every step, then repair them one at a time in check order. Each
  // repair must reveal the next error in the locked order.
  InferenceEnvelopeView view = NominalEnvelope();
  view.common.frame_id = {};
  view.common.source_time_present = false;
  view.common.deadline_present = false;
  view.common.validity_horizon_present = false;
  view.oip_request = OipIdentifierView{true, {}, {}, {}};
  view.common.provenance_present = false;
  view.common.confidence = 2.0;
  view.common.uncertainty = -1.0;
  view.common.input_digest = {};
  view.common.world_snapshot_id = "bad";

  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kMissingFrame);
  view.common.frame_id = "world_enu";
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kCreationTime);
  view.common.source_time_present = true;
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kDeadline);
  view.common.deadline_present = true;
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kValidityHorizon);
  view.common.validity_horizon_present = true;
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kOipIdentifier);
  view.oip_request = NominalOip();
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kProvenance);
  view.common.provenance_present = true;
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kConfidence);
  view.common.confidence = 1.0;
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kUncertainty);
  view.common.uncertainty = 0.0;
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kDigest);
  view.common.input_digest = "sha256:in";
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kSnapshotId);
  view.common.world_snapshot_id = kSnapshot;
  EXPECT_EQ(AssessInferenceEnvelope(view).error, Error::kNone);
  EXPECT_TRUE(AssessInferenceEnvelope(view).accepted);
}

TEST(InferenceEnvelopeContractTest, ResultDigestWinsOverSnapshotId) {
  InferenceResultView result = NominalResult();
  result.output_digest = {};
  result.common.world_snapshot_id = "bad";
  EXPECT_EQ(AssessInferenceResult(result).error, Error::kDigest);
}

}  // namespace
}  // namespace intrinsic::inference
