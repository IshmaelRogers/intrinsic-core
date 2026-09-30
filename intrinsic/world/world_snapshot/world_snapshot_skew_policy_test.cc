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

#include "intrinsic/world/world_snapshot/world_snapshot_skew_policy.h"

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"
#include "intrinsic/world/world_snapshot/world_snapshot_builder.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_assessor.h"

namespace intrinsic::world {
namespace {

using intrinsic_proto::world::WorldSnapshotDescriptor;
using Kinds = std::vector<std::string>;

constexpr char kBathymetry[] = "bathymetry_reference";
constexpr char kCurrent[] = "current_field";
constexpr char kOccupancy[] = "occupancy_reference";
constexpr char kContacts[] = "semantic_contacts";

constexpr int64_t kBaseSeconds = 1700000000;
constexpr int64_t kMs = 1000000;
constexpr int64_t kSecond = 1000000000;

// Time at kBaseSeconds plus offset_ns.
TimeParts At(int64_t offset_ns) {
  int64_t seconds = kBaseSeconds + offset_ns / kSecond;
  int64_t nanos = offset_ns % kSecond;
  if (nanos < 0) {
    nanos += kSecond;
    --seconds;
  }
  return {seconds, static_cast<int32_t>(nanos)};
}

WorldSnapshotSkewPolicy TestPolicy() {
  WorldSnapshotSkewPolicy policy;
  policy.required_kinds = {kBathymetry, kCurrent};
  policy.optional_kinds = {kOccupancy, kContacts};
  return policy;
}

WorldSnapshotView ViewOf(const Kinds& kinds) {
  WorldSnapshotView view;
  view.present = true;
  view.snapshot_id = "id";
  view.creation_time_present = true;
  view.creation_time = {kBaseSeconds, 0};
  for (const std::string& kind : kinds) {
    view.components.push_back({kind, 1});
  }
  return view;
}

WorldSnapshotView AllKinds() {
  return ViewOf({kBathymetry, kCurrent, kOccupancy, kContacts});
}

ComponentTiming Timing(const std::string& kind, int64_t observation_offset_ns,
                       TimeParts horizon = {10, 0}) {
  return {kind, At(observation_offset_ns), horizon, "source"};
}

ComponentTimings AllTimings(int64_t offset_ns = 0) {
  return {Timing(kBathymetry, offset_ns), Timing(kCurrent, offset_ns),
          Timing(kOccupancy, offset_ns), Timing(kContacts, offset_ns)};
}

bool SameTime(TimeParts a, TimeParts b) {
  return a.seconds == b.seconds && a.nanos == b.nanos;
}

bool SameAssessment(const WorldSnapshotPolicyAssessment& a,
                    const WorldSnapshotPolicyAssessment& b) {
  return a.status == b.status && a.error == b.error &&
         a.descriptor_error == b.descriptor_error && a.accepted == b.accepted &&
         a.withhold == b.withhold && a.offending_kinds == b.offending_kinds &&
         a.missing_optional_kinds == b.missing_optional_kinds &&
         a.measured_skew_present == b.measured_skew_present &&
         SameTime(a.measured_skew, b.measured_skew) &&
         a.earliest_kind == b.earliest_kind && a.latest_kind == b.latest_kind;
}

TEST(SkewPolicyTest, DefaultsAre200MsSkewAnd2SecondAge) {
  const WorldSnapshotSkewPolicy policy;
  EXPECT_TRUE(SameTime(policy.max_skew, {0, 200000000}));
  EXPECT_TRUE(SameTime(policy.max_age, {2, 0}));
  EXPECT_TRUE(policy.required_kinds.empty());
  EXPECT_TRUE(policy.optional_kinds.empty());
}

TEST(SkewPolicyTest, StatusNumbersAreLocked) {
  EXPECT_EQ(static_cast<int>(SnapshotPolicyStatus::kUnspecified), 0);
  EXPECT_EQ(static_cast<int>(SnapshotPolicyStatus::kFresh), 1);
  EXPECT_EQ(static_cast<int>(SnapshotPolicyStatus::kPartial), 2);
  EXPECT_EQ(static_cast<int>(SnapshotPolicyStatus::kStale), 3);
  EXPECT_EQ(static_cast<int>(SnapshotPolicyStatus::kExcessiveSkew), 4);
  EXPECT_EQ(static_cast<int>(SnapshotPolicyStatus::kIncomplete), 5);
}

TEST(SkewPolicyTest, FreshWhenAllKindsPresentAndWithinBounds) {
  const SnapshotSkewAssessment result = AssessWorldSnapshotSkew(
      AllKinds(), AllTimings(), At(500 * kMs), TestPolicy());
  EXPECT_EQ(result.error, SnapshotPolicyError::kNone);
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kFresh);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.withhold);
  EXPECT_TRUE(result.offending_kinds.empty());
  EXPECT_TRUE(result.missing_optional_kinds.empty());
  EXPECT_TRUE(result.measured_skew_present);
  EXPECT_TRUE(SameTime(result.measured_skew, {0, 0}));
}

TEST(SkewPolicyTest, EmptyRequiredAndOptionalListsAreFresh) {
  const auto result =
      AssessWorldSnapshotSkew(ViewOf({}), {}, At(0), WorldSnapshotSkewPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kFresh);
  EXPECT_FALSE(result.measured_skew_present);
}

TEST(SkewPolicyTest, PermittedPartialWhenOptionalKindAbsent) {
  const auto result = AssessWorldSnapshotSkew(
      ViewOf({kBathymetry, kCurrent, kOccupancy}),
      {Timing(kBathymetry, 0), Timing(kCurrent, 50 * kMs),
       Timing(kOccupancy, 100 * kMs)},
      At(500 * kMs), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kPartial);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.withhold);
  EXPECT_TRUE(result.offending_kinds.empty());
  EXPECT_EQ(result.missing_optional_kinds, Kinds({kContacts}));
  EXPECT_TRUE(SameTime(result.measured_skew, {0, 100000000}));
}

TEST(SkewPolicyTest, AllOptionalKindsAbsentIsPartial) {
  const auto result = AssessWorldSnapshotSkew(
      ViewOf({kBathymetry, kCurrent}),
      {Timing(kBathymetry, 0), Timing(kCurrent, 0)}, At(0), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kPartial);
  EXPECT_EQ(result.missing_optional_kinds, Kinds({kOccupancy, kContacts}));
}

TEST(SkewPolicyTest, KindsOutsideRequiredAndOptionalAreIgnored) {
  WorldSnapshotView view = AllKinds();
  view.components.push_back({"custom_kind", 3});
  ComponentTimings timings = AllTimings();
  timings.push_back(Timing("custom_kind", -100 * kSecond, {0, 1}));
  const auto result =
      AssessWorldSnapshotSkew(view, timings, At(500 * kMs), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kFresh);
  EXPECT_TRUE(SameTime(result.measured_skew, {0, 0}));
}

TEST(SkewPolicyTest, TimingsForKindsAbsentFromDescriptorAreIgnored) {
  ComponentTimings timings = {Timing(kBathymetry, 0), Timing(kCurrent, 0),
                              Timing(kContacts, -50 * kSecond, {0, 1})};
  const auto result = AssessWorldSnapshotSkew(ViewOf({kBathymetry, kCurrent}),
                                              timings, At(0), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kPartial);
  EXPECT_TRUE(SameTime(result.measured_skew, {0, 0}));
}

TEST(SkewPolicyTest, HorizonBoundaryIsFreshAndNextNanosecondIsStale) {
  ComponentTimings timings = AllTimings();
  timings[1].validity_horizon = {0, 500000000};

  const auto at_deadline =
      AssessWorldSnapshotSkew(AllKinds(), timings, At(500 * kMs), TestPolicy());
  EXPECT_EQ(at_deadline.status, SnapshotPolicyStatus::kFresh);

  const auto past_deadline = AssessWorldSnapshotSkew(
      AllKinds(), timings, At(500 * kMs + 1), TestPolicy());
  EXPECT_EQ(past_deadline.status, SnapshotPolicyStatus::kStale);
  EXPECT_FALSE(past_deadline.accepted);
  EXPECT_TRUE(past_deadline.withhold);
  EXPECT_EQ(past_deadline.offending_kinds, Kinds({kCurrent}));
}

TEST(SkewPolicyTest, MaxAgeBoundaryIsFreshAndNextNanosecondIsStale) {
  const auto at_limit = AssessWorldSnapshotSkew(AllKinds(), AllTimings(),
                                                At(2 * kSecond), TestPolicy());
  EXPECT_EQ(at_limit.status, SnapshotPolicyStatus::kFresh);

  const auto past_limit = AssessWorldSnapshotSkew(
      AllKinds(), AllTimings(), At(2 * kSecond + 1), TestPolicy());
  EXPECT_EQ(past_limit.status, SnapshotPolicyStatus::kStale);
  EXPECT_TRUE(past_limit.withhold);
  EXPECT_EQ(past_limit.offending_kinds,
            Kinds({kBathymetry, kCurrent, kOccupancy, kContacts}));
}

TEST(SkewPolicyTest, MaxAgeIsEvaluatedPerKind) {
  ComponentTimings timings = AllTimings(1500 * kMs);
  timings[1].observation_time = At(0);
  const auto result =
      AssessWorldSnapshotSkew(AllKinds(), timings, At(2 * kSecond + 1), [] {
        WorldSnapshotSkewPolicy policy = TestPolicy();
        policy.max_skew = {5, 0};
        return policy;
      }());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kStale);
  EXPECT_EQ(result.offending_kinds, Kinds({kCurrent}));
}

TEST(SkewPolicyTest, ObservationAfterQueryTimeIsNotStale) {
  const auto result = AssessWorldSnapshotSkew(AllKinds(), AllTimings(100 * kMs),
                                              At(0), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kFresh);
}

TEST(SkewPolicyTest, PresentOptionalKindWithExpiredHorizonIsStale) {
  ComponentTimings timings = AllTimings();
  timings[2].validity_horizon = {0, 1};
  const auto result =
      AssessWorldSnapshotSkew(AllKinds(), timings, At(2), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kStale);
  EXPECT_EQ(result.offending_kinds, Kinds({kOccupancy}));
}

TEST(SkewPolicyTest, PresentKindWithoutTimingIsStale) {
  const auto required_missing = AssessWorldSnapshotSkew(
      AllKinds(),
      {Timing(kBathymetry, 0), Timing(kOccupancy, 0), Timing(kContacts, 0)},
      At(0), TestPolicy());
  EXPECT_EQ(required_missing.status, SnapshotPolicyStatus::kStale);
  EXPECT_EQ(required_missing.offending_kinds, Kinds({kCurrent}));

  const auto optional_missing = AssessWorldSnapshotSkew(
      AllKinds(),
      {Timing(kBathymetry, 0), Timing(kCurrent, 0), Timing(kOccupancy, 0)},
      At(0), TestPolicy());
  EXPECT_EQ(optional_missing.status, SnapshotPolicyStatus::kStale);
  EXPECT_EQ(optional_missing.offending_kinds, Kinds({kContacts}));
}

TEST(SkewPolicyTest, UnusableTimingIsStaleAndDoesNotJoinSkewSet) {
  ComponentTimings timings = AllTimings();
  timings[0].observation_time = {kBaseSeconds, 1000000000};
  const auto bad_observation =
      AssessWorldSnapshotSkew(AllKinds(), timings, At(0), TestPolicy());
  EXPECT_EQ(bad_observation.status, SnapshotPolicyStatus::kStale);
  EXPECT_EQ(bad_observation.offending_kinds, Kinds({kBathymetry}));

  timings = AllTimings();
  timings[0].validity_horizon = {-1, 0};
  const auto bad_horizon =
      AssessWorldSnapshotSkew(AllKinds(), timings, At(0), TestPolicy());
  EXPECT_EQ(bad_horizon.status, SnapshotPolicyStatus::kStale);
  EXPECT_EQ(bad_horizon.offending_kinds, Kinds({kBathymetry}));
}

TEST(SkewPolicyTest, SkewBoundaryIsWithinLimit) {
  ComponentTimings timings = AllTimings();
  timings[3].observation_time = At(200 * kMs);
  const auto result =
      AssessWorldSnapshotSkew(AllKinds(), timings, At(300 * kMs), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kFresh);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.withhold);
  EXPECT_TRUE(result.measured_skew_present);
  EXPECT_TRUE(SameTime(result.measured_skew, {0, 200000000}));
}

TEST(SkewPolicyTest, SkewOneNanosecondOverWithholdsAndNamesEarliestLatest) {
  ComponentTimings timings = AllTimings(50 * kMs);
  timings[1].observation_time = At(0);
  timings[3].observation_time = At(200 * kMs + 1);
  const auto result =
      AssessWorldSnapshotSkew(AllKinds(), timings, At(300 * kMs), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kExcessiveSkew);
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.withhold);
  EXPECT_EQ(result.earliest_kind, kCurrent);
  EXPECT_EQ(result.latest_kind, kContacts);
  EXPECT_EQ(result.offending_kinds, Kinds({kCurrent, kContacts}));
  EXPECT_TRUE(result.measured_skew_present);
  EXPECT_TRUE(SameTime(result.measured_skew, {0, 200000001}));
}

TEST(SkewPolicyTest, SkewAcrossSecondBoundaryIsMeasured) {
  ComponentTimings timings = AllTimings(900 * kMs);
  timings[0].observation_time = At(900 * kMs);
  timings[2].observation_time = At(1100 * kMs);
  const auto result = AssessWorldSnapshotSkew(AllKinds(), timings,
                                              At(1200 * kMs), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kFresh);
  EXPECT_TRUE(SameTime(result.measured_skew, {0, 200000000}));
}

TEST(SkewPolicyTest, SkewTiesGoToSmallestKind) {
  ComponentTimings timings = AllTimings(0);
  timings[0].observation_time = At(0);
  timings[1].observation_time = At(0);
  timings[2].observation_time = At(300 * kMs);
  timings[3].observation_time = At(300 * kMs);
  WorldSnapshotSkewPolicy policy = TestPolicy();
  const auto result =
      AssessWorldSnapshotSkew(AllKinds(), timings, At(400 * kMs), policy);
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kExcessiveSkew);
  EXPECT_EQ(result.earliest_kind, kBathymetry);
  EXPECT_EQ(result.latest_kind, kOccupancy);
  EXPECT_EQ(result.offending_kinds, Kinds({kBathymetry, kOccupancy}));
}

TEST(SkewPolicyTest, CustomMaxSkewIsHonored) {
  ComponentTimings timings = AllTimings();
  timings[0].observation_time = At(1);
  WorldSnapshotSkewPolicy policy = TestPolicy();
  policy.max_skew = {0, 0};
  const auto result =
      AssessWorldSnapshotSkew(AllKinds(), timings, At(1), policy);
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kExcessiveSkew);
  policy.max_skew = {0, 1};
  EXPECT_EQ(AssessWorldSnapshotSkew(AllKinds(), timings, At(1), policy).status,
            SnapshotPolicyStatus::kFresh);
}

TEST(SkewPolicyTest, SingleTimedKindHasNoMeasuredSkew) {
  WorldSnapshotSkewPolicy policy;
  policy.required_kinds = {kBathymetry};
  const auto result = AssessWorldSnapshotSkew(
      ViewOf({kBathymetry}), {Timing(kBathymetry, 0)}, At(0), policy);
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kFresh);
  EXPECT_FALSE(result.measured_skew_present);
  EXPECT_TRUE(SameTime(result.measured_skew, {0, 0}));
  EXPECT_TRUE(result.earliest_kind.empty());
  EXPECT_TRUE(result.latest_kind.empty());
}

TEST(SkewPolicyTest, MissingRequiredKindIsIncompleteNotPartial) {
  const auto result = AssessWorldSnapshotSkew(
      ViewOf({kBathymetry}), {Timing(kBathymetry, 0)}, At(0), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kIncomplete);
  EXPECT_NE(result.status, SnapshotPolicyStatus::kPartial);
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.withhold);
  EXPECT_EQ(result.offending_kinds, Kinds({kCurrent}));
  EXPECT_EQ(result.missing_optional_kinds, Kinds({kOccupancy, kContacts}));
}

TEST(SkewPolicyTest, EmptyDescriptorWithRequiredKindsIsIncomplete) {
  const auto result =
      AssessWorldSnapshotSkew(ViewOf({}), {}, At(0), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kIncomplete);
  EXPECT_EQ(result.offending_kinds, Kinds({kBathymetry, kCurrent}));
}

TEST(SkewPolicyTest, IncompleteBeatsStaleAndSkew) {
  const auto result = AssessWorldSnapshotSkew(
      ViewOf({kBathymetry, kOccupancy}),
      {Timing(kBathymetry, -100 * kSecond), Timing(kOccupancy, 0)}, At(0),
      TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kIncomplete);
  EXPECT_EQ(result.offending_kinds, Kinds({kCurrent}));
}

TEST(SkewPolicyTest, StaleBeatsExcessiveSkew) {
  ComponentTimings timings = AllTimings();
  timings[0].observation_time = At(-1 * kSecond);
  timings[1].validity_horizon = {0, 1};
  timings[1].observation_time = At(0);
  const auto result = AssessWorldSnapshotSkew(AllKinds(), timings,
                                              At(1 * kSecond), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kStale);
  EXPECT_EQ(result.offending_kinds, Kinds({kCurrent}));
  EXPECT_TRUE(result.measured_skew_present);
  EXPECT_TRUE(SameTime(result.measured_skew, {1, 0}));
}

TEST(SkewPolicyTest, ExcessiveSkewBeatsPartial) {
  ComponentTimings timings = {Timing(kBathymetry, 0),
                              Timing(kCurrent, 201 * kMs)};
  const auto result = AssessWorldSnapshotSkew(
      ViewOf({kBathymetry, kCurrent}), timings, At(300 * kMs), TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kExcessiveSkew);
  EXPECT_EQ(result.missing_optional_kinds, Kinds({kOccupancy, kContacts}));
}

TEST(SkewPolicyTest, StructuralDescriptorDefectSkipsThePolicy) {
  WorldSnapshotView view = AllKinds();
  view.snapshot_id.clear();
  auto result =
      AssessWorldSnapshotSkew(view, AllTimings(), At(0), TestPolicy());
  EXPECT_EQ(result.error, SnapshotPolicyError::kDescriptor);
  EXPECT_EQ(result.descriptor_error, SnapshotError::kSnapshotId);
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kUnspecified);
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.withhold);

  view = AllKinds();
  view.components.push_back({kCurrent, 9});
  result = AssessWorldSnapshotSkew(view, AllTimings(), At(0), TestPolicy());
  EXPECT_EQ(result.error, SnapshotPolicyError::kDescriptor);
  EXPECT_EQ(result.descriptor_error, SnapshotError::kDuplicateComponentKind);

  view = AllKinds();
  view.creation_time_present = false;
  result = AssessWorldSnapshotSkew(view, AllTimings(), At(0), TestPolicy());
  EXPECT_EQ(result.descriptor_error, SnapshotError::kCreationTime);

  view = AllKinds();
  view.present = false;
  result = AssessWorldSnapshotSkew(view, AllTimings(), At(0), TestPolicy());
  EXPECT_EQ(result.error, SnapshotPolicyError::kDescriptorNotPresent);
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kUnspecified);
  EXPECT_FALSE(result.accepted);
}

TEST(SkewPolicyTest, InvalidPolicyTimingsAndQueryTimeAreRejected) {
  WorldSnapshotSkewPolicy policy = TestPolicy();
  policy.max_skew = {-1, 0};
  EXPECT_EQ(
      AssessWorldSnapshotSkew(AllKinds(), AllTimings(), At(0), policy).error,
      SnapshotPolicyError::kInvalidPolicy);
  policy = TestPolicy();
  policy.max_age = {0, 1000000000};
  EXPECT_EQ(
      AssessWorldSnapshotSkew(AllKinds(), AllTimings(), At(0), policy).error,
      SnapshotPolicyError::kInvalidPolicy);
  policy = TestPolicy();
  policy.optional_kinds.push_back(kCurrent);
  EXPECT_EQ(
      AssessWorldSnapshotSkew(AllKinds(), AllTimings(), At(0), policy).error,
      SnapshotPolicyError::kInvalidPolicy);
  policy = TestPolicy();
  policy.required_kinds.push_back("");
  EXPECT_EQ(
      AssessWorldSnapshotSkew(AllKinds(), AllTimings(), At(0), policy).error,
      SnapshotPolicyError::kInvalidPolicy);

  ComponentTimings timings = AllTimings();
  timings.push_back(Timing(kCurrent, 0));
  EXPECT_EQ(
      AssessWorldSnapshotSkew(AllKinds(), timings, At(0), TestPolicy()).error,
      SnapshotPolicyError::kInvalidTimings);
  timings = AllTimings();
  timings.push_back(Timing("", 0));
  EXPECT_EQ(
      AssessWorldSnapshotSkew(AllKinds(), timings, At(0), TestPolicy()).error,
      SnapshotPolicyError::kInvalidTimings);

  EXPECT_EQ(AssessWorldSnapshotSkew(AllKinds(), AllTimings(),
                                    {kBaseSeconds, -1}, TestPolicy())
                .error,
            SnapshotPolicyError::kInvalidQueryTime);
}

TEST(SkewPolicyTest, ResultIsDeterministicForIdenticalInputs) {
  const auto first = AssessWorldSnapshotSkew(AllKinds(), AllTimings(),
                                             At(2 * kSecond + 1), TestPolicy());
  const auto second = AssessWorldSnapshotSkew(
      AllKinds(), AllTimings(), At(2 * kSecond + 1), TestPolicy());
  EXPECT_TRUE(SameAssessment(first, second));
}

// Live source store: revisions and observation times change while the
// validator only sees the latched copies.
class LiveStore {
 public:
  void Set(const std::string& kind, uint64_t revision, TimeParts observation) {
    std::lock_guard<std::mutex> lock(mutex_);
    revisions_[kind] = revision;
    observations_[kind] = observation;
  }
  void Bump(const std::string& kind, TimeParts observation) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++revisions_[kind];
    observations_[kind] = observation;
  }
  uint64_t Revision(const std::string& kind) {
    std::lock_guard<std::mutex> lock(mutex_);
    return revisions_.at(kind);
  }
  ComponentTimings LatchTimings(const Kinds& kinds) {
    std::lock_guard<std::mutex> lock(mutex_);
    ComponentTimings timings;
    for (const std::string& kind : kinds) {
      timings.push_back({kind, observations_.at(kind), {10, 0}, "live"});
    }
    return timings;
  }

 private:
  std::mutex mutex_;
  std::map<std::string, uint64_t> revisions_;
  std::map<std::string, TimeParts> observations_;
};

WorldSnapshotDescriptor LatchDescriptor(LiveStore& store, const Kinds& kinds) {
  WorldSnapshotBuilder builder;
  builder.SetStateEpoch(5).SetCreationTime({kBaseSeconds, 0});
  for (const std::string& kind : kinds) {
    builder.AddComponentRevisionSource(
        kind, [&store, kind]() { return store.Revision(kind); });
  }
  auto descriptor = builder.Build();
  EXPECT_TRUE(descriptor.ok()) << descriptor.status();
  return *descriptor;
}

TEST(SkewPolicyTest, LatchedInputsAreImmutableUnderConcurrentSourceChanges) {
  const Kinds kinds = {kBathymetry, kCurrent, kOccupancy, kContacts};
  LiveStore store;
  for (const std::string& kind : kinds) {
    store.Set(kind, 1, At(0));
  }
  const WorldSnapshotSkewPolicy policy = TestPolicy();
  const TimeParts query_time = At(300 * kMs);

  const WorldSnapshotDescriptor latched = LatchDescriptor(store, kinds);
  const ComponentTimings latched_timings = store.LatchTimings(kinds);
  const std::string latched_bytes = latched.SerializeAsString();
  const auto expected = AssessWorldSnapshotDescriptorSkew(
      latched, latched_timings, query_time, policy);
  ASSERT_EQ(expected.status, SnapshotPolicyStatus::kFresh);

  std::atomic<bool> stop{false};
  std::thread mutator([&store, &kinds, &stop]() {
    int64_t step = 0;
    while (!stop.load()) {
      ++step;
      for (const std::string& kind : kinds) {
        store.Bump(kind, At(step * kSecond * (kind == kContacts ? 1 : 0)));
      }
    }
  });
  bool all_identical = true;
  for (int i = 0; i < 2000; ++i) {
    all_identical = all_identical &&
                    SameAssessment(expected, AssessWorldSnapshotDescriptorSkew(
                                                 latched, latched_timings,
                                                 query_time, policy));
  }
  stop.store(true);
  mutator.join();
  EXPECT_TRUE(all_identical);
  store.Bump(kContacts, At(3 * kSecond));

  EXPECT_EQ(latched.SerializeAsString(), latched_bytes);
  EXPECT_TRUE(SameAssessment(
      expected, AssessWorldSnapshotDescriptorSkew(latched, latched_timings,
                                                  query_time, policy)));

  const WorldSnapshotDescriptor relatched = LatchDescriptor(store, kinds);
  const ComponentTimings relatched_timings = store.LatchTimings(kinds);
  EXPECT_NE(relatched.snapshot_id(), latched.snapshot_id());
  const auto updated = AssessWorldSnapshotDescriptorSkew(
      relatched, relatched_timings, query_time, policy);
  EXPECT_FALSE(SameAssessment(expected, updated));
  EXPECT_EQ(updated.status, SnapshotPolicyStatus::kExcessiveSkew);
}

TEST(SkewPolicyTest, NewLatchAfterObservationChangeReflectsSkew) {
  const Kinds kinds = {kBathymetry, kCurrent};
  LiveStore store;
  store.Set(kBathymetry, 1, At(0));
  store.Set(kCurrent, 1, At(0));
  const WorldSnapshotSkewPolicy policy = TestPolicy();

  const WorldSnapshotDescriptor before = LatchDescriptor(store, kinds);
  const ComponentTimings before_timings = store.LatchTimings(kinds);
  store.Bump(kCurrent, At(250 * kMs));
  const auto old_latch = AssessWorldSnapshotDescriptorSkew(
      before, before_timings, At(300 * kMs), policy);
  EXPECT_EQ(old_latch.status, SnapshotPolicyStatus::kPartial);

  const WorldSnapshotDescriptor after = LatchDescriptor(store, kinds);
  const ComponentTimings after_timings = store.LatchTimings(kinds);
  const auto new_latch = AssessWorldSnapshotDescriptorSkew(
      after, after_timings, At(300 * kMs), policy);
  EXPECT_EQ(new_latch.status, SnapshotPolicyStatus::kExcessiveSkew);
  EXPECT_NE(before.snapshot_id(), after.snapshot_id());
}

TEST(SkewPolicyTest, AssessmentDoesNotChangeDescriptorOrDigest) {
  constexpr char kGoldenSnapshotId[] =
      "b43197bb590584583288de254822664ae70709d0b9103698d296b2bc7ef30f7e";
  WorldSnapshotBuilder builder;
  builder.SetStateEpoch(42)
      .AddComponentRevision(kCurrent, 7)
      .AddComponentRevision(kBathymetry, 3)
      .SetCreationTime({kBaseSeconds, 250000000});
  auto built = builder.Build();
  ASSERT_TRUE(built.ok()) << built.status();
  const std::string before = built->SerializeAsString();
  const auto result = AssessWorldSnapshotDescriptorSkew(
      *built, {Timing(kBathymetry, 0), Timing(kCurrent, 0)}, At(0),
      TestPolicy());
  EXPECT_EQ(result.status, SnapshotPolicyStatus::kPartial);
  EXPECT_EQ(built->SerializeAsString(), before);
  EXPECT_EQ(built->snapshot_id(), kGoldenSnapshotId);
}

TEST(SkewPolicyTest, ProtoDescriptorWithStructuralDefectIsRejected) {
  const WorldSnapshotDescriptor empty;
  const auto result = AssessWorldSnapshotDescriptorSkew(
      empty, {}, At(0), WorldSnapshotSkewPolicy());
  EXPECT_EQ(result.error, SnapshotPolicyError::kDescriptor);
  EXPECT_EQ(result.descriptor_error, SnapshotError::kSnapshotId);
  EXPECT_FALSE(result.accepted);
}

}  // namespace
}  // namespace intrinsic::world
