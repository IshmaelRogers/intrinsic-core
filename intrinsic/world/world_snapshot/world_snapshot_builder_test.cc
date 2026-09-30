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

#include "intrinsic/world/world_snapshot/world_snapshot_builder.h"

#include <cstdint>
#include <map>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "gtest/gtest.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"

namespace intrinsic::world {
namespace {

using intrinsic_proto::world::WorldSnapshotDescriptor;

constexpr char kGoldenSnapshotId[] =
    "b43197bb590584583288de254822664ae70709d0b9103698d296b2bc7ef30f7e";

constexpr SnapshotTime kFixedTime = {1700000000, 250000000};

// Fake source store that counts every read.
class FakeStore {
 public:
  void Set(const std::string& kind, uint64_t revision) {
    revisions_[kind] = revision;
  }
  void SetEpoch(uint64_t epoch) { epoch_ = epoch; }

  uint64_t ReadEpoch() {
    ++epoch_reads_;
    return epoch_;
  }
  uint64_t ReadRevision(const std::string& kind) {
    ++reads_[kind];
    return revisions_.at(kind);
  }
  int epoch_reads() const { return epoch_reads_; }
  int reads(const std::string& kind) const {
    auto it = reads_.find(kind);
    return it == reads_.end() ? 0 : it->second;
  }
  int total_reads() const {
    int total = epoch_reads_;
    for (const auto& [kind, count] : reads_) {
      total += count;
    }
    return total;
  }

 private:
  uint64_t epoch_ = 0;
  int epoch_reads_ = 0;
  std::map<std::string, uint64_t> revisions_;
  std::map<std::string, int> reads_;
};

void AddStoreKind(WorldSnapshotBuilder& builder, FakeStore& store,
                  const std::string& kind) {
  builder.AddComponentRevisionSource(
      kind, [&store, kind]() { return store.ReadRevision(kind); });
}

void SetStoreEpoch(WorldSnapshotBuilder& builder, FakeStore& store) {
  builder.SetStateEpochSource([&store]() { return store.ReadEpoch(); });
}

TEST(WorldSnapshotBuilderTest, GoldenDescriptorFromSources) {
  FakeStore store;
  store.SetEpoch(42);
  store.Set("current_field", 7);
  store.Set("bathymetry_reference", 3);
  WorldSnapshotBuilder builder;
  SetStoreEpoch(builder, store);
  AddStoreKind(builder, store, "current_field");
  AddStoreKind(builder, store, "bathymetry_reference");
  builder.SetCreationTime(kFixedTime);

  auto result = builder.Build();
  ASSERT_TRUE(result.ok()) << result.status();
  const WorldSnapshotDescriptor& descriptor = *result;
  EXPECT_EQ(descriptor.snapshot_id(), kGoldenSnapshotId);
  EXPECT_EQ(descriptor.creation_time().seconds(), 1700000000);
  EXPECT_EQ(descriptor.creation_time().nanos(), 250000000);
  EXPECT_EQ(descriptor.state_epoch(), 42);
  ASSERT_EQ(descriptor.components_size(), 2);
  EXPECT_EQ(descriptor.components(0).component_kind(), "bathymetry_reference");
  EXPECT_EQ(descriptor.components(0).revision(), 3);
  EXPECT_EQ(descriptor.components(1).component_kind(), "current_field");
  EXPECT_EQ(descriptor.components(1).revision(), 7);
  EXPECT_TRUE(AssessWorldSnapshotDescriptor(descriptor).accepted);
}

TEST(WorldSnapshotBuilderTest, SnapshotIdEqualsPolicyDigest) {
  WorldSnapshotBuilder builder;
  builder.SetStateEpoch(9)
      .AddComponentRevision("semantic_contacts", 4)
      .AddComponentRevision("occupancy_reference", 0)
      .SetCreationTime(kFixedTime);
  auto result = builder.Build();
  ASSERT_TRUE(result.ok()) << result.status();
  EXPECT_EQ(result->snapshot_id(),
            ComputeSnapshotId(
                9, {{"occupancy_reference", 0}, {"semantic_contacts", 4}}));
}

TEST(WorldSnapshotBuilderTest, SameEpochAndRevisionsGiveSameIdAcrossTimes) {
  auto build = [](SnapshotTime time) {
    WorldSnapshotBuilder builder;
    builder.SetStateEpoch(42)
        .AddComponentRevision("bathymetry_reference", 3)
        .AddComponentRevision("current_field", 7)
        .SetCreationTime(time);
    return builder.Build();
  };
  auto first = build({1700000000, 0});
  auto second = build({1800000000, 999999999});
  ASSERT_TRUE(first.ok() && second.ok());
  EXPECT_EQ(first->snapshot_id(), second->snapshot_id());
  EXPECT_EQ(first->snapshot_id(), kGoldenSnapshotId);
  EXPECT_NE(first->creation_time().seconds(),
            second->creation_time().seconds());
}

TEST(WorldSnapshotBuilderTest, InputOrderDoesNotChangeIdOrComponentOrder) {
  WorldSnapshotBuilder forward;
  forward.SetStateEpoch(42)
      .AddComponentRevision("bathymetry_reference", 3)
      .AddComponentRevision("current_field", 7)
      .SetCreationTime(kFixedTime);
  WorldSnapshotBuilder reversed;
  reversed.SetStateEpoch(42)
      .AddComponentRevision("current_field", 7)
      .AddComponentRevision("bathymetry_reference", 3)
      .SetCreationTime(kFixedTime);
  auto a = forward.Build();
  auto b = reversed.Build();
  ASSERT_TRUE(a.ok() && b.ok());
  EXPECT_EQ(a->snapshot_id(), b->snapshot_id());
  EXPECT_EQ(a->SerializeAsString(), b->SerializeAsString());
  EXPECT_EQ(b->components(0).component_kind(), "bathymetry_reference");
}

TEST(WorldSnapshotBuilderTest, ComponentsAreSortedAscendingIncludingUnknown) {
  WorldSnapshotBuilder builder;
  builder.SetStateEpoch(1)
      .AddComponentRevision("vendor_x", 1)
      .AddComponentRevision("semantic_contacts", 2)
      .AddComponentRevision("current_field", 3)
      .AddComponentRevision("occupancy_reference", 4)
      .AddComponentRevision("bathymetry_reference", 5)
      .SetCreationTime(kFixedTime);
  auto result = builder.Build();
  ASSERT_TRUE(result.ok()) << result.status();
  ASSERT_EQ(result->components_size(), 5);
  EXPECT_EQ(result->components(0).component_kind(), "bathymetry_reference");
  EXPECT_EQ(result->components(1).component_kind(), "current_field");
  EXPECT_EQ(result->components(2).component_kind(), "occupancy_reference");
  EXPECT_EQ(result->components(3).component_kind(), "semantic_contacts");
  EXPECT_EQ(result->components(4).component_kind(), "vendor_x");
}

TEST(WorldSnapshotBuilderTest, DescriptorIsImmutableAfterSourceMutation) {
  FakeStore store;
  store.SetEpoch(42);
  store.Set("current_field", 7);
  store.Set("bathymetry_reference", 3);
  WorldSnapshotBuilder builder;
  SetStoreEpoch(builder, store);
  AddStoreKind(builder, store, "current_field");
  AddStoreKind(builder, store, "bathymetry_reference");
  builder.SetCreationTime(kFixedTime);

  auto result = builder.Build();
  ASSERT_TRUE(result.ok()) << result.status();
  const WorldSnapshotDescriptor before = *result;
  const std::string wire_before = result->SerializeAsString();

  store.SetEpoch(43);
  store.Set("current_field", 8);
  store.Set("bathymetry_reference", 4);

  EXPECT_EQ(result->SerializeAsString(), wire_before);
  EXPECT_EQ(result->state_epoch(), 42);
  EXPECT_EQ(result->components(1).revision(), 7);
  EXPECT_EQ(result->snapshot_id(), before.snapshot_id());

  auto later = builder.Build();
  ASSERT_TRUE(later.ok());
  EXPECT_EQ(later->state_epoch(), 43);
  EXPECT_NE(later->snapshot_id(), before.snapshot_id());
  EXPECT_EQ(result->SerializeAsString(), wire_before);
}

TEST(WorldSnapshotBuilderTest, RepeatedReadsOfDescriptorAreStable) {
  WorldSnapshotBuilder builder;
  builder.SetStateEpoch(42)
      .AddComponentRevision("current_field", 7)
      .AddComponentRevision("bathymetry_reference", 3)
      .SetCreationTime(kFixedTime);
  auto result = builder.Build();
  ASSERT_TRUE(result.ok());
  const std::string first = result->SerializeAsString();
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(result->SerializeAsString(), first);
    EXPECT_EQ(result->snapshot_id(), kGoldenSnapshotId);
    EXPECT_EQ(result->components_size(), 2);
  }
  WorldSnapshotDescriptor reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(first));
  EXPECT_EQ(reparsed.SerializeAsString(), first);
}

TEST(WorldSnapshotBuilderTest, EachSourceIsReadExactlyOncePerBuild) {
  FakeStore store;
  store.SetEpoch(42);
  store.Set("current_field", 7);
  store.Set("bathymetry_reference", 3);
  store.Set("occupancy_reference", 1);
  store.Set("semantic_contacts", 2);
  WorldSnapshotBuilder builder;
  SetStoreEpoch(builder, store);
  for (const char* kind : {"semantic_contacts", "current_field",
                           "occupancy_reference", "bathymetry_reference"}) {
    AddStoreKind(builder, store, kind);
  }
  int clock_reads = 0;
  builder.SetClock([&clock_reads]() {
    ++clock_reads;
    return kFixedTime;
  });

  auto result = builder.Build();
  ASSERT_TRUE(result.ok()) << result.status();
  EXPECT_EQ(store.epoch_reads(), 1);
  EXPECT_EQ(store.reads("bathymetry_reference"), 1);
  EXPECT_EQ(store.reads("current_field"), 1);
  EXPECT_EQ(store.reads("occupancy_reference"), 1);
  EXPECT_EQ(store.reads("semantic_contacts"), 1);
  EXPECT_EQ(store.total_reads(), 5);
  EXPECT_EQ(clock_reads, 1);

  ASSERT_TRUE(builder.Build().ok());
  EXPECT_EQ(store.epoch_reads(), 2);
  EXPECT_EQ(store.reads("current_field"), 2);
}

TEST(WorldSnapshotBuilderTest, EpochOnlySnapshotIsAllowed) {
  WorldSnapshotBuilder builder;
  builder.SetStateEpoch(0).SetCreationTime(kFixedTime);
  auto result = builder.Build();
  ASSERT_TRUE(result.ok()) << result.status();
  EXPECT_EQ(result->state_epoch(), 0);
  EXPECT_EQ(result->components_size(), 0);
  EXPECT_EQ(result->snapshot_id(), ComputeSnapshotId(0, {}));
  EXPECT_TRUE(AssessWorldSnapshotDescriptor(*result).accepted);
}

TEST(WorldSnapshotBuilderTest, UnsetEpochDefaultsToZero) {
  WorldSnapshotBuilder builder;
  builder.AddComponentRevision("current_field", 1).SetCreationTime(kFixedTime);
  auto result = builder.Build();
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result->state_epoch(), 0);
}

TEST(WorldSnapshotBuilderTest, RevisionZeroKindIsKept) {
  WorldSnapshotBuilder builder;
  builder.SetStateEpoch(5)
      .AddComponentRevision("current_field", 0)
      .SetCreationTime(kFixedTime);
  auto result = builder.Build();
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result->components_size(), 1);
  EXPECT_EQ(result->components(0).revision(), 0);
}

TEST(WorldSnapshotBuilderTest, EmptyKindIsRejectedBeforeAnySourceRead) {
  FakeStore store;
  store.SetEpoch(1);
  store.Set("", 1);
  store.Set("current_field", 2);
  WorldSnapshotBuilder builder;
  SetStoreEpoch(builder, store);
  AddStoreKind(builder, store, "current_field");
  AddStoreKind(builder, store, "");
  builder.SetCreationTime(kFixedTime);
  auto result = builder.Build();
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_NE(result.status().message().find("empty_component_kind"),
            std::string::npos);
  EXPECT_EQ(store.total_reads(), 0);
}

TEST(WorldSnapshotBuilderTest, DuplicateKindIsRejectedBeforeAnySourceRead) {
  FakeStore store;
  store.SetEpoch(1);
  store.Set("current_field", 2);
  WorldSnapshotBuilder builder;
  SetStoreEpoch(builder, store);
  AddStoreKind(builder, store, "current_field");
  AddStoreKind(builder, store, "current_field");
  builder.SetCreationTime(kFixedTime);
  auto result = builder.Build();
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_NE(result.status().message().find("duplicate_component_kind"),
            std::string::npos);
  EXPECT_EQ(store.total_reads(), 0);
}

TEST(WorldSnapshotBuilderTest, BadCreationNanosAreRejected) {
  for (int32_t nanos : {-1, 1000000000}) {
    WorldSnapshotBuilder builder;
    builder.SetStateEpoch(1).SetCreationTime({1700000000, nanos});
    auto result = builder.Build();
    ASSERT_FALSE(result.ok()) << nanos;
    EXPECT_EQ(result.status().code(), absl::StatusCode::kInvalidArgument);
    EXPECT_NE(result.status().message().find("creation_time"),
              std::string::npos);
  }
}

TEST(WorldSnapshotBuilderTest, MissingCreationTimeIsRejected) {
  WorldSnapshotBuilder builder;
  builder.SetStateEpoch(1);
  auto result = builder.Build();
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), absl::StatusCode::kInvalidArgument);
}

TEST(WorldSnapshotBuilderTest, NullComponentSourceIsRejected) {
  WorldSnapshotBuilder builder;
  builder.AddComponentRevisionSource("current_field", nullptr)
      .SetCreationTime(kFixedTime);
  EXPECT_EQ(builder.Build().status().code(),
            absl::StatusCode::kInvalidArgument);
}

TEST(WorldSnapshotBuilderTest, NeverWithholdsAWellFormedSnapshot) {
  // Old creation time, huge epoch, and revision zero are all just values.
  WorldSnapshotBuilder builder;
  builder.SetStateEpoch(UINT64_MAX)
      .AddComponentRevision("semantic_contacts", 0)
      .SetCreationTime({0, 0});
  EXPECT_TRUE(builder.Build().ok());
}

TEST(WorldSnapshotBuilderTest, DescriptorAssessmentMapsProtoDefects) {
  WorldSnapshotBuilder builder;
  builder.SetStateEpoch(42)
      .AddComponentRevision("current_field", 7)
      .SetCreationTime(kFixedTime);
  WorldSnapshotDescriptor good = *builder.Build();

  EXPECT_FALSE(
      AssessWorldSnapshotDescriptor(WorldSnapshotDescriptor()).accepted);

  WorldSnapshotDescriptor no_id = good;
  no_id.clear_snapshot_id();
  EXPECT_EQ(AssessWorldSnapshotDescriptor(no_id).error,
            SnapshotError::kSnapshotId);

  WorldSnapshotDescriptor no_time = good;
  no_time.clear_creation_time();
  EXPECT_EQ(AssessWorldSnapshotDescriptor(no_time).error,
            SnapshotError::kCreationTime);

  WorldSnapshotDescriptor bad_nanos = good;
  bad_nanos.mutable_creation_time()->set_nanos(1000000000);
  EXPECT_EQ(AssessWorldSnapshotDescriptor(bad_nanos).error,
            SnapshotError::kCreationTime);

  WorldSnapshotDescriptor empty_kind = good;
  empty_kind.add_components();
  EXPECT_EQ(AssessWorldSnapshotDescriptor(empty_kind).error,
            SnapshotError::kEmptyComponentKind);

  WorldSnapshotDescriptor duplicate = good;
  duplicate.add_components()->set_component_kind("current_field");
  const SnapshotAssessment assessment =
      AssessWorldSnapshotDescriptor(duplicate);
  EXPECT_EQ(assessment.error, SnapshotError::kDuplicateComponentKind);
  EXPECT_EQ(assessment.component_index, 1);
}

}  // namespace
}  // namespace intrinsic::world
