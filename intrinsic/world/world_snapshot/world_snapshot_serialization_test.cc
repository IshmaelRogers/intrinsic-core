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

#include <cstddef>
#include <string>
#include <string_view>

#include "google/protobuf/unknown_field_set.h"
#include "gtest/gtest.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"

namespace intrinsic::world {
namespace {

using intrinsic_proto::world::WorldComponentRevision;
using intrinsic_proto::world::WorldSnapshotDescriptor;

// Canonical serialization of FillFixture(). Keep in sync with
// world_snapshot_serialization_test.py.
constexpr std::string_view kGoldenHex =
    "0a40623433313937626235393035383435383332383864653235343832323636346165"
    "37303730396430623931303336393864323936623262633765663330663765120b0880"
    "e2cfaa061080e59a77182a22180a1462617468796d657472795f7265666572656e6365"
    "100322110a0d63757272656e745f6669656c641007";

constexpr std::string_view kEpochOnlyHex =
    "0a40366333646430393434653934326266613735313136336535623534326132333063"
    "34393963663835656364306636626132646635303161646661653034623864120b0880"
    "e2cfaa061080e59a77";

constexpr std::string_view kZeroRevisionHex =
    "0a40623533613735313633656662386564646638373834373833666366643965346161"
    "6536343338623261386563386462393566313234386463626564343632616412020801"
    "1805220f0a0d63757272656e745f6669656c64";

std::string FromHex(std::string_view hex) {
  std::string out;
  out.reserve(hex.size() / 2);
  auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    return c - 'a' + 10;
  };
  for (size_t i = 0; i < hex.size(); i += 2) {
    out.push_back(
        static_cast<char>((nibble(hex[i]) << 4) | nibble(hex[i + 1])));
  }
  return out;
}

void FillFixture(WorldSnapshotDescriptor* message) {
  message->set_snapshot_id(
      "b43197bb590584583288de254822664ae70709d0b9103698d296b2bc7ef30f7e");
  message->mutable_creation_time()->set_seconds(1700000000);
  message->mutable_creation_time()->set_nanos(250000000);
  message->set_state_epoch(42);
  WorldComponentRevision* bathymetry = message->add_components();
  bathymetry->set_component_kind("bathymetry_reference");
  bathymetry->set_revision(3);
  WorldComponentRevision* current = message->add_components();
  current->set_component_kind("current_field");
  current->set_revision(7);
}

TEST(WorldSnapshotSerializationTest, DefaultMessageSerializesToZeroBytes) {
  EXPECT_TRUE(WorldSnapshotDescriptor().SerializeAsString().empty());
  EXPECT_TRUE(WorldComponentRevision().SerializeAsString().empty());
}

TEST(WorldSnapshotSerializationTest, SchemaIsLocked) {
  const auto* descriptor = WorldSnapshotDescriptor::descriptor();
  ASSERT_EQ(descriptor->field_count(), 4);
  EXPECT_EQ(descriptor->field(0)->number(), 1);
  EXPECT_EQ(descriptor->field(0)->name(), "snapshot_id");
  EXPECT_EQ(descriptor->field(1)->number(), 2);
  EXPECT_EQ(descriptor->field(1)->name(), "creation_time");
  EXPECT_EQ(descriptor->field(2)->number(), 3);
  EXPECT_EQ(descriptor->field(2)->name(), "state_epoch");
  EXPECT_EQ(descriptor->field(3)->number(), 4);
  EXPECT_EQ(descriptor->field(3)->name(), "components");
  EXPECT_TRUE(descriptor->field(3)->is_repeated());

  const auto* revision = WorldComponentRevision::descriptor();
  ASSERT_EQ(revision->field_count(), 2);
  EXPECT_EQ(revision->field(0)->number(), 1);
  EXPECT_EQ(revision->field(0)->name(), "component_kind");
  EXPECT_EQ(revision->field(1)->number(), 2);
  EXPECT_EQ(revision->field(1)->name(), "revision");
}

TEST(WorldSnapshotSerializationTest, SchemaHasNoSkewOrPayloadFields) {
  const auto* descriptor = WorldSnapshotDescriptor::descriptor();
  for (const char* name : {"skew", "age", "max_age", "stale", "status",
                           "partial", "payload", "freshness"}) {
    EXPECT_EQ(descriptor->FindFieldByName(name), nullptr) << name;
  }
  const auto* revision = WorldComponentRevision::descriptor();
  for (const char* name : {"payload", "component", "data", "validity"}) {
    EXPECT_EQ(revision->FindFieldByName(name), nullptr) << name;
  }
}

TEST(WorldSnapshotSerializationTest, GoldenRoundTrip) {
  WorldSnapshotDescriptor message;
  FillFixture(&message);
  const std::string golden = FromHex(kGoldenHex);
  EXPECT_EQ(message.SerializeAsString(), golden);

  WorldSnapshotDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.snapshot_id(),
            "b43197bb590584583288de254822664ae70709d0b9103698d296b2bc7ef30f7e");
  EXPECT_EQ(parsed.creation_time().seconds(), 1700000000);
  EXPECT_EQ(parsed.creation_time().nanos(), 250000000);
  EXPECT_EQ(parsed.state_epoch(), 42);
  ASSERT_EQ(parsed.components_size(), 2);
  EXPECT_EQ(parsed.components(0).component_kind(), "bathymetry_reference");
  EXPECT_EQ(parsed.components(0).revision(), 3);
  EXPECT_EQ(parsed.components(1).component_kind(), "current_field");
  EXPECT_EQ(parsed.components(1).revision(), 7);
  EXPECT_EQ(parsed.SerializeAsString(), golden);
}

TEST(WorldSnapshotSerializationTest, EpochOnlyGoldenRoundTrip) {
  WorldSnapshotDescriptor message;
  message.set_snapshot_id(
      "6c3dd0944e942bfa751163e5b542a230c499cf85ecd0f6ba2df501adfae04b8d");
  message.mutable_creation_time()->set_seconds(1700000000);
  message.mutable_creation_time()->set_nanos(250000000);
  const std::string golden = FromHex(kEpochOnlyHex);
  EXPECT_EQ(message.SerializeAsString(), golden);

  WorldSnapshotDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.state_epoch(), 0);
  EXPECT_EQ(parsed.components_size(), 0);
}

TEST(WorldSnapshotSerializationTest, ZeroRevisionKindStaysPresentOnTheWire) {
  WorldSnapshotDescriptor message;
  message.set_snapshot_id(
      "b53a75163efb8eddf8784783fcfd9e4aae6438b2a8ec8db95f1248dcbed462ad");
  message.mutable_creation_time()->set_seconds(1);
  message.set_state_epoch(5);
  message.add_components()->set_component_kind("current_field");
  const std::string golden = FromHex(kZeroRevisionHex);
  EXPECT_EQ(message.SerializeAsString(), golden);

  WorldSnapshotDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  ASSERT_EQ(parsed.components_size(), 1);
  EXPECT_EQ(parsed.components(0).component_kind(), "current_field");
  EXPECT_EQ(parsed.components(0).revision(), 0);
}

TEST(WorldSnapshotSerializationTest, UnknownFieldsRoundTrip) {
  WorldSnapshotDescriptor message;
  FillFixture(&message);
  message.GetReflection()->MutableUnknownFields(&message)->AddVarint(99, 12345);
  message.mutable_components(0)
      ->GetReflection()
      ->MutableUnknownFields(message.mutable_components(0))
      ->AddLengthDelimited(77, "future");
  const std::string wire = message.SerializeAsString();

  WorldSnapshotDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(wire));
  EXPECT_EQ(parsed.GetReflection()->GetUnknownFields(parsed).field_count(), 1);
  EXPECT_EQ(parsed.components(0)
                .GetReflection()
                ->GetUnknownFields(parsed.components(0))
                .field_count(),
            1);
  EXPECT_EQ(parsed.SerializeAsString(), wire);
  EXPECT_EQ(parsed.state_epoch(), 42);
  EXPECT_EQ(parsed.components(1).revision(), 7);
}

}  // namespace
}  // namespace intrinsic::world
