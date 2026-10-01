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

#ifndef INTRINSIC_PERCEPTION_SONAR_OBSERVATION_BUNDLE_BUILDER_H_
#define INTRINSIC_PERCEPTION_SONAR_OBSERVATION_BUNDLE_BUILDER_H_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "intrinsic/perception/proto/sonar/observation_bundle.pb.h"

namespace intrinsic::perception::sonar {

// Thin assembler for MultimodalObservationBundle. Copies the caller-supplied
// fields onto the wire message. Does not fetch reference ids, does not embed
// FLS, SSS, camera, point-cloud, VehicleState, or World payloads, and does
// not rewrite strings, times, or snapshot ids.

struct ObservationSlotParts {
  uint64_t sequence = 0;
  int64_t source_seconds = 0;
  int32_t source_nanos = 0;
  bool set_receive = false;
  int64_t receive_seconds = 0;
  int32_t receive_nanos = 0;
  std::string source_id;
  std::string frame_id;
  std::string clock_domain;
  bool set_validity = false;
  int validity_state = 0;
  std::string reference_id;
  std::string content_type;
  bool set_byte_size = false;
  uint64_t byte_size = 0;
};

struct StateReferenceParts {
  bool set = false;
  ObservationSlotParts stamp;
  uint64_t state_epoch = 0;
  std::string world_snapshot_id;
};

struct AbsentModalityParts {
  int modality = 0;
  int reason = 0;
};

struct ObservationBundleParts {
  bool set_header = false;
  ObservationSlotParts header;
  bool set_max_skew = false;
  int64_t max_skew_seconds = 0;
  int32_t max_skew_nanos = 0;
  bool set_fls = false;
  ObservationSlotParts fls;
  bool set_sss = false;
  ObservationSlotParts sss;
  bool set_optical = false;
  ObservationSlotParts optical;
  bool set_point_cloud = false;
  ObservationSlotParts point_cloud;
  StateReferenceParts vehicle_state;
  std::vector<AbsentModalityParts> absent;
  std::vector<std::pair<std::string, std::string>> metadata;
};

inline void FillStampedHeader(
    intrinsic_proto::embodiment::StampedHeader* header,
    const ObservationSlotParts& parts) {
  header->set_sequence(parts.sequence);
  header->mutable_source_time()->set_seconds(parts.source_seconds);
  header->mutable_source_time()->set_nanos(parts.source_nanos);
  if (parts.set_receive) {
    header->mutable_receive_time()->set_seconds(parts.receive_seconds);
    header->mutable_receive_time()->set_nanos(parts.receive_nanos);
  }
  header->set_source_id(parts.source_id);
  header->set_frame_id(parts.frame_id);
  header->set_clock_domain(parts.clock_domain);
  if (parts.set_validity) {
    header->mutable_validity()->set_state(
        static_cast<intrinsic_proto::embodiment::Validity::State>(
            parts.validity_state));
  }
}

inline void FillObservationSlot(
    intrinsic_proto::perception::sonar::ObservationSlot* slot,
    const ObservationSlotParts& parts) {
  FillStampedHeader(slot->mutable_header(), parts);
  slot->set_reference_id(parts.reference_id);
  slot->set_content_type(parts.content_type);
  if (parts.set_byte_size) {
    slot->set_byte_size(parts.byte_size);
  }
}

inline void BuildMultimodalObservationBundle(
    const ObservationBundleParts& parts,
    intrinsic_proto::perception::sonar::MultimodalObservationBundle* bundle) {
  bundle->Clear();
  if (parts.set_header) {
    FillStampedHeader(bundle->mutable_header(), parts.header);
  }
  if (parts.set_max_skew) {
    bundle->mutable_max_skew()->set_seconds(parts.max_skew_seconds);
    bundle->mutable_max_skew()->set_nanos(parts.max_skew_nanos);
  }
  if (parts.set_fls) {
    FillObservationSlot(bundle->mutable_fls(), parts.fls);
  }
  if (parts.set_sss) {
    FillObservationSlot(bundle->mutable_sss(), parts.sss);
  }
  if (parts.set_optical) {
    FillObservationSlot(bundle->mutable_optical(), parts.optical);
  }
  if (parts.set_point_cloud) {
    FillObservationSlot(bundle->mutable_point_cloud(), parts.point_cloud);
  }
  if (parts.vehicle_state.set) {
    auto* state = bundle->mutable_vehicle_state();
    FillStampedHeader(state->mutable_header(), parts.vehicle_state.stamp);
    state->set_state_epoch(parts.vehicle_state.state_epoch);
    state->set_world_snapshot_id(parts.vehicle_state.world_snapshot_id);
  }
  for (const AbsentModalityParts& entry : parts.absent) {
    auto* absent = bundle->add_absent();
    absent->set_modality(
        static_cast<intrinsic_proto::perception::sonar::ObservationModality>(
            entry.modality));
    absent->set_reason(
        static_cast<intrinsic_proto::perception::sonar::AbsentModalityReason>(
            entry.reason));
  }
  for (const auto& item : parts.metadata) {
    (*bundle->mutable_metadata())[item.first] = item.second;
  }
}

}  // namespace intrinsic::perception::sonar

#endif  // INTRINSIC_PERCEPTION_SONAR_OBSERVATION_BUNDLE_BUILDER_H_
