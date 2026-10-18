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

#ifndef INTRINSIC_DATA_EPISODE_EPISODE_EVENT_VIEW_H_
#define INTRINSIC_DATA_EPISODE_EPISODE_EVENT_VIEW_H_

#include "google/protobuf/message.h"
#include "google/protobuf/unknown_field_set.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"

namespace intrinsic::episode {

// Fills the plain-value view that AssessEpisodeEvent reads. The episode writer
// and the episode reader share this one helper so both judge an event the same
// way. The returned view points into `message`.
//
// Same fields as the #127 serialization tests fill.
inline EpisodeEventView EpisodeEventViewOf(
    const intrinsic_proto::episode::EpisodeEvent& message) {
  const auto& header = message.header();
  EpisodeEventView view;
  view.header_present = message.has_header();
  view.source_id = header.source_id();
  view.frame_id = header.frame_id();
  view.source_time_present = header.has_source_time();
  view.source_time = {header.source_time().seconds(),
                      header.source_time().nanos()};
  view.receive_time_present = header.has_receive_time();
  view.receive_time = {header.receive_time().seconds(),
                       header.receive_time().nanos()};
  view.schema_version = message.schema_version();
  view.snapshot_id = message.snapshot_id();
  view.config_digest = message.config_digest();
  view.provenance_present = message.has_provenance();
  view.payload_case = static_cast<int>(message.payload_case());
  view.has_unknown_fields =
      !message.GetReflection()->GetUnknownFields(message).empty();
  if (message.has_episode_metadata()) {
    view.marine_randomization_metadata =
        message.episode_metadata().marine_randomization_metadata();
  }
  return view;
}

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_EPISODE_EVENT_VIEW_H_
