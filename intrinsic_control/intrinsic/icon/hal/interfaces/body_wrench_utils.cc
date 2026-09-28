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

#include "intrinsic/icon/hal/interfaces/body_wrench_utils.h"

#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "intrinsic/icon/hal/interfaces/body_wrench.fbs.h"

namespace intrinsic_fbs {

flatbuffers::DetachedBuffer BuildBodyWrench() {
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);
  BodyWrenchBuilder wrench_builder(builder);
  wrench_builder.add_sequence(0);
  wrench_builder.add_source_time_present(false);
  wrench_builder.add_source_time_seconds(0);
  wrench_builder.add_source_time_nanos(0);
  wrench_builder.add_receive_time_present(false);
  wrench_builder.add_receive_time_seconds(0);
  wrench_builder.add_receive_time_nanos(0);
  wrench_builder.add_validity_present(false);
  wrench_builder.add_validity_state(0);
  builder.Finish(wrench_builder.Finish());
  return builder.Release();
}

}  // namespace intrinsic_fbs
