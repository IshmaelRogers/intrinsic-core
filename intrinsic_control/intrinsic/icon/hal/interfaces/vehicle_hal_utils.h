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

#ifndef INTRINSIC_ICON_HAL_INTERFACES_VEHICLE_HAL_UTILS_H_
#define INTRINSIC_ICON_HAL_INTERFACES_VEHICLE_HAL_UTILS_H_

#include <stddef.h>
#include <stdint.h>

#include <string_view>

#include "flatbuffers/detached_buffer.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal.fbs.h"

namespace intrinsic_fbs {

// Copies `text` into `out` and zeros the unused tail. Returns false when
// `out` is null or `text` is longer than 64 bytes. A failed call leaves
// `out` unchanged. Length 0 is an absent string, not a truncated one.
inline bool AssignFixedString64(FixedString64* out, std::string_view text) {
  if (out == nullptr) {
    return false;
  }
  auto* data = out->mutable_data();
  if (text.size() > data->size()) {
    return false;
  }
  for (uint16_t i = 0; i < data->size(); ++i) {
    const uint8_t byte = i < text.size() ? static_cast<uint8_t>(text[i])
                                         : static_cast<uint8_t>(0);
    data->Mutate(i, byte);
  }
  out->mutate_length(static_cast<uint8_t>(text.size()));
  return true;
}

// The bytes `value.length()` names. Empty when the length is past the array.
// The view points at `value` and is not NUL-terminated.
inline std::string_view ViewFixedString64(const FixedString64& value) {
  const auto* data = value.data();
  if (data == nullptr || value.length() > data->size()) {
    return {};
  }
  return std::string_view(reinterpret_cast<const char*>(data->Data()),
                          value.length());
}

// Neutral samples. Every field is written. Presence flags are false, strings
// are empty, and numeric payloads are zero. A zero wrench payload is the
// neutral sample only together with frame id "body"; this builder leaves the
// frame empty, so it is not a command.
flatbuffers::DetachedBuffer BuildBodyState();
flatbuffers::DetachedBuffer BuildBodyWrench();
flatbuffers::DetachedBuffer BuildVehicleLimits();

}  // namespace intrinsic_fbs

#endif  // INTRINSIC_ICON_HAL_INTERFACES_VEHICLE_HAL_UTILS_H_
