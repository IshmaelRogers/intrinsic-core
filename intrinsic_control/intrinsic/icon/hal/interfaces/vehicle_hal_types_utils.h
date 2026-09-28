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

#ifndef INTRINSIC_ICON_HAL_INTERFACES_VEHICLE_HAL_TYPES_UTILS_H_
#define INTRINSIC_ICON_HAL_INTERFACES_VEHICLE_HAL_TYPES_UTILS_H_

#include "absl/strings/string_view.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal_types.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic_fbs {

// LINT.IfChange(FixedText256)
inline constexpr int kFixedText256Capacity = 256;
// LINT.ThenChange(//intrinsic_control/intrinsic/icon/hal/interfaces/vehicle_hal_types.fbs:FixedText256)

// Room for a trailing NUL. A longer value is rejected.
inline constexpr int kFixedText256MaxChars = kFixedText256Capacity - 1;

inline constexpr int kSpatialDof = 6;
inline constexpr int kMatrix6dValues = kSpatialDof * kSpatialDof;

// Row-major index. row and col are in [0, 5].
inline constexpr int Matrix6Index(int row, int col) {
  return row * kSpatialDof + col;
}

// Copies `value` into `text`, including a trailing NUL. Returns an error if
// `text` is null or `value` does not fit. Does not truncate.
intrinsic::icon::RealtimeStatus SetFixedText(FixedText256* text,
                                             absl::string_view value);

// Bytes of a present FixedText256, up to the first NUL. Does not allocate.
absl::string_view ReadFixedText(const FixedText256& text);

}  // namespace intrinsic_fbs

#endif  // INTRINSIC_ICON_HAL_INTERFACES_VEHICLE_HAL_TYPES_UTILS_H_
