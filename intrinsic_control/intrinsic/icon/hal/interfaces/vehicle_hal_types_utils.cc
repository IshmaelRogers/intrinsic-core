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

#include "intrinsic/icon/hal/interfaces/vehicle_hal_types_utils.h"

#include <cstring>

#include "absl/strings/string_view.h"
#include "intrinsic/icon/flatbuffers/fixed_string.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal_types.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic_fbs {

intrinsic::icon::RealtimeStatus SetFixedText(FixedText256 *text,
                                             absl::string_view value) {
  if (text == nullptr) {
    return intrinsic::icon::InvalidArgumentError("FixedText256 is null");
  }
  // StringCopy uses SNPrintF("%s"), which returns the untruncated length.
  // That length equals value.size() even when the destination keeps only
  // capacity - 1 bytes, so overflow is rejected here instead of truncated.
  if (value.size() > static_cast<size_t>(kFixedText256MaxChars)) {
    return intrinsic::icon::InvalidArgumentError(
        "FixedText256 value does not fit");
  }
  return StringCopy(text->mutable_data(), value);
}

absl::string_view ReadFixedText(const FixedText256 &text) {
  const auto *data = text.data();
  if (data == nullptr || data->data() == nullptr) {
    return absl::string_view();
  }
  const char *bytes = reinterpret_cast<const char *>(data->data());
  return absl::string_view(bytes, ::strnlen(bytes, kFixedText256Capacity));
}

}  // namespace intrinsic_fbs
