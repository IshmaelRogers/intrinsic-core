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

#ifndef INTRINSIC_ICON_HAL_INTERFACES_BODY_WRENCH_UTILS_H_
#define INTRINSIC_ICON_HAL_INTERFACES_BODY_WRENCH_UTILS_H_

#include "flatbuffers/detached_buffer.h"
#include "intrinsic/icon/hal/interfaces/body_wrench.fbs.h"

namespace intrinsic_fbs {

// Default body wrench. The wrench field stays null, so this is not a command
// and not the neutral zero wrench. Frame id stays null and is not filled in.
flatbuffers::DetachedBuffer BuildBodyWrench();

}  // namespace intrinsic_fbs

#endif  // INTRINSIC_ICON_HAL_INTERFACES_BODY_WRENCH_UTILS_H_
