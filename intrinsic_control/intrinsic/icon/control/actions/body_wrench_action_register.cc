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

#include "intrinsic/icon/actions/body_wrench_action_info.h"
#include "intrinsic/icon/actions/body_wrench_action_signature.h"
#include "intrinsic/icon/control/actions/body_wrench_action.h"
#include "intrinsic/icon/control/rtcl_action_factory_registry.h"

// This registers the factory for the BodyWrenchAction. It is opt-in: no
// default action set or server plugin list depends on it. We keep this in a
// separate cc file so that no header gets the linker flag alwayslink=True.

namespace intrinsic::icon {
namespace {

const auto kUnused = GetGlobalRtclActionFactoryRegistry().RegisterNoParameters(
    BodyWrenchActionInfo::kActionTypeName, &BodyWrenchAction::Create,
    GetBodyWrenchActionSignature());

}  // namespace
}  // namespace intrinsic::icon
