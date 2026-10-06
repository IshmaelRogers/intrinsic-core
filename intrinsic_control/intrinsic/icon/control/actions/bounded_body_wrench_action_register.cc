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

#include "intrinsic/icon/actions/bounded_body_wrench_info.h"
#include "intrinsic/icon/actions/bounded_body_wrench_signature.h"
#include "intrinsic/icon/control/actions/bounded_body_wrench_action.h"
#include "intrinsic/icon/control/rtcl_action_factory_registry.h"

// This registers the factory for the BoundedBodyWrenchAction.
// We have this separate cc file so no header file gets the linker flag
// alwayslink=True, which would have far reaching effects and is discouraged.
//
// This target is opt-in. It is intentionally not part of
// rtcl_controller_default_actions.

namespace intrinsic::icon {
namespace {

const auto kUnused =
    GetGlobalRtclActionFactoryRegistry()
        .RegisterTyped<BoundedBodyWrenchInfo::FixedParams>(
            BoundedBodyWrenchInfo::kActionTypeName,
            &BoundedBodyWrenchAction::Create, GetBoundedBodyWrenchSignature());

}  // namespace
}  // namespace intrinsic::icon
