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

#ifndef INTRINSIC_EMBODIMENT_CAPABILITY_DESCRIPTOR_FIXTURE_H_
#define INTRINSIC_EMBODIMENT_CAPABILITY_DESCRIPTOR_FIXTURE_H_

#include <string>

#include "intrinsic/embodiment/capability_descriptor.h"
#include "intrinsic/embodiment/proto/capability_descriptor.pb.h"

namespace intrinsic::embodiment {

// Fills the industrial manipulator compatibility profile.
//
// Pure data. Does not read hardware, register ICON features, start a
// simulator, or command motion. Vehicle capabilities are absent. Callers
// that never call this keep an empty descriptor.
inline void FillManipulatorDescriptor(
    intrinsic_proto::embodiment::EmbodimentDescriptor* descriptor) {
  descriptor->Clear();
  descriptor->set_resource_id(std::string(kManipulatorResourceId));
  for (const CapabilityDeclarationView& capability : kManipulatorCapabilities) {
    auto* added = descriptor->add_capabilities();
    added->set_id(std::string(capability.id));
    added->set_category(
        static_cast<intrinsic_proto::embodiment::CapabilityCategory>(
            capability.category));
    added->set_interface_name(std::string(capability.interface_name));
  }
}

}  // namespace intrinsic::embodiment

#endif  // INTRINSIC_EMBODIMENT_CAPABILITY_DESCRIPTOR_FIXTURE_H_
