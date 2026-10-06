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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_HYDRODYNAMICS_HYDRODYNAMICS_PLUGIN_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_HYDRODYNAMICS_HYDRODYNAMICS_PLUGIN_H_

#include <memory>
#include <optional>
#include <string>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "intrinsic/simulation/gazebo/plugins/hydrodynamics/hydrodynamics_adapter.h"
#include "sdf/Element.hh"

namespace intrinsic::simulation {

// Opt-in marine hydrodynamics. Attach under a <model>:
//
// <plugin filename="static://intrinsic::simulation::Hydrodynamics"
//         name="intrinsic::simulation::Hydrodynamics">
//   <link_name>hull</link_name>  <!-- optional; default is canonical link -->
//   <current_field>
//     <frame_id>world_enu</frame_id>
//     <velocity_m_s>0.2 -0.1 0</velocity_m_s>
//   </current_field>
// </plugin>
//
// No <current_field> applies nothing and does not change link gravity.
// The force comes from HydrodynamicsAdapter, which calls
// MarineForceDynamics::Evaluate. Link gravity is disabled only on steps
// that apply that wrench, because the wrench already includes restoring.
class Hydrodynamics final : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemConfigurePriority,
                            public gz::sim::ISystemPreUpdate,
                            public gz::sim::ISystemReset {
 public:
  Hydrodynamics() = default;
  ~Hydrodynamics() override = default;

  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& event_mgr) override;

  gz::sim::System::PriorityType ConfigurePriority() override;

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override;

  void Reset(const gz::sim::UpdateInfo& info,
             gz::sim::EntityComponentManager& ecm) override;

 private:
  void Report(const HydrodynamicsStepResult& result);

  HydrodynamicsAdapter adapter_;
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_ = gz::sim::kNullEntity;
  bool gravity_disabled_ = false;
  std::optional<HydrodynamicsErrorCode> last_reported_;
  std::string last_reported_message_;
};

// Defined next to the static plugin registration. Tests call it so that
// unit is linked and the alias is registered.
int HydrodynamicsStaticPluginAnchor();

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_HYDRODYNAMICS_HYDRODYNAMICS_PLUGIN_H_
