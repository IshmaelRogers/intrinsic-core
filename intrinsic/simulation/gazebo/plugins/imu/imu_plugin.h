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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_IMU_IMU_PLUGIN_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_IMU_IMU_PLUGIN_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "intrinsic/simulation/gazebo/plugins/imu/imu_adapter.h"
#include "sdf/Element.hh"

namespace intrinsic::simulation {

// Opt-in IMU sensor system plugin for UUV simulation. Sim and test only.
//
// Attach to a model. Each non-paused PostUpdate copies the target link's
// world pose, world angular velocity, and world linear acceleration and calls
// ImuAdapter::Step. Configure and Reset enable the link's velocity and
// acceleration checks, which create components in the ECM. PostUpdate
// receives a const ECM, so afterwards the plugin never mutates world state.
// Link poses are the same with and without this plugin.
//
// There is no gz-transport topic. The latest adapter result is available
// through LastResultForTesting() for tests. See imu_adapter.h for the SDF
// parameters, and README.md for the contract.
class Imu final : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPostUpdate,
                  public gz::sim::ISystemReset {
 public:
  Imu() = default;
  ~Imu() override = default;

  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& event_mgr) override;

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override;

  void Reset(const gz::sim::UpdateInfo& info,
             gz::sim::EntityComponentManager& ecm) override;

  // Test-only accessors. Not part of the plugin contract and not for
  // production callers.
  //
  // Result of the most recent non-paused step. Default before the first.
  [[nodiscard]] const ImuStepResult& LastResultForTesting() const {
    return last_result_;
  }
  [[nodiscard]] const ImuAdapter& AdapterForTesting() const { return adapter_; }

 private:
  void Report(const ImuSimStatus& status);

  ImuAdapter adapter_;
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_ = gz::sim::kNullEntity;
  ImuStepResult last_result_;
  std::optional<ImuSimErrorCode> last_reported_;
  std::string last_reported_message_;
};

// Test-only. Lets a test reference the registration translation unit so the
// linker keeps it.
int ImuStaticPluginAnchorForTesting();

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_IMU_IMU_PLUGIN_H_
