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

#include "intrinsic/hardware/marine/fake_thruster_array.h"

#include <optional>
#include <vector>

#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/hardware/marine/thruster_array.pb.h"
#include "intrinsic/hardware/marine/thruster_array_policy.h"
#include "gtest/gtest.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::ThrusterArrayCommand;
using intrinsic_proto::hardware::marine::ThrusterArrayFeedback;
using intrinsic_proto::hardware::marine::ThrusterHealth;

void FillHeader(intrinsic_proto::embodiment::StampedHeader *header,
                int64_t source_seconds) {
  header->set_sequence(7);
  header->mutable_source_time()->set_seconds(source_seconds);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(source_seconds + 1);
  header->set_source_id("thruster_array");
  header->set_frame_id("body");
  header->set_clock_domain("monotonic");
  header->mutable_validity()->set_state(Validity::STATE_VALID);
}

ThrusterArrayCommand MakeCommand(double thrust_n, int64_t source_seconds,
                                 bool set_enable = false, bool enable = true) {
  ThrusterArrayCommand command;
  FillHeader(command.mutable_header(), source_seconds);
  auto *element = command.add_thrusters();
  element->set_name("surge_port");
  element->set_thrust_n(thrust_n);
  if (set_enable) {
    element->set_enable(enable);
  }
  return command;
}

ThrusterArrayAssessment Assess(const ThrusterArrayFeedback &message) {
  std::vector<ThrusterFeedbackElementView> elements;
  elements.reserve(message.thrusters_size());
  for (const auto &element : message.thrusters()) {
    ThrusterFeedbackElementView view;
    view.name_present = element.has_name();
    view.name = element.name();
    view.commanded_thrust_present = element.has_commanded_thrust_n();
    view.commanded_thrust_n = element.commanded_thrust_n();
    view.measured_thrust_present = element.has_measured_thrust_n();
    view.measured_thrust_n = element.measured_thrust_n();
    view.saturated_present = element.has_saturated();
    view.saturated = element.saturated();
    view.health_present = element.has_health();
    view.health = element.health();
    view.health_derate_present = element.has_health_derate();
    view.health_derate = element.health_derate();
    view.efficiency_present = element.has_efficiency();
    view.efficiency = element.efficiency();
    elements.push_back(view);
  }
  ThrusterArrayFeedbackView sample;
  const auto &header = message.header();
  sample.header.header_present = message.has_header();
  sample.header.frame_id = header.frame_id();
  sample.header.source_time_present = header.has_source_time();
  if (sample.header.source_time_present) {
    sample.header.source_time = embodiment::ClockReading{
        header.source_time().seconds(), header.source_time().nanos()};
  }
  sample.header.receive_time_present = header.has_receive_time();
  if (sample.header.receive_time_present) {
    sample.header.receive_time = embodiment::ClockReading{
        header.receive_time().seconds(), header.receive_time().nanos()};
  }
  sample.header.header_validity_present = header.has_validity();
  sample.header.header_validity_state = header.validity().state();
  sample.thrusters = elements;
  return AssessThrusterArrayFeedback(sample);
}

TEST(FakeThrusterArrayTest, NominalRoundTripIsAcceptedAndRepeatable) {
  const ThrusterArrayCommand command = MakeCommand(10.0, 1700000000);
  FakeThrusterArray first;
  FakeThrusterArray second;
  const std::optional<ThrusterArrayFeedback> left = first.Apply(command);
  const std::optional<ThrusterArrayFeedback> right = second.Apply(command);
  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_EQ(left->SerializeAsString(), right->SerializeAsString());
  ASSERT_EQ(left->thrusters_size(), 1);
  EXPECT_EQ(left->header().sequence(), 42u);
  EXPECT_EQ(left->header().frame_id(), "body");
  EXPECT_DOUBLE_EQ(left->thrusters(0).commanded_thrust_n(), 10.0);
  EXPECT_DOUBLE_EQ(left->thrusters(0).measured_thrust_n(), 10.0);
  EXPECT_FALSE(left->thrusters(0).saturated());
  EXPECT_EQ(left->thrusters(0).health(),
            ThrusterHealth::THRUSTER_HEALTH_NOMINAL);
  EXPECT_DOUBLE_EQ(left->thrusters(0).health_derate(), 1.0);
  EXPECT_DOUBLE_EQ(left->thrusters(0).efficiency(), 1.0);
  EXPECT_TRUE(Assess(*left).accepted);

  const std::optional<ThrusterArrayFeedback> next =
      first.Apply(MakeCommand(0.0, 1700000002));
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->header().sequence(), 43u);
  EXPECT_DOUBLE_EQ(next->thrusters(0).measured_thrust_n(), 0.0);
  EXPECT_TRUE(Assess(*next).accepted);
}

TEST(FakeThrusterArrayTest, LagDelaysThrustAndDoesNotRepairIt) {
  FakeThrusterArrayConfig config;
  config.lag_steps = 1;
  FakeThrusterArray fake(config);
  const ThrusterArrayCommand first_command = MakeCommand(10.0, 100);
  const ThrusterArrayCommand second_command = MakeCommand(25.0, 200);
  const std::optional<ThrusterArrayFeedback> first = fake.Apply(first_command);
  const std::optional<ThrusterArrayFeedback> second =
      fake.Apply(second_command);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->header().source_time().seconds(), 100);
  EXPECT_DOUBLE_EQ(first->thrusters(0).commanded_thrust_n(), 0.0);
  EXPECT_DOUBLE_EQ(first->thrusters(0).measured_thrust_n(), 0.0);
  EXPECT_EQ(second->header().source_time().seconds(), 200);
  EXPECT_EQ(second->header().sequence(), 43u);
  EXPECT_DOUBLE_EQ(second->thrusters(0).commanded_thrust_n(), 10.0);
  EXPECT_DOUBLE_EQ(second->thrusters(0).measured_thrust_n(), 10.0);
  EXPECT_TRUE(Assess(*first).accepted);
  EXPECT_TRUE(Assess(*second).accepted);
}

TEST(FakeThrusterArrayTest, SaturationClampsAndExactBoundsDoNot) {
  FakeThrusterArray high;
  const auto over = high.Apply(MakeCommand(1000.0, 1700000000));
  ASSERT_TRUE(over.has_value());
  EXPECT_DOUBLE_EQ(over->thrusters(0).commanded_thrust_n(), 1000.0);
  EXPECT_DOUBLE_EQ(over->thrusters(0).measured_thrust_n(), 50.0);
  EXPECT_TRUE(over->thrusters(0).saturated());
  EXPECT_TRUE(Assess(*over).accepted);

  FakeThrusterArray low;
  const auto under = low.Apply(MakeCommand(-1000.0, 1700000000));
  ASSERT_TRUE(under.has_value());
  EXPECT_DOUBLE_EQ(under->thrusters(0).measured_thrust_n(), -35.0);
  EXPECT_TRUE(under->thrusters(0).saturated());

  FakeThrusterArray at_forward;
  const auto forward = at_forward.Apply(MakeCommand(50.0, 1700000000));
  ASSERT_TRUE(forward.has_value());
  EXPECT_DOUBLE_EQ(forward->thrusters(0).measured_thrust_n(), 50.0);
  EXPECT_FALSE(forward->thrusters(0).saturated());

  FakeThrusterArray at_reverse;
  const auto reverse = at_reverse.Apply(MakeCommand(-35.0, 1700000000));
  ASSERT_TRUE(reverse.has_value());
  EXPECT_DOUBLE_EQ(reverse->thrusters(0).measured_thrust_n(), -35.0);
  EXPECT_FALSE(reverse->thrusters(0).saturated());

  ThrusterArrayCommand heave;
  FillHeader(heave.mutable_header(), 1700000000);
  // Slot 0 is surge. Put the heave command on slot 4.
  for (int i = 0; i < 4; ++i) {
    auto *pad = heave.add_thrusters();
    pad->set_thrust_n(0.0);
  }
  auto *heave_slot = heave.add_thrusters();
  heave_slot->set_name("heave_fore");
  heave_slot->set_thrust_n(-100.0);
  FakeThrusterArray heave_fake;
  const auto heave_feedback = heave_fake.Apply(heave);
  ASSERT_TRUE(heave_feedback.has_value());
  EXPECT_DOUBLE_EQ(heave_feedback->thrusters(4).measured_thrust_n(), -25.0);
  EXPECT_TRUE(heave_feedback->thrusters(4).saturated());
  EXPECT_TRUE(Assess(*heave_feedback).accepted);
}

TEST(FakeThrusterArrayTest, StuckOffEfficiencyDisableAndFailed) {
  FakeThrusterArrayConfig stuck_config;
  stuck_config.slots[0].fault_health_present = true;
  stuck_config.slots[0].fault_health =
      ThrusterHealth::THRUSTER_HEALTH_STUCK_OFF;
  stuck_config.slots[0].efficiency = 0.5;
  FakeThrusterArray stuck(stuck_config);
  const auto stuck_feedback = stuck.Apply(MakeCommand(10.0, 1700000000));
  ASSERT_TRUE(stuck_feedback.has_value());
  EXPECT_DOUBLE_EQ(stuck_feedback->thrusters(0).commanded_thrust_n(), 10.0);
  EXPECT_DOUBLE_EQ(stuck_feedback->thrusters(0).measured_thrust_n(), 0.0);
  EXPECT_EQ(stuck_feedback->thrusters(0).health(),
            ThrusterHealth::THRUSTER_HEALTH_STUCK_OFF);
  EXPECT_DOUBLE_EQ(stuck_feedback->thrusters(0).health_derate(), 0.0);
  EXPECT_DOUBLE_EQ(stuck_feedback->thrusters(0).efficiency(), 0.5);
  EXPECT_FALSE(stuck_feedback->thrusters(0).saturated());
  EXPECT_TRUE(Assess(*stuck_feedback).accepted);

  FakeThrusterArrayConfig loss_config;
  loss_config.slots[0].efficiency = 0.5;
  FakeThrusterArray loss(loss_config);
  const auto loss_feedback = loss.Apply(MakeCommand(10.0, 1700000000));
  ASSERT_TRUE(loss_feedback.has_value());
  EXPECT_DOUBLE_EQ(loss_feedback->thrusters(0).measured_thrust_n(), 5.0);
  EXPECT_EQ(loss_feedback->thrusters(0).health(),
            ThrusterHealth::THRUSTER_HEALTH_DERATED);
  EXPECT_DOUBLE_EQ(loss_feedback->thrusters(0).health_derate(), 0.5);
  EXPECT_FALSE(loss_feedback->thrusters(0).saturated());
  EXPECT_TRUE(Assess(*loss_feedback).accepted);

  FakeThrusterArrayConfig derate_config;
  derate_config.slots[0].fault_health_present = true;
  derate_config.slots[0].fault_health = ThrusterHealth::THRUSTER_HEALTH_DERATED;
  derate_config.slots[0].fault_derate = 0.5;
  FakeThrusterArray derated(derate_config);
  const auto derated_feedback = derated.Apply(MakeCommand(10.0, 1700000000));
  ASSERT_TRUE(derated_feedback.has_value());
  EXPECT_DOUBLE_EQ(derated_feedback->thrusters(0).measured_thrust_n(), 10.0);
  EXPECT_EQ(derated_feedback->thrusters(0).health(),
            ThrusterHealth::THRUSTER_HEALTH_DERATED);
  EXPECT_DOUBLE_EQ(derated_feedback->thrusters(0).health_derate(), 0.5);
  EXPECT_TRUE(Assess(*derated_feedback).accepted);

  const auto disabled =
      FakeThrusterArray().Apply(MakeCommand(10.0, 1700000000, true, false));
  ASSERT_TRUE(disabled.has_value());
  EXPECT_DOUBLE_EQ(disabled->thrusters(0).commanded_thrust_n(), 10.0);
  EXPECT_DOUBLE_EQ(disabled->thrusters(0).measured_thrust_n(), 0.0);
  EXPECT_EQ(disabled->thrusters(0).health(),
            ThrusterHealth::THRUSTER_HEALTH_DISABLED);
  EXPECT_DOUBLE_EQ(disabled->thrusters(0).health_derate(), 0.0);
  EXPECT_TRUE(Assess(*disabled).accepted);

  FakeThrusterArrayConfig failed_config;
  failed_config.slots[0].fault_health_present = true;
  failed_config.slots[0].fault_health = ThrusterHealth::THRUSTER_HEALTH_FAILED;
  FakeThrusterArray failed(failed_config);
  const auto failed_feedback =
      failed.Apply(MakeCommand(10.0, 1700000000, true, false));
  ASSERT_TRUE(failed_feedback.has_value());
  EXPECT_DOUBLE_EQ(failed_feedback->thrusters(0).measured_thrust_n(), 0.0);
  EXPECT_EQ(failed_feedback->thrusters(0).health(),
            ThrusterHealth::THRUSTER_HEALTH_FAILED);
  EXPECT_DOUBLE_EQ(failed_feedback->thrusters(0).health_derate(), 0.0);
  EXPECT_TRUE(Assess(*failed_feedback).accepted);
}

TEST(FakeThrusterArrayTest, SlewMovesTowardTheCommand) {
  FakeThrusterArrayConfig config;
  config.slew = true;
  config.dt_s = 0.1;
  config.slots = {config.slots[0]};
  config.slots[0].max_forward_slew_n_per_s = 100.0;
  config.slots[0].max_reverse_slew_n_per_s = 100.0;
  FakeThrusterArray fake(config);
  const auto first = fake.Apply(MakeCommand(50.0, 1700000000));
  const auto second = fake.Apply(MakeCommand(50.0, 1700000001));
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_DOUBLE_EQ(first->thrusters(0).measured_thrust_n(), 10.0);
  EXPECT_FALSE(first->thrusters(0).saturated());
  EXPECT_DOUBLE_EQ(second->thrusters(0).measured_thrust_n(), 20.0);
  EXPECT_TRUE(Assess(*first).accepted);
  EXPECT_TRUE(Assess(*second).accepted);
}

TEST(FakeThrusterArrayTest, DropoutOmitsFeedback) {
  FakeThrusterArrayConfig config;
  config.dropout = true;
  FakeThrusterArray fake(config);
  EXPECT_FALSE(fake.Apply(MakeCommand(10.0, 1700000000)).has_value());
}

TEST(FakeThrusterArrayTest, FixtureHasSixExampleBounds) {
  const std::vector<FakeThrusterSlotConfig> slots = SixThrusterFixtureSlots();
  ASSERT_EQ(slots.size(), 6u);
  EXPECT_EQ(slots[0].name, "surge_port");
  EXPECT_DOUBLE_EQ(slots[0].max_thrust_n, 50.0);
  EXPECT_DOUBLE_EQ(slots[0].min_thrust_n, -35.0);
  EXPECT_DOUBLE_EQ(slots[4].name == "heave_fore" ? slots[4].min_thrust_n : 0.0,
                   -25.0);
  EXPECT_EQ(slots[4].name, "heave_fore");
  EXPECT_DOUBLE_EQ(slots[4].max_thrust_n, 40.0);
}

} // namespace
} // namespace intrinsic::hardware::marine
