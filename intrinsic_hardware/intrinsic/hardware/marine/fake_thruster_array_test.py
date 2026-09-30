# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Determinism and fault-injection tests for FakeThrusterArray."""

import dataclasses
import unittest

from intrinsic.hardware.marine import fake_thruster_array
from intrinsic.hardware.marine import thruster_array_pb2
from intrinsic.hardware.marine import thruster_array_policy

from intrinsic.embodiment.proto import stamped_header_pb2

_HEALTH = thruster_array_pb2


def _fill_header(header, source_seconds):
  header.sequence = 7
  header.source_time.seconds = source_seconds
  header.source_time.nanos = 250000000
  header.receive_time.seconds = source_seconds + 1
  header.source_id = "thruster_array"
  header.frame_id = "body"
  header.clock_domain = "monotonic"
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID


def _command(thrust_n, source_seconds, set_enable=False, enable=True):
  command = thruster_array_pb2.ThrusterArrayCommand()
  _fill_header(command.header, source_seconds)
  element = command.thrusters.add()
  element.name = "surge_port"
  element.thrust_n = thrust_n
  if set_enable:
    element.enable = enable
  return command


def _assess(message):
  elements = []
  for element in message.thrusters:
    elements.append(
        thruster_array_policy.ThrusterFeedbackElementView(
            name_present=element.HasField("name"),
            name=element.name,
            commanded_thrust_present=element.HasField("commanded_thrust_n"),
            commanded_thrust_n=element.commanded_thrust_n,
            measured_thrust_present=element.HasField("measured_thrust_n"),
            measured_thrust_n=element.measured_thrust_n,
            saturated_present=element.HasField("saturated"),
            saturated=element.saturated,
            health_present=element.HasField("health"),
            health=element.health,
            health_derate_present=element.HasField("health_derate"),
            health_derate=element.health_derate,
            efficiency_present=element.HasField("efficiency"),
            efficiency=element.efficiency,
        )
    )
  header = message.header
  return thruster_array_policy.assess_thruster_array_feedback(
      thruster_array_policy.ThrusterArrayFeedbackView(
          header=thruster_array_policy.ThrusterHeaderView(
              header_present=message.HasField("header"),
              frame_id=header.frame_id,
              source_time_present=header.HasField("source_time"),
              source_time=(
                  header.source_time.seconds,
                  header.source_time.nanos,
              ),
              receive_time_present=header.HasField("receive_time"),
              receive_time=(
                  header.receive_time.seconds,
                  header.receive_time.nanos,
              ),
              header_validity_present=header.HasField("validity"),
              header_validity_state=header.validity.state,
          ),
          thrusters=tuple(elements),
      )
  )


class FakeThrusterArrayTest(unittest.TestCase):

  def test_nominal_round_trip_is_accepted_and_repeatable(self):
    command = _command(10.0, 1700000000)
    left = fake_thruster_array.FakeThrusterArray().apply(command)
    right = fake_thruster_array.FakeThrusterArray().apply(command)
    self.assertEqual(left.SerializeToString(), right.SerializeToString())
    self.assertEqual(left.header.sequence, 42)
    self.assertEqual(left.header.frame_id, "body")
    self.assertEqual(left.thrusters[0].commanded_thrust_n, 10.0)
    self.assertEqual(left.thrusters[0].measured_thrust_n, 10.0)
    self.assertFalse(left.thrusters[0].saturated)
    self.assertEqual(left.thrusters[0].health, _HEALTH.THRUSTER_HEALTH_NOMINAL)
    self.assertEqual(left.thrusters[0].health_derate, 1.0)
    self.assertEqual(left.thrusters[0].efficiency, 1.0)
    self.assertTrue(_assess(left).accepted)
    fake = fake_thruster_array.FakeThrusterArray()
    fake.apply(command)
    nxt = fake.apply(_command(0.0, 1700000002))
    self.assertEqual(nxt.header.sequence, 43)
    self.assertEqual(nxt.thrusters[0].measured_thrust_n, 0.0)
    self.assertTrue(_assess(nxt).accepted)

  def test_lag_delays_thrust_and_does_not_repair_it(self):
    config = fake_thruster_array.FakeThrusterArrayConfig(lag_steps=1)
    fake = fake_thruster_array.FakeThrusterArray(config)
    first = fake.apply(_command(10.0, 100))
    second = fake.apply(_command(25.0, 200))
    self.assertEqual(first.header.source_time.seconds, 100)
    self.assertEqual(first.thrusters[0].commanded_thrust_n, 0.0)
    self.assertEqual(first.thrusters[0].measured_thrust_n, 0.0)
    self.assertEqual(second.header.source_time.seconds, 200)
    self.assertEqual(second.header.sequence, 43)
    self.assertEqual(second.thrusters[0].commanded_thrust_n, 10.0)
    self.assertEqual(second.thrusters[0].measured_thrust_n, 10.0)
    self.assertTrue(_assess(first).accepted)
    self.assertTrue(_assess(second).accepted)

  def test_saturation_clamps_and_exact_bounds_do_not(self):
    over = fake_thruster_array.FakeThrusterArray().apply(
        _command(1000.0, 1700000000)
    )
    self.assertEqual(over.thrusters[0].commanded_thrust_n, 1000.0)
    self.assertEqual(over.thrusters[0].measured_thrust_n, 50.0)
    self.assertTrue(over.thrusters[0].saturated)
    self.assertTrue(_assess(over).accepted)

    under = fake_thruster_array.FakeThrusterArray().apply(
        _command(-1000.0, 1700000000)
    )
    self.assertEqual(under.thrusters[0].measured_thrust_n, -35.0)
    self.assertTrue(under.thrusters[0].saturated)

    forward = fake_thruster_array.FakeThrusterArray().apply(
        _command(50.0, 1700000000)
    )
    self.assertEqual(forward.thrusters[0].measured_thrust_n, 50.0)
    self.assertFalse(forward.thrusters[0].saturated)

    reverse = fake_thruster_array.FakeThrusterArray().apply(
        _command(-35.0, 1700000000)
    )
    self.assertEqual(reverse.thrusters[0].measured_thrust_n, -35.0)
    self.assertFalse(reverse.thrusters[0].saturated)

    heave = thruster_array_pb2.ThrusterArrayCommand()
    _fill_header(heave.header, 1700000000)
    for _ in range(4):
      pad = heave.thrusters.add()
      pad.thrust_n = 0.0
    slot = heave.thrusters.add()
    slot.name = "heave_fore"
    slot.thrust_n = -100.0
    feedback = fake_thruster_array.FakeThrusterArray().apply(heave)
    self.assertEqual(feedback.thrusters[4].measured_thrust_n, -25.0)
    self.assertTrue(feedback.thrusters[4].saturated)
    self.assertTrue(_assess(feedback).accepted)

  def test_stuck_off_efficiency_disable_and_failed(self):
    stuck_config = fake_thruster_array.FakeThrusterArrayConfig()
    stuck_config.slots[0].fault_health = _HEALTH.THRUSTER_HEALTH_STUCK_OFF
    stuck_config.slots[0].efficiency = 0.5
    stuck = fake_thruster_array.FakeThrusterArray(stuck_config).apply(
        _command(10.0, 1700000000)
    )
    self.assertEqual(stuck.thrusters[0].commanded_thrust_n, 10.0)
    self.assertEqual(stuck.thrusters[0].measured_thrust_n, 0.0)
    self.assertEqual(
        stuck.thrusters[0].health, _HEALTH.THRUSTER_HEALTH_STUCK_OFF
    )
    self.assertEqual(stuck.thrusters[0].health_derate, 0.0)
    self.assertEqual(stuck.thrusters[0].efficiency, 0.5)
    self.assertFalse(stuck.thrusters[0].saturated)
    self.assertTrue(_assess(stuck).accepted)

    loss_config = fake_thruster_array.FakeThrusterArrayConfig()
    loss_config.slots[0].efficiency = 0.5
    loss = fake_thruster_array.FakeThrusterArray(loss_config).apply(
        _command(10.0, 1700000000)
    )
    self.assertEqual(loss.thrusters[0].measured_thrust_n, 5.0)
    self.assertEqual(loss.thrusters[0].health, _HEALTH.THRUSTER_HEALTH_DERATED)
    self.assertEqual(loss.thrusters[0].health_derate, 0.5)
    self.assertFalse(loss.thrusters[0].saturated)
    self.assertTrue(_assess(loss).accepted)

    derate_config = fake_thruster_array.FakeThrusterArrayConfig()
    derate_config.slots[0].fault_health = _HEALTH.THRUSTER_HEALTH_DERATED
    derate_config.slots[0].fault_derate = 0.5
    derated = fake_thruster_array.FakeThrusterArray(derate_config).apply(
        _command(10.0, 1700000000)
    )
    self.assertEqual(derated.thrusters[0].measured_thrust_n, 10.0)
    self.assertEqual(
        derated.thrusters[0].health, _HEALTH.THRUSTER_HEALTH_DERATED
    )
    self.assertEqual(derated.thrusters[0].health_derate, 0.5)
    self.assertTrue(_assess(derated).accepted)

    disabled = fake_thruster_array.FakeThrusterArray().apply(
        _command(10.0, 1700000000, True, False)
    )
    self.assertEqual(disabled.thrusters[0].commanded_thrust_n, 10.0)
    self.assertEqual(disabled.thrusters[0].measured_thrust_n, 0.0)
    self.assertEqual(
        disabled.thrusters[0].health, _HEALTH.THRUSTER_HEALTH_DISABLED
    )
    self.assertEqual(disabled.thrusters[0].health_derate, 0.0)
    self.assertTrue(_assess(disabled).accepted)

    failed_config = fake_thruster_array.FakeThrusterArrayConfig()
    failed_config.slots[0].fault_health = _HEALTH.THRUSTER_HEALTH_FAILED
    failed = fake_thruster_array.FakeThrusterArray(failed_config).apply(
        _command(10.0, 1700000000, True, False)
    )
    self.assertEqual(failed.thrusters[0].measured_thrust_n, 0.0)
    self.assertEqual(failed.thrusters[0].health, _HEALTH.THRUSTER_HEALTH_FAILED)
    self.assertEqual(failed.thrusters[0].health_derate, 0.0)
    self.assertTrue(_assess(failed).accepted)

  def test_slew_moves_toward_the_command(self):
    config = fake_thruster_array.FakeThrusterArrayConfig(slew=True, dt_s=0.1)
    slot = dataclasses.replace(
        config.slots[0],
        max_forward_slew_n_per_s=100.0,
        max_reverse_slew_n_per_s=100.0,
    )
    config.slots = [slot]
    fake = fake_thruster_array.FakeThrusterArray(config)
    first = fake.apply(_command(50.0, 1700000000))
    second = fake.apply(_command(50.0, 1700000001))
    self.assertEqual(first.thrusters[0].measured_thrust_n, 10.0)
    self.assertFalse(first.thrusters[0].saturated)
    self.assertEqual(second.thrusters[0].measured_thrust_n, 20.0)
    self.assertTrue(_assess(first).accepted)
    self.assertTrue(_assess(second).accepted)

  def test_dropout_omits_feedback(self):
    config = fake_thruster_array.FakeThrusterArrayConfig(dropout=True)
    fake = fake_thruster_array.FakeThrusterArray(config)
    self.assertIsNone(fake.apply(_command(10.0, 1700000000)))

  def test_fixture_has_six_example_bounds(self):
    slots = fake_thruster_array.six_thruster_fixture_slots()
    self.assertEqual(len(slots), 6)
    self.assertEqual(slots[0].name, "surge_port")
    self.assertEqual(slots[0].max_thrust_n, 50.0)
    self.assertEqual(slots[0].min_thrust_n, -35.0)
    self.assertEqual(slots[4].name, "heave_fore")
    self.assertEqual(slots[4].min_thrust_n, -25.0)
    self.assertEqual(slots[4].max_thrust_n, 40.0)


if __name__ == "__main__":
  unittest.main()
