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

"""Validation tests for the thruster array policy."""

import math
import unittest

from intrinsic.hardware.marine import thruster_array_policy

from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy

_ERROR = thruster_array_policy.ThrusterArrayError
_HEALTH = thruster_array_policy.ThrusterHealthKind
_NAN = float("nan")
_INF = float("inf")


def _header():
  return thruster_array_policy.ThrusterHeaderView(
      header_present=True,
      frame_id=vehicle_contract_policy.BODY_FRAME_ID,
      source_time_present=True,
      source_time=(1700000000, 250000000),
      receive_time_present=True,
      receive_time=(1700000001, 0),
      header_validity_present=True,
      header_validity_state=1,
  )


def _command(thrust_n, name="surge_port"):
  return thruster_array_policy.ThrusterCommandElementView(
      name_present=True,
      name=name,
      thrust_present=True,
      thrust_n=thrust_n,
  )


def _feedback(thrust_n, health=0, derate=1.0, efficiency=1.0):
  return thruster_array_policy.ThrusterFeedbackElementView(
      name_present=True,
      name="surge_port",
      commanded_thrust_present=True,
      commanded_thrust_n=thrust_n,
      measured_thrust_present=True,
      measured_thrust_n=thrust_n,
      saturated_present=True,
      saturated=False,
      health_present=True,
      health=health,
      health_derate_present=True,
      health_derate=derate,
      efficiency_present=True,
      efficiency=efficiency,
  )


def _assess_command(header, elements):
  return thruster_array_policy.assess_thruster_array_command(
      thruster_array_policy.ThrusterArrayCommandView(header, tuple(elements))
  )


def _assess_feedback(header, elements):
  return thruster_array_policy.assess_thruster_array_feedback(
      thruster_array_policy.ThrusterArrayFeedbackView(header, tuple(elements))
  )


class ThrusterArrayPolicyTest(unittest.TestCase):

  def test_empty_messages_are_absent_and_not_errors(self):
    command = thruster_array_policy.assess_thruster_array_command(
        thruster_array_policy.ThrusterArrayCommandView()
    )
    self.assertEqual(command.error, _ERROR.NONE)
    self.assertEqual(
        command.header_validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertFalse(command.accepted)
    feedback = thruster_array_policy.assess_thruster_array_feedback(
        thruster_array_policy.ThrusterArrayFeedbackView()
    )
    self.assertEqual(feedback.error, _ERROR.NONE)
    self.assertFalse(feedback.accepted)

  def test_nominal_command_and_zero_thrust_are_accepted(self):
    assessment = _assess_command(
        _header(), (_command(10.0), _command(0.0, "sway_fore"))
    )
    self.assertEqual(assessment.error, _ERROR.NONE)
    self.assertEqual(
        assessment.header_validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertTrue(assessment.accepted)

  def test_unset_enable_is_enabled_and_explicit_false_is_kept(self):
    unset = _command(4.0)
    self.assertFalse(unset.enable_present)
    self.assertTrue(thruster_array_policy.thruster_command_enabled(unset))
    disabled = thruster_array_policy.ThrusterCommandElementView(
        name_present=True,
        name="surge_port",
        thrust_present=True,
        thrust_n=4.0,
        enable_present=True,
        enable=False,
    )
    self.assertFalse(thruster_array_policy.thruster_command_enabled(disabled))
    self.assertTrue(_assess_command(_header(), (disabled,)).accepted)

  def test_header_defects_are_first_and_ordered(self):
    elements = (_command(_NAN),)
    self.assertEqual(
        _assess_command(
            thruster_array_policy.ThrusterHeaderView(), elements
        ).error,
        _ERROR.MISSING_HEADER,
    )
    empty_frame = _header()
    self.assertEqual(
        _assess_command(
            thruster_array_policy.ThrusterHeaderView(
                header_present=True,
                frame_id="",
                source_time_present=empty_frame.source_time_present,
                source_time=empty_frame.source_time,
                receive_time_present=empty_frame.receive_time_present,
                receive_time=empty_frame.receive_time,
                header_validity_present=True,
                header_validity_state=1,
            ),
            elements,
        ).error,
        _ERROR.MISSING_FRAME,
    )
    world = thruster_array_policy.ThrusterHeaderView(
        header_present=True,
        frame_id="world_enu",
        source_time_present=True,
        source_time=(1700000000, 250000000),
        receive_time_present=True,
        receive_time=(1700000001, 0),
        header_validity_present=True,
        header_validity_state=1,
    )
    self.assertEqual(_assess_command(world, elements).error, _ERROR.WRONG_FRAME)
    reversed_header = thruster_array_policy.ThrusterHeaderView(
        header_present=True,
        frame_id="body",
        source_time_present=True,
        source_time=(1700000000, 250000000),
        receive_time_present=True,
        receive_time=(1700000000, 1),
        header_validity_present=True,
        header_validity_state=1,
    )
    self.assertEqual(
        _assess_command(reversed_header, elements).error, _ERROR.TIME_REVERSAL
    )

  def test_empty_array_and_element_defects(self):
    self.assertEqual(_assess_command(_header(), ()).error, _ERROR.EMPTY_ARRAY)
    empty_name = thruster_array_policy.ThrusterCommandElementView(
        name_present=True,
        name="",
        thrust_present=True,
        thrust_n=1.0,
    )
    self.assertEqual(
        _assess_command(_header(), (empty_name, _command(_NAN))).error,
        _ERROR.EMPTY_NAME,
    )
    missing = thruster_array_policy.ThrusterCommandElementView(
        name_present=True, name="surge_port"
    )
    self.assertEqual(
        _assess_command(_header(), (missing,)).error, _ERROR.MISSING_THRUST
    )
    for value in (_NAN, _INF, -_INF):
      assessment = _assess_command(_header(), (_command(value),))
      self.assertEqual(assessment.error, _ERROR.NON_FINITE)
      self.assertFalse(assessment.accepted)

  def test_invalid_header_validity_does_not_reject(self):
    header = thruster_array_policy.ThrusterHeaderView(
        header_present=True,
        frame_id="body",
        source_time_present=True,
        source_time=(1700000000, 250000000),
        receive_time_present=True,
        receive_time=(1700000001, 0),
        header_validity_present=True,
        header_validity_state=2,
    )
    assessment = _assess_command(header, (_command(1.0),))
    self.assertEqual(
        assessment.header_validity, stamped_header_policy.ValidityKind.INVALID
    )
    self.assertTrue(assessment.accepted)

  def test_nominal_feedback_derate_and_efficiency_boundaries(self):
    header = _header()
    self.assertTrue(_assess_feedback(header, (_feedback(0.0),)).accepted)
    self.assertTrue(
        _assess_feedback(header, (_feedback(5.0, 2, 0.5, 1.0),)).accepted
    )
    self.assertTrue(
        _assess_feedback(header, (_feedback(5.0, 2, 0.25, 0.25),)).accepted
    )
    for health in (1, 3, 4):
      self.assertTrue(
          _assess_feedback(
              header, (_feedback(0.0, health, 0.0, 1.0),)
          ).accepted,
          health,
      )

  def test_feedback_metadata_defects(self):
    header = _header()
    commanded = _feedback(1.0)
    commanded = thruster_array_policy.ThrusterFeedbackElementView(
        **{**commanded.__dict__, "commanded_thrust_n": _NAN}
    )
    self.assertEqual(
        _assess_feedback(header, (commanded,)).error, _ERROR.NON_FINITE
    )
    measured = thruster_array_policy.ThrusterFeedbackElementView(
        **{**_feedback(1.0).__dict__, "measured_thrust_n": _INF}
    )
    self.assertEqual(
        _assess_feedback(header, (measured,)).error, _ERROR.NON_FINITE
    )
    derate = thruster_array_policy.ThrusterFeedbackElementView(
        **{**_feedback(1.0).__dict__, "health_derate": _NAN}
    )
    self.assertEqual(
        _assess_feedback(header, (derate,)).error, _ERROR.NON_FINITE
    )
    efficiency = thruster_array_policy.ThrusterFeedbackElementView(
        **{**_feedback(1.0).__dict__, "efficiency": _NAN}
    )
    self.assertEqual(
        _assess_feedback(header, (efficiency,)).error, _ERROR.NON_FINITE
    )
    unknown = _feedback(1.0, 99, 0.0)
    self.assertEqual(_assess_feedback(header, (unknown,)).error, _ERROR.HEALTH)
    self.assertEqual(
        thruster_array_policy.classify_thruster_health(True, 99),
        _HEALTH.UNRECOGNIZED,
    )
    self.assertEqual(
        thruster_array_policy.classify_thruster_health(False, 0), _HEALTH.ABSENT
    )
    mismatch = _feedback(1.0, 0, 0.5)
    self.assertEqual(
        _assess_feedback(header, (mismatch,)).error, _ERROR.HEALTH_DERATE
    )
    self.assertEqual(
        _assess_feedback(header, (_feedback(1.0, 2, 1.0),)).error,
        _ERROR.HEALTH_DERATE,
    )
    self.assertEqual(
        _assess_feedback(header, (_feedback(1.0, 2, 0.0),)).error,
        _ERROR.HEALTH_DERATE,
    )
    self.assertEqual(
        _assess_feedback(header, (_feedback(1.0, 1, 1.0),)).error,
        _ERROR.HEALTH_DERATE,
    )
    missing_derate = thruster_array_policy.ThrusterFeedbackElementView(
        **{**_feedback(1.0).__dict__, "health_derate_present": False}
    )
    self.assertEqual(
        _assess_feedback(header, (missing_derate,)).error, _ERROR.HEALTH_DERATE
    )
    self.assertEqual(
        _assess_feedback(header, (_feedback(1.0, 0, 1.0, 0.0),)).error,
        _ERROR.EFFICIENCY,
    )
    self.assertEqual(
        _assess_feedback(header, (_feedback(1.0, 0, 1.0, 1.1),)).error,
        _ERROR.EFFICIENCY,
    )

  def test_feedback_optional_absence_and_first_defect(self):
    sparse = thruster_array_policy.ThrusterFeedbackElementView(
        measured_thrust_present=True, measured_thrust_n=0.0
    )
    self.assertTrue(_assess_feedback(_header(), (sparse,)).accepted)
    empty_frame = thruster_array_policy.ThrusterHeaderView(
        header_present=True, frame_id=""
    )
    bad = thruster_array_policy.ThrusterFeedbackElementView(
        **{**_feedback(1.0, 99, 0.0).__dict__, "efficiency": 4.0}
    )
    self.assertEqual(
        _assess_feedback(empty_frame, (bad,)).error, _ERROR.MISSING_FRAME
    )
    self.assertEqual(_assess_feedback(_header(), ()).error, _ERROR.EMPTY_ARRAY)

  def test_health_kind_matches_wire_numbers(self):
    self.assertEqual(
        thruster_array_policy.classify_thruster_health(True, 0), _HEALTH.NOMINAL
    )
    self.assertEqual(
        thruster_array_policy.classify_thruster_health(True, 1),
        _HEALTH.DISABLED,
    )
    self.assertEqual(
        thruster_array_policy.classify_thruster_health(True, 2), _HEALTH.DERATED
    )
    self.assertEqual(
        thruster_array_policy.classify_thruster_health(True, 3),
        _HEALTH.STUCK_OFF,
    )
    self.assertEqual(
        thruster_array_policy.classify_thruster_health(True, 4), _HEALTH.FAILED
    )
    self.assertTrue(math.isfinite(1.0))


if __name__ == "__main__":
  unittest.main()
