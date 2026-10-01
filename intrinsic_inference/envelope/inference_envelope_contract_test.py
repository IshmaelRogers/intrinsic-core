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

"""Plain-value tests for the inference envelope host policy."""

import dataclasses
import math
import unittest

from intrinsic.embodiment import stamped_header_policy
from intrinsic_inference.envelope import inference_envelope_contract_policy as policy

_Error = policy.InferenceEnvelopeContractError
_Kind = stamped_header_policy.ValidityKind
_SNAPSHOT = "0123456789abcdef" * 4


def _common(**overrides):
  fields = dict(
      header_present=True,
      validity_present=True,
      validity_state=1,
      frame_id="world_enu",
      source_time_present=True,
      source_time=(1700000000, 0),
      state_epoch=42,
      world_snapshot_id=_SNAPSHOT,
      deadline_present=True,
      deadline=(1700000000, 500000000),
      validity_horizon_present=True,
      validity_horizon=(2, 0),
      confidence_present=True,
      confidence=0.9,
      uncertainty_present=True,
      uncertainty=0.05,
      input_digest="sha256:in",
      provenance_present=True,
      provenance_model_id="pose_estimator",
      metadata_present=True,
  )
  fields.update(overrides)
  return policy.InferenceCommonView(**fields)


def _oip(**overrides):
  fields = dict(
      message_set=True,
      model_name="pose_estimator",
      model_version="3",
      request_id="req-0001",
  )
  fields.update(overrides)
  return policy.OipIdentifierView(**fields)


def _envelope(oip=None, **overrides):
  return policy.InferenceEnvelopeView(
      common=_common(**overrides), oip_request=oip or _oip()
  )


def _result(oip=None, output_digest="sha256:out", **overrides):
  return policy.InferenceResultView(
      common=_common(**overrides),
      oip_result=oip or _oip(),
      output_digest=output_digest,
  )


def _assess_envelope(**overrides):
  return policy.assess_inference_envelope(_envelope(**overrides))


def _assess_result(**overrides):
  return policy.assess_inference_result(_result(**overrides))


class InferenceEnvelopeContractTest(unittest.TestCase):

  def test_empty_envelope_is_not_engaged(self):
    assessment = policy.assess_inference_envelope(
        policy.InferenceEnvelopeView()
    )
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.ABSENT)
    self.assertFalse(assessment.accepted)

  def test_empty_result_is_not_engaged(self):
    assessment = policy.assess_inference_result(policy.InferenceResultView())
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.ABSENT)
    self.assertFalse(assessment.accepted)

  def test_state_epoch_zero_alone_is_not_engaged(self):
    view = policy.InferenceEnvelopeView(
        common=policy.InferenceCommonView(state_epoch=0)
    )
    self.assertFalse(policy.assess_inference_envelope(view).accepted)
    self.assertIs(policy.assess_inference_envelope(view).error, _Error.NONE)

  def test_any_single_signal_engages(self):
    signals = (
        dict(header_present=True),
        dict(state_epoch=1),
        dict(world_snapshot_id=_SNAPSHOT),
        dict(deadline_present=True),
        dict(validity_horizon_present=True),
        dict(confidence_present=True),
        dict(uncertainty_present=True),
        dict(input_digest="d"),
        dict(provenance_present=True),
        dict(metadata_present=True),
    )
    for signal in signals:
      with self.subTest(signal=signal):
        view = policy.InferenceEnvelopeView(
            common=policy.InferenceCommonView(**signal)
        )
        self.assertIs(
            policy.assess_inference_envelope(view).error, _Error.MISSING_FRAME
        )

  def test_empty_but_set_oip_message_engages(self):
    view = policy.InferenceEnvelopeView(
        oip_request=policy.OipIdentifierView(message_set=True)
    )
    self.assertIs(
        policy.assess_inference_envelope(view).error, _Error.MISSING_FRAME
    )

  def test_nominal_envelope_is_accepted(self):
    assessment = _assess_envelope()
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.VALID)
    self.assertTrue(assessment.accepted)

  def test_nominal_result_is_accepted(self):
    assessment = _assess_result()
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.VALID)
    self.assertTrue(assessment.accepted)

  def test_optional_fields_may_be_unset(self):
    assessment = _assess_envelope(
        confidence_present=False,
        uncertainty_present=False,
        world_snapshot_id="",
        state_epoch=0,
        metadata_present=False,
    )
    self.assertTrue(assessment.accepted)

  def test_empty_model_version_is_tolerated(self):
    view = _envelope(oip=_oip(model_version=""))
    self.assertTrue(policy.assess_inference_envelope(view).accepted)

  def test_validity_kinds_other_than_valid_are_not_accepted(self):
    cases = (
        (dict(validity_present=False), _Kind.ABSENT),
        (dict(validity_state=0), _Kind.UNSPECIFIED),
        (dict(validity_state=2), _Kind.INVALID),
    )
    for overrides, kind in cases:
      with self.subTest(kind=kind):
        assessment = _assess_envelope(**overrides)
        self.assertIs(assessment.error, _Error.NONE)
        self.assertIs(assessment.validity, kind)
        self.assertFalse(assessment.accepted)

  def test_defect_with_valid_header_is_not_accepted(self):
    assessment = _assess_envelope(frame_id="")
    self.assertIs(assessment.validity, _Kind.VALID)
    self.assertFalse(assessment.accepted)

  def test_missing_frame(self):
    self.assertIs(_assess_envelope(frame_id="").error, _Error.MISSING_FRAME)
    self.assertIs(_assess_result(frame_id="").error, _Error.MISSING_FRAME)

  def test_creation_time(self):
    self.assertIs(
        _assess_envelope(source_time_present=False).error, _Error.CREATION_TIME
    )
    self.assertIs(
        _assess_envelope(source_time=(1700000000, 1000000000)).error,
        _Error.CREATION_TIME,
    )
    self.assertIs(
        _assess_envelope(source_time=(1700000000, -1)).error,
        _Error.CREATION_TIME,
    )

  def test_deadline_before_creation_is_rejected(self):
    for deadline in ((1699999999, 999999999), (1699999999, 0), (0, 0)):
      with self.subTest(deadline=deadline):
        self.assertIs(
            _assess_envelope(deadline=deadline).error, _Error.DEADLINE
        )
        self.assertIs(_assess_result(deadline=deadline).error, _Error.DEADLINE)

  def test_deadline_nanos_before_creation_nanos_is_rejected(self):
    assessment = _assess_envelope(
        source_time=(1700000000, 500), deadline=(1700000000, 499)
    )
    self.assertIs(assessment.error, _Error.DEADLINE)

  def test_deadline_equal_to_creation_is_accepted(self):
    self.assertTrue(
        _assess_envelope(
            source_time=(1700000000, 7), deadline=(1700000000, 7)
        ).accepted
    )
    self.assertTrue(
        _assess_result(
            source_time=(1700000000, 7), deadline=(1700000000, 7)
        ).accepted
    )

  def test_deadline_one_nanosecond_after_creation_is_accepted(self):
    self.assertTrue(
        _assess_envelope(
            source_time=(1700000000, 7), deadline=(1700000000, 8)
        ).accepted
    )

  def test_deadline_missing_or_bad_nanos_is_rejected(self):
    self.assertIs(
        _assess_envelope(deadline_present=False).error, _Error.DEADLINE
    )
    self.assertIs(
        _assess_envelope(deadline=(1700000001, 1000000000)).error,
        _Error.DEADLINE,
    )
    self.assertIs(
        _assess_envelope(deadline=(1700000001, -1)).error, _Error.DEADLINE
    )

  def test_validity_horizon(self):
    self.assertIs(
        _assess_envelope(validity_horizon_present=False).error,
        _Error.VALIDITY_HORIZON,
    )
    self.assertIs(
        _assess_envelope(validity_horizon=(-1, 0)).error,
        _Error.VALIDITY_HORIZON,
    )
    self.assertIs(
        _assess_envelope(validity_horizon=(0, -1)).error,
        _Error.VALIDITY_HORIZON,
    )
    self.assertIs(
        _assess_envelope(validity_horizon=(1, 1000000000)).error,
        _Error.VALIDITY_HORIZON,
    )

  def test_zero_validity_horizon_is_accepted(self):
    self.assertTrue(_assess_envelope(validity_horizon=(0, 0)).accepted)

  def test_oip_identifier(self):
    for oip in (
        _oip(model_name=""),
        _oip(request_id=""),
        policy.OipIdentifierView(),
    ):
      with self.subTest(oip=oip):
        self.assertIs(
            policy.assess_inference_envelope(_envelope(oip=oip)).error,
            _Error.OIP_IDENTIFIER,
        )
        self.assertIs(
            policy.assess_inference_result(_result(oip=oip)).error,
            _Error.OIP_IDENTIFIER,
        )

  def test_provenance(self):
    self.assertIs(
        _assess_envelope(provenance_present=False).error, _Error.PROVENANCE
    )
    self.assertIs(
        _assess_envelope(provenance_model_id="").error, _Error.PROVENANCE
    )
    self.assertIs(
        _assess_result(provenance_model_id="").error, _Error.PROVENANCE
    )

  def test_confidence_boundaries(self):
    self.assertTrue(_assess_envelope(confidence=0.0).accepted)
    self.assertTrue(_assess_envelope(confidence=1.0).accepted)
    for bad in (
        -0.0000001,
        1.0000001,
        1.5,
        -1.0,
        math.nan,
        math.inf,
        -math.inf,
    ):
      with self.subTest(confidence=bad):
        self.assertIs(_assess_envelope(confidence=bad).error, _Error.CONFIDENCE)
        self.assertIs(_assess_result(confidence=bad).error, _Error.CONFIDENCE)

  def test_unset_confidence_is_not_zero(self):
    self.assertTrue(_assess_envelope(confidence_present=False).accepted)
    self.assertIs(
        _assess_envelope(confidence_present=False, confidence=7.0).error,
        _Error.NONE,
    )

  def test_uncertainty_boundaries(self):
    self.assertTrue(_assess_envelope(uncertainty=0.0).accepted)
    self.assertTrue(_assess_envelope(uncertainty=1e9).accepted)
    for bad in (-0.0000001, -1.0, math.nan, math.inf, -math.inf):
      with self.subTest(uncertainty=bad):
        self.assertIs(
            _assess_envelope(uncertainty=bad).error, _Error.UNCERTAINTY
        )
        self.assertIs(_assess_result(uncertainty=bad).error, _Error.UNCERTAINTY)

  def test_unset_uncertainty_is_not_zero(self):
    self.assertTrue(
        _assess_envelope(uncertainty_present=False, uncertainty=-5.0).accepted
    )

  def test_digests(self):
    self.assertIs(_assess_envelope(input_digest="").error, _Error.DIGEST)
    self.assertIs(_assess_result(input_digest="").error, _Error.DIGEST)
    self.assertIs(_assess_result(output_digest="").error, _Error.DIGEST)

  def test_envelope_does_not_need_output_digest(self):
    self.assertTrue(_assess_envelope().accepted)

  def test_snapshot_id(self):
    self.assertTrue(_assess_envelope(world_snapshot_id="").accepted)
    bad_ids = (
        "not-a-hex",
        "0123456789abcdef" * 4 + "0",
        "0123456789abcde",
        "0123456789ABCDEF" * 4,
        "g" + "0" * 63,
        " " + "0" * 63,
    )
    for bad in bad_ids:
      with self.subTest(snapshot=bad):
        self.assertIs(
            _assess_envelope(world_snapshot_id=bad).error, _Error.SNAPSHOT_ID
        )
        self.assertIs(
            _assess_result(world_snapshot_id=bad).error, _Error.SNAPSHOT_ID
        )

  def test_metadata_is_always_tolerated(self):
    self.assertTrue(_assess_envelope(metadata_present=True).accepted)
    self.assertTrue(_assess_envelope(metadata_present=False).accepted)

  def test_state_epoch_values_are_tolerated(self):
    for epoch in (0, 1, 2**64 - 1):
      with self.subTest(epoch=epoch):
        self.assertTrue(_assess_envelope(state_epoch=epoch).accepted)

  def test_first_defect_wins(self):
    # Each step is broken together with every later step.
    everything_later_broken = dict(
        source_time_present=False,
        deadline_present=False,
        validity_horizon_present=False,
        provenance_present=False,
        confidence=2.0,
        uncertainty=-1.0,
        input_digest="",
        world_snapshot_id="bad",
    )
    broken_oip = policy.OipIdentifierView(message_set=True)
    order = (
        ("frame_id", _Error.MISSING_FRAME),
        ("source_time_present", _Error.CREATION_TIME),
        ("deadline_present", _Error.DEADLINE),
        ("validity_horizon_present", _Error.VALIDITY_HORIZON),
        ("oip", _Error.OIP_IDENTIFIER),
        ("provenance_present", _Error.PROVENANCE),
        ("confidence", _Error.CONFIDENCE),
        ("uncertainty", _Error.UNCERTAINTY),
        ("input_digest", _Error.DIGEST),
        ("world_snapshot_id", _Error.SNAPSHOT_ID),
    )
    keys = [key for key, _ in order]
    for index, (key, expected) in enumerate(order):
      remaining = set(keys[index:])
      overrides = {
          name: value
          for name, value in everything_later_broken.items()
          if name in remaining
      }
      if key == "frame_id":
        overrides["frame_id"] = ""
      oip = broken_oip if "oip" in remaining else _oip()
      with self.subTest(first=key):
        assessment = policy.assess_inference_envelope(
            policy.InferenceEnvelopeView(
                common=_common(**overrides), oip_request=oip
            )
        )
        self.assertIs(assessment.error, expected)

  def test_result_output_digest_is_checked_with_input_digest(self):
    both = _assess_result(input_digest="", output_digest="")
    self.assertIs(both.error, _Error.DIGEST)
    self.assertIs(
        _assess_result(output_digest="", world_snapshot_id="bad").error,
        _Error.DIGEST,
    )

  def test_views_are_frozen(self):
    with self.assertRaises(dataclasses.FrozenInstanceError):
      _common().frame_id = "x"


if __name__ == "__main__":
  unittest.main()
