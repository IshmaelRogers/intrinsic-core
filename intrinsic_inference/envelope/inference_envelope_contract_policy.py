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

"""Plain-value checks for InferenceEnvelope and InferenceResult.

Policy: intrinsic_apis/intrinsic/inference/proto/README.md.

These helpers do not parse protobuf, do not call OIP or Triton, do not
queue, submit, poll, or cancel, and do not call World, ICON, or a HAL.
Expiry consumption is a later leaf. This policy rejects structurally
invalid envelopes only.
"""

from dataclasses import dataclass
import enum
import math

from intrinsic.embodiment import stamped_header_policy

# seconds, nanos. Nanos are in [0, 1000000000) when the time is usable.
TimeParts = tuple[int, int]


class InferenceEnvelopeContractError(enum.Enum):
  NONE = 0
  MISSING_FRAME = 1
  CREATION_TIME = 2
  DEADLINE = 3
  VALIDITY_HORIZON = 4
  OIP_IDENTIFIER = 5
  PROVENANCE = 6
  CONFIDENCE = 7
  UNCERTAINTY = 8
  DIGEST = 9
  SNAPSHOT_ID = 10


@dataclass(frozen=True)
class InferenceEnvelopeContractAssessment:
  """accepted is true only for no defect and a STATE_VALID header."""

  error: InferenceEnvelopeContractError = InferenceEnvelopeContractError.NONE
  validity: stamped_header_policy.ValidityKind = (
      stamped_header_policy.ValidityKind.ABSENT
  )
  accepted: bool = False


@dataclass(frozen=True)
class OipIdentifierView:
  """message_set is wire presence of the OIP identifier message."""

  message_set: bool = False
  model_name: str = ""
  model_version: str = ""
  request_id: str = ""


@dataclass(frozen=True)
class InferenceCommonView:
  """Fields shared by InferenceEnvelope and InferenceResult."""

  header_present: bool = False
  validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  # Header source_time is the creation time the deadline is checked against.
  source_time_present: bool = False
  source_time: TimeParts = (0, 0)
  state_epoch: int = 0
  world_snapshot_id: str = ""
  deadline_present: bool = False
  deadline: TimeParts = (0, 0)
  validity_horizon_present: bool = False
  validity_horizon: TimeParts = (0, 0)
  confidence_present: bool = False
  confidence: float = 0.0
  uncertainty_present: bool = False
  uncertainty: float = 0.0
  input_digest: str = ""
  provenance_present: bool = False
  provenance_model_id: str = ""
  # True when the metadata map has any entry. Values are not inspected.
  metadata_present: bool = False


@dataclass(frozen=True)
class InferenceEnvelopeView:
  common: InferenceCommonView = InferenceCommonView()
  oip_request: OipIdentifierView = OipIdentifierView()


@dataclass(frozen=True)
class InferenceResultView:
  common: InferenceCommonView = InferenceCommonView()
  oip_result: OipIdentifierView = OipIdentifierView()
  output_digest: str = ""


def oip_identifier_set(oip: OipIdentifierView) -> bool:
  return bool(
      oip.message_set or oip.model_name or oip.model_version or oip.request_id
  )


def inference_common_engaged(
    common: InferenceCommonView, oip: OipIdentifierView, output_digest: str
) -> bool:
  """Any signal engages. state_epoch 0 alone does not."""
  return bool(
      common.header_present
      or common.validity_present
      or common.frame_id
      or common.source_time_present
      or oip_identifier_set(oip)
      or common.state_epoch != 0
      or common.world_snapshot_id
      or common.deadline_present
      or common.validity_horizon_present
      or common.confidence_present
      or common.uncertainty_present
      or common.input_digest
      or output_digest
      or common.provenance_present
      or common.provenance_model_id
      or common.metadata_present
  )


def is_lowercase_sha256_hex(value: str) -> bool:
  return len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def confidence_in_range(confidence: float) -> bool:
  return math.isfinite(confidence) and 0.0 <= confidence <= 1.0


def uncertainty_in_range(uncertainty: float) -> bool:
  return math.isfinite(uncertainty) and uncertainty >= 0.0


def duration_non_negative(duration: TimeParts) -> bool:
  return duration[0] >= 0 and stamped_header_policy.nanos_in_range(duration[1])


def _time_before(a: TimeParts, b: TimeParts) -> bool:
  return a[0] < b[0] or (a[0] == b[0] and a[1] < b[1])


def _make_assessment(
    error: InferenceEnvelopeContractError,
    validity: stamped_header_policy.ValidityKind,
) -> InferenceEnvelopeContractAssessment:
  accepted = (
      error is InferenceEnvelopeContractError.NONE
      and stamped_header_policy.sample_accepted(validity, True)
  )
  return InferenceEnvelopeContractAssessment(error, validity, accepted)


def assess_inference_common(
    common: InferenceCommonView,
    oip: OipIdentifierView,
    output_digest: str,
    require_output_digest: bool,
) -> InferenceEnvelopeContractError:
  """First defect wins.

  Check order: frame, creation time, deadline, validity horizon, OIP ids,
  provenance, confidence, uncertainty, digests, snapshot id. Metadata is
  always tolerated. The digest step needs input_digest, and also
  output_digest when require_output_digest is true (results).
  """
  error = InferenceEnvelopeContractError
  nanos_in_range = stamped_header_policy.nanos_in_range
  if not common.frame_id:
    return error.MISSING_FRAME
  if not common.source_time_present or not nanos_in_range(
      common.source_time[1]
  ):
    return error.CREATION_TIME
  if (
      not common.deadline_present
      or not nanos_in_range(common.deadline[1])
      or _time_before(common.deadline, common.source_time)
  ):
    return error.DEADLINE
  if not common.validity_horizon_present or not duration_non_negative(
      common.validity_horizon
  ):
    return error.VALIDITY_HORIZON
  if not oip.model_name or not oip.request_id:
    return error.OIP_IDENTIFIER
  if not common.provenance_present or not common.provenance_model_id:
    return error.PROVENANCE
  if common.confidence_present and not confidence_in_range(common.confidence):
    return error.CONFIDENCE
  if common.uncertainty_present and not uncertainty_in_range(
      common.uncertainty
  ):
    return error.UNCERTAINTY
  if not common.input_digest or (require_output_digest and not output_digest):
    return error.DIGEST
  if common.world_snapshot_id and not is_lowercase_sha256_hex(
      common.world_snapshot_id
  ):
    return error.SNAPSHOT_ID
  return error.NONE


def assess_inference_envelope(
    envelope: InferenceEnvelopeView,
) -> InferenceEnvelopeContractAssessment:
  """An empty envelope is not engaged: NONE, ABSENT, not accepted."""
  if not inference_common_engaged(envelope.common, envelope.oip_request, ""):
    return InferenceEnvelopeContractAssessment()
  validity = stamped_header_policy.classify_validity(
      envelope.common.validity_present, envelope.common.validity_state
  )
  return _make_assessment(
      assess_inference_common(envelope.common, envelope.oip_request, "", False),
      validity,
  )


def assess_inference_result(
    result: InferenceResultView,
) -> InferenceEnvelopeContractAssessment:
  """An empty result is not engaged: NONE, ABSENT, not accepted."""
  if not inference_common_engaged(
      result.common, result.oip_result, result.output_digest
  ):
    return InferenceEnvelopeContractAssessment()
  validity = stamped_header_policy.classify_validity(
      result.common.validity_present, result.common.validity_state
  )
  return _make_assessment(
      assess_inference_common(
          result.common, result.oip_result, result.output_digest, True
      ),
      validity,
  )
