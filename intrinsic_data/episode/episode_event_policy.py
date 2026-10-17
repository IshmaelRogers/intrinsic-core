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

"""Plain-value checks for EpisodeEvent.

Policy: intrinsic_apis/intrinsic/episode/proto/README.md.

These helpers do not parse protobuf, do not write, read, or replay a stream,
do not check sequence advance, do not order times, and do not re-validate
inner payloads. Consumers use the SafetyDecision, VehicleTrajectory, and
Inference* policies for those.
"""

from dataclasses import dataclass
import enum
import hashlib

from intrinsic.embodiment import stamped_header_policy

# seconds, nanos. Nanos are in [0, 1000000000) when the time is usable.
TimeParts = tuple[int, int]

# Payload oneof case numbers. They equal the EpisodeEvent field numbers. The
# serialization tests assert this against the generated descriptor.
PAYLOAD_CASE_NOT_SET = 0
PAYLOAD_CASE_SAFETY_DECISION = 10
PAYLOAD_CASE_VEHICLE_TRAJECTORY = 11
PAYLOAD_CASE_INFERENCE_ENVELOPE = 12
PAYLOAD_CASE_INFERENCE_RESULT = 13
PAYLOAD_CASE_EPISODE_METADATA = 14
PAYLOAD_CASE_EPISODE_LOSS = 15

SHA256_HEX_LENGTH = 64


class EpisodeEventError(enum.Enum):
  """Append-only. Numbers are permanent."""

  NONE = 0
  MISSING_HEADER = 1
  SOURCE_ID = 2
  MISSING_FRAME = 3
  SOURCE_TIME = 4
  RECEIVE_TIME = 5
  SCHEMA_VERSION = 6
  SNAPSHOT_ID = 7
  CONFIG_DIGEST = 8
  CONFIG_DIGEST_MISMATCH = 9
  MISSING_PAYLOAD = 10


class PayloadKind(enum.Enum):
  # The payload oneof is not set and there are no unknown fields.
  NONE = 0
  SAFETY_DECISION = 1
  VEHICLE_TRAJECTORY = 2
  INFERENCE_ENVELOPE = 3
  INFERENCE_RESULT = 4
  EPISODE_METADATA = 5
  # The oneof is not set but the message carries unknown fields: a payload
  # case this reader does not know.
  UNKNOWN = 6
  # Writer-emitted loss record (case 15). Appended after UNKNOWN so no
  # existing number changes.
  EPISODE_LOSS = 7


@dataclass(frozen=True)
class EpisodeEventAssessment:
  """skip is true for a valid event with a payload this reader does not know.

  The reader skips it and keeps the unknown bytes. engaged is false for an
  empty message: absent, not an error.
  """

  error: EpisodeEventError = EpisodeEventError.NONE
  payload_kind: PayloadKind = PayloadKind.NONE
  skip: bool = False
  engaged: bool = False


@dataclass(frozen=True)
class EpisodeEventView:
  header_present: bool = False
  source_id: str = ""
  frame_id: str = ""
  source_time_present: bool = False
  source_time: TimeParts = (0, 0)
  receive_time_present: bool = False
  receive_time: TimeParts = (0, 0)
  schema_version: int = 0
  snapshot_id: str = ""
  config_digest: str = ""
  provenance_present: bool = False
  # The oneof case number, 0 when not set.
  payload_case: int = PAYLOAD_CASE_NOT_SET
  # True when the message carries any unknown field (UnknownFieldSet).
  has_unknown_fields: bool = False
  # EpisodeMetadata.marine_randomization_metadata. Read only for case 14.
  marine_randomization_metadata: str = ""


_PAYLOAD_KINDS = {
    PAYLOAD_CASE_SAFETY_DECISION: PayloadKind.SAFETY_DECISION,
    PAYLOAD_CASE_VEHICLE_TRAJECTORY: PayloadKind.VEHICLE_TRAJECTORY,
    PAYLOAD_CASE_INFERENCE_ENVELOPE: PayloadKind.INFERENCE_ENVELOPE,
    PAYLOAD_CASE_INFERENCE_RESULT: PayloadKind.INFERENCE_RESULT,
    PAYLOAD_CASE_EPISODE_METADATA: PayloadKind.EPISODE_METADATA,
    PAYLOAD_CASE_EPISODE_LOSS: PayloadKind.EPISODE_LOSS,
}


def is_lowercase_sha256_hex(value: str) -> bool:
  return len(value) == SHA256_HEX_LENGTH and all(
      c in "0123456789abcdef" for c in value
  )


def compute_config_digest(marine_metadata_text: str) -> str:
  """Lowercase hex SHA-256 over exactly the bytes of the text.

  No prefix, suffix, or normalization. Covers only the FormatMarineMetadata
  realized parameters. Empty text has no digest: the result is "" (no config
  recorded). The text is hashed as UTF-8, which equals ASCII for the
  FormatMarineMetadata output and matches the C++ byte hash for any string.
  """
  if not marine_metadata_text:
    return ""
  return hashlib.sha256(marine_metadata_text.encode("utf-8")).hexdigest()


def classify_payload(
    payload_case: int, has_unknown_fields: bool
) -> PayloadKind:
  kind = _PAYLOAD_KINDS.get(payload_case)
  if kind is not None:
    return kind
  return PayloadKind.UNKNOWN if has_unknown_fields else PayloadKind.NONE


def episode_event_engaged(view: EpisodeEventView) -> bool:
  """Any signal engages. An empty message (0 bytes) is not engaged."""
  return bool(
      view.header_present
      or view.source_id
      or view.frame_id
      or view.source_time_present
      or view.receive_time_present
      or view.schema_version != 0
      or view.snapshot_id
      or view.config_digest
      or view.provenance_present
      or view.payload_case != PAYLOAD_CASE_NOT_SET
      or view.has_unknown_fields
  )


def assess_episode_event(view: EpisodeEventView) -> EpisodeEventAssessment:
  """First failing check wins.

  An empty message is not engaged: NONE, not an error, engaged is false.
  """
  if not episode_event_engaged(view):
    return EpisodeEventAssessment()
  error = EpisodeEventError
  nanos_in_range = stamped_header_policy.nanos_in_range
  kind = classify_payload(view.payload_case, view.has_unknown_fields)

  def result(
      failure: EpisodeEventError, skip: bool = False
  ) -> EpisodeEventAssessment:
    return EpisodeEventAssessment(failure, kind, skip, True)

  if not view.header_present:
    return result(error.MISSING_HEADER)
  if not view.source_id:
    return result(error.SOURCE_ID)
  if not view.frame_id:
    return result(error.MISSING_FRAME)
  if not view.source_time_present or not nanos_in_range(view.source_time[1]):
    return result(error.SOURCE_TIME)
  if not view.receive_time_present or not nanos_in_range(view.receive_time[1]):
    return result(error.RECEIVE_TIME)
  if view.schema_version < 1:
    return result(error.SCHEMA_VERSION)
  if view.snapshot_id and not is_lowercase_sha256_hex(view.snapshot_id):
    return result(error.SNAPSHOT_ID)
  if view.config_digest and not is_lowercase_sha256_hex(view.config_digest):
    return result(error.CONFIG_DIGEST)
  if view.payload_case == PAYLOAD_CASE_EPISODE_METADATA and (
      view.config_digest
      != compute_config_digest(view.marine_randomization_metadata)
  ):
    return result(error.CONFIG_DIGEST_MISMATCH)
  if kind is PayloadKind.NONE:
    return result(error.MISSING_PAYLOAD)
  if kind is PayloadKind.UNKNOWN:
    return result(error.NONE, skip=True)
  return result(error.NONE)
