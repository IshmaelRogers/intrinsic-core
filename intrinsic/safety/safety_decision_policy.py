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

"""Plain-value checks for the SafetyDecision proto.

Policy: intrinsic_apis/intrinsic/safety/proto/README.md.

These helpers do not parse protobuf, do not evaluate any safety rule, and do
not call ICON, a HAL, or an actuator API.
"""

from dataclasses import dataclass
import enum
import re

from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy

SHA256_HEX_LENGTH = 64

_SHA256_HEX_PATTERN = re.compile(r"[0-9a-f]{64}")


class SafetyDecisionKind(enum.Enum):
  UNSPECIFIED = 0
  ACCEPT = 1
  PROJECT = 2
  REJECT = 3
  ABORT = 4
  SURFACE = 5
  # Any other wire number. Never rewritten to REJECT or ABORT.
  UNKNOWN = 6


class SafetyFindingSeverityKind(enum.Enum):
  UNSPECIFIED = 0
  INFO = 1
  WARNING = 2
  ERROR = 3
  CRITICAL = 4
  # Any other wire number. Kept as is. Not an error.
  UNKNOWN = 5


class SafetyDecisionError(enum.Enum):
  NONE = 0
  MISSING_HEADER = 1
  HEADER_TIME = 2
  HEADER_INVALID = 3
  UNSPECIFIED_KIND = 4
  ORIGINAL_INTENT_DIGEST = 5
  SNAPSHOT_ID = 6
  APPLIED_INTENT_MISSING = 7
  APPLIED_INTENT_UNEXPECTED = 8
  APPLIED_INTENT_INVALID = 9
  FINDING_RULE_ID = 10


_KINDS = {
    0: SafetyDecisionKind.UNSPECIFIED,
    1: SafetyDecisionKind.ACCEPT,
    2: SafetyDecisionKind.PROJECT,
    3: SafetyDecisionKind.REJECT,
    4: SafetyDecisionKind.ABORT,
    5: SafetyDecisionKind.SURFACE,
}

_SEVERITIES = {
    0: SafetyFindingSeverityKind.UNSPECIFIED,
    1: SafetyFindingSeverityKind.INFO,
    2: SafetyFindingSeverityKind.WARNING,
    3: SafetyFindingSeverityKind.ERROR,
    4: SafetyFindingSeverityKind.CRITICAL,
}


def classify_safety_decision_kind(kind: int) -> SafetyDecisionKind:
  """Wire values other than 0..5 are UNKNOWN."""
  return _KINDS.get(kind, SafetyDecisionKind.UNKNOWN)


def classify_safety_finding_severity(
    severity: int,
) -> SafetyFindingSeverityKind:
  """Wire values other than 0..4 are UNKNOWN."""
  return _SEVERITIES.get(severity, SafetyFindingSeverityKind.UNKNOWN)


def is_known_committed_kind(kind: SafetyDecisionKind) -> bool:
  """True for ACCEPT, PROJECT, REJECT, ABORT, and SURFACE."""
  return kind not in (
      SafetyDecisionKind.UNSPECIFIED,
      SafetyDecisionKind.UNKNOWN,
  )


def kind_requires_applied_intent(kind: SafetyDecisionKind) -> bool:
  return kind in (SafetyDecisionKind.PROJECT, SafetyDecisionKind.SURFACE)


def kind_forbids_applied_intent(kind: SafetyDecisionKind) -> bool:
  return kind in (
      SafetyDecisionKind.ACCEPT,
      SafetyDecisionKind.REJECT,
      SafetyDecisionKind.ABORT,
  )


def is_lowercase_sha256_hex(value: str) -> bool:
  """Exactly 64 characters in [0-9a-f]. Uppercase is rejected."""
  return _SHA256_HEX_PATTERN.fullmatch(value) is not None


@dataclass(frozen=True)
class SafetyFindingView:
  rule_id: str = ""
  # Raw wire number. Classify with classify_safety_finding_severity.
  severity: int = 0


@dataclass(frozen=True)
class SafetyDecisionView:
  header_present: bool = False
  header_validity_present: bool = False
  header_validity_state: int = 0
  header_source_time_present: bool = False
  header_source_time_nanos: int = 0
  header_receive_time_present: bool = False
  header_receive_time_nanos: int = 0
  # Raw wire number. Classify with classify_safety_decision_kind.
  kind: int = 0
  original_intent_digest: str = ""
  applied_intent_present: bool = False
  applied_intent: vehicle_contract_policy.DesiredMotionView = (
      vehicle_contract_policy.DesiredMotionView()
  )
  snapshot_id: str = ""
  findings: tuple[SafetyFindingView, ...] = ()
  decision_epoch_present: bool = False
  decision_epoch: int = 0


@dataclass(frozen=True)
class SafetyDecisionAssessment:
  # False for an empty decision. An unengaged decision is not an error and
  # is not accepted.
  engaged: bool = False
  error: SafetyDecisionError = SafetyDecisionError.NONE
  # Index of the first finding with an empty rule_id. -1 otherwise.
  finding_index: int = -1
  # Delegated DesiredMotion defect. NONE unless error is
  # APPLIED_INTENT_INVALID.
  applied_intent_error: vehicle_contract_policy.ContractError = (
      vehicle_contract_policy.ContractError.NONE
  )
  # Classification of the wire kind. Unknown stays UNKNOWN.
  kind: SafetyDecisionKind = SafetyDecisionKind.UNSPECIFIED
  header_validity: stamped_header_policy.ValidityKind = (
      stamped_header_policy.ValidityKind.ABSENT
  )
  # True only when the decision is engaged, has no structural defect, the
  # header is STATE_VALID, and kind is ACCEPT, PROJECT, REJECT, ABORT, or
  # SURFACE. An unknown kind is never accepted. The message is not changed.
  accepted: bool = False


def decision_engaged(decision: SafetyDecisionView) -> bool:
  return (
      decision.header_present
      or decision.kind != 0
      or decision.original_intent_digest != ""
      or decision.applied_intent_present
      or decision.snapshot_id != ""
      or len(decision.findings) > 0
      or decision.decision_epoch_present
      or decision.decision_epoch != 0
  )


def _structural_error(
    decision: SafetyDecisionView,
    kind: SafetyDecisionKind,
    header_validity: stamped_header_policy.ValidityKind,
) -> tuple[SafetyDecisionError, vehicle_contract_policy.ContractError]:
  none = vehicle_contract_policy.ContractError.NONE
  if not decision.header_present:
    return SafetyDecisionError.MISSING_HEADER, none
  if (
      decision.header_source_time_present
      and not stamped_header_policy.nanos_in_range(
          decision.header_source_time_nanos
      )
  ) or (
      decision.header_receive_time_present
      and not stamped_header_policy.nanos_in_range(
          decision.header_receive_time_nanos
      )
  ):
    return SafetyDecisionError.HEADER_TIME, none
  if header_validity is stamped_header_policy.ValidityKind.INVALID:
    return SafetyDecisionError.HEADER_INVALID, none
  if kind is SafetyDecisionKind.UNSPECIFIED:
    return SafetyDecisionError.UNSPECIFIED_KIND, none
  if not is_lowercase_sha256_hex(decision.original_intent_digest):
    return SafetyDecisionError.ORIGINAL_INTENT_DIGEST, none
  if not is_lowercase_sha256_hex(decision.snapshot_id):
    return SafetyDecisionError.SNAPSHOT_ID, none
  if kind_requires_applied_intent(kind):
    if not decision.applied_intent_present:
      return SafetyDecisionError.APPLIED_INTENT_MISSING, none
    applied = vehicle_contract_policy.assess_desired_motion(
        decision.applied_intent
    )
    if not applied.accepted:
      return SafetyDecisionError.APPLIED_INTENT_INVALID, applied.error
  elif kind_forbids_applied_intent(kind) and decision.applied_intent_present:
    return SafetyDecisionError.APPLIED_INTENT_UNEXPECTED, none
  return SafetyDecisionError.NONE, none


def assess_safety_decision(
    decision: SafetyDecisionView,
) -> SafetyDecisionAssessment:
  """First defect wins, in the same order as the C++ helper.

  Order: header presence and stamp, unspecified kind, original_intent_digest,
  snapshot_id, applied_intent presence and DesiredMotion assessment, finding
  rule ids. An unknown kind is not a structural defect: it is classified
  UNKNOWN, the applied_intent rules are not applied to it, and it is never
  accepted. Severity is never a defect.
  """
  if not decision_engaged(decision):
    return SafetyDecisionAssessment()
  kind = classify_safety_decision_kind(decision.kind)
  header_validity = stamped_header_policy.classify_validity(
      decision.header_validity_present, decision.header_validity_state
  )
  error, applied_error = _structural_error(decision, kind, header_validity)
  finding_index = -1
  if error is SafetyDecisionError.NONE:
    for index, finding in enumerate(decision.findings):
      if finding.rule_id == "":
        error = SafetyDecisionError.FINDING_RULE_ID
        finding_index = index
        break
  accepted = (
      error is SafetyDecisionError.NONE
      and is_known_committed_kind(kind)
      and stamped_header_policy.sample_accepted(header_validity, True)
  )
  return SafetyDecisionAssessment(
      engaged=True,
      error=error,
      finding_index=finding_index,
      applied_intent_error=applied_error,
      kind=kind,
      header_validity=header_validity,
      accepted=accepted,
  )
