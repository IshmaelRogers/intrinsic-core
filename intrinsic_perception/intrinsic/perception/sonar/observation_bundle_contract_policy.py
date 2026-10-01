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

"""Plain-value checks for MultimodalObservationBundle.

Policy: intrinsic_apis/intrinsic/perception/proto/sonar/README.md.

These helpers do not parse protobuf, do not fetch references, do not run
inference, do not beamform, and do not call World, ICON, or a HAL.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import enum

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy

# Fixture examples use 200ms. The assessor does not apply this when max_skew
# is unset.
FIXTURE_MAX_SKEW_SECONDS = 0
FIXTURE_MAX_SKEW_NANOS = 200000000

_INT64_MAX = 9223372036854775807
_INT64_MIN = -9223372036854775808
_NANOS_PER_SECOND = stamped_header_policy.NANOS_PER_SECOND


class ObservationModality(enum.IntEnum):
  UNSPECIFIED = 0
  SONAR_FLS = 1
  SONAR_SSS = 2
  OPTICAL = 3
  POINT_CLOUD = 4
  VEHICLE_STATE = 5


class AbsentModalityReason(enum.IntEnum):
  UNSPECIFIED = 0
  NOT_CONFIGURED = 1
  SENSOR_OFFLINE = 2
  OUT_OF_RANGE = 3
  DROPPED_FOR_SKEW = 4
  INTENTIONALLY_OMITTED = 5


class ObservationBundleContractError(enum.IntEnum):
  NONE = 0
  MISSING_FRAME = 1
  MAX_SKEW = 2
  SLOT = 3
  ABSENT_LIST = 4
  CLOCK_DOMAIN = 5
  FRAME_MISMATCH = 6
  EXCESSIVE_SKEW = 7
  TIME_REVERSAL = 8
  SNAPSHOT_ID = 9
  REFERENCE = 10


@dataclass(frozen=True)
class ObservationBundleContractAssessment:
  """accepted is true only for no error and STATE_VALID.

  An empty message is not accepted and is not an error. measured_skew is
  (seconds, nanos). It is filled when the skew set has at least one
  source_time and the check reached the skew step. Earlier defects leave
  measured_skew_present false and the duration at zero.
  """

  error: ObservationBundleContractError
  validity: stamped_header_policy.ValidityKind
  accepted: bool
  measured_skew_present: bool = False
  measured_skew: tuple[int, int] = (0, 0)


@dataclass(frozen=True)
class ObservationSlotView:
  """Present iff reference_id is non-empty. The header is then required."""

  reference_id: str = ""
  content_type: str = ""
  byte_size_present: bool = False
  byte_size: int = 0
  frame_id: str = ""
  source_time_present: bool = False
  source_time: tuple[int, int] = (0, 0)
  receive_time_present: bool = False
  receive_time: tuple[int, int] = (0, 0)
  clock_domain: str = ""


@dataclass(frozen=True)
class StateReferenceView:
  """Present iff message_set and frame_id is non-empty."""

  message_set: bool = False
  frame_id: str = ""
  source_time_present: bool = False
  source_time: tuple[int, int] = (0, 0)
  receive_time_present: bool = False
  receive_time: tuple[int, int] = (0, 0)
  clock_domain: str = ""
  state_epoch: int = 0
  world_snapshot_id: str = ""


@dataclass(frozen=True)
class AbsentModalityEntryView:
  modality: int = 0
  reason: int = 0


@dataclass(frozen=True)
class ObservationBundleView:
  """Plain values of one MultimodalObservationBundle."""

  header_present: bool = False
  validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  clock_domain: str = ""
  source_time_present: bool = False
  source_time: tuple[int, int] = (0, 0)
  receive_time_present: bool = False
  receive_time: tuple[int, int] = (0, 0)
  max_skew_present: bool = False
  max_skew: tuple[int, int] = (0, 0)
  fls: ObservationSlotView = ObservationSlotView()
  sss: ObservationSlotView = ObservationSlotView()
  optical: ObservationSlotView = ObservationSlotView()
  point_cloud: ObservationSlotView = ObservationSlotView()
  vehicle_state: StateReferenceView = StateReferenceView()
  absent: tuple[AbsentModalityEntryView, ...] = ()
  # True when the metadata map has any entry. Values are not inspected.
  metadata_present: bool = False


def slot_present(slot: ObservationSlotView) -> bool:
  return slot.reference_id != ""


def state_present(state: StateReferenceView) -> bool:
  return state.message_set and state.frame_id != ""


def _max_skew_nonzero(bundle: ObservationBundleView) -> bool:
  return bundle.max_skew_present and (
      bundle.max_skew[0] != 0 or bundle.max_skew[1] != 0
  )


def observation_bundle_engaged(bundle: ObservationBundleView) -> bool:
  """A present zero max_skew does not engage by itself."""
  return (
      slot_present(bundle.fls)
      or slot_present(bundle.sss)
      or slot_present(bundle.optical)
      or slot_present(bundle.point_cloud)
      or state_present(bundle.vehicle_state)
      or len(bundle.absent) > 0
      or _max_skew_nonzero(bundle)
      or bundle.metadata_present
      or bundle.header_present
  )


def known_modality(modality: int) -> bool:
  return (
      int(ObservationModality.SONAR_FLS)
      <= modality
      <= int(ObservationModality.VEHICLE_STATE)
  )


def known_absent_reason(reason: int) -> bool:
  return (
      int(AbsentModalityReason.NOT_CONFIGURED)
      <= reason
      <= int(AbsentModalityReason.INTENTIONALLY_OMITTED)
  )


def is_lowercase_sha256_hex(value: str) -> bool:
  if len(value) != 64:
    return False
  for char in value:
    if char not in "0123456789abcdef":
      return False
  return True


def _is_world_frame(frame_id: str) -> bool:
  return frame_id in (
      frame_policy.WORLD_ENU_FRAME_ID,
      frame_policy.WORLD_NED_FRAME_ID,
  )


def _before(left: tuple[int, int], right: tuple[int, int]) -> bool:
  return left[0] < right[0] or (left[0] == right[0] and left[1] < right[1])


def _duration_greater(value: tuple[int, int], seconds: int, nanos: int) -> bool:
  return value[0] > seconds or (value[0] == seconds and value[1] > nanos)


def _difference(
    later: tuple[int, int], earlier: tuple[int, int]
) -> tuple[int, int]:
  """later - earlier. Saturates when the difference does not fit in int64."""
  if earlier[0] < 0 and later[0] > _INT64_MAX + earlier[0]:
    return (_INT64_MAX, 999999999)
  if earlier[0] > 0 and later[0] < _INT64_MIN + earlier[0]:
    return (_INT64_MAX, 999999999)
  seconds = later[0] - earlier[0]
  nanos = later[1] - earlier[1]
  if nanos < 0:
    if seconds == _INT64_MIN:
      return (_INT64_MAX, 999999999)
    seconds -= 1
    nanos += _NANOS_PER_SECOND
  return (seconds, nanos)


def _max_skew_ok(bundle: ObservationBundleView) -> bool:
  return (
      bundle.max_skew_present
      and bundle.max_skew[0] >= 0
      and stamped_header_policy.nanos_in_range(bundle.max_skew[1])
  )


def _make(
    error: ObservationBundleContractError,
    validity: stamped_header_policy.ValidityKind,
    measured_skew_present: bool,
    measured_skew: tuple[int, int],
) -> ObservationBundleContractAssessment:
  accepted = (
      error is ObservationBundleContractError.NONE
      and stamped_header_policy.sample_accepted(validity, True)
  )
  return ObservationBundleContractAssessment(
      error=error,
      validity=validity,
      accepted=accepted,
      measured_skew_present=measured_skew_present,
      measured_skew=measured_skew,
  )


def _assess_slot(slot: ObservationSlotView) -> ObservationBundleContractError:
  if (
      slot.frame_id == ""
      or not slot.source_time_present
      or not stamped_header_policy.nanos_in_range(slot.source_time[1])
  ):
    return ObservationBundleContractError.SLOT
  if slot.reference_id == "" or (
      slot.byte_size_present and slot.byte_size == 0
  ):
    return ObservationBundleContractError.REFERENCE
  return ObservationBundleContractError.NONE


def _assess_state(state: StateReferenceView) -> ObservationBundleContractError:
  if (
      state.frame_id == ""
      or not state.source_time_present
      or not stamped_header_policy.nanos_in_range(state.source_time[1])
  ):
    return ObservationBundleContractError.SLOT
  if state.world_snapshot_id != "" and not is_lowercase_sha256_hex(
      state.world_snapshot_id
  ):
    return ObservationBundleContractError.SNAPSHOT_ID
  return ObservationBundleContractError.NONE


def _present_flags(bundle: ObservationBundleView) -> list[bool]:
  present = [False] * 6
  if slot_present(bundle.fls):
    present[int(ObservationModality.SONAR_FLS)] = True
  if slot_present(bundle.sss):
    present[int(ObservationModality.SONAR_SSS)] = True
  if slot_present(bundle.optical):
    present[int(ObservationModality.OPTICAL)] = True
  if slot_present(bundle.point_cloud):
    present[int(ObservationModality.POINT_CLOUD)] = True
  if state_present(bundle.vehicle_state):
    present[int(ObservationModality.VEHICLE_STATE)] = True
  return present


def _absent_list_ok(bundle: ObservationBundleView) -> bool:
  """A modality listed in neither place is a subset and is not a defect."""
  present = _present_flags(bundle)
  seen = [False] * 6
  for entry in bundle.absent:
    if (
        not known_modality(entry.modality)
        or not known_absent_reason(entry.reason)
        or seen[entry.modality]
        or present[entry.modality]
    ):
      return False
    seen[entry.modality] = True
  return True


def _any_present(bundle: ObservationBundleView) -> bool:
  return (
      slot_present(bundle.fls)
      or slot_present(bundle.sss)
      or slot_present(bundle.optical)
      or slot_present(bundle.point_cloud)
      or state_present(bundle.vehicle_state)
  )


def _assess_present_slots(
    bundle: ObservationBundleView,
) -> ObservationBundleContractError:
  for slot in (bundle.fls, bundle.sss, bundle.optical, bundle.point_cloud):
    if not slot_present(slot):
      continue
    error = _assess_slot(slot)
    if error is not ObservationBundleContractError.NONE:
      return error
  if state_present(bundle.vehicle_state):
    return _assess_state(bundle.vehicle_state)
  return ObservationBundleContractError.NONE


def _clocks_agree(bundle: ObservationBundleView) -> bool:
  domain = bundle.clock_domain
  for slot in (bundle.fls, bundle.sss, bundle.optical, bundle.point_cloud):
    if slot_present(slot) and slot.clock_domain != domain:
      return False
  if (
      state_present(bundle.vehicle_state)
      and bundle.vehicle_state.clock_domain != domain
  ):
    return False
  return True


def _world_frames_agree(bundle: ObservationBundleView) -> bool:
  world = ""
  have_world = False
  for slot in (bundle.fls, bundle.sss, bundle.optical, bundle.point_cloud):
    if not slot_present(slot) or not _is_world_frame(slot.frame_id):
      continue
    if have_world and slot.frame_id != world:
      return False
    have_world = True
    world = slot.frame_id
  return True


def _source_time_reversed(
    source_present: bool,
    source: tuple[int, int],
    receive_present: bool,
    receive: tuple[int, int],
) -> bool:
  if not source_present or not receive_present:
    return False
  if not stamped_header_policy.nanos_in_range(
      source[1]
  ) or not stamped_header_policy.nanos_in_range(receive[1]):
    return True
  return _before(receive, source)


def _any_source_time_reversed(bundle: ObservationBundleView) -> bool:
  if _source_time_reversed(
      bundle.source_time_present,
      bundle.source_time,
      bundle.receive_time_present,
      bundle.receive_time,
  ):
    return True
  for slot in (bundle.fls, bundle.sss, bundle.optical, bundle.point_cloud):
    if slot_present(slot) and _source_time_reversed(
        slot.source_time_present,
        slot.source_time,
        slot.receive_time_present,
        slot.receive_time,
    ):
      return True
  state = bundle.vehicle_state
  return state_present(state) and _source_time_reversed(
      state.source_time_present,
      state.source_time,
      state.receive_time_present,
      state.receive_time,
  )


def _measure_skew(
    bundle: ObservationBundleView,
) -> tuple[bool, tuple[int, int]]:
  have = False
  earliest = (0, 0)
  latest = (0, 0)

  def consider(present: bool, time: tuple[int, int]) -> None:
    nonlocal have, earliest, latest
    if not present:
      return
    if not have:
      earliest = time
      latest = time
      have = True
      return
    if _before(time, earliest):
      earliest = time
    if _before(latest, time):
      latest = time

  if slot_present(bundle.fls):
    consider(bundle.fls.source_time_present, bundle.fls.source_time)
  if slot_present(bundle.sss):
    consider(bundle.sss.source_time_present, bundle.sss.source_time)
  if slot_present(bundle.optical):
    consider(bundle.optical.source_time_present, bundle.optical.source_time)
  if slot_present(bundle.point_cloud):
    consider(
        bundle.point_cloud.source_time_present, bundle.point_cloud.source_time
    )
  if state_present(bundle.vehicle_state):
    consider(
        bundle.vehicle_state.source_time_present,
        bundle.vehicle_state.source_time,
    )
  if not have:
    return False, (0, 0)
  return True, _difference(latest, earliest)


def assess_multimodal_observation_bundle(
    bundle: ObservationBundleView,
) -> ObservationBundleContractAssessment:
  """First defect wins. An empty view is not engaged.

  Engaged with no present slot and no absent entry is SLOT. Engaged with no
  present slot and a non-empty valid absent list (all-absent) can be accepted.
  """
  if not observation_bundle_engaged(bundle):
    return _make(
        ObservationBundleContractError.NONE,
        stamped_header_policy.ValidityKind.ABSENT,
        False,
        (0, 0),
    )
  validity = stamped_header_policy.classify_validity(
      bundle.validity_present, bundle.validity_state
  )

  def fail(
      error: ObservationBundleContractError,
  ) -> ObservationBundleContractAssessment:
    return _make(error, validity, False, (0, 0))

  if bundle.frame_id == "":
    return fail(ObservationBundleContractError.MISSING_FRAME)
  if not _max_skew_ok(bundle):
    return fail(ObservationBundleContractError.MAX_SKEW)
  if not _absent_list_ok(bundle):
    return fail(ObservationBundleContractError.ABSENT_LIST)
  if not _any_present(bundle):
    if len(bundle.absent) == 0:
      return fail(ObservationBundleContractError.SLOT)
  else:
    slot_error = _assess_present_slots(bundle)
    if slot_error is not ObservationBundleContractError.NONE:
      return fail(slot_error)
  if not _clocks_agree(bundle):
    return fail(ObservationBundleContractError.CLOCK_DOMAIN)
  if not _world_frames_agree(bundle):
    return fail(ObservationBundleContractError.FRAME_MISMATCH)
  if _any_source_time_reversed(bundle):
    return fail(ObservationBundleContractError.TIME_REVERSAL)
  measured_skew_present, measured_skew = _measure_skew(bundle)
  if measured_skew_present and _duration_greater(
      measured_skew, bundle.max_skew[0], bundle.max_skew[1]
  ):
    return _make(
        ObservationBundleContractError.EXCESSIVE_SKEW,
        validity,
        True,
        measured_skew,
    )
  return _make(
      ObservationBundleContractError.NONE,
      validity,
      measured_skew_present,
      measured_skew,
  )


def absent_entries(
    pairs: Sequence[tuple[int, int]],
) -> tuple[AbsentModalityEntryView, ...]:
  """Thin helper: copies (modality, reason) pairs. Does not fetch payloads."""
  return tuple(
      AbsentModalityEntryView(modality=modality, reason=reason)
      for modality, reason in pairs
  )
