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

"""Thin assembler for MultimodalObservationBundle.

Copies caller-supplied fields onto the wire message. Does not fetch reference
ids, does not embed FLS, SSS, camera, point-cloud, VehicleState, or World
payloads, and does not rewrite strings, times, or snapshot ids.
"""

from collections.abc import Sequence
from dataclasses import dataclass

from intrinsic.perception.proto.sonar import observation_bundle_pb2


@dataclass(frozen=True)
class ObservationSlotParts:
  sequence: int = 0
  source_seconds: int = 0
  source_nanos: int = 0
  set_receive: bool = False
  receive_seconds: int = 0
  receive_nanos: int = 0
  source_id: str = ""
  frame_id: str = ""
  clock_domain: str = ""
  set_validity: bool = False
  validity_state: int = 0
  reference_id: str = ""
  content_type: str = ""
  set_byte_size: bool = False
  byte_size: int = 0


@dataclass(frozen=True)
class StateReferenceParts:
  set: bool = False
  stamp: ObservationSlotParts = ObservationSlotParts()
  state_epoch: int = 0
  world_snapshot_id: str = ""


@dataclass(frozen=True)
class AbsentModalityParts:
  modality: int = 0
  reason: int = 0


@dataclass(frozen=True)
class ObservationBundleParts:
  set_header: bool = False
  header: ObservationSlotParts = ObservationSlotParts()
  set_max_skew: bool = False
  max_skew_seconds: int = 0
  max_skew_nanos: int = 0
  set_fls: bool = False
  fls: ObservationSlotParts = ObservationSlotParts()
  set_sss: bool = False
  sss: ObservationSlotParts = ObservationSlotParts()
  set_optical: bool = False
  optical: ObservationSlotParts = ObservationSlotParts()
  set_point_cloud: bool = False
  point_cloud: ObservationSlotParts = ObservationSlotParts()
  vehicle_state: StateReferenceParts = StateReferenceParts()
  absent: tuple[AbsentModalityParts, ...] = ()
  metadata: tuple[tuple[str, str], ...] = ()


def _fill_header(header, parts: ObservationSlotParts) -> None:
  header.sequence = parts.sequence
  header.source_time.seconds = parts.source_seconds
  header.source_time.nanos = parts.source_nanos
  if parts.set_receive:
    header.receive_time.seconds = parts.receive_seconds
    header.receive_time.nanos = parts.receive_nanos
  header.source_id = parts.source_id
  header.frame_id = parts.frame_id
  header.clock_domain = parts.clock_domain
  if parts.set_validity:
    header.validity.state = parts.validity_state


def _fill_slot(slot, parts: ObservationSlotParts) -> None:
  _fill_header(slot.header, parts)
  slot.reference_id = parts.reference_id
  slot.content_type = parts.content_type
  if parts.set_byte_size:
    slot.byte_size = parts.byte_size


def build_multimodal_observation_bundle(
    parts: ObservationBundleParts,
) -> observation_bundle_pb2.MultimodalObservationBundle:
  """Assembles one bundle. Omitted slots stay unset on the wire."""
  bundle = observation_bundle_pb2.MultimodalObservationBundle()
  if parts.set_header:
    _fill_header(bundle.header, parts.header)
  if parts.set_max_skew:
    bundle.max_skew.seconds = parts.max_skew_seconds
    bundle.max_skew.nanos = parts.max_skew_nanos
  if parts.set_fls:
    _fill_slot(bundle.fls, parts.fls)
  if parts.set_sss:
    _fill_slot(bundle.sss, parts.sss)
  if parts.set_optical:
    _fill_slot(bundle.optical, parts.optical)
  if parts.set_point_cloud:
    _fill_slot(bundle.point_cloud, parts.point_cloud)
  if parts.vehicle_state.set:
    _fill_header(bundle.vehicle_state.header, parts.vehicle_state.stamp)
    bundle.vehicle_state.state_epoch = parts.vehicle_state.state_epoch
    bundle.vehicle_state.world_snapshot_id = (
        parts.vehicle_state.world_snapshot_id
    )
  for entry in parts.absent:
    absent = bundle.absent.add()
    absent.modality = entry.modality
    absent.reason = entry.reason
  for key, value in parts.metadata:
    bundle.metadata[key] = value
  return bundle


def absent_parts(
    pairs: Sequence[tuple[int, int]],
) -> tuple[AbsentModalityParts, ...]:
  return tuple(
      AbsentModalityParts(modality=modality, reason=reason)
      for modality, reason in pairs
  )
