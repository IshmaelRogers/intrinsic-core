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

"""Deterministic test double for EstimatorService.

Policy: intrinsic/estimation/README.md. Mirrors fake_estimator_service.h.

The initial pose and twist come from EstimatorConfig.seed. Predict holds the
kinematics and covariance and only advances the stamp. No process noise, no
error state, no integration.
"""

import dataclasses
from typing import Callable, Optional

from google.protobuf import timestamp_pb2

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.estimation import estimator_pb2
from intrinsic.estimation import estimator_service
from intrinsic.vehicle import vehicle_contract_policy as vehicle_policy
from intrinsic.vehicle.proto import vehicle_state_pb2

_DEFAULT_ESTIMATOR_ID = "estimator"
_DEFAULT_CLOCK_DOMAIN = stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
_DEFAULT_UNHEALTHY_AFTER = 3
_NANOS_PER_SECOND = stamped_header_policy.NANOS_PER_SECOND
_MASK64 = (1 << 64) - 1

_AIDING_KINDS = frozenset(
    {
        estimator_pb2.MEASUREMENT_KIND_DVL,
        estimator_pb2.MEASUREMENT_KIND_PRESSURE,
        estimator_pb2.MEASUREMENT_KIND_ALTIMETER,
        estimator_pb2.MEASUREMENT_KIND_SURFACE_FIX,
    }
)

PayloadValidator = Callable[[estimator_pb2.MeasurementEnvelope], bool]


def _to_nanos(time: timestamp_pb2.Timestamp) -> int:
  return time.seconds * _NANOS_PER_SECOND + time.nanos


def _timestamp_valid(time: timestamp_pb2.Timestamp) -> bool:
  return stamped_header_policy.nanos_in_range(time.nanos)


class _SplitMix64:
  """SplitMix64. The C++ fake uses the same constants."""

  def __init__(self, seed: int):
    self._state = seed & _MASK64

  def draw(self) -> float:
    """A multiple of 1/1024 in [0, 1), exact in binary floating point."""
    self._state = (self._state + 0x9E3779B97F4A7C15) & _MASK64
    z = self._state
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & _MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & _MASK64
    z ^= z >> 31
    return (z >> 54) / 1024.0


def _has_accepted_pose(state: vehicle_state_pb2.VehicleState) -> bool:
  """True when state passes AssessVehicleState accept and holds a pose."""
  header = state.header
  pose_present = state.HasField("pose_world_from_body")
  pose = state.pose_world_from_body
  twist = state.body_twist
  accel = state.body_acceleration
  view = vehicle_policy.VehicleStateView(
      validity_present=header.HasField("validity"),
      validity_state=header.validity.state,
      frame_id=header.frame_id,
      pose_present=pose_present,
      position=(pose.position.x, pose.position.y, pose.position.z),
      orientation=(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w,
      ),
      twist_present=state.HasField("body_twist"),
      twist=(
          twist.linear_x_m_s,
          twist.linear_y_m_s,
          twist.linear_z_m_s,
          twist.angular_x_rad_s,
          twist.angular_y_rad_s,
          twist.angular_z_rad_s,
      ),
      acceleration_present=state.HasField("body_acceleration"),
      acceleration=(
          accel.linear_x_m_s2,
          accel.linear_y_m_s2,
          accel.linear_z_m_s2,
          accel.angular_x_rad_s2,
          accel.angular_y_rad_s2,
          accel.angular_z_rad_s2,
      ),
      pose_covariance_present=state.HasField("pose_covariance"),
      pose_covariance=tuple(state.pose_covariance.values),
      twist_covariance_present=state.HasField("twist_covariance"),
      twist_covariance=tuple(state.twist_covariance.values),
      sources=tuple(
          vehicle_policy.SourceView(
              source_id=source.source_id,
              validity_present=source.HasField("validity"),
              validity_state=source.validity.state,
          )
          for source in state.sources
      ),
  )
  return pose_present and vehicle_policy.assess_vehicle_state(view).accepted


@dataclasses.dataclass
class _Source:
  status: estimator_pb2.SourceStatus
  last_accept_ns: int = 0
  accepted_once: bool = False


class FakeEstimatorService(estimator_service.EstimatorService):
  """Fixed-seed EstimatorService."""

  def __init__(self):
    self._initialized = False
    self._faulted = False
    self._aided_since_predict = False
    self._epoch = 0
    self._sequence = 0
    self._config = estimator_pb2.EstimatorConfig()
    self._max_future_skew_ns = 0
    self._state = vehicle_state_pb2.VehicleState()
    self._sources: list[_Source] = []
    self._last_predict_ns: Optional[int] = None
    self._validator: Optional[PayloadValidator] = None

  def set_payload_validator(self, validator: Optional[PayloadValidator]):
    """validator returns True when the payload is usable."""
    self._validator = validator

  def _result(self, reason: int) -> estimator_pb2.EstimatorResult:
    return estimator_pb2.EstimatorResult(
        ok=reason == estimator_pb2.REJECT_REASON_NONE,
        reason=reason,
        estimator_epoch=self._epoch,
    )

  def _reset_buffers(self):
    self._faulted = False
    self._aided_since_predict = False
    self._sources = []
    self._last_predict_ns = None
    self._state = vehicle_state_pb2.VehicleState()

  def _fill_identity_header(self):
    header = self._state.header
    header.sequence = self._sequence
    header.source_id = self._config.estimator_id
    header.frame_id = self._config.world_frame_id
    header.clock_domain = self._config.clock_domain

  def initialize(self, config):
    skew = config.max_future_skew
    if not vehicle_policy.duration_non_negative(skew.seconds, skew.nanos):
      return self._result(estimator_pb2.REJECT_REASON_INVALID_CONFIG)
    self._config = estimator_pb2.EstimatorConfig()
    self._config.CopyFrom(config)
    if not self._config.estimator_id:
      self._config.estimator_id = _DEFAULT_ESTIMATOR_ID
    if not self._config.world_frame_id:
      self._config.world_frame_id = frame_policy.WORLD_ENU_FRAME_ID
    if not self._config.clock_domain:
      self._config.clock_domain = _DEFAULT_CLOCK_DOMAIN
    if self._config.unhealthy_after_consecutive_rejects == 0:
      self._config.unhealthy_after_consecutive_rejects = (
          _DEFAULT_UNHEALTHY_AFTER
      )
    self._max_future_skew_ns = skew.seconds * _NANOS_PER_SECOND + skew.nanos

    self._reset_buffers()
    self._initialized = True
    self._epoch += 1
    self._sequence += 1

    rng = _SplitMix64(self._config.seed)
    x = rng.draw() * 64.0
    y = rng.draw() * 64.0
    z = -rng.draw() * 16.0
    surge = rng.draw() * 2.0

    self._fill_identity_header()
    self._state.header.validity.state = stamped_header_pb2.Validity.STATE_VALID
    pose = self._state.pose_world_from_body
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1
    self._state.body_twist.linear_x_m_s = surge
    if self._config.publish_pose_covariance:
      values = [0.0] * vehicle_policy.COVARIANCE_VALUES
      for i in range(vehicle_policy.SPATIAL_DOF):
        values[vehicle_policy.covariance_index(i, i)] = (
            0.25 if i < 3 else 0.0625
        )
      self._state.pose_covariance.values.extend(values)
    self._state.mode = vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING
    self._state.estimator_epoch = self._epoch
    return self._result(estimator_pb2.REJECT_REASON_NONE)

  def _find_or_add_source(self, source_id: str) -> _Source:
    for source in self._sources:
      if source.status.source_id == source_id:
        return source
    source = _Source(status=estimator_pb2.SourceStatus(source_id=source_id))
    self._sources.append(source)
    return source

  def _reject(self, source: _Source, reason: int):
    status = source.status
    status.last_reject_reason = reason
    status.consecutive_rejects += 1
    if (
        status.consecutive_rejects
        >= self._config.unhealthy_after_consecutive_rejects
    ):
      status.healthy = False
    return self._result(reason)

  def ingest(self, envelope):
    if not self._initialized:
      return self._result(estimator_pb2.REJECT_REASON_NOT_INITIALIZED)
    if self._faulted:
      return self._result(estimator_pb2.REJECT_REASON_ESTIMATOR_FAULTED)
    if not envelope.source_id:
      return self._result(estimator_pb2.REJECT_REASON_INVALID_ENVELOPE)
    source = self._find_or_add_source(envelope.source_id)
    header = envelope.header

    kind_valid = (
        envelope.kind != estimator_pb2.MEASUREMENT_KIND_UNSPECIFIED
        and envelope.kind in estimator_pb2.MeasurementKind.values()
    )
    if (
        not kind_valid
        or not header.HasField("source_time")
        or not header.HasField("receive_time")
        or not _timestamp_valid(header.source_time)
        or not _timestamp_valid(header.receive_time)
        or (envelope.payload and not envelope.payload_type)
    ):
      return self._reject(source, estimator_pb2.REJECT_REASON_INVALID_ENVELOPE)
    if header.clock_domain != self._config.clock_domain:
      return self._reject(
          source, estimator_pb2.REJECT_REASON_CLOCK_DOMAIN_MISMATCH
      )
    validity = stamped_header_policy.classify_validity(
        header.HasField("validity"), header.validity.state
    )
    if not stamped_header_policy.sample_accepted(validity, True):
      return self._reject(source, estimator_pb2.REJECT_REASON_INVALID_PAYLOAD)

    source_ns = _to_nanos(header.source_time)
    if source_ns > _to_nanos(header.receive_time) + self._max_future_skew_ns:
      return self._reject(
          source, estimator_pb2.REJECT_REASON_FUTURE_MEASUREMENT
      )
    if source.accepted_once:
      if source_ns < source.last_accept_ns:
        return self._reject(source, estimator_pb2.REJECT_REASON_OUT_OF_ORDER)
      if source_ns == source.last_accept_ns:
        return self._reject(
            source, estimator_pb2.REJECT_REASON_DUPLICATE_TIMESTAMP
        )
    if self._validator is not None and not self._validator(envelope):
      return self._reject(source, estimator_pb2.REJECT_REASON_INVALID_PAYLOAD)

    source.accepted_once = True
    source.last_accept_ns = source_ns
    source.status.healthy = True
    source.status.consecutive_rejects = 0
    source.status.last_accept_source_time.CopyFrom(header.source_time)
    if envelope.kind in _AIDING_KINDS:
      self._aided_since_predict = True
    return self._result(estimator_pb2.REJECT_REASON_NONE)

  def predict(self, to_time):
    if not self._initialized:
      return self._result(estimator_pb2.REJECT_REASON_NOT_INITIALIZED)
    if not _timestamp_valid(to_time) or (
        self._last_predict_ns is not None
        and _to_nanos(to_time) < self._last_predict_ns
    ):
      return self._result(estimator_pb2.REJECT_REASON_INVALID_TIME)
    self._last_predict_ns = _to_nanos(to_time)
    self._sequence += 1
    header = self._state.header
    header.sequence = self._sequence
    header.source_time.CopyFrom(to_time)
    header.receive_time.CopyFrom(to_time)

    if self._faulted:
      self._state.mode = vehicle_state_pb2.NAVIGATION_MODE_FAULTED
    elif _has_accepted_pose(self._state):
      self._state.mode = (
          vehicle_state_pb2.NAVIGATION_MODE_AIDED
          if self._aided_since_predict
          else vehicle_state_pb2.NAVIGATION_MODE_DEAD_RECKONING
      )
    self._aided_since_predict = False
    return self._result(estimator_pb2.REJECT_REASON_NONE)

  def reset(self, prior=None):
    if not self._initialized:
      return self._result(estimator_pb2.REJECT_REASON_NOT_INITIALIZED)
    self._reset_buffers()
    self._epoch += 1
    self._sequence += 1
    prior_accepted = prior is not None and _has_accepted_pose(prior)
    if prior_accepted:
      self._state.CopyFrom(prior)
      self._state.ClearField("sources")
      self._sequence = max(self._sequence, self._state.header.sequence)
      if self._state.header.HasField("source_time") and _timestamp_valid(
          self._state.header.source_time
      ):
        self._last_predict_ns = _to_nanos(self._state.header.source_time)
    else:
      self._fill_identity_header()
    self._state.mode = vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING
    self._state.estimator_epoch = self._epoch
    result = self._result(estimator_pb2.REJECT_REASON_NONE)
    result.prior_accepted = prior_accepted
    return result

  def inject_fault(self) -> bool:
    """Forces FAULTED until the next initialize or reset."""
    if not self._initialized:
      return False
    self._faulted = True
    self._state.mode = vehicle_state_pb2.NAVIGATION_MODE_FAULTED
    return True

  def inject_source_fault(self, source_id: str) -> bool:
    """Marks one known source unhealthy until its next accept."""
    for source in self._sources:
      if source.status.source_id == source_id:
        source.status.healthy = False
        return True
    return False

  def get_state(self):
    response = estimator_pb2.GetStateResponse()
    if not self._initialized:
      return response
    response.state.CopyFrom(self._state)
    for source in self._sources:
      response.source_status.add().CopyFrom(source.status)
      if source.accepted_once:
        health = response.state.sources.add()
        health.source_id = source.status.source_id
        health.validity.state = (
            stamped_header_pb2.Validity.STATE_VALID
            if source.status.healthy
            else stamped_header_pb2.Validity.STATE_INVALID
        )
    return response

  def state_digest(self) -> int:
    """FNV-1a 64 of the deterministic serialization of get_state().state."""
    response = self.get_state()
    if not response.HasField("state"):
      return 0
    digest = 0xCBF29CE484222325
    for byte in response.state.SerializeToString(deterministic=True):
      digest = ((digest ^ byte) * 0x100000001B3) & _MASK64
    return digest
