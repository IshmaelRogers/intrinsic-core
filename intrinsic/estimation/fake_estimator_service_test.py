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

"""Tests for FakeEstimatorService. Mirrors fake_estimator_service_test.cc."""

import os
import unittest

from google.protobuf import duration_pb2
from google.protobuf import text_format
from google.protobuf import timestamp_pb2

from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.estimation import estimator_pb2
from intrinsic.estimation import fake_estimator_service
from intrinsic.vehicle import vehicle_contract_policy as vehicle_policy
from intrinsic.vehicle.proto import vehicle_state_pb2

_VALID = stamped_header_pb2.Validity.STATE_VALID
_INVALID = stamped_header_pb2.Validity.STATE_INVALID

# FNV-1a 64 of the deterministic serialization of the nominal snapshot. Keep in
# sync with fake_estimator_service_test.cc.
_NOMINAL_DIGEST = 0x603D34D9F8B840DD


def _ts(seconds, nanos=0):
  return timestamp_pb2.Timestamp(seconds=seconds, nanos=nanos)


def _config(**kwargs):
  return estimator_pb2.EstimatorConfig(**kwargs)


def _envelope(
    source_id,
    kind,
    source_time,
    receive_time=None,
    validity=_VALID,
    clock_domain="monotonic",
):
  envelope = estimator_pb2.MeasurementEnvelope(
      source_id=source_id,
      kind=kind,
      payload_type="intrinsic_proto.marine.Sample",
      payload=b"\x08\x01",
  )
  header = envelope.header
  header.source_id = source_id
  header.clock_domain = clock_domain
  header.source_time.CopyFrom(source_time)
  header.receive_time.CopyFrom(receive_time or source_time)
  if validity is not None:
    header.validity.state = validity
  return envelope


def _load_golden(name):
  root = os.environ.get("TEST_SRCDIR", "")
  for dirpath, _, filenames in os.walk(root):
    if name in filenames and os.path.basename(dirpath) == "testdata":
      with open(os.path.join(dirpath, name), encoding="utf-8") as handle:
        return handle.read()
  raise AssertionError("missing %s under %s" % (name, root))


def _run_nominal(service):
  """init -> DVL + depth ingest -> predict. Used by the golden and replay."""
  assert service.initialize(_config(seed=29, publish_pose_covariance=True)).ok
  assert service.ingest(
      _envelope(
          "dvl",
          estimator_pb2.MEASUREMENT_KIND_DVL,
          _ts(1700000000, 250000000),
          _ts(1700000000, 300000000),
      )
  ).ok
  assert service.ingest(
      _envelope(
          "depth",
          estimator_pb2.MEASUREMENT_KIND_PRESSURE,
          _ts(1700000000, 500000000),
      )
  ).ok
  assert service.predict(_ts(1700000001)).ok


class FakeEstimatorServiceTest(unittest.TestCase):

  def _ready(self, **config):
    service = fake_estimator_service.FakeEstimatorService()
    self.assertTrue(service.initialize(_config(**config)).ok)
    return service

  def test_never_initialized_state_is_absent(self):
    service = fake_estimator_service.FakeEstimatorService()
    response = service.get_state()
    self.assertFalse(response.HasField("state"))
    self.assertEqual(service.state_digest(), 0)
    self.assertEqual(
        service.ingest(
            _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(1))
        ).reason,
        estimator_pb2.REJECT_REASON_NOT_INITIALIZED,
    )
    self.assertEqual(
        service.predict(_ts(1)).reason,
        estimator_pb2.REJECT_REASON_NOT_INITIALIZED,
    )
    self.assertEqual(
        service.reset().reason, estimator_pb2.REJECT_REASON_NOT_INITIALIZED
    )
    self.assertFalse(service.inject_fault())

  def test_initialize_publishes_initializing_with_epoch_one(self):
    service = fake_estimator_service.FakeEstimatorService()
    result = service.initialize(_config(seed=7))
    self.assertTrue(result.ok)
    self.assertEqual(result.estimator_epoch, 1)
    state = service.get_state().state
    self.assertEqual(state.mode, vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING)
    self.assertEqual(state.estimator_epoch, 1)
    self.assertEqual(state.header.source_id, "estimator")
    self.assertEqual(state.header.frame_id, "world_enu")
    self.assertEqual(state.header.clock_domain, "monotonic")
    self.assertTrue(state.HasField("pose_world_from_body"))

  def test_invalid_config_changes_nothing(self):
    service = self._ready(seed=1)
    before = service.state_digest()
    bad = _config(max_future_skew=duration_pb2.Duration(seconds=-1))
    result = service.initialize(bad)
    self.assertEqual(result.reason, estimator_pb2.REJECT_REASON_INVALID_CONFIG)
    self.assertEqual(service.get_state().state.estimator_epoch, 1)
    self.assertEqual(service.state_digest(), before)

  def test_reinitialize_is_a_full_reset_with_epoch_bump(self):
    service = self._ready(seed=1)
    service.ingest(
        _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10))
    )
    service.predict(_ts(11))
    result = service.initialize(_config(seed=1))
    self.assertEqual(result.estimator_epoch, 2)
    response = service.get_state()
    self.assertEqual(
        response.state.mode, vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING
    )
    self.assertFalse(response.state.header.HasField("source_time"))
    self.assertEqual(len(response.source_status), 0)
    self.assertEqual(len(response.state.sources), 0)
    self.assertTrue(
        service.ingest(
            _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10))
        ).ok
    )

  def test_covariance_present_or_absent(self):
    with_covariance = self._ready(seed=3, publish_pose_covariance=True)
    state = with_covariance.get_state().state
    self.assertTrue(state.HasField("pose_covariance"))
    self.assertEqual(
        vehicle_policy.assess_covariance(
            True, tuple(state.pose_covariance.values)
        ),
        vehicle_policy.CovarianceError.NONE,
    )
    self.assertFalse(state.HasField("twist_covariance"))
    without = self._ready(seed=3)
    state = without.get_state().state
    self.assertFalse(state.HasField("pose_covariance"))
    self.assertFalse(state.HasField("twist_covariance"))

  def test_published_initial_state_passes_the_17_accept_path(self):
    state = self._ready(seed=5).get_state().state
    self.assertTrue(fake_estimator_service._has_accepted_pose(state))

  def test_mode_initializing_then_dead_reckoning_on_first_predict(self):
    service = self._ready(seed=9)
    self.assertTrue(service.predict(_ts(100)).ok)
    state = service.get_state().state
    self.assertEqual(
        state.mode, vehicle_state_pb2.NAVIGATION_MODE_DEAD_RECKONING
    )
    self.assertEqual(state.header.source_time.seconds, 100)
    self.assertEqual(state.header.receive_time.seconds, 100)
    self.assertTrue(service.predict(_ts(101)).ok)
    self.assertEqual(
        service.get_state().state.mode,
        vehicle_state_pb2.NAVIGATION_MODE_DEAD_RECKONING,
    )

  def test_accepted_aiding_ingest_makes_next_predict_aided_once(self):
    service = self._ready(seed=9)
    self.assertTrue(
        service.ingest(
            _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10))
        ).ok
    )
    self.assertEqual(
        service.get_state().state.mode,
        vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING,
    )
    service.predict(_ts(11))
    self.assertEqual(
        service.get_state().state.mode, vehicle_state_pb2.NAVIGATION_MODE_AIDED
    )
    service.predict(_ts(12))
    self.assertEqual(
        service.get_state().state.mode,
        vehicle_state_pb2.NAVIGATION_MODE_DEAD_RECKONING,
    )

  def test_propagation_kinds_do_not_aid(self):
    service = self._ready(seed=9)
    for index, kind in enumerate(
        (
            estimator_pb2.MEASUREMENT_KIND_IMU,
            estimator_pb2.MEASUREMENT_KIND_INS,
            estimator_pb2.MEASUREMENT_KIND_THRUSTER_FEEDBACK,
        )
    ):
      self.assertTrue(
          service.ingest(_envelope("src%d" % index, kind, _ts(10))).ok
      )
    service.predict(_ts(11))
    self.assertEqual(
        service.get_state().state.mode,
        vehicle_state_pb2.NAVIGATION_MODE_DEAD_RECKONING,
    )

  def test_predict_holds_kinematics_and_rejects_time_regression(self):
    service = self._ready(seed=9, publish_pose_covariance=True)
    before = service.get_state().state
    service.predict(_ts(50))
    after = service.get_state().state
    self.assertEqual(after.pose_world_from_body, before.pose_world_from_body)
    self.assertEqual(after.body_twist, before.body_twist)
    self.assertEqual(after.pose_covariance, before.pose_covariance)
    self.assertGreater(after.header.sequence, before.header.sequence)
    self.assertEqual(
        service.predict(_ts(49)).reason,
        estimator_pb2.REJECT_REASON_INVALID_TIME,
    )
    self.assertEqual(service.get_state().state.header.source_time.seconds, 50)
    self.assertTrue(service.predict(_ts(50)).ok)
    self.assertEqual(
        service.predict(_ts(60, 1000000000)).reason,
        estimator_pb2.REJECT_REASON_INVALID_TIME,
    )

  def test_reset_bumps_epoch_and_empties_the_body_without_prior(self):
    service = self._ready(seed=9)
    service.predict(_ts(5))
    result = service.reset()
    self.assertTrue(result.ok)
    self.assertFalse(result.prior_accepted)
    self.assertEqual(result.estimator_epoch, 2)
    state = service.get_state().state
    self.assertFalse(state.HasField("pose_world_from_body"))
    self.assertFalse(state.HasField("body_twist"))
    self.assertEqual(state.mode, vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING)
    self.assertEqual(state.estimator_epoch, 2)
    service.predict(_ts(6))
    self.assertEqual(
        service.get_state().state.mode,
        vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING,
    )
    self.assertEqual(service.reset().estimator_epoch, 3)

  def test_reset_with_accepted_prior_becomes_the_body(self):
    service = self._ready(seed=9)
    prior = vehicle_state_pb2.VehicleState()
    text_format.Parse(
        """
        header { sequence: 500 frame_id: "world_enu" clock_domain: "monotonic"
                 source_time { seconds: 40 } validity { state: STATE_VALID } }
        pose_world_from_body { position { x: 1 y: 2 z: -3 } orientation { w: 1 } }
        mode: NAVIGATION_MODE_AIDED
        sources { source_id: "stale" }
        estimator_epoch: 99
        """,
        prior,
    )
    result = service.reset(prior)
    self.assertTrue(result.prior_accepted)
    state = service.get_state().state
    self.assertEqual(state.pose_world_from_body, prior.pose_world_from_body)
    self.assertEqual(state.mode, vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING)
    self.assertEqual(state.estimator_epoch, 2)
    self.assertEqual(len(state.sources), 0)
    self.assertEqual(
        service.predict(_ts(39)).reason,
        estimator_pb2.REJECT_REASON_INVALID_TIME,
    )
    self.assertTrue(service.predict(_ts(41)).ok)
    state = service.get_state().state
    self.assertEqual(
        state.mode, vehicle_state_pb2.NAVIGATION_MODE_DEAD_RECKONING
    )
    self.assertGreater(state.header.sequence, 500)

  def test_reset_with_rejected_prior_is_empty(self):
    service = self._ready(seed=9)
    prior = vehicle_state_pb2.VehicleState()
    prior.header.frame_id = "world_enu"
    prior.header.validity.state = _VALID
    prior.pose_world_from_body.position.x = float("nan")
    prior.pose_world_from_body.orientation.w = 1
    result = service.reset(prior)
    self.assertTrue(result.ok)
    self.assertFalse(result.prior_accepted)
    self.assertFalse(service.get_state().state.HasField("pose_world_from_body"))
    no_validity = vehicle_state_pb2.VehicleState()
    no_validity.header.frame_id = "world_enu"
    no_validity.pose_world_from_body.orientation.w = 1
    self.assertFalse(service.reset(no_validity).prior_accepted)

  def test_ingest_accept_and_source_health(self):
    service = self._ready(seed=2)
    result = service.ingest(
        _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10, 5))
    )
    self.assertTrue(result.ok)
    self.assertEqual(result.reason, estimator_pb2.REJECT_REASON_NONE)
    response = service.get_state()
    self.assertEqual(len(response.source_status), 1)
    status = response.source_status[0]
    self.assertEqual(status.source_id, "dvl")
    self.assertTrue(status.healthy)
    self.assertEqual(status.last_accept_source_time, _ts(10, 5))
    self.assertFalse(status.HasField("last_reject_reason"))
    self.assertEqual(len(response.state.sources), 1)
    self.assertEqual(response.state.sources[0].source_id, "dvl")
    self.assertEqual(response.state.sources[0].validity.state, _VALID)

  def test_reject_leaves_published_state_unchanged(self):
    service = self._ready(
        seed=2,
        publish_pose_covariance=True,
        unhealthy_after_consecutive_rejects=10,
    )
    service.ingest(
        _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10))
    )
    service.predict(_ts(11))
    before = service.get_state().state.SerializeToString(deterministic=True)
    digest = service.state_digest()
    rejects = (
        _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(9)),
        _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10)),
        _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(20), _ts(12)),
    )
    for envelope in rejects:
      self.assertFalse(service.ingest(envelope).ok)
    self.assertEqual(
        service.get_state().state.SerializeToString(deterministic=True), before
    )
    self.assertEqual(service.state_digest(), digest)
    # Unknown sources that never had an accept are not published either.
    service.ingest(
        _envelope("ghost", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(99), _ts(1))
    )
    self.assertEqual(
        service.get_state().state.SerializeToString(deterministic=True), before
    )
    self.assertEqual(len(service.get_state().source_status), 2)

  def test_future_measurement_rejected_with_default_skew_zero(self):
    service = self._ready(seed=2)
    result = service.ingest(
        _envelope(
            "dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10, 1), _ts(10)
        )
    )
    self.assertEqual(
        result.reason, estimator_pb2.REJECT_REASON_FUTURE_MEASUREMENT
    )
    self.assertTrue(
        service.ingest(
            _envelope(
                "dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10), _ts(10)
            )
        ).ok
    )

  def test_future_skew_is_honored(self):
    service = self._ready(
        seed=2, max_future_skew=duration_pb2.Duration(nanos=500000000)
    )
    self.assertTrue(
        service.ingest(
            _envelope(
                "dvl",
                estimator_pb2.MEASUREMENT_KIND_DVL,
                _ts(10, 500000000),
                _ts(10),
            )
        ).ok
    )
    self.assertEqual(
        service.ingest(
            _envelope(
                "dvl",
                estimator_pb2.MEASUREMENT_KIND_DVL,
                _ts(11, 500000001),
                _ts(11),
            )
        ).reason,
        estimator_pb2.REJECT_REASON_FUTURE_MEASUREMENT,
    )

  def test_out_of_order_and_duplicate_are_rejected_per_source(self):
    service = self._ready(seed=2)
    self.assertTrue(
        service.ingest(
            _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10))
        ).ok
    )
    self.assertEqual(
        service.ingest(
            _envelope(
                "dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(9, 999999999)
            )
        ).reason,
        estimator_pb2.REJECT_REASON_OUT_OF_ORDER,
    )
    self.assertEqual(
        service.ingest(
            _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10))
        ).reason,
        estimator_pb2.REJECT_REASON_DUPLICATE_TIMESTAMP,
    )
    # Another source keeps its own clock.
    self.assertTrue(
        service.ingest(
            _envelope("depth", estimator_pb2.MEASUREMENT_KIND_PRESSURE, _ts(5))
        ).ok
    )
    status = {s.source_id: s for s in service.get_state().source_status}
    self.assertEqual(
        status["dvl"].last_reject_reason,
        estimator_pb2.REJECT_REASON_DUPLICATE_TIMESTAMP,
    )
    self.assertEqual(status["dvl"].last_accept_source_time, _ts(10))

  def test_delayed_but_in_order_is_accepted(self):
    service = self._ready(seed=2)
    service.predict(_ts(100))
    result = service.ingest(
        _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(10), _ts(99))
    )
    self.assertTrue(result.ok)
    state = service.get_state().state
    self.assertEqual(state.header.source_time.seconds, 100)
    status = service.get_state().source_status[0]
    self.assertEqual(status.last_accept_source_time, _ts(10))

  def test_invalid_envelope_and_payload_are_rejected(self):
    service = self._ready(seed=2)
    kind = estimator_pb2.MEASUREMENT_KIND_DVL
    self.assertEqual(
        service.ingest(_envelope("", kind, _ts(1))).reason,
        estimator_pb2.REJECT_REASON_INVALID_ENVELOPE,
    )
    self.assertEqual(
        service.ingest(
            _envelope("a", estimator_pb2.MEASUREMENT_KIND_UNSPECIFIED, _ts(1))
        ).reason,
        estimator_pb2.REJECT_REASON_INVALID_ENVELOPE,
    )
    self.assertEqual(
        service.ingest(_envelope("a", 99, _ts(1))).reason,
        estimator_pb2.REJECT_REASON_INVALID_ENVELOPE,
    )
    no_time = _envelope("a", kind, _ts(1))
    no_time.header.ClearField("receive_time")
    self.assertEqual(
        service.ingest(no_time).reason,
        estimator_pb2.REJECT_REASON_INVALID_ENVELOPE,
    )
    untyped = _envelope("a", kind, _ts(1))
    untyped.payload_type = ""
    self.assertEqual(
        service.ingest(untyped).reason,
        estimator_pb2.REJECT_REASON_INVALID_ENVELOPE,
    )
    self.assertEqual(
        service.ingest(_envelope("a", kind, _ts(1), clock_domain="utc")).reason,
        estimator_pb2.REJECT_REASON_CLOCK_DOMAIN_MISMATCH,
    )
    for validity in (
        None,
        _INVALID,
        stamped_header_pb2.Validity.STATE_UNSPECIFIED,
    ):
      self.assertEqual(
          service.ingest(
              _envelope("a", kind, _ts(1), validity=validity)
          ).reason,
          estimator_pb2.REJECT_REASON_INVALID_PAYLOAD,
      )
    service.set_payload_validator(lambda envelope: len(envelope.payload) > 4)
    self.assertEqual(
        service.ingest(_envelope("a", kind, _ts(1))).reason,
        estimator_pb2.REJECT_REASON_INVALID_PAYLOAD,
    )
    service.set_payload_validator(None)
    self.assertTrue(service.ingest(_envelope("a", kind, _ts(1))).ok)

  def test_source_becomes_unhealthy_after_consecutive_rejects(self):
    service = self._ready(seed=2, unhealthy_after_consecutive_rejects=2)
    kind = estimator_pb2.MEASUREMENT_KIND_DVL
    service.ingest(_envelope("dvl", kind, _ts(10)))
    service.ingest(_envelope("dvl", kind, _ts(9)))
    self.assertTrue(service.get_state().source_status[0].healthy)
    self.assertEqual(
        service.get_state().state.sources[0].validity.state, _VALID
    )
    service.ingest(_envelope("dvl", kind, _ts(8)))
    status = service.get_state().source_status[0]
    self.assertFalse(status.healthy)
    self.assertEqual(status.consecutive_rejects, 2)
    self.assertEqual(
        service.get_state().state.sources[0].validity.state, _INVALID
    )
    self.assertTrue(service.ingest(_envelope("dvl", kind, _ts(11))).ok)
    status = service.get_state().source_status[0]
    self.assertTrue(status.healthy)
    self.assertEqual(status.consecutive_rejects, 0)
    self.assertEqual(
        status.last_reject_reason, estimator_pb2.REJECT_REASON_OUT_OF_ORDER
    )

  def test_default_unhealthy_threshold_is_three(self):
    service = self._ready(seed=2)
    kind = estimator_pb2.MEASUREMENT_KIND_DVL
    service.ingest(_envelope("dvl", kind, _ts(10)))
    for _ in range(2):
      service.ingest(_envelope("dvl", kind, _ts(1)))
    self.assertTrue(service.get_state().source_status[0].healthy)
    service.ingest(_envelope("dvl", kind, _ts(1)))
    self.assertFalse(service.get_state().source_status[0].healthy)

  def test_source_fault_injection(self):
    service = self._ready(seed=2)
    kind = estimator_pb2.MEASUREMENT_KIND_DVL
    service.ingest(_envelope("dvl", kind, _ts(10)))
    self.assertFalse(service.inject_source_fault("missing"))
    self.assertTrue(service.inject_source_fault("dvl"))
    self.assertEqual(
        service.get_state().state.sources[0].validity.state, _INVALID
    )
    service.ingest(_envelope("dvl", kind, _ts(11)))
    self.assertEqual(
        service.get_state().state.sources[0].validity.state, _VALID
    )

  def test_fault_injection_forces_faulted_until_reset_or_initialize(self):
    service = self._ready(seed=2)
    self.assertTrue(service.inject_fault())
    self.assertEqual(
        service.get_state().state.mode,
        vehicle_state_pb2.NAVIGATION_MODE_FAULTED,
    )
    self.assertEqual(
        service.ingest(
            _envelope("dvl", estimator_pb2.MEASUREMENT_KIND_DVL, _ts(1))
        ).reason,
        estimator_pb2.REJECT_REASON_ESTIMATOR_FAULTED,
    )
    self.assertTrue(service.predict(_ts(5)).ok)
    state = service.get_state().state
    self.assertEqual(state.mode, vehicle_state_pb2.NAVIGATION_MODE_FAULTED)
    self.assertEqual(state.header.source_time.seconds, 5)
    service.reset()
    self.assertEqual(
        service.get_state().state.mode,
        vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING,
    )
    service.inject_fault()
    service.initialize(_config(seed=2))
    self.assertEqual(
        service.get_state().state.mode,
        vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING,
    )

  def test_navigation_mode_wire_values_are_the_17_values(self):
    self.assertEqual(vehicle_state_pb2.NAVIGATION_MODE_UNSPECIFIED, 0)
    self.assertEqual(vehicle_state_pb2.NAVIGATION_MODE_INITIALIZING, 1)
    self.assertEqual(vehicle_state_pb2.NAVIGATION_MODE_DEAD_RECKONING, 2)
    self.assertEqual(vehicle_state_pb2.NAVIGATION_MODE_AIDED, 3)
    self.assertEqual(vehicle_state_pb2.NAVIGATION_MODE_FAULTED, 4)
    service = self._ready(seed=2)
    for step in range(3):
      service.predict(_ts(step + 1))
      kind = vehicle_policy.classify_navigation_mode(
          service.get_state().state.mode
      )
      self.assertIsNot(kind, vehicle_policy.NavigationModeKind.UNKNOWN)

  def test_epoch_is_monotonic_across_init_and_reset(self):
    service = fake_estimator_service.FakeEstimatorService()
    epochs = [
        service.initialize(_config()).estimator_epoch,
        service.reset().estimator_epoch,
        service.initialize(_config()).estimator_epoch,
        service.reset().estimator_epoch,
    ]
    self.assertEqual(epochs, [1, 2, 3, 4])
    self.assertEqual(service.get_state().state.estimator_epoch, 4)

  def test_replay_digests_repeat_for_a_fixed_seed(self):
    first = fake_estimator_service.FakeEstimatorService()
    second = fake_estimator_service.FakeEstimatorService()
    _run_nominal(first)
    _run_nominal(second)
    self.assertEqual(first.state_digest(), second.state_digest())
    self.assertNotEqual(first.state_digest(), 0)
    other = fake_estimator_service.FakeEstimatorService()
    other.initialize(_config(seed=30, publish_pose_covariance=True))
    other.predict(_ts(1700000001))
    self.assertNotEqual(first.state_digest(), other.state_digest())

  def test_nominal_snapshot_matches_the_golden(self):
    service = fake_estimator_service.FakeEstimatorService()
    _run_nominal(service)
    expected = vehicle_state_pb2.VehicleState()
    text_format.Parse(_load_golden("nominal_vehicle_state.textproto"), expected)
    actual = service.get_state().state
    self.assertEqual(actual, expected)
    self.assertEqual(
        actual.SerializeToString(deterministic=True),
        expected.SerializeToString(deterministic=True),
    )
    self.assertEqual(service.state_digest(), _NOMINAL_DIGEST)
    self.assertEqual(actual.mode, vehicle_state_pb2.NAVIGATION_MODE_AIDED)
    self.assertEqual(actual.estimator_epoch, 1)
    self.assertTrue(fake_estimator_service._has_accepted_pose(actual))


if __name__ == "__main__":
  unittest.main()
