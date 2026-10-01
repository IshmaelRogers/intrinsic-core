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

"""Tests for the snapshot-backed vehicle trajectory validity adapter."""

import dataclasses
import math
import unittest

from intrinsic.motion_planning.vehicle import vehicle_state_space
from intrinsic.motion_planning.vehicle import vehicle_trajectory_validity as validity
from intrinsic.world.world_snapshot import world_snapshot_policy as snapshot

_Error = validity.TrajectoryValidityError

_FRAME = "world_enu"
_REGION = "ops-box"
_OCCUPANCY = snapshot.OCCUPANCY_REFERENCE_KIND
_COMPONENTS = (snapshot.ComponentRevisionView(_OCCUPANCY, 1),)


def _view():
  return snapshot.WorldSnapshotView(
      present=True,
      snapshot_id=snapshot.compute_snapshot_id(1, _COMPONENTS),
      creation_time_present=True,
      creation_time=(100, 0),
      state_epoch=1,
      components=_COMPONENTS,
  )


def _sample(t, x, y=0.0, z=-2.0):
  return validity.PropagationSample(
      time_s=t,
      state=vehicle_state_space.VehiclePlanningState(position=(x, y, z)),
  )


def _clear(clearance_m=2.0, source=validity.ClearanceSource.OBSTACLE, **kwargs):
  return validity.ClearanceSample(
      frame_id=_FRAME, source=source, clearance_m=clearance_m, **kwargs
  )


def _request(**overrides):
  view = _view()
  fields = dict(
      snapshot_id=view.snapshot_id,
      descriptor=view,
      samples=(_sample(0.0, 0.0), _sample(0.5, 1.0), _sample(1.0, 2.0)),
      bounds=validity.VehicleStateBounds(
          position_limits_present=True,
          position_min=(-50.0, -50.0, -50.0),
          position_max=(50.0, 50.0, 50.0),
      ),
      fence=validity.AabbGeofence(
          _FRAME, _REGION, -10.0, -10.0, -10.0, 10.0, 10.0, 10.0
      ),
      clearance_samples=(_clear(), _clear(), _clear()),
  )
  fields.update(overrides)
  return validity.TrajectoryValidityRequest(**fields)


def _stale_timings():
  return (
      validity.ComponentTiming(
          component_kind=_OCCUPANCY,
          observation_time=(0, 0),
          validity_horizon=(1, 0),
      ),
  )


def _skew_policy():
  return validity.WorldSnapshotSkewPolicy(required_kinds=(_OCCUPANCY,))


class TrajectoryValidityTest(unittest.TestCase):

  def assert_result(self, result, error, index, rule):
    self.assertEqual(result.error, error)
    self.assertEqual(result.first_invalid_sample, index)
    self.assertEqual(result.failed_rule, rule)

  def test_enum_wire_values(self):
    self.assertEqual(
        [e.value for e in _Error],
        [0, 1, 2, 3, 4, 5, 6],
    )

  def test_snapshot_id_is_sha256_hex(self):
    self.assertEqual(len(_view().snapshot_id), 64)

  def test_free_trajectory_is_ok(self):
    result = validity.check_trajectory_validity(_request())
    self.assert_result(result, _Error.OK, -1, "")

  def test_free_with_fresh_skew_is_ok(self):
    timings = (
        validity.ComponentTiming(
            component_kind=_OCCUPANCY,
            observation_time=(100, 0),
            validity_horizon=(10, 0),
        ),
    )
    result = validity.check_trajectory_validity(
        _request(
            assess_skew=True,
            timings=timings,
            query_time=(101, 0),
            skew_policy=_skew_policy(),
        )
    )
    self.assert_result(result, _Error.OK, -1, "")

  def test_clearance_exactly_at_minimum_is_ok(self):
    result = validity.check_trajectory_validity(
        _request(clearance_samples=(_clear(0.5), _clear(0.5), _clear(0.5)))
    )
    self.assert_result(result, _Error.OK, -1, "")

  def test_collision_reports_first_sample_below_minimum(self):
    request = _request(
        clearance_samples=(_clear(), _clear(0.4), _clear(0.01)),
    )
    result = validity.check_trajectory_validity(request)
    self.assert_result(result, _Error.CLEARANCE, 1, "clearance.min")

  def test_collision_later_worse_samples_do_not_move_first_index(self):
    shallow = _request(clearance_samples=(_clear(), _clear(0.4), _clear()))
    deep = _request(clearance_samples=(_clear(), _clear(0.4), _clear(-3.0)))
    self.assertEqual(
        validity.check_trajectory_validity(shallow),
        validity.check_trajectory_validity(deep),
    )

  def test_low_altitude_seafloor_at_first_sample(self):
    seafloor = _clear(0.1, validity.ClearanceSource.SEAFLOOR)
    result = validity.check_trajectory_validity(
        _request(clearance_samples=(seafloor, _clear(), _clear()))
    )
    self.assert_result(result, _Error.CLEARANCE, 0, "clearance.min")

  def test_unknown_map_is_clearance_failure(self):
    unknown = _clear(9.0, validity.ClearanceSource.UNKNOWN_MAP)
    result = validity.check_trajectory_validity(
        _request(clearance_samples=(_clear(), _clear(), unknown))
    )
    self.assert_result(result, _Error.CLEARANCE, 2, "clearance.min")

  def test_custom_min_clearance_is_applied(self):
    result = validity.check_trajectory_validity(_request(min_clearance_m=3.0))
    self.assert_result(result, _Error.CLEARANCE, 0, "clearance.min")

  def test_stale_skew_withholds_before_samples(self):
    result = validity.check_trajectory_validity(
        _request(
            assess_skew=True,
            timings=_stale_timings(),
            query_time=(5, 0),
            skew_policy=_skew_policy(),
            bounds=validity.VehicleStateBounds(
                position_limits_present=True,
                position_min=(100.0, 100.0, 100.0),
                position_max=(101.0, 101.0, 101.0),
            ),
        )
    )
    self.assert_result(result, _Error.STALE_SNAPSHOT, -1, "snapshot")

  def test_missing_timing_for_required_kind_is_stale(self):
    result = validity.check_trajectory_validity(
        _request(assess_skew=True, skew_policy=_skew_policy())
    )
    self.assert_result(result, _Error.STALE_SNAPSHOT, -1, "snapshot")

  def test_stale_skew_ignored_when_not_assessed(self):
    result = validity.check_trajectory_validity(
        _request(
            assess_skew=False,
            timings=_stale_timings(),
            query_time=(5, 0),
            skew_policy=_skew_policy(),
        )
    )
    self.assert_result(result, _Error.OK, -1, "")

  def test_sample_local_snapshot_unusable_is_stale(self):
    unusable = _clear(snapshot_usable=False)
    result = validity.check_trajectory_validity(
        _request(clearance_samples=(_clear(), unusable, _clear()))
    )
    self.assert_result(result, _Error.STALE_SNAPSHOT, 1, "clearance.min")

  def test_frame_error_from_pose_frame_override(self):
    result = validity.check_trajectory_validity(
        _request(pose_frame="world_ned")
    )
    self.assert_result(result, _Error.FRAME_ERROR, 0, "geofence.aabb")

  def test_pose_frame_equal_to_fence_frame_is_ok(self):
    result = validity.check_trajectory_validity(_request(pose_frame=_FRAME))
    self.assert_result(result, _Error.OK, -1, "")

  def test_invalid_fence_is_bad_request_at_sample(self):
    fence = validity.AabbGeofence(
        _FRAME, _REGION, 10.0, -10.0, -10.0, -10.0, 10.0, 10.0
    )
    result = validity.check_trajectory_validity(_request(fence=fence))
    self.assert_result(result, _Error.BAD_REQUEST, 0, "geofence.aabb")

  def test_empty_samples_is_bad_request(self):
    result = validity.check_trajectory_validity(
        _request(samples=(), clearance_samples=())
    )
    self.assert_result(result, _Error.BAD_REQUEST, -1, "")

  def test_clearance_length_mismatch_is_bad_request(self):
    result = validity.check_trajectory_validity(
        _request(clearance_samples=(_clear(), _clear()))
    )
    self.assert_result(result, _Error.BAD_REQUEST, -1, "")

  def test_empty_snapshot_id_is_bad_request(self):
    result = validity.check_trajectory_validity(_request(snapshot_id=""))
    self.assert_result(result, _Error.BAD_REQUEST, -1, "snapshot")

  def test_snapshot_id_mismatch_is_bad_request(self):
    result = validity.check_trajectory_validity(
        _request(snapshot_id=snapshot.compute_snapshot_id(2, _COMPONENTS))
    )
    self.assert_result(result, _Error.BAD_REQUEST, -1, "snapshot")

  def test_absent_descriptor_is_bad_request(self):
    result = validity.check_trajectory_validity(
        _request(descriptor=snapshot.WorldSnapshotView())
    )
    self.assert_result(result, _Error.BAD_REQUEST, -1, "snapshot")

  def test_structurally_defective_descriptor_is_bad_request(self):
    view = dataclasses.replace(_view(), creation_time_present=False)
    result = validity.check_trajectory_validity(_request(descriptor=view))
    self.assert_result(result, _Error.BAD_REQUEST, -1, "snapshot")

  def test_duplicate_component_kind_is_bad_request(self):
    view = dataclasses.replace(
        _view(),
        components=_COMPONENTS + _COMPONENTS,
    )
    result = validity.check_trajectory_validity(_request(descriptor=view))
    self.assert_result(result, _Error.BAD_REQUEST, -1, "snapshot")

  def test_shape_defects_precede_snapshot_defects(self):
    result = validity.check_trajectory_validity(
        _request(samples=(), snapshot_id="", clearance_samples=(_clear(),))
    )
    self.assert_result(result, _Error.BAD_REQUEST, -1, "")
    result = validity.check_trajectory_validity(
        _request(snapshot_id="", clearance_samples=(_clear(),))
    )
    self.assert_result(result, _Error.BAD_REQUEST, -1, "")

  def test_snapshot_defect_precedes_skew_withhold(self):
    result = validity.check_trajectory_validity(
        _request(
            snapshot_id="",
            assess_skew=True,
            timings=_stale_timings(),
            query_time=(5, 0),
            skew_policy=_skew_policy(),
        )
    )
    self.assert_result(result, _Error.BAD_REQUEST, -1, "snapshot")

  def test_bounds_failure_position_box(self):
    bounds = validity.VehicleStateBounds(
        position_limits_present=True,
        position_min=(-1.5, -1.0, -3.0),
        position_max=(1.5, 1.0, -1.0),
    )
    result = validity.check_trajectory_validity(_request(bounds=bounds))
    self.assert_result(result, _Error.BOUNDS, 2, "bounds")

  def test_bounds_failure_non_finite_state(self):
    samples = (_sample(0.0, 0.0), _sample(0.5, math.nan), _sample(1.0, 2.0))
    result = validity.check_trajectory_validity(_request(samples=samples))
    self.assert_result(result, _Error.BOUNDS, 1, "bounds")

  def test_bounds_failure_bad_bounds_configuration(self):
    bounds = validity.VehicleStateBounds(
        max_linear_speed_present=True, max_linear_speed_m_s=-1.0
    )
    result = validity.check_trajectory_validity(_request(bounds=bounds))
    self.assert_result(result, _Error.BOUNDS, 0, "bounds")

  def test_geofence_outside(self):
    samples = (_sample(0.0, 0.0), _sample(0.5, 10.5), _sample(1.0, 2.0))
    result = validity.check_trajectory_validity(_request(samples=samples))
    self.assert_result(result, _Error.GEOFENCE, 1, "geofence.aabb")

  def test_geofence_boundary_is_inside(self):
    samples = (_sample(0.0, 10.0), _sample(0.5, -10.0), _sample(1.0, 2.0))
    result = validity.check_trajectory_validity(_request(samples=samples))
    self.assert_result(result, _Error.OK, -1, "")

  def test_bounds_precede_geofence_and_clearance_at_same_sample(self):
    samples = (_sample(0.0, 0.0), _sample(0.5, 20.0), _sample(1.0, 2.0))
    bounds = validity.VehicleStateBounds(
        position_limits_present=True,
        position_min=(-5.0, -5.0, -5.0),
        position_max=(5.0, 5.0, 5.0),
    )
    result = validity.check_trajectory_validity(
        _request(
            samples=samples,
            bounds=bounds,
            clearance_samples=(_clear(), _clear(0.0), _clear()),
        )
    )
    self.assert_result(result, _Error.BOUNDS, 1, "bounds")

  def test_geofence_precedes_clearance_at_same_sample(self):
    samples = (_sample(0.0, 0.0), _sample(0.5, 20.0), _sample(1.0, 2.0))
    result = validity.check_trajectory_validity(
        _request(
            samples=samples,
            clearance_samples=(_clear(), _clear(0.0), _clear()),
        )
    )
    self.assert_result(result, _Error.GEOFENCE, 1, "geofence.aabb")

  def test_earlier_clearance_failure_wins_over_later_bounds_failure(self):
    samples = (_sample(0.0, 0.0), _sample(0.5, 1.0), _sample(1.0, math.inf))
    result = validity.check_trajectory_validity(
        _request(
            samples=samples,
            clearance_samples=(_clear(0.1), _clear(), _clear()),
        )
    )
    self.assert_result(result, _Error.CLEARANCE, 0, "clearance.min")

  def test_deterministic_repeat(self):
    request = _request(clearance_samples=(_clear(), _clear(0.4), _clear()))
    first = validity.check_trajectory_validity(request)
    for _ in range(5):
      self.assertEqual(validity.check_trajectory_validity(request), first)

  def test_does_not_mutate_request_samples(self):
    request = _request()
    before = (request.samples, request.clearance_samples, request.descriptor)
    validity.check_trajectory_validity(request)
    self.assertEqual(
        before,
        (request.samples, request.clearance_samples, request.descriptor),
    )


if __name__ == "__main__":
  unittest.main()
