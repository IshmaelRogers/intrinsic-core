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

"""Serialization and textproto tests for vehicle trajectories."""

import os
import unittest

from google.protobuf import text_format

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import trajectory_contract_policy
from intrinsic.vehicle.proto import vehicle_trajectory_pb2

# Canonical serialization of _fill_two_sample. Keep in sync with
# trajectory_contract_serialization_test.cc.
_VEHICLE_TRAJECTORY_GOLDEN_HEX = (
    "0a3a080b120b088ae2cfaa061080e59a771a06088be2cfaa062207706c616e6e65722a09776f"
    "726c645f656e7532096d6f6e6f746f6e69633a020801120a7472616a5f616c7068611a460a06"
    "088ae2cfaa0612280a1b09000000000000f03f1100000000000000401900000000000008c012"
    "0921000000000000f03f1a1209000000000000e03f31000000000000c03f1a430a0c088be2cf"
    "aa061080cab5ee0112280a1b09000000000000f83f1100000000000000401900000000000008"
    "c0120921000000000000f03f1a0909000000000000e03f421e0a0b7575765f706c616e6e6572"
    "120276311a0b7368613235363a7472616a"
)


def _fill_header(header, sequence, seconds, source_id, frame_id):
  header.sequence = sequence
  header.source_time.seconds = seconds
  header.source_time.nanos = 250000000
  header.receive_time.seconds = seconds + 1
  header.source_id = source_id
  header.frame_id = frame_id
  header.clock_domain = stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID


def _fill_sample(sample, seconds, nanos, x, angular_z):
  sample.time.seconds = seconds
  if nanos != 0:
    sample.time.nanos = nanos
  sample.pose_world_from_body.position.x = x
  sample.pose_world_from_body.position.y = 2
  sample.pose_world_from_body.position.z = -3
  sample.pose_world_from_body.orientation.w = 1
  sample.body_twist.linear_x_m_s = 0.5
  if angular_z:
    sample.body_twist.angular_z_rad_s = 0.125


def _fill_two_sample():
  trajectory = vehicle_trajectory_pb2.VehicleTrajectory()
  _fill_header(
      trajectory.header,
      11,
      1700000010,
      "planner",
      frame_policy.WORLD_ENU_FRAME_ID,
  )
  trajectory.trajectory_id = "traj_alpha"
  _fill_sample(trajectory.samples.add(), 1700000010, 0, 1, True)
  _fill_sample(trajectory.samples.add(), 1700000011, 500000000, 1.5, False)
  trajectory.provenance.model_id = "uuv_planner"
  trajectory.provenance.model_version = "v1"
  trajectory.provenance.digest = "sha256:traj"
  return trajectory


def _sample_view(sample):
  pose = sample.pose_world_from_body
  twist = sample.body_twist
  acceleration = sample.body_acceleration
  return trajectory_contract_policy.TrajectorySampleView(
      time_present=sample.HasField("time"),
      seconds=sample.time.seconds,
      nanos=sample.time.nanos,
      position=(pose.position.x, pose.position.y, pose.position.z),
      orientation=(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w,
      ),
      twist_present=sample.HasField("body_twist"),
      twist=(
          twist.linear_x_m_s,
          twist.linear_y_m_s,
          twist.linear_z_m_s,
          twist.angular_x_rad_s,
          twist.angular_y_rad_s,
          twist.angular_z_rad_s,
      ),
      acceleration_present=sample.HasField("body_acceleration"),
      acceleration=(
          acceleration.linear_x_m_s2,
          acceleration.linear_y_m_s2,
          acceleration.linear_z_m_s2,
          acceleration.angular_x_rad_s2,
          acceleration.angular_y_rad_s2,
          acceleration.angular_z_rad_s2,
      ),
  )


def _trajectory_view(message):
  tolerances = message.tolerances
  return trajectory_contract_policy.VehicleTrajectoryView(
      header_present=message.HasField("header"),
      validity_present=message.header.HasField("validity"),
      validity_state=message.header.validity.state,
      frame_id=message.header.frame_id,
      trajectory_id=message.trajectory_id,
      samples=tuple(_sample_view(sample) for sample in message.samples),
      tolerances_present=message.HasField("tolerances"),
      position_tolerance_m=tolerances.position_m,
      orientation_tolerance_rad=tolerances.orientation_rad,
      linear_velocity_tolerance_m_s=tolerances.linear_velocity_m_s,
      angular_velocity_tolerance_rad_s=tolerances.angular_velocity_rad_s,
      cost_present=message.HasField("cost"),
      cost=message.cost,
      risk_present=message.HasField("risk"),
      risk=message.risk,
      uncertainty_present=message.HasField("uncertainty"),
      uncertainty=tuple(message.uncertainty.values),
      provenance_present=message.HasField("provenance"),
      model_id=message.provenance.model_id,
      metadata_present=len(message.metadata) > 0,
  )


def _load_example(name):
  root = os.environ.get("TEST_SRCDIR", "")
  matches = []
  for dirpath, _, filenames in os.walk(root):
    if name in filenames:
      matches.append(os.path.join(dirpath, name))
  if not matches:
    raise AssertionError("missing %s under %s" % (name, root))
  matches.sort(key=lambda path: ("examples" not in path, len(path)))
  with open(matches[0], encoding="utf-8") as handle:
    return handle.read()


def _assess_text(name):
  parsed = vehicle_trajectory_pb2.VehicleTrajectory()
  text_format.Parse(_load_example(name), parsed)
  return trajectory_contract_policy.assess_vehicle_trajectory(
      _trajectory_view(parsed)
  )


class TrajectoryContractSerializationTest(unittest.TestCase):

  def test_no_robot_type_and_locked_field_numbers(self):
    descriptor = vehicle_trajectory_pb2.VehicleTrajectory.DESCRIPTOR
    self.assertEqual(descriptor.file.package, "intrinsic_proto.vehicle")
    self.assertIsNone(descriptor.fields_by_name.get("robot_type"))
    self.assertIsNone(descriptor.file.enum_types_by_name.get("RobotType"))
    self.assertEqual(descriptor.fields_by_name["header"].number, 1)
    self.assertEqual(descriptor.fields_by_name["trajectory_id"].number, 2)
    self.assertEqual(descriptor.fields_by_name["samples"].number, 3)
    self.assertEqual(descriptor.fields_by_name["tolerances"].number, 4)
    self.assertEqual(descriptor.fields_by_name["cost"].number, 5)
    self.assertEqual(descriptor.fields_by_name["risk"].number, 6)
    self.assertEqual(descriptor.fields_by_name["uncertainty"].number, 7)
    self.assertEqual(descriptor.fields_by_name["provenance"].number, 8)
    self.assertEqual(descriptor.fields_by_name["metadata"].number, 100)
    sample = descriptor.file.message_types_by_name["TrajectorySample"]
    self.assertEqual(sample.fields_by_name["time"].number, 1)
    self.assertEqual(sample.fields_by_name["pose_world_from_body"].number, 2)
    self.assertEqual(sample.fields_by_name["body_twist"].number, 3)
    self.assertEqual(sample.fields_by_name["body_acceleration"].number, 4)

  def test_default_is_zero_bytes_and_not_accepted(self):
    empty = vehicle_trajectory_pb2.VehicleTrajectory()
    self.assertEqual(empty.SerializeToString(), b"")
    self.assertEqual(empty.ByteSize(), 0)
    assessment = trajectory_contract_policy.assess_vehicle_trajectory(
        _trajectory_view(empty)
    )
    self.assertIs(
        assessment.error,
        trajectory_contract_policy.TrajectoryContractError.NONE,
    )
    self.assertFalse(assessment.accepted)
    self.assertFalse(empty.HasField("cost"))
    self.assertFalse(empty.HasField("risk"))
    self.assertFalse(empty.HasField("uncertainty"))
    self.assertFalse(empty.HasField("tolerances"))

  def test_golden_round_trip(self):
    trajectory = _fill_two_sample()
    self.assertEqual(
        trajectory.SerializeToString().hex(), _VEHICLE_TRAJECTORY_GOLDEN_HEX
    )
    parsed = vehicle_trajectory_pb2.VehicleTrajectory()
    parsed.ParseFromString(bytes.fromhex(_VEHICLE_TRAJECTORY_GOLDEN_HEX))
    self.assertEqual(parsed.SerializeToString(), trajectory.SerializeToString())
    self.assertEqual(parsed.trajectory_id, "traj_alpha")
    self.assertEqual(parsed.header.frame_id, frame_policy.WORLD_ENU_FRAME_ID)
    self.assertEqual(len(parsed.samples), 2)
    self.assertEqual(parsed.samples[1].time.nanos, 500000000)
    self.assertFalse(parsed.HasField("uncertainty"))
    self.assertEqual(parsed.provenance.model_id, "uuv_planner")
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            _trajectory_view(parsed)
        ).accepted
    )

  def test_unknown_field_is_preserved(self):
    trajectory = _fill_two_sample()
    with_unknown = trajectory.SerializeToString() + bytes([0xA8, 0x06, 0x07])
    parsed = vehicle_trajectory_pb2.VehicleTrajectory()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.trajectory_id, "traj_alpha")
    self.assertEqual(parsed.SerializeToString(), with_unknown)
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            _trajectory_view(parsed)
        ).accepted
    )

  def test_metadata_shuffle_stays_accepted(self):
    first = _fill_two_sample()
    first.metadata["planner"] = "direct"
    first.metadata["note"] = ""
    second = _fill_two_sample()
    second.metadata["note"] = ""
    second.metadata["planner"] = "direct"
    self.assertEqual(dict(first.metadata), dict(second.metadata))
    first_assessment = trajectory_contract_policy.assess_vehicle_trajectory(
        _trajectory_view(first)
    )
    second_assessment = trajectory_contract_policy.assess_vehicle_trajectory(
        _trajectory_view(second)
    )
    self.assertTrue(first_assessment.accepted)
    self.assertTrue(second_assessment.accepted)
    self.assertIs(first_assessment.error, second_assessment.error)
    first.metadata[""] = "nan"
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            _trajectory_view(first)
        ).accepted
    )

  def test_zero_cost_is_present(self):
    trajectory = vehicle_trajectory_pb2.VehicleTrajectory()
    self.assertFalse(trajectory.HasField("cost"))
    trajectory.cost = 0
    self.assertTrue(trajectory.HasField("cost"))
    with_zero = trajectory.SerializeToString()
    trajectory.ClearField("cost")
    self.assertFalse(trajectory.HasField("cost"))
    self.assertNotEqual(trajectory.SerializeToString(), with_zero)

  def test_present_zero_twist_is_not_absence(self):
    trajectory = vehicle_trajectory_pb2.VehicleTrajectory()
    trajectory.trajectory_id = "traj_alpha"
    sample = trajectory.samples.add()
    sample.time.seconds = 1
    sample.pose_world_from_body.orientation.w = 1
    sample.body_twist.SetInParent()
    self.assertTrue(sample.HasField("body_twist"))
    parsed = vehicle_trajectory_pb2.VehicleTrajectory()
    parsed.ParseFromString(trajectory.SerializeToString())
    self.assertTrue(parsed.samples[0].HasField("body_twist"))
    self.assertEqual(parsed.samples[0].body_twist.linear_x_m_s, 0)

  def test_textproto_fixtures(self):
    parsed = vehicle_trajectory_pb2.VehicleTrajectory()
    text_format.Parse(
        _load_example("vehicle_trajectory_two_sample.textproto"), parsed
    )
    self.assertEqual(
        parsed.SerializeToString(), _fill_two_sample().SerializeToString()
    )
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            _trajectory_view(parsed)
        ).accepted
    )
    with_metadata = vehicle_trajectory_pb2.VehicleTrajectory()
    text_format.Parse(
        _load_example("vehicle_trajectory_with_metadata.textproto"),
        with_metadata,
    )
    self.assertFalse(with_metadata.HasField("uncertainty"))
    self.assertEqual(with_metadata.metadata["planner"], "direct")
    self.assertEqual(with_metadata.metadata["note"], "")
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            _trajectory_view(with_metadata)
        ).accepted
    )
    non_monotonic = _assess_text("vehicle_trajectory_non_monotonic.textproto")
    self.assertIs(
        non_monotonic.error,
        trajectory_contract_policy.TrajectoryContractError.NON_MONOTONIC_TIME,
    )
    self.assertIs(
        non_monotonic.validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertFalse(non_monotonic.accepted)
    bad_frame = _assess_text("vehicle_trajectory_bad_frame.textproto")
    self.assertIs(
        bad_frame.error,
        trajectory_contract_policy.TrajectoryContractError.MISSING_FRAME,
    )
    non_finite = _assess_text("vehicle_trajectory_non_finite.textproto")
    self.assertIs(
        non_finite.error,
        trajectory_contract_policy.TrajectoryContractError.NON_FINITE,
    )
    self.assertIs(non_finite.validity, stamped_header_policy.ValidityKind.VALID)
    empty_id = _assess_text("vehicle_trajectory_empty_id.textproto")
    self.assertIs(
        empty_id.error,
        trajectory_contract_policy.TrajectoryContractError.TRAJECTORY_ID,
    )
    self.assertIs(empty_id.validity, stamped_header_policy.ValidityKind.VALID)


if __name__ == "__main__":
  unittest.main()
