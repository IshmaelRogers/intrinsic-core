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

"""Serialization and textproto tests for vehicle contracts."""

import os
import unittest

from google.protobuf import text_format

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy
from intrinsic.vehicle.proto import vehicle_command_pb2
from intrinsic.vehicle.proto import vehicle_state_pb2

# Canonical serializations of the fillers below. Keep in sync with
# vehicle_contract_serialization_test.cc.
_VEHICLE_STATE_GOLDEN_HEX = "pending"
_DESIRED_MOTION_GOLDEN_HEX = "pending"
_BODY_WRENCH_GOLDEN_HEX = "pending"


def _pose_covariance():
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[0] = 0.25
  values[7] = 0.25
  values[14] = 0.25
  values[21] = 0.0625
  values[28] = 0.0625
  values[35] = 0.0625
  return values


def _fill_header(header, sequence, seconds, source_id, frame_id):
  header.sequence = sequence
  header.source_time.seconds = seconds
  header.source_time.nanos = 250000000
  header.receive_time.seconds = seconds + 1
  header.source_id = source_id
  header.frame_id = frame_id
  header.clock_domain = stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID


def _fill_vehicle_state():
  state = vehicle_state_pb2.VehicleState()
  _fill_header(
      state.header, 42, 1700000000, "eskf", frame_policy.WORLD_ENU_FRAME_ID
  )
  state.pose_world_from_body.position.x = 1
  state.pose_world_from_body.position.y = 2
  state.pose_world_from_body.position.z = -3
  state.pose_world_from_body.orientation.w = 1
  state.body_twist.linear_x_m_s = 0.5
  state.body_twist.angular_z_rad_s = 0.125
  state.body_acceleration.linear_z_m_s2 = 0.25
  state.pose_covariance.values.extend(_pose_covariance())
  state.mode = vehicle_state_pb2.NAVIGATION_MODE_AIDED
  dvl = state.sources.add()
  dvl.source_id = "dvl"
  dvl.validity.state = stamped_header_pb2.Validity.STATE_VALID
  state.sources.add().source_id = "depth"
  state.estimator_epoch = 3
  return state


def _fill_desired_motion_pose():
  motion = vehicle_command_pb2.DesiredMotion()
  _fill_header(
      motion.header, 7, 1700000002, "policy", frame_policy.WORLD_ENU_FRAME_ID
  )
  pose = motion.pose.pose_world_from_body
  pose.position.x = 4
  pose.position.z = -2
  pose.orientation.w = 1
  motion.horizon.seconds = 5
  motion.confidence = 0.5
  motion.provenance.model_id = "uuv_intent"
  motion.provenance.model_version = "v1"
  motion.provenance.digest = "sha256:abc"
  return motion


def _fill_desired_motion_twist():
  motion = vehicle_command_pb2.DesiredMotion()
  _fill_header(
      motion.header,
      8,
      1700000002,
      "policy",
      vehicle_contract_policy.BODY_FRAME_ID,
  )
  motion.twist.body_twist.linear_x_m_s = 0.5
  motion.horizon.seconds = 1
  motion.confidence = 0.5
  return motion


def _fill_body_wrench():
  wrench = vehicle_command_pb2.BodyWrench()
  _fill_header(
      wrench.header,
      9,
      1700000003,
      "guidance",
      vehicle_contract_policy.BODY_FRAME_ID,
  )
  wrench.force_x_n = 1.5
  wrench.torque_z_n_m = 0.25
  return wrench


def _state_view(state):
  pose = state.pose_world_from_body
  twist = state.body_twist
  acceleration = state.body_acceleration
  return vehicle_contract_policy.VehicleStateView(
      validity_present=state.header.HasField("validity"),
      validity_state=state.header.validity.state,
      frame_id=state.header.frame_id,
      pose_present=state.HasField("pose_world_from_body"),
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
          acceleration.linear_x_m_s2,
          acceleration.linear_y_m_s2,
          acceleration.linear_z_m_s2,
          acceleration.angular_x_rad_s2,
          acceleration.angular_y_rad_s2,
          acceleration.angular_z_rad_s2,
      ),
      pose_covariance_present=state.HasField("pose_covariance"),
      pose_covariance=tuple(state.pose_covariance.values),
      twist_covariance_present=state.HasField("twist_covariance"),
      twist_covariance=tuple(state.twist_covariance.values),
      sources=tuple(
          vehicle_contract_policy.SourceView(
              source.source_id,
              source.HasField("validity"),
              source.validity.state,
          )
          for source in state.sources
      ),
  )


def _motion_view(motion):
  objective = vehicle_contract_policy.ObjectiveKind.ABSENT
  position = (0.0, 0.0, 0.0)
  orientation = (0.0, 0.0, 0.0, 1.0)
  twist = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  trajectory_id = ""
  which = motion.WhichOneof("objective")
  if which == "pose":
    objective = vehicle_contract_policy.ObjectiveKind.POSE
    pose = motion.pose.pose_world_from_body
    position = (pose.position.x, pose.position.y, pose.position.z)
    orientation = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
  elif which == "twist":
    objective = vehicle_contract_policy.ObjectiveKind.TWIST
    body = motion.twist.body_twist
    twist = (
        body.linear_x_m_s,
        body.linear_y_m_s,
        body.linear_z_m_s,
        body.angular_x_rad_s,
        body.angular_y_rad_s,
        body.angular_z_rad_s,
    )
  elif which == "trajectory":
    objective = vehicle_contract_policy.ObjectiveKind.TRAJECTORY
    trajectory_id = motion.trajectory.trajectory_id
  return vehicle_contract_policy.DesiredMotionView(
      header_present=motion.HasField("header"),
      validity_present=motion.header.HasField("validity"),
      validity_state=motion.header.validity.state,
      frame_id=motion.header.frame_id,
      objective=objective,
      position=position,
      orientation=orientation,
      twist=twist,
      trajectory_id=trajectory_id,
      confidence_present=motion.HasField("confidence"),
      confidence=motion.confidence,
      horizon_present=motion.HasField("horizon"),
      horizon_seconds=motion.horizon.seconds,
      horizon_nanos=motion.horizon.nanos,
      provenance_present=motion.HasField("provenance"),
      model_id=motion.provenance.model_id,
  )


def _wrench_view(wrench):
  return vehicle_contract_policy.BodyWrenchView(
      engaged=wrench.ByteSize() != 0,
      validity_present=wrench.header.HasField("validity"),
      validity_state=wrench.header.validity.state,
      frame_id=wrench.header.frame_id,
      force_torque=(
          wrench.force_x_n,
          wrench.force_y_n,
          wrench.force_z_n,
          wrench.torque_x_n_m,
          wrench.torque_y_n_m,
          wrench.torque_z_n_m,
      ),
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


class VehicleContractSerializationTest(unittest.TestCase):

  def test_no_robot_type_on_the_contracts(self):
    descriptor = vehicle_state_pb2.VehicleState.DESCRIPTOR
    self.assertEqual(descriptor.file.package, "intrinsic_proto.vehicle")
    self.assertIsNone(descriptor.fields_by_name.get("robot_type"))
    self.assertIsNone(descriptor.fields_by_name.get("embodiment"))
    motion = vehicle_command_pb2.DesiredMotion.DESCRIPTOR
    wrench = vehicle_command_pb2.BodyWrench.DESCRIPTOR
    self.assertIsNone(motion.fields_by_name.get("robot_type"))
    self.assertIsNone(wrench.fields_by_name.get("actuator_command"))
    self.assertIsNone(descriptor.file.enum_types_by_name.get("RobotType"))

  def test_defaults_are_empty_and_opt_in(self):
    self.assertEqual(vehicle_state_pb2.VehicleState().SerializeToString(), b"")
    self.assertEqual(
        vehicle_command_pb2.DesiredMotion().SerializeToString(), b""
    )
    self.assertEqual(vehicle_command_pb2.BodyWrench().SerializeToString(), b"")
    state = vehicle_state_pb2.VehicleState()
    self.assertFalse(state.HasField("pose_world_from_body"))
    self.assertFalse(state.HasField("body_twist"))
    self.assertFalse(state.HasField("pose_covariance"))
    motion = vehicle_command_pb2.DesiredMotion()
    self.assertFalse(motion.HasField("confidence"))
    self.assertFalse(motion.HasField("horizon"))
    self.assertIsNone(motion.WhichOneof("objective"))

  def test_vehicle_state_golden_round_trip(self):
    state = _fill_vehicle_state()
    self.assertEqual(state.SerializeToString().hex(), _VEHICLE_STATE_GOLDEN_HEX)
    golden = bytes.fromhex(_VEHICLE_STATE_GOLDEN_HEX)
    parsed = vehicle_state_pb2.VehicleState()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.SerializeToString(), golden)
    self.assertFalse(parsed.HasField("twist_covariance"))
    self.assertEqual(parsed.pose_covariance.values[0], 0.25)
    self.assertEqual(len(parsed.sources), 2)
    self.assertTrue(parsed.sources[0].HasField("validity"))
    self.assertFalse(parsed.sources[1].HasField("validity"))
    assessment = vehicle_contract_policy.assess_vehicle_state(
        _state_view(parsed)
    )
    self.assertTrue(assessment.accepted)
    self.assertFalse(
        vehicle_contract_policy.covariance_is_unknown(
            vehicle_contract_policy.assess_covariance(
                parsed.HasField("pose_covariance"),
                parsed.pose_covariance.values,
            )
        )
    )
    self.assertTrue(
        vehicle_contract_policy.covariance_is_unknown(
            vehicle_contract_policy.assess_covariance(
                parsed.HasField("twist_covariance"), ()
            )
        )
    )

  def test_epoch_clear_is_a_prefix_of_the_golden(self):
    state = _fill_vehicle_state()
    full = state.SerializeToString()
    state.ClearField("estimator_epoch")
    self.assertTrue(full.startswith(state.SerializeToString()))
    self.assertNotEqual(full, state.SerializeToString())

  def test_present_zero_twist_is_not_absence(self):
    state = vehicle_state_pb2.VehicleState()
    state.body_twist.SetInParent()
    self.assertTrue(state.HasField("body_twist"))
    self.assertGreater(state.ByteSize(), 0)
    parsed = vehicle_state_pb2.VehicleState()
    parsed.ParseFromString(state.SerializeToString())
    self.assertTrue(parsed.HasField("body_twist"))
    self.assertEqual(parsed.body_twist.linear_x_m_s, 0)

  def test_zero_covariance_is_not_the_unknown_encoding(self):
    absent = vehicle_state_pb2.VehicleState()
    zeros = vehicle_state_pb2.VehicleState()
    zeros.pose_covariance.values.extend(
        [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
    )
    self.assertFalse(absent.HasField("pose_covariance"))
    self.assertTrue(zeros.HasField("pose_covariance"))
    self.assertNotEqual(absent.SerializeToString(), zeros.SerializeToString())
    self.assertTrue(
        vehicle_contract_policy.is_all_zero_covariance(
            zeros.pose_covariance.values
        )
    )

  def test_unknown_field_and_unknown_mode_are_kept(self):
    state = _fill_vehicle_state()
    with_unknown = state.SerializeToString() + bytes([0xA0, 0x06, 0x07])
    parsed = vehicle_state_pb2.VehicleState()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.estimator_epoch, 3)
    self.assertEqual(parsed.SerializeToString(), with_unknown)
    parsed.mode = 100
    reparsed = vehicle_state_pb2.VehicleState()
    reparsed.ParseFromString(parsed.SerializeToString())
    self.assertEqual(reparsed.mode, 100)
    self.assertIs(
        vehicle_contract_policy.classify_navigation_mode(reparsed.mode),
        vehicle_contract_policy.NavigationModeKind.UNKNOWN,
    )

  def test_desired_motion_and_wrench_goldens(self):
    motion = _fill_desired_motion_pose()
    self.assertEqual(
        motion.SerializeToString().hex(), _DESIRED_MOTION_GOLDEN_HEX
    )
    parsed = vehicle_command_pb2.DesiredMotion()
    parsed.ParseFromString(bytes.fromhex(motion.SerializeToString().hex()))
    self.assertEqual(parsed.WhichOneof("objective"), "pose")
    self.assertTrue(parsed.HasField("confidence"))
    self.assertEqual(parsed.confidence, 0.5)
    self.assertTrue(
        vehicle_contract_policy.assess_desired_motion(
            _motion_view(parsed)
        ).accepted
    )
    wrench = _fill_body_wrench()
    self.assertEqual(wrench.SerializeToString().hex(), _BODY_WRENCH_GOLDEN_HEX)
    parsed_wrench = vehicle_command_pb2.BodyWrench()
    parsed_wrench.ParseFromString(wrench.SerializeToString())
    self.assertEqual(
        parsed_wrench.header.frame_id, vehicle_contract_policy.BODY_FRAME_ID
    )
    self.assertTrue(
        vehicle_contract_policy.assess_body_wrench(
            _wrench_view(parsed_wrench)
        ).accepted
    )

  def test_oneof_keeps_the_last_objective(self):
    motion = vehicle_command_pb2.DesiredMotion()
    motion.pose.pose_world_from_body.orientation.w = 1
    motion.twist.body_twist.linear_x_m_s = 0.5
    self.assertEqual(motion.WhichOneof("objective"), "twist")
    parsed = vehicle_command_pb2.DesiredMotion()
    parsed.ParseFromString(motion.SerializeToString())
    self.assertEqual(parsed.WhichOneof("objective"), "twist")

  def test_zero_confidence_is_present(self):
    motion = vehicle_command_pb2.DesiredMotion()
    self.assertFalse(motion.HasField("confidence"))
    motion.confidence = 0
    self.assertTrue(motion.HasField("confidence"))
    with_zero = motion.SerializeToString()
    motion.ClearField("confidence")
    self.assertFalse(motion.HasField("confidence"))
    self.assertNotEqual(motion.SerializeToString(), with_zero)

  def test_stamp_uses_the_existing_time_policy(self):
    state = _fill_vehicle_state()
    age = stamped_header_policy.monotonic_age_seconds(
        (state.header.source_time.seconds, state.header.source_time.nanos),
        (state.header.receive_time.seconds, state.header.receive_time.nanos),
        state.header.clock_domain,
    )
    self.assertEqual(age, 0.75)
    self.assertIsNone(
        stamped_header_policy.monotonic_age_seconds(
            (state.header.source_time.seconds, state.header.source_time.nanos),
            (
                state.header.receive_time.seconds,
                state.header.receive_time.nanos,
            ),
            stamped_header_policy.CLOCK_DOMAIN_UTC,
        )
    )
    self.assertTrue(stamped_header_policy.sequence_advances(2, 3))
    self.assertFalse(stamped_header_policy.sequence_advances(3, 3))

  def test_textproto_examples_match_the_fixtures(self):
    parsed_state = vehicle_state_pb2.VehicleState()
    text_format.Parse(_load_example("vehicle_state.textproto"), parsed_state)
    self.assertEqual(
        parsed_state.SerializeToString(),
        _fill_vehicle_state().SerializeToString(),
    )
    unknown = vehicle_state_pb2.VehicleState()
    text_format.Parse(
        _load_example("vehicle_state_unknown_covariance.textproto"), unknown
    )
    self.assertFalse(unknown.HasField("pose_covariance"))
    self.assertFalse(unknown.HasField("twist_covariance"))
    self.assertTrue(
        vehicle_contract_policy.assess_vehicle_state(
            _state_view(unknown)
        ).accepted
    )
    parsed_pose = vehicle_command_pb2.DesiredMotion()
    text_format.Parse(
        _load_example("desired_motion_pose.textproto"), parsed_pose
    )
    self.assertEqual(
        parsed_pose.SerializeToString(),
        _fill_desired_motion_pose().SerializeToString(),
    )
    parsed_twist = vehicle_command_pb2.DesiredMotion()
    text_format.Parse(
        _load_example("desired_motion_twist.textproto"), parsed_twist
    )
    self.assertEqual(
        parsed_twist.SerializeToString(),
        _fill_desired_motion_twist().SerializeToString(),
    )
    parsed_wrench = vehicle_command_pb2.BodyWrench()
    text_format.Parse(_load_example("body_wrench.textproto"), parsed_wrench)
    self.assertEqual(
        parsed_wrench.SerializeToString(),
        _fill_body_wrench().SerializeToString(),
    )


if __name__ == "__main__":
  unittest.main()
