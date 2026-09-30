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

"""Proto wrappers for the SafetyDecision host assessment.

Policy: intrinsic_apis/intrinsic/safety/proto/README.md.

These helpers read a decision message once and never modify it. Unknown enum
numbers are passed to the plain-value policy as raw ints, so they are
classified as unknown and are not rewritten.
"""

import hashlib

from intrinsic.safety import safety_decision_policy
from intrinsic.safety.proto import safety_decision_pb2
from intrinsic.vehicle import vehicle_contract_policy
from intrinsic.vehicle.proto import vehicle_command_pb2

_VEHICLE = vehicle_contract_policy


def desired_motion_view_from_proto(
    motion: vehicle_command_pb2.DesiredMotion,
) -> _VEHICLE.DesiredMotionView:
  """Maps a DesiredMotion onto the plain view used by the vehicle policy."""
  objective = _VEHICLE.ObjectiveKind.ABSENT
  position = (0.0, 0.0, 0.0)
  orientation = (0.0, 0.0, 0.0, 1.0)
  twist = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  trajectory_id = ""
  which = motion.WhichOneof("objective")
  if which == "pose":
    objective = _VEHICLE.ObjectiveKind.POSE
    pose = motion.pose.pose_world_from_body
    position = (pose.position.x, pose.position.y, pose.position.z)
    orientation = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
  elif which == "twist":
    objective = _VEHICLE.ObjectiveKind.TWIST
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
    objective = _VEHICLE.ObjectiveKind.TRAJECTORY
    trajectory_id = motion.trajectory.trajectory_id
  return _VEHICLE.DesiredMotionView(
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


def safety_decision_view_from_proto(
    decision: safety_decision_pb2.SafetyDecision,
) -> safety_decision_policy.SafetyDecisionView:
  """Maps a decision onto the plain view. Enum numbers stay raw ints."""
  header = decision.header
  applied_intent = _VEHICLE.DesiredMotionView()
  if decision.HasField("applied_intent"):
    applied_intent = desired_motion_view_from_proto(decision.applied_intent)
  return safety_decision_policy.SafetyDecisionView(
      header_present=decision.HasField("header"),
      header_validity_present=header.HasField("validity"),
      header_validity_state=header.validity.state,
      header_source_time_present=header.HasField("source_time"),
      header_source_time_nanos=header.source_time.nanos,
      header_receive_time_present=header.HasField("receive_time"),
      header_receive_time_nanos=header.receive_time.nanos,
      kind=int(decision.kind),
      original_intent_digest=decision.original_intent_digest,
      applied_intent_present=decision.HasField("applied_intent"),
      applied_intent=applied_intent,
      snapshot_id=decision.snapshot_id,
      findings=tuple(
          safety_decision_policy.SafetyFindingView(
              rule_id=finding.rule_id, severity=int(finding.severity)
          )
          for finding in decision.findings
      ),
      decision_epoch_present=decision.HasField("decision_epoch"),
      decision_epoch=decision.decision_epoch,
  )


def assess_safety_decision_proto(
    decision: safety_decision_pb2.SafetyDecision,
) -> safety_decision_policy.SafetyDecisionAssessment:
  """Structural assessment of a decision message.

  An empty message is not engaged, not an error, and not accepted. An
  unknown kind is never accepted and is not rewritten.
  """
  return safety_decision_policy.assess_safety_decision(
      safety_decision_view_from_proto(decision)
  )


def compute_original_intent_digest(
    motion: vehicle_command_pb2.DesiredMotion,
) -> str:
  """Lowercase hex SHA-256 of the deterministic serialization of motion."""
  return hashlib.sha256(
      motion.SerializeToString(deterministic=True)
  ).hexdigest()
