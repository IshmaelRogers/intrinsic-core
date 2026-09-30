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

"""Python serialization tests for SemanticContactsComponent."""

from dataclasses import dataclass
from dataclasses import field
import unittest

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.world.marine_component_validity import marine_component_validity_policy
from intrinsic.world.proto import semantic_contacts_component_pb2
from intrinsic.world.semantic_contacts_component import semantic_contacts_component_policy

# Canonical serialization of _fill_fixture(). Keep in sync with
# semantic_contacts_component_serialization_test.cc.
# Contact 0 is buoy_1 with every field set. Contact 1 is dock_2 with an
# explicit zero velocity and age (present, all zero) and no confidence.
_NO_CONTACTS_HEX = (
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75"
)
_CONTACT_0_HEX = (
    "1a6c0a0662756f795f3112280a1b090000000000002840110000000000000cc0190000"
    "00000000f4bf120921000000000000f03f1a1f0a12099a9999999999b93f199a999999"
    "9999a9bf1209197b14ae47e17a943f220462756f7929cdccccccccccec3f3208080410"
    "80cab5ee01"
)
_CONTACT_1_HEX = (
    "1a330a06646f636b5f32121f0a1209000000000000444011000000000000"
    "2040120919000000000000f03f1a002204646f636b3200"
)
_GOLDEN_HEX = _NO_CONTACTS_HEX + _CONTACT_0_HEX + _CONTACT_1_HEX
_POLICY = semantic_contacts_component_policy
_VALIDITY = marine_component_validity_policy
_PB2 = semantic_contacts_component_pb2
_QUERY = (1700000010, 750000000)


@dataclass
class _ContactsPlain:
  """Test-only plain value. Unknown fields are not copied."""

  present: bool = False
  validity_message_present: bool = False
  source_id: str = ""
  observation_time_present: bool = False
  observation_time: tuple[int, int] = (0, 0)
  validity_horizon_present: bool = False
  validity_horizon: tuple[int, int] = (0, 0)
  confidence_present: bool = False
  confidence: float = 0.0
  uncertainty_reference: str = ""
  embodiment_validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  contacts: list[_POLICY.SemanticContactView] = field(default_factory=list)


def _fill_fixture():
  message = _PB2.SemanticContactsComponent()
  message.validity.source_id = "fusion_0"
  message.validity.observation_time.seconds = 1700000000
  message.validity.observation_time.nanos = 250000000
  message.validity.validity_horizon.seconds = 10
  message.validity.validity_horizon.nanos = 500000000
  message.validity.confidence = 0.75
  message.validity.uncertainty_reference = "cov-ref-7"
  message.validity.validity.state = stamped_header_pb2.Validity.STATE_VALID
  message.frame_id = "world_enu"

  buoy = message.contacts.add()
  buoy.contact_id = "buoy_1"
  buoy.pose.position.x = 12.0
  buoy.pose.position.y = -3.5
  buoy.pose.position.z = -1.25
  buoy.pose.orientation.w = 1.0
  buoy.velocity.linear.x = 0.1
  buoy.velocity.linear.z = -0.05
  buoy.velocity.angular.z = 0.02
  buoy.classification = "buoy"
  buoy.confidence = 0.9
  buoy.age.seconds = 4
  buoy.age.nanos = 500000000

  dock = message.contacts.add()
  dock.contact_id = "dock_2"
  dock.pose.position.x = 40.0
  dock.pose.position.y = 8.0
  dock.pose.orientation.z = 1.0
  dock.velocity.SetInParent()
  dock.classification = "dock"
  dock.age.SetInParent()
  return message


def _contact_from_proto(contact):
  pose = contact.pose
  velocity = contact.velocity
  return _POLICY.SemanticContactView(
      contact_id=contact.contact_id,
      pose_present=contact.HasField("pose"),
      position=(pose.position.x, pose.position.y, pose.position.z),
      orientation=(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w,
      ),
      velocity_present=contact.HasField("velocity"),
      linear_velocity_m_s=(
          velocity.linear.x,
          velocity.linear.y,
          velocity.linear.z,
      ),
      angular_velocity_rad_s=(
          velocity.angular.x,
          velocity.angular.y,
          velocity.angular.z,
      ),
      classification=contact.classification,
      confidence_present=contact.HasField("confidence"),
      confidence=contact.confidence,
      age_present=contact.HasField("age"),
      age=(contact.age.seconds, contact.age.nanos),
  )


def _plain_from_proto(message, present):
  validity = message.validity
  return _ContactsPlain(
      present=present,
      validity_message_present=message.HasField("validity"),
      source_id=validity.source_id,
      observation_time_present=validity.HasField("observation_time"),
      observation_time=(
          validity.observation_time.seconds,
          validity.observation_time.nanos,
      ),
      validity_horizon_present=validity.HasField("validity_horizon"),
      validity_horizon=(
          validity.validity_horizon.seconds,
          validity.validity_horizon.nanos,
      ),
      confidence_present=validity.HasField("confidence"),
      confidence=validity.confidence,
      uncertainty_reference=validity.uncertainty_reference,
      embodiment_validity_present=validity.HasField("validity"),
      validity_state=validity.validity.state,
      frame_id=message.frame_id,
      contacts=[_contact_from_proto(c) for c in message.contacts],
  )


def _set_nonzero(target, names, values):
  # Zero components are left unset so that empty nested messages are not
  # created. The wire image is identical because proto3 omits zero scalars.
  for name, value in zip(names, values):
    if value != 0.0:
      setattr(target, name, value)


def _apply_contact(view, contact):
  contact.contact_id = view.contact_id
  if view.pose_present:
    contact.pose.SetInParent()
    _set_nonzero(contact.pose.position, "xyz", view.position)
    _set_nonzero(contact.pose.orientation, "xyzw", view.orientation)
  if view.velocity_present:
    contact.velocity.SetInParent()
    _set_nonzero(contact.velocity.linear, "xyz", view.linear_velocity_m_s)
    _set_nonzero(contact.velocity.angular, "xyz", view.angular_velocity_rad_s)
  contact.classification = view.classification
  if view.confidence_present:
    contact.confidence = view.confidence
  if view.age_present:
    contact.age.SetInParent()
    _set_nonzero(contact.age, ("seconds", "nanos"), view.age)


def _apply_plain(plain, message):
  message.Clear()
  if plain.validity_message_present:
    validity = message.validity
    validity.source_id = plain.source_id
    if plain.observation_time_present:
      validity.observation_time.seconds = plain.observation_time[0]
      validity.observation_time.nanos = plain.observation_time[1]
    if plain.validity_horizon_present:
      validity.validity_horizon.seconds = plain.validity_horizon[0]
      validity.validity_horizon.nanos = plain.validity_horizon[1]
    if plain.confidence_present:
      validity.confidence = plain.confidence
    validity.uncertainty_reference = plain.uncertainty_reference
    if plain.embodiment_validity_present:
      validity.validity.state = plain.validity_state
  message.frame_id = plain.frame_id
  for view in plain.contacts:
    _apply_contact(view, message.contacts.add())


def _view_from_plain(plain):
  return _POLICY.SemanticContactsView(
      present=plain.present,
      validity=_VALIDITY.MarineComponentValidityView(
          present=plain.validity_message_present,
          source_id=plain.source_id,
          observation_time_present=plain.observation_time_present,
          observation_time=plain.observation_time,
          validity_horizon_present=plain.validity_horizon_present,
          validity_horizon=plain.validity_horizon,
          confidence_present=plain.confidence_present,
          confidence=plain.confidence,
          uncertainty_reference=plain.uncertainty_reference,
          validity_present=plain.embodiment_validity_present,
          validity_state=plain.validity_state,
      ),
      frame_id=plain.frame_id,
      contacts=tuple(plain.contacts),
  )


def _assess_message(message):
  plain = _plain_from_proto(message, True)
  return _POLICY.assess_semantic_contacts(_view_from_plain(plain), _QUERY)


class SemanticContactsSerializationTest(unittest.TestCase):

  def test_schema_has_no_perception_fields(self):
    descriptor = _PB2.SemanticContactsComponent.DESCRIPTOR
    self.assertEqual(
        [(f.number, f.name) for f in descriptor.fields],
        [(1, "validity"), (2, "frame_id"), (3, "contacts")],
    )
    contact = descriptor.fields_by_name["contacts"].message_type
    self.assertEqual(contact.name, "SemanticContact")
    self.assertEqual(contact.containing_type, descriptor)
    self.assertEqual(
        [(f.number, f.name) for f in contact.fields],
        [
            (1, "contact_id"),
            (2, "pose"),
            (3, "velocity"),
            (4, "classification"),
            (5, "confidence"),
            (6, "age"),
        ],
    )
    for name in ("track", "detections", "embedding", "image", "points"):
      self.assertIsNone(descriptor.fields_by_name.get(name))
      self.assertIsNone(contact.fields_by_name.get(name))

  def test_golden_round_trip(self):
    message = _fill_fixture()
    golden = bytes.fromhex(_GOLDEN_HEX)
    self.assertEqual(message.SerializeToString(), golden)
    parsed = _PB2.SemanticContactsComponent()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.frame_id, "world_enu")
    self.assertEqual(parsed.validity.source_id, "fusion_0")
    self.assertEqual(
        parsed.validity.validity.state, stamped_header_pb2.Validity.STATE_VALID
    )
    self.assertEqual(
        [c.contact_id for c in parsed.contacts], ["buoy_1", "dock_2"]
    )
    buoy, dock = parsed.contacts
    self.assertEqual(buoy.pose.position.x, 12.0)
    self.assertEqual(buoy.pose.position.y, -3.5)
    self.assertEqual(buoy.pose.position.z, -1.25)
    self.assertEqual(buoy.pose.orientation.w, 1.0)
    self.assertEqual(buoy.velocity.linear.x, 0.1)
    self.assertEqual(buoy.velocity.linear.z, -0.05)
    self.assertEqual(buoy.velocity.angular.z, 0.02)
    self.assertEqual(buoy.classification, "buoy")
    self.assertTrue(buoy.HasField("confidence"))
    self.assertEqual(buoy.confidence, 0.9)
    self.assertEqual((buoy.age.seconds, buoy.age.nanos), (4, 500000000))
    self.assertEqual(dock.pose.orientation.z, 1.0)
    self.assertTrue(dock.HasField("velocity"))
    self.assertTrue(dock.HasField("age"))
    self.assertFalse(dock.HasField("confidence"))
    self.assertEqual(parsed.SerializeToString(), golden)

    plain = _plain_from_proto(parsed, True)
    again = _PB2.SemanticContactsComponent()
    _apply_plain(plain, again)
    self.assertEqual(again.SerializeToString(), golden)
    assessment = _POLICY.assess_semantic_contacts(
        _view_from_plain(plain), _QUERY
    )
    self.assertIs(assessment.error, _POLICY.SemanticContactsError.NONE)
    self.assertEqual(assessment.contact_index, -1)
    self.assertTrue(assessment.accepted)

  def test_contact_order_is_preserved(self):
    swapped = _PB2.SemanticContactsComponent()
    swapped.CopyFrom(_fill_fixture())
    first = _PB2.SemanticContactsComponent.SemanticContact()
    first.CopyFrom(swapped.contacts[0])
    del swapped.contacts[0]
    swapped.contacts.append(first)
    self.assertEqual(
        [c.contact_id for c in swapped.contacts], ["dock_2", "buoy_1"]
    )
    parsed = _PB2.SemanticContactsComponent()
    parsed.ParseFromString(swapped.SerializeToString())
    self.assertEqual(
        [c.contact_id for c in parsed.contacts], ["dock_2", "buoy_1"]
    )
    self.assertNotEqual(swapped.SerializeToString(), bytes.fromhex(_GOLDEN_HEX))

  def test_empty_contact_list_is_a_clear_set(self):
    message = _fill_fixture()
    del message.contacts[:]
    prefix = bytes.fromhex(_NO_CONTACTS_HEX)
    self.assertEqual(message.SerializeToString(), prefix)
    self.assertTrue(bytes.fromhex(_GOLDEN_HEX).startswith(prefix))
    assessment = _assess_message(message)
    self.assertIs(assessment.error, _POLICY.SemanticContactsError.NONE)
    self.assertTrue(assessment.accepted)

  def test_absent_submessages_are_omitted_and_rejected(self):
    cases = (
        ("pose", _POLICY.SemanticContactError.POSE),
        ("velocity", _POLICY.SemanticContactError.VELOCITY),
        ("age", _POLICY.SemanticContactError.AGE),
    )
    golden = bytes.fromhex(_GOLDEN_HEX)
    for name, expected in cases:
      message = _fill_fixture()
      message.contacts[1].ClearField(name)
      self.assertFalse(message.contacts[1].HasField(name))
      self.assertLess(len(message.SerializeToString()), len(golden), name)
      plain = _plain_from_proto(message, True)
      assessment = _POLICY.assess_semantic_contacts(
          _view_from_plain(plain), _QUERY
      )
      self.assertIs(assessment.contact_error, expected, name)
      self.assertEqual(assessment.contact_index, 1, name)
      self.assertFalse(assessment.accepted, name)

  def test_explicit_zero_velocity_and_age_are_present(self):
    message = _fill_fixture()
    dock = message.contacts[1]
    self.assertTrue(dock.HasField("velocity"))
    self.assertTrue(dock.HasField("age"))
    plain = _plain_from_proto(message, True)
    self.assertTrue(plain.contacts[1].velocity_present)
    self.assertEqual(plain.contacts[1].linear_velocity_m_s, (0.0, 0.0, 0.0))
    self.assertEqual(plain.contacts[1].angular_velocity_rad_s, (0.0, 0.0, 0.0))
    self.assertTrue(plain.contacts[1].age_present)
    self.assertEqual(plain.contacts[1].age, (0, 0))
    self.assertTrue(_assess_message(message).accepted)
    again = _PB2.SemanticContactsComponent()
    _apply_plain(plain, again)
    self.assertEqual(again.SerializeToString(), message.SerializeToString())

  def test_confidence_zero_is_distinct_from_unset(self):
    unset = _fill_fixture()
    self.assertFalse(unset.contacts[1].HasField("confidence"))
    zero = _PB2.SemanticContactsComponent()
    zero.CopyFrom(unset)
    zero.contacts[1].confidence = 0.0
    self.assertTrue(zero.contacts[1].HasField("confidence"))
    self.assertNotEqual(unset.SerializeToString(), zero.SerializeToString())
    plain = _plain_from_proto(zero, True)
    self.assertTrue(plain.contacts[1].confidence_present)
    self.assertEqual(plain.contacts[1].confidence, 0.0)
    self.assertTrue(_assess_message(zero).accepted)
    self.assertFalse(
        _plain_from_proto(unset, True).contacts[1].confidence_present
    )

  def test_confidence_boundaries_survive_the_wire(self):
    for confidence, accepted in (
        (0.0, True),
        (1.0, True),
        (-1e-12, False),
        (1.0000000000000002, False),
    ):
      message = _fill_fixture()
      message.contacts[0].confidence = confidence
      parsed = _PB2.SemanticContactsComponent()
      parsed.ParseFromString(message.SerializeToString())
      assessment = _assess_message(parsed)
      self.assertEqual(assessment.accepted, accepted, confidence)
      if not accepted:
        self.assertIs(
            assessment.contact_error, _POLICY.SemanticContactError.CONFIDENCE
        )

  def test_negative_age_survives_the_wire_and_is_rejected(self):
    message = _fill_fixture()
    message.contacts[0].age.seconds = -1
    message.contacts[0].age.nanos = 0
    parsed = _PB2.SemanticContactsComponent()
    parsed.ParseFromString(message.SerializeToString())
    self.assertEqual(parsed.contacts[0].age.seconds, -1)
    assessment = _assess_message(parsed)
    self.assertIs(assessment.contact_error, _POLICY.SemanticContactError.AGE)
    self.assertEqual(assessment.contact_index, 0)

  def test_non_finite_geometry_survives_the_wire_and_is_rejected(self):
    message = _fill_fixture()
    message.contacts[1].pose.position.z = float("nan")
    parsed = _PB2.SemanticContactsComponent()
    parsed.ParseFromString(message.SerializeToString())
    assessment = _assess_message(parsed)
    self.assertIs(assessment.contact_error, _POLICY.SemanticContactError.POSE)
    self.assertEqual(assessment.contact_index, 1)

    message = _fill_fixture()
    message.contacts[0].velocity.angular.y = float("inf")
    parsed = _PB2.SemanticContactsComponent()
    parsed.ParseFromString(message.SerializeToString())
    assessment = _assess_message(parsed)
    self.assertIs(
        assessment.contact_error, _POLICY.SemanticContactError.VELOCITY
    )
    self.assertEqual(assessment.contact_index, 0)

  def test_duplicate_contact_ids_survive_the_wire_and_are_rejected(self):
    message = _fill_fixture()
    message.contacts[1].contact_id = "buoy_1"
    parsed = _PB2.SemanticContactsComponent()
    parsed.ParseFromString(message.SerializeToString())
    self.assertEqual([c.contact_id for c in parsed.contacts], ["buoy_1"] * 2)
    assessment = _assess_message(parsed)
    self.assertIs(
        assessment.contact_error,
        _POLICY.SemanticContactError.DUPLICATE_CONTACT_ID,
    )
    self.assertEqual(assessment.contact_index, 1)

  def test_default_message_is_empty(self):
    message = _PB2.SemanticContactsComponent()
    self.assertEqual(message.SerializeToString(), b"")
    self.assertFalse(message.HasField("validity"))
    self.assertEqual(len(message.contacts), 0)
    parsed = _PB2.SemanticContactsComponent()
    parsed.ParseFromString(b"")
    plain = _plain_from_proto(parsed, False)
    again = _PB2.SemanticContactsComponent()
    _apply_plain(plain, again)
    self.assertEqual(again.SerializeToString(), b"")
    empty = _POLICY.assess_semantic_contacts(_view_from_plain(plain), (0, 0))
    self.assertIs(empty.error, _POLICY.SemanticContactsError.NONE)
    self.assertFalse(empty.accepted)
    present = _POLICY.assess_semantic_contacts(
        _view_from_plain(_ContactsPlain(**{**plain.__dict__, "present": True})),
        (0, 0),
    )
    self.assertIs(present.error, _POLICY.SemanticContactsError.VALIDITY)

  def test_unknown_fields_are_preserved(self):
    golden = bytes.fromhex(_GOLDEN_HEX)
    with_unknown = golden + bytes((0xA0, 0x06, 0x07))
    parsed = _PB2.SemanticContactsComponent()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.frame_id, "world_enu")
    self.assertEqual(len(parsed.contacts), 2)
    self.assertEqual(parsed.SerializeToString(), with_unknown)

    # Unknown varint field 100 appended inside the dock_2 contact.
    dock = bytes.fromhex(_CONTACT_1_HEX)
    extra = bytes((0xA0, 0x06, 0x07))
    grown = bytes((dock[0], dock[1] + len(extra))) + dock[2:] + extra
    nested_raw = bytes.fromhex(_NO_CONTACTS_HEX + _CONTACT_0_HEX) + grown
    nested = _PB2.SemanticContactsComponent()
    nested.ParseFromString(nested_raw)
    self.assertEqual(nested.contacts[1].contact_id, "dock_2")
    self.assertEqual(nested.SerializeToString(), nested_raw)
    self.assertTrue(_assess_message(nested).accepted)

  def test_unknown_validity_enum_round_trips(self):
    raw = bytes.fromhex("0a0432020863")
    parsed = _PB2.SemanticContactsComponent()
    parsed.ParseFromString(raw)
    self.assertTrue(parsed.HasField("validity"))
    self.assertEqual(parsed.validity.validity.state, 99)
    self.assertEqual(parsed.SerializeToString(), raw)
    plain = _plain_from_proto(parsed, True)
    self.assertIs(
        stamped_header_policy.classify_validity(True, plain.validity_state),
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )
    nested_raw = bytes.fromhex("0a06320408011805")
    nested = _PB2.SemanticContactsComponent()
    nested.ParseFromString(nested_raw)
    self.assertEqual(
        nested.validity.validity.state, stamped_header_pb2.Validity.STATE_VALID
    )
    self.assertEqual(nested.SerializeToString(), nested_raw)


if __name__ == "__main__":
  unittest.main()
