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

"""E9: the EpisodeEvent schema freeze.

Every tag, type, name, and oneof membership is permanent. A change fails this
test and needs senior review. Keep in sync with
episode_event_schema_freeze_test.cc.
"""

import os
import unittest

from google.protobuf import descriptor
from google.protobuf import descriptor_pb2
from intrinsic.episode.proto import episode_event_pb2

_FREEZE_RELATIVE = (
    "intrinsic_data/episode/testdata/episode_event_schema_freeze.txt"
)

_TYPE_NAMES = {
    descriptor.FieldDescriptor.TYPE_DOUBLE: "double",
    descriptor.FieldDescriptor.TYPE_FLOAT: "float",
    descriptor.FieldDescriptor.TYPE_INT64: "int64",
    descriptor.FieldDescriptor.TYPE_UINT64: "uint64",
    descriptor.FieldDescriptor.TYPE_INT32: "int32",
    descriptor.FieldDescriptor.TYPE_FIXED64: "fixed64",
    descriptor.FieldDescriptor.TYPE_FIXED32: "fixed32",
    descriptor.FieldDescriptor.TYPE_BOOL: "bool",
    descriptor.FieldDescriptor.TYPE_STRING: "string",
    descriptor.FieldDescriptor.TYPE_GROUP: "group",
    descriptor.FieldDescriptor.TYPE_MESSAGE: "message",
    descriptor.FieldDescriptor.TYPE_BYTES: "bytes",
    descriptor.FieldDescriptor.TYPE_UINT32: "uint32",
    descriptor.FieldDescriptor.TYPE_ENUM: "enum",
    descriptor.FieldDescriptor.TYPE_SFIXED32: "sfixed32",
    descriptor.FieldDescriptor.TYPE_SFIXED64: "sfixed64",
    descriptor.FieldDescriptor.TYPE_SINT32: "sint32",
    descriptor.FieldDescriptor.TYPE_SINT64: "sint64",
}


def _freeze_lines():
  root = os.environ.get("TEST_SRCDIR", "")
  workspace = os.environ.get("TEST_WORKSPACE", "")
  for prefix in (os.path.join(root, workspace), os.path.join(root, "_main")):
    path = os.path.join(prefix, _FREEZE_RELATIVE)
    if os.path.isfile(path):
      with open(path, encoding="utf-8") as handle:
        return [line for line in handle.read().splitlines() if line]
  raise AssertionError("missing %s under %s" % (_FREEZE_RELATIVE, root))


def _real_oneof_name(message, field):
  """Like C++ real_containing_oneof: proto3 optional synthetic oneofs excluded."""
  if field.containing_oneof is None:
    return None
  proto = descriptor_pb2.DescriptorProto()
  message.CopyToProto(proto)
  field_proto = next(f for f in proto.field if f.name == field.name)
  if field_proto.proto3_optional:
    return None
  return field.containing_oneof.name


def _presence_label(message, field):
  """repeated, oneof:<name>, explicit, or implicit, as in the C++ test."""
  if field.is_repeated:
    return "repeated"
  oneof_name = _real_oneof_name(message, field)
  if oneof_name is not None:
    return "oneof:" + oneof_name
  return "explicit" if field.has_presence else "implicit"


def _field_line(message, field):
  type_name = _TYPE_NAMES[field.type]
  if field.type == field.TYPE_MESSAGE:
    type_name += ":" + field.message_type.full_name
  label = _presence_label(message, field)
  return "%s.%s %d %s %s" % (
      message.name,
      field.name,
      field.number,
      type_name,
      label,
  )


def _descriptor_lines():
  file_descriptor = episode_event_pb2.DESCRIPTOR
  lines = []
  for message in file_descriptor.message_types_by_name.values():
    for field in message.fields:
      lines.append(_field_line(message, field))
  return sorted(lines)


class EpisodeEventSchemaFreezeTest(unittest.TestCase):

  def test_descriptor_equals_the_freeze_file(self):
    frozen = _freeze_lines()
    self.assertTrue(frozen)
    self.assertEqual(_descriptor_lines(), frozen)

  def test_freeze_file_is_sorted_and_unique(self):
    frozen = _freeze_lines()
    self.assertEqual(frozen, sorted(frozen))
    self.assertEqual(len(set(frozen)), len(frozen))

  def test_package_and_messages_are_frozen(self):
    file_descriptor = episode_event_pb2.DESCRIPTOR
    self.assertEqual(file_descriptor.package, "intrinsic_proto.episode")
    self.assertEqual(
        set(file_descriptor.message_types_by_name),
        {"EpisodeEvent", "EpisodeLoss", "EpisodeMetadata"},
    )
    self.assertEqual(len(file_descriptor.enum_types_by_name), 0)

  def test_loss_reason_enum_is_frozen(self):
    loss = episode_event_pb2.DESCRIPTOR.message_types_by_name["EpisodeLoss"]
    reason = loss.enum_types_by_name["LossReason"]
    self.assertEqual(
        {
            loss.full_name + "." + value.name: value.number
            for value in reason.values
        },
        {
            "intrinsic_proto.episode.EpisodeLoss.LOSS_REASON_UNSPECIFIED": 0,
            "intrinsic_proto.episode.EpisodeLoss.LOSS_REASON_OVERFLOW": 1,
            "intrinsic_proto.episode.EpisodeLoss.LOSS_REASON_TOO_LARGE": 2,
        },
    )

  def test_no_field_is_declared_reserved(self):
    for message in episode_event_pb2.DESCRIPTOR.message_types_by_name.values():
      with self.subTest(message=message.name):
        proto = descriptor_pb2.DescriptorProto()
        message.CopyToProto(proto)
        self.assertEqual(len(proto.reserved_range), 0)
        self.assertEqual(len(proto.reserved_name), 0)


if __name__ == "__main__":
  unittest.main()
