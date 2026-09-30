// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/unknown_field_set.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/world/proto/semantic_contacts_component.pb.h"
#include "intrinsic/world/semantic_contacts_component/semantic_contacts_component_policy.h"

namespace intrinsic::world {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::world::SemanticContactsComponent;
using Contact = SemanticContactsComponent::SemanticContact;

// Canonical serialization of FillFixture(). Keep in sync with
// semantic_contacts_component_serialization_test.py.
// Contact 0 is buoy_1 with every field set. Contact 1 is dock_2 with an
// explicit zero velocity and age (present, all zero) and no confidence.
constexpr std::string_view kNoContactsHex =
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75";
constexpr std::string_view kContact0Hex =
    "1a6c0a0662756f795f3112280a1b090000000000002840110000000000000cc0190000"
    "00000000f4bf120921000000000000f03f1a1f0a12099a9999999999b93f199a999999"
    "9999a9bf1209197b14ae47e17a943f220462756f7929cdccccccccccec3f3208080410"
    "80cab5ee01";
constexpr std::string_view kContact1Hex =
    "1a330a06646f636b5f32121f0a1209000000000000444011000000000000"
    "2040120919000000000000f03f1a002204646f636b3200";

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

TimeParts Query() { return TimeParts{1700000010, 750000000}; }

// Test-only plain contact. Unknown fields are not copied.
struct ContactPlain {
  std::string contact_id;
  bool pose_present = false;
  embodiment::Vec3 position;
  embodiment::Quaternion orientation;
  bool velocity_present = false;
  embodiment::Vec3 linear_velocity_m_s;
  embodiment::Vec3 angular_velocity_rad_s;
  std::string classification;
  bool confidence_present = false;
  double confidence = 0;
  bool age_present = false;
  TimeParts age;
};

// Test-only plain value. Unknown fields are not copied.
struct ContactsPlain {
  bool present = false;
  bool validity_message_present = false;
  std::string source_id;
  bool observation_time_present = false;
  TimeParts observation_time;
  bool validity_horizon_present = false;
  TimeParts validity_horizon;
  bool confidence_present = false;
  double confidence = 0;
  std::string uncertainty_reference;
  bool embodiment_validity_present = false;
  int validity_state = 0;
  std::string frame_id;
  std::vector<ContactPlain> contacts;
};

std::string FromHex(std::string_view hex) {
  std::string out;
  out.reserve(hex.size() / 2);
  auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    return c - 'a' + 10;
  };
  for (size_t i = 0; i < hex.size(); i += 2) {
    out.push_back(
        static_cast<char>((nibble(hex[i]) << 4) | nibble(hex[i + 1])));
  }
  return out;
}

std::string GoldenBytes() {
  return FromHex(kNoContactsHex) + FromHex(kContact0Hex) +
         FromHex(kContact1Hex);
}

void FillFixture(SemanticContactsComponent *message) {
  auto *validity = message->mutable_validity();
  validity->set_source_id("fusion_0");
  validity->mutable_observation_time()->set_seconds(1700000000);
  validity->mutable_observation_time()->set_nanos(250000000);
  validity->mutable_validity_horizon()->set_seconds(10);
  validity->mutable_validity_horizon()->set_nanos(500000000);
  validity->set_confidence(0.75);
  validity->set_uncertainty_reference("cov-ref-7");
  validity->mutable_validity()->set_state(Validity::STATE_VALID);
  message->set_frame_id("world_enu");

  Contact *buoy = message->add_contacts();
  buoy->set_contact_id("buoy_1");
  buoy->mutable_pose()->mutable_position()->set_x(12.0);
  buoy->mutable_pose()->mutable_position()->set_y(-3.5);
  buoy->mutable_pose()->mutable_position()->set_z(-1.25);
  buoy->mutable_pose()->mutable_orientation()->set_w(1.0);
  buoy->mutable_velocity()->mutable_linear()->set_x(0.1);
  buoy->mutable_velocity()->mutable_linear()->set_z(-0.05);
  buoy->mutable_velocity()->mutable_angular()->set_z(0.02);
  buoy->set_classification("buoy");
  buoy->set_confidence(0.9);
  buoy->mutable_age()->set_seconds(4);
  buoy->mutable_age()->set_nanos(500000000);

  Contact *dock = message->add_contacts();
  dock->set_contact_id("dock_2");
  dock->mutable_pose()->mutable_position()->set_x(40.0);
  dock->mutable_pose()->mutable_position()->set_y(8.0);
  dock->mutable_pose()->mutable_orientation()->set_z(1.0);
  dock->mutable_velocity();
  dock->set_classification("dock");
  dock->mutable_age();
}

ContactPlain ContactFromProto(const Contact &contact) {
  ContactPlain plain;
  plain.contact_id = contact.contact_id();
  plain.pose_present = contact.has_pose();
  plain.position = embodiment::Vec3{contact.pose().position().x(),
                                    contact.pose().position().y(),
                                    contact.pose().position().z()};
  plain.orientation = embodiment::Quaternion{
      contact.pose().orientation().x(), contact.pose().orientation().y(),
      contact.pose().orientation().z(), contact.pose().orientation().w()};
  plain.velocity_present = contact.has_velocity();
  plain.linear_velocity_m_s = embodiment::Vec3{contact.velocity().linear().x(),
                                               contact.velocity().linear().y(),
                                               contact.velocity().linear().z()};
  plain.angular_velocity_rad_s = embodiment::Vec3{
      contact.velocity().angular().x(), contact.velocity().angular().y(),
      contact.velocity().angular().z()};
  plain.classification = contact.classification();
  plain.confidence_present = contact.has_confidence();
  plain.confidence = contact.confidence();
  plain.age_present = contact.has_age();
  plain.age = TimeParts{contact.age().seconds(), contact.age().nanos()};
  return plain;
}

ContactsPlain PlainFromProto(const SemanticContactsComponent &message,
                             bool present) {
  ContactsPlain plain;
  plain.present = present;
  plain.validity_message_present = message.has_validity();
  plain.source_id = message.validity().source_id();
  plain.observation_time_present = message.validity().has_observation_time();
  plain.observation_time =
      TimeParts{message.validity().observation_time().seconds(),
                message.validity().observation_time().nanos()};
  plain.validity_horizon_present = message.validity().has_validity_horizon();
  plain.validity_horizon =
      TimeParts{message.validity().validity_horizon().seconds(),
                message.validity().validity_horizon().nanos()};
  plain.confidence_present = message.validity().has_confidence();
  plain.confidence = message.validity().confidence();
  plain.uncertainty_reference = message.validity().uncertainty_reference();
  plain.embodiment_validity_present = message.validity().has_validity();
  plain.validity_state =
      static_cast<int>(message.validity().validity().state());
  plain.frame_id = message.frame_id();
  for (const Contact &contact : message.contacts()) {
    plain.contacts.push_back(ContactFromProto(contact));
  }
  return plain;
}

// Zero components are left unset so that empty nested messages are not
// created. The wire image is identical because proto3 omits zero scalars.
void ApplyContact(const ContactPlain &plain, Contact *contact) {
  contact->set_contact_id(plain.contact_id);
  if (plain.pose_present) {
    auto *pose = contact->mutable_pose();
    if (plain.position.x != 0)
      pose->mutable_position()->set_x(plain.position.x);
    if (plain.position.y != 0)
      pose->mutable_position()->set_y(plain.position.y);
    if (plain.position.z != 0)
      pose->mutable_position()->set_z(plain.position.z);
    auto &q = plain.orientation;
    if (q.x != 0) pose->mutable_orientation()->set_x(q.x);
    if (q.y != 0) pose->mutable_orientation()->set_y(q.y);
    if (q.z != 0) pose->mutable_orientation()->set_z(q.z);
    if (q.w != 0) pose->mutable_orientation()->set_w(q.w);
  }
  if (plain.velocity_present) {
    auto *velocity = contact->mutable_velocity();
    auto &l = plain.linear_velocity_m_s;
    if (l.x != 0) velocity->mutable_linear()->set_x(l.x);
    if (l.y != 0) velocity->mutable_linear()->set_y(l.y);
    if (l.z != 0) velocity->mutable_linear()->set_z(l.z);
    auto &a = plain.angular_velocity_rad_s;
    if (a.x != 0) velocity->mutable_angular()->set_x(a.x);
    if (a.y != 0) velocity->mutable_angular()->set_y(a.y);
    if (a.z != 0) velocity->mutable_angular()->set_z(a.z);
  }
  contact->set_classification(plain.classification);
  if (plain.confidence_present) {
    contact->set_confidence(plain.confidence);
  }
  if (plain.age_present) {
    auto *age = contact->mutable_age();
    if (plain.age.seconds != 0) age->set_seconds(plain.age.seconds);
    if (plain.age.nanos != 0) age->set_nanos(plain.age.nanos);
  }
}

void ApplyPlain(const ContactsPlain &plain,
                SemanticContactsComponent *message) {
  message->Clear();
  if (plain.validity_message_present) {
    auto *validity = message->mutable_validity();
    validity->set_source_id(plain.source_id);
    if (plain.observation_time_present) {
      validity->mutable_observation_time()->set_seconds(
          plain.observation_time.seconds);
      validity->mutable_observation_time()->set_nanos(
          plain.observation_time.nanos);
    }
    if (plain.validity_horizon_present) {
      validity->mutable_validity_horizon()->set_seconds(
          plain.validity_horizon.seconds);
      validity->mutable_validity_horizon()->set_nanos(
          plain.validity_horizon.nanos);
    }
    if (plain.confidence_present) {
      validity->set_confidence(plain.confidence);
    }
    validity->set_uncertainty_reference(plain.uncertainty_reference);
    if (plain.embodiment_validity_present) {
      validity->mutable_validity()->set_state(
          static_cast<Validity::State>(plain.validity_state));
    }
  }
  message->set_frame_id(plain.frame_id);
  for (const ContactPlain &contact : plain.contacts) {
    ApplyContact(contact, message->add_contacts());
  }
}

// string_view members alias plain. plain must outlive the returned view.
SemanticContactsView ViewFromPlain(const ContactsPlain &plain) {
  SemanticContactsView view;
  view.present = plain.present;
  view.validity.present = plain.validity_message_present;
  view.validity.source_id = plain.source_id;
  view.validity.observation_time_present = plain.observation_time_present;
  view.validity.observation_time = plain.observation_time;
  view.validity.validity_horizon_present = plain.validity_horizon_present;
  view.validity.validity_horizon = plain.validity_horizon;
  view.validity.confidence_present = plain.confidence_present;
  view.validity.confidence = plain.confidence;
  view.validity.uncertainty_reference = plain.uncertainty_reference;
  view.validity.validity_present = plain.embodiment_validity_present;
  view.validity.validity_state = plain.validity_state;
  view.frame_id = plain.frame_id;
  for (const ContactPlain &contact : plain.contacts) {
    SemanticContactView item;
    item.contact_id = contact.contact_id;
    item.pose_present = contact.pose_present;
    item.position = contact.position;
    item.orientation = contact.orientation;
    item.velocity_present = contact.velocity_present;
    item.linear_velocity_m_s = contact.linear_velocity_m_s;
    item.angular_velocity_rad_s = contact.angular_velocity_rad_s;
    item.classification = contact.classification;
    item.confidence_present = contact.confidence_present;
    item.confidence = contact.confidence;
    item.age_present = contact.age_present;
    item.age = contact.age;
    view.contacts.push_back(item);
  }
  return view;
}

// Wire round trip so the assessed value is what a reader would see.
SemanticContactsAssessment AssessViaWire(const SemanticContactsComponent &in) {
  SemanticContactsComponent parsed;
  EXPECT_TRUE(parsed.ParseFromString(in.SerializeAsString()));
  const ContactsPlain plain = PlainFromProto(parsed, true);
  return AssessSemanticContacts(ViewFromPlain(plain), Query());
}

TEST(SemanticContactsSerializationTest, SchemaHasNoPerceptionFields) {
  const auto *descriptor = SemanticContactsComponent::descriptor();
  ASSERT_EQ(descriptor->field_count(), 3);
  EXPECT_EQ(descriptor->field(0)->number(), 1);
  EXPECT_EQ(descriptor->field(0)->name(), "validity");
  EXPECT_EQ(descriptor->field(1)->number(), 2);
  EXPECT_EQ(descriptor->field(1)->name(), "frame_id");
  EXPECT_EQ(descriptor->field(2)->number(), 3);
  EXPECT_EQ(descriptor->field(2)->name(), "contacts");
  EXPECT_TRUE(descriptor->field(2)->is_repeated());

  const auto *contact = descriptor->field(2)->message_type();
  ASSERT_EQ(contact->name(), "SemanticContact");
  EXPECT_EQ(contact->containing_type(), descriptor);
  ASSERT_EQ(contact->field_count(), 6);
  const char *const names[] = {"contact_id",     "pose",       "velocity",
                               "classification", "confidence", "age"};
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(contact->field(i)->number(), i + 1);
    EXPECT_EQ(contact->field(i)->name(), names[i]);
  }
  for (const char *name :
       {"track", "detections", "embedding", "image", "points"}) {
    EXPECT_EQ(descriptor->FindFieldByName(name), nullptr) << name;
    EXPECT_EQ(contact->FindFieldByName(name), nullptr) << name;
  }
}

TEST(SemanticContactsSerializationTest, GoldenRoundTrip) {
  SemanticContactsComponent message;
  FillFixture(&message);
  const std::string golden = GoldenBytes();
  EXPECT_EQ(message.SerializeAsString(), golden);

  SemanticContactsComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.frame_id(), "world_enu");
  EXPECT_EQ(parsed.validity().source_id(), "fusion_0");
  EXPECT_EQ(parsed.validity().validity().state(), Validity::STATE_VALID);
  ASSERT_EQ(parsed.contacts_size(), 2);
  const Contact &buoy = parsed.contacts(0);
  const Contact &dock = parsed.contacts(1);
  EXPECT_EQ(buoy.contact_id(), "buoy_1");
  EXPECT_DOUBLE_EQ(buoy.pose().position().x(), 12.0);
  EXPECT_DOUBLE_EQ(buoy.pose().position().y(), -3.5);
  EXPECT_DOUBLE_EQ(buoy.pose().position().z(), -1.25);
  EXPECT_DOUBLE_EQ(buoy.pose().orientation().w(), 1.0);
  EXPECT_DOUBLE_EQ(buoy.velocity().linear().x(), 0.1);
  EXPECT_DOUBLE_EQ(buoy.velocity().linear().z(), -0.05);
  EXPECT_DOUBLE_EQ(buoy.velocity().angular().z(), 0.02);
  EXPECT_EQ(buoy.classification(), "buoy");
  EXPECT_TRUE(buoy.has_confidence());
  EXPECT_DOUBLE_EQ(buoy.confidence(), 0.9);
  EXPECT_EQ(buoy.age().seconds(), 4);
  EXPECT_EQ(buoy.age().nanos(), 500000000);
  EXPECT_EQ(dock.contact_id(), "dock_2");
  EXPECT_DOUBLE_EQ(dock.pose().orientation().z(), 1.0);
  EXPECT_TRUE(dock.has_velocity());
  EXPECT_TRUE(dock.has_age());
  EXPECT_FALSE(dock.has_confidence());
  EXPECT_EQ(parsed.SerializeAsString(), golden);

  const ContactsPlain plain = PlainFromProto(parsed, true);
  SemanticContactsComponent again;
  ApplyPlain(plain, &again);
  EXPECT_EQ(again.SerializeAsString(), golden);
  const SemanticContactsAssessment assessment =
      AssessSemanticContacts(ViewFromPlain(plain), Query());
  EXPECT_EQ(assessment.error, SemanticContactsError::kNone);
  EXPECT_EQ(assessment.contact_index, -1);
  EXPECT_TRUE(assessment.accepted);
}

TEST(SemanticContactsSerializationTest, ContactOrderIsPreserved) {
  SemanticContactsComponent message;
  FillFixture(&message);
  message.mutable_contacts()->SwapElements(0, 1);
  EXPECT_NE(message.SerializeAsString(), GoldenBytes());
  SemanticContactsComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(message.SerializeAsString()));
  ASSERT_EQ(parsed.contacts_size(), 2);
  EXPECT_EQ(parsed.contacts(0).contact_id(), "dock_2");
  EXPECT_EQ(parsed.contacts(1).contact_id(), "buoy_1");
}

TEST(SemanticContactsSerializationTest, EmptyContactListIsAClearSet) {
  SemanticContactsComponent message;
  FillFixture(&message);
  message.clear_contacts();
  const std::string prefix = FromHex(kNoContactsHex);
  EXPECT_EQ(message.SerializeAsString(), prefix);
  EXPECT_TRUE(GoldenBytes().starts_with(prefix));
  const SemanticContactsAssessment assessment = AssessViaWire(message);
  EXPECT_EQ(assessment.error, SemanticContactsError::kNone);
  EXPECT_TRUE(assessment.accepted);
}

TEST(SemanticContactsSerializationTest,
     AbsentSubmessagesAreOmittedAndRejected) {
  {
    SemanticContactsComponent message;
    FillFixture(&message);
    message.mutable_contacts(1)->clear_pose();
    EXPECT_FALSE(message.contacts(1).has_pose());
    EXPECT_LT(message.SerializeAsString().size(), GoldenBytes().size());
    const SemanticContactsAssessment assessment = AssessViaWire(message);
    EXPECT_EQ(assessment.contact_error, SemanticContactError::kPose);
    EXPECT_EQ(assessment.contact_index, 1);
    EXPECT_FALSE(assessment.accepted);
  }
  {
    SemanticContactsComponent message;
    FillFixture(&message);
    message.mutable_contacts(1)->clear_velocity();
    EXPECT_FALSE(message.contacts(1).has_velocity());
    EXPECT_LT(message.SerializeAsString().size(), GoldenBytes().size());
    const SemanticContactsAssessment assessment = AssessViaWire(message);
    EXPECT_EQ(assessment.contact_error, SemanticContactError::kVelocity);
    EXPECT_EQ(assessment.contact_index, 1);
    EXPECT_FALSE(assessment.accepted);
  }
  {
    SemanticContactsComponent message;
    FillFixture(&message);
    message.mutable_contacts(1)->clear_age();
    EXPECT_FALSE(message.contacts(1).has_age());
    EXPECT_LT(message.SerializeAsString().size(), GoldenBytes().size());
    const SemanticContactsAssessment assessment = AssessViaWire(message);
    EXPECT_EQ(assessment.contact_error, SemanticContactError::kAge);
    EXPECT_EQ(assessment.contact_index, 1);
    EXPECT_FALSE(assessment.accepted);
  }
}

TEST(SemanticContactsSerializationTest, ExplicitZeroVelocityAndAgeArePresent) {
  SemanticContactsComponent message;
  FillFixture(&message);
  EXPECT_TRUE(message.contacts(1).has_velocity());
  EXPECT_TRUE(message.contacts(1).has_age());
  const ContactsPlain plain = PlainFromProto(message, true);
  EXPECT_TRUE(plain.contacts[1].velocity_present);
  EXPECT_DOUBLE_EQ(plain.contacts[1].linear_velocity_m_s.x, 0.0);
  EXPECT_DOUBLE_EQ(plain.contacts[1].angular_velocity_rad_s.z, 0.0);
  EXPECT_TRUE(plain.contacts[1].age_present);
  EXPECT_EQ(plain.contacts[1].age.seconds, 0);
  EXPECT_EQ(plain.contacts[1].age.nanos, 0);
  EXPECT_TRUE(AssessSemanticContacts(ViewFromPlain(plain), Query()).accepted);
  SemanticContactsComponent again;
  ApplyPlain(plain, &again);
  EXPECT_EQ(again.SerializeAsString(), message.SerializeAsString());
}

TEST(SemanticContactsSerializationTest, ConfidenceZeroIsDistinctFromUnset) {
  SemanticContactsComponent unset;
  FillFixture(&unset);
  EXPECT_FALSE(unset.contacts(1).has_confidence());
  SemanticContactsComponent zero = unset;
  zero.mutable_contacts(1)->set_confidence(0.0);
  EXPECT_TRUE(zero.contacts(1).has_confidence());
  EXPECT_NE(unset.SerializeAsString(), zero.SerializeAsString());
  const ContactsPlain plain = PlainFromProto(zero, true);
  EXPECT_TRUE(plain.contacts[1].confidence_present);
  EXPECT_DOUBLE_EQ(plain.contacts[1].confidence, 0.0);
  EXPECT_TRUE(AssessSemanticContacts(ViewFromPlain(plain), Query()).accepted);
  EXPECT_FALSE(PlainFromProto(unset, true).contacts[1].confidence_present);
}

TEST(SemanticContactsSerializationTest, ConfidenceBoundariesSurviveTheWire) {
  const struct {
    double confidence;
    bool accepted;
  } cases[] = {
      {0.0, true},
      {1.0, true},
      {-1e-12, false},
      {std::nextafter(1.0, 2.0), false},
  };
  for (const auto &test_case : cases) {
    SemanticContactsComponent message;
    FillFixture(&message);
    message.mutable_contacts(0)->set_confidence(test_case.confidence);
    const SemanticContactsAssessment assessment = AssessViaWire(message);
    EXPECT_EQ(assessment.accepted, test_case.accepted) << test_case.confidence;
    if (!test_case.accepted) {
      EXPECT_EQ(assessment.contact_error, SemanticContactError::kConfidence);
    }
  }
}

TEST(SemanticContactsSerializationTest,
     NegativeAgeSurvivesTheWireAndIsRejected) {
  SemanticContactsComponent message;
  FillFixture(&message);
  message.mutable_contacts(0)->mutable_age()->set_seconds(-1);
  message.mutable_contacts(0)->mutable_age()->set_nanos(0);
  const SemanticContactsAssessment assessment = AssessViaWire(message);
  EXPECT_EQ(assessment.contact_error, SemanticContactError::kAge);
  EXPECT_EQ(assessment.contact_index, 0);
}

TEST(SemanticContactsSerializationTest,
     NonFiniteGeometrySurvivesTheWireAndIsRejected) {
  SemanticContactsComponent message;
  FillFixture(&message);
  message.mutable_contacts(1)->mutable_pose()->mutable_position()->set_z(kNan);
  SemanticContactsAssessment assessment = AssessViaWire(message);
  EXPECT_EQ(assessment.contact_error, SemanticContactError::kPose);
  EXPECT_EQ(assessment.contact_index, 1);

  message.Clear();
  FillFixture(&message);
  message.mutable_contacts(0)->mutable_velocity()->mutable_angular()->set_y(
      kInf);
  assessment = AssessViaWire(message);
  EXPECT_EQ(assessment.contact_error, SemanticContactError::kVelocity);
  EXPECT_EQ(assessment.contact_index, 0);
}

TEST(SemanticContactsSerializationTest,
     DuplicateContactIdsSurviveTheWireAndAreRejected) {
  SemanticContactsComponent message;
  FillFixture(&message);
  message.mutable_contacts(1)->set_contact_id("buoy_1");
  SemanticContactsComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(message.SerializeAsString()));
  ASSERT_EQ(parsed.contacts_size(), 2);
  EXPECT_EQ(parsed.contacts(0).contact_id(), "buoy_1");
  EXPECT_EQ(parsed.contacts(1).contact_id(), "buoy_1");
  const SemanticContactsAssessment assessment = AssessViaWire(message);
  EXPECT_EQ(assessment.contact_error,
            SemanticContactError::kDuplicateContactId);
  EXPECT_EQ(assessment.contact_index, 1);
}

TEST(SemanticContactsSerializationTest, DefaultMessageIsEmpty) {
  const SemanticContactsComponent message;
  EXPECT_EQ(message.SerializeAsString(), "");
  EXPECT_FALSE(message.has_validity());
  EXPECT_EQ(message.contacts_size(), 0);
  SemanticContactsComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  const ContactsPlain plain = PlainFromProto(parsed, false);
  SemanticContactsComponent again;
  ApplyPlain(plain, &again);
  EXPECT_EQ(again.SerializeAsString(), "");
  const SemanticContactsAssessment empty =
      AssessSemanticContacts(ViewFromPlain(plain), TimeParts{0, 0});
  EXPECT_EQ(empty.error, SemanticContactsError::kNone);
  EXPECT_FALSE(empty.accepted);
  ContactsPlain present_plain = plain;
  present_plain.present = true;
  EXPECT_EQ(
      AssessSemanticContacts(ViewFromPlain(present_plain), TimeParts{0, 0})
          .error,
      SemanticContactsError::kValidity);
}

TEST(SemanticContactsSerializationTest, UnknownFieldsArePreserved) {
  const std::string golden = GoldenBytes();
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  SemanticContactsComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.frame_id(), "world_enu");
  EXPECT_EQ(parsed.contacts_size(), 2);
  const google::protobuf::UnknownFieldSet &unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);

  // Unknown varint field 100 appended inside the dock_2 contact.
  std::string dock = FromHex(kContact1Hex);
  const std::string extra("\xA0\x06\x07", 3);
  dock[1] = static_cast<char>(dock[1] + extra.size());
  dock += extra;
  const std::string nested_raw =
      FromHex(kNoContactsHex) + FromHex(kContact0Hex) + dock;
  SemanticContactsComponent nested;
  ASSERT_TRUE(nested.ParseFromString(nested_raw));
  EXPECT_EQ(nested.contacts(1).contact_id(), "dock_2");
  const google::protobuf::UnknownFieldSet &contact_unknown =
      nested.contacts(1).GetReflection()->GetUnknownFields(nested.contacts(1));
  ASSERT_EQ(contact_unknown.field_count(), 1);
  EXPECT_EQ(contact_unknown.field(0).number(), 100);
  EXPECT_EQ(nested.SerializeAsString(), nested_raw);
  EXPECT_TRUE(AssessViaWire(nested).accepted);
}

TEST(SemanticContactsSerializationTest, UnknownValidityEnumRoundTrips) {
  const std::string bytes = FromHex("0a0432020863");
  SemanticContactsComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_TRUE(parsed.has_validity());
  EXPECT_EQ(static_cast<int>(parsed.validity().validity().state()), 99);
  EXPECT_FALSE(Validity::State_IsValid(99));
  EXPECT_EQ(parsed.SerializeAsString(), bytes);
  const ContactsPlain plain = PlainFromProto(parsed, true);
  EXPECT_EQ(embodiment::ClassifyValidity(true, plain.validity_state),
            embodiment::ValidityKind::kUnspecified);

  const std::string with_unknown_field = FromHex("0a06320408011805");
  SemanticContactsComponent nested;
  ASSERT_TRUE(nested.ParseFromString(with_unknown_field));
  EXPECT_EQ(nested.validity().validity().state(), Validity::STATE_VALID);
  const google::protobuf::UnknownFieldSet &unknown =
      nested.validity().validity().GetReflection()->GetUnknownFields(
          nested.validity().validity());
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 3);
  EXPECT_EQ(unknown.field(0).varint(), 5u);
  EXPECT_EQ(nested.SerializeAsString(), with_unknown_field);
}

}  // namespace
}  // namespace intrinsic::world
