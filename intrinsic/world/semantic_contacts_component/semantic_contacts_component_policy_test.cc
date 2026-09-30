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

#include "intrinsic/world/semantic_contacts_component/semantic_contacts_component_policy.h"

#include <cmath>
#include <limits>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::world {
namespace {

using embodiment::ValidityKind;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

MarineComponentValidityView ValidValidity() {
  MarineComponentValidityView view;
  view.present = true;
  view.source_id = "fusion_0";
  view.observation_time_present = true;
  view.observation_time = TimeParts{1700000000, 250000000};
  view.validity_horizon_present = true;
  view.validity_horizon = TimeParts{10, 500000000};
  view.confidence_present = true;
  view.confidence = 0.75;
  view.uncertainty_reference = "cov-ref-7";
  view.validity_present = true;
  view.validity_state = 1;
  return view;
}

SemanticContactView Contact(std::string_view id = "buoy_1",
                            std::string_view classification = "buoy") {
  SemanticContactView contact;
  contact.contact_id = id;
  contact.pose_present = true;
  contact.position = embodiment::Vec3{12.0, -3.5, -1.25};
  contact.orientation = embodiment::Quaternion{0.0, 0.0, 0.0, 1.0};
  contact.velocity_present = true;
  contact.linear_velocity_m_s = embodiment::Vec3{0.1, 0.0, -0.05};
  contact.angular_velocity_rad_s = embodiment::Vec3{0.0, 0.0, 0.02};
  contact.classification = classification;
  contact.confidence_present = true;
  contact.confidence = 0.9;
  contact.age_present = true;
  contact.age = TimeParts{4, 500000000};
  return contact;
}

SemanticContactsView ValidContacts() {
  SemanticContactsView view;
  view.present = true;
  view.validity = ValidValidity();
  view.frame_id = embodiment::kWorldEnuFrameId;
  view.contacts = {Contact("buoy_1", "buoy"), Contact("dock_2", "dock")};
  return view;
}

// 1700000000.250s + 10.500s = 1700000010.750s.
TimeParts Deadline() { return TimeParts{1700000010, 750000000}; }

SemanticContactsAssessment Assess(const SemanticContactsView &view) {
  return AssessSemanticContacts(view, Deadline());
}

TEST(SemanticContactsPolicyTest, EmptyMessageIsNotAnError) {
  const SemanticContactsAssessment assessment = Assess(SemanticContactsView{});
  EXPECT_EQ(assessment.error, SemanticContactsError::kNone);
  EXPECT_EQ(assessment.contact_error, SemanticContactError::kNone);
  EXPECT_EQ(assessment.contact_index, -1);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kNone);
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kAbsent);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SemanticContactsPolicyTest, MissingValidityIsFirstDefect) {
  SemanticContactsView view = ValidContacts();
  view.validity.present = false;
  view.frame_id = "robot";
  view.contacts = {Contact("")};
  const SemanticContactsAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.error, SemanticContactsError::kValidity);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kNone);
  EXPECT_EQ(assessment.contact_index, -1);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SemanticContactsPolicyTest, FirstDefectWins) {
  SemanticContactsView view = ValidContacts();
  view.validity.source_id = "";
  view.frame_id = "robot";
  view.contacts = {Contact("")};
  SemanticContactsAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.error, SemanticContactsError::kValidity);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kSourceId);

  view.validity = ValidValidity();
  EXPECT_EQ(Assess(view).error, SemanticContactsError::kFrameId);

  view.frame_id = embodiment::kWorldNedFrameId;
  assessment = Assess(view);
  EXPECT_EQ(assessment.error, SemanticContactsError::kContact);
  EXPECT_EQ(assessment.contact_error, SemanticContactError::kContactId);
  EXPECT_EQ(assessment.contact_index, 0);

  view.contacts = {Contact()};
  assessment = Assess(view);
  EXPECT_EQ(assessment.error, SemanticContactsError::kNone);
  EXPECT_EQ(assessment.contact_index, -1);
  EXPECT_TRUE(assessment.accepted);
}

TEST(SemanticContactsPolicyTest, FrameAllowList) {
  SemanticContactsView view = ValidContacts();
  for (const std::string_view frame_id :
       {embodiment::kWorldEnuFrameId, embodiment::kWorldNedFrameId}) {
    view.frame_id = frame_id;
    EXPECT_TRUE(Assess(view).accepted) << frame_id;
  }
  for (const std::string_view frame_id :
       {"", "enu", "ned", "world_ENU", "body", "robot", "world_enu "}) {
    view.frame_id = frame_id;
    EXPECT_EQ(Assess(view).error, SemanticContactsError::kFrameId) << frame_id;
  }
}

TEST(SemanticContactsPolicyTest, EmptyContactListIsAccepted) {
  SemanticContactsView view = ValidContacts();
  view.contacts.clear();
  const SemanticContactsAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.error, SemanticContactsError::kNone);
  EXPECT_EQ(assessment.contact_index, -1);
  EXPECT_TRUE(assessment.accepted);
}

TEST(SemanticContactsPolicyTest, ContactIdEmptyAndDuplicate) {
  SemanticContactsView view = ValidContacts();
  view.contacts[1].contact_id = "";
  SemanticContactsAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.error, SemanticContactsError::kContact);
  EXPECT_EQ(assessment.contact_error, SemanticContactError::kContactId);
  EXPECT_EQ(assessment.contact_index, 1);

  view.contacts[1].contact_id = "buoy_1";
  assessment = Assess(view);
  EXPECT_EQ(assessment.error, SemanticContactsError::kContact);
  EXPECT_EQ(assessment.contact_error,
            SemanticContactError::kDuplicateContactId);
  EXPECT_EQ(assessment.contact_index, 1);
  EXPECT_FALSE(assessment.accepted);

  for (const std::string_view near_miss : {"Buoy_1", "buoy_1 ", " buoy_1"}) {
    view.contacts[1].contact_id = near_miss;
    EXPECT_TRUE(Assess(view).accepted) << near_miss;
  }
}

TEST(SemanticContactsPolicyTest, FirstDuplicateIsTheDefect) {
  SemanticContactsView view = ValidContacts();
  view.contacts = {Contact("a"), Contact("b"), Contact("b"), Contact("a")};
  const SemanticContactsAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.contact_error,
            SemanticContactError::kDuplicateContactId);
  EXPECT_EQ(assessment.contact_index, 2);
}

TEST(SemanticContactsPolicyTest,
     DuplicateIdPrecedesLaterFieldDefectsOfSameContact) {
  SemanticContactsView view = ValidContacts();
  view.contacts = {Contact("a"), Contact("a")};
  view.contacts[1].classification = "";
  view.contacts[1].age_present = false;
  EXPECT_EQ(Assess(view).contact_error,
            SemanticContactError::kDuplicateContactId);
}

TEST(SemanticContactsPolicyTest, EarlierContactDefectWinsOverLaterDuplicate) {
  SemanticContactsView view = ValidContacts();
  view.contacts = {Contact("a"), Contact("a")};
  view.contacts[0].classification = "";
  const SemanticContactsAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.contact_error, SemanticContactError::kClassification);
  EXPECT_EQ(assessment.contact_index, 0);
}

TEST(SemanticContactsPolicyTest, PoseMustBePresentAndFinite) {
  SemanticContactsView view = ValidContacts();
  view.contacts[0].pose_present = false;
  SemanticContactsAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.contact_error, SemanticContactError::kPose);
  EXPECT_EQ(assessment.contact_index, 0);

  for (const embodiment::Vec3 position :
       {embodiment::Vec3{kNan, 0, 0}, embodiment::Vec3{0, kInf, 0},
        embodiment::Vec3{0, 0, -kInf}}) {
    view = ValidContacts();
    view.contacts[1].position = position;
    assessment = Assess(view);
    EXPECT_EQ(assessment.contact_error, SemanticContactError::kPose);
    EXPECT_EQ(assessment.contact_index, 1);
  }
  for (const embodiment::Quaternion orientation :
       {embodiment::Quaternion{kNan, 0, 0, 1},
        embodiment::Quaternion{0, kInf, 0, 1},
        embodiment::Quaternion{0, 0, -kInf, 1},
        embodiment::Quaternion{0, 0, 0, kNan}}) {
    view = ValidContacts();
    view.contacts[0].orientation = orientation;
    EXPECT_EQ(Assess(view).contact_error, SemanticContactError::kPose);
  }
}

TEST(SemanticContactsPolicyTest, QuaternionIsNotRenormalized) {
  SemanticContactsView view = ValidContacts();
  view.contacts[0].orientation = embodiment::Quaternion{0, 0, 0, 0};
  EXPECT_TRUE(Assess(view).accepted);
  view.contacts[0].orientation = embodiment::Quaternion{3, 4, 0, 5};
  EXPECT_TRUE(Assess(view).accepted);
}

TEST(SemanticContactsPolicyTest, VelocityMustBePresentAndFinite) {
  SemanticContactsView view = ValidContacts();
  view.contacts[0].velocity_present = false;
  EXPECT_EQ(Assess(view).contact_error, SemanticContactError::kVelocity);

  view = ValidContacts();
  view.contacts[0].linear_velocity_m_s = embodiment::Vec3{0, 0, 0};
  view.contacts[0].angular_velocity_rad_s = embodiment::Vec3{0, 0, 0};
  EXPECT_TRUE(Assess(view).accepted);

  const double bad[] = {kNan, kInf, -kInf};
  for (int axis = 0; axis < 3; ++axis) {
    embodiment::Vec3 vector;
    (axis == 0 ? vector.x : axis == 1 ? vector.y : vector.z) = bad[axis];
    view = ValidContacts();
    view.contacts[1].linear_velocity_m_s = vector;
    SemanticContactsAssessment assessment = Assess(view);
    EXPECT_EQ(assessment.contact_error, SemanticContactError::kVelocity);
    EXPECT_EQ(assessment.contact_index, 1);

    view = ValidContacts();
    view.contacts[1].angular_velocity_rad_s = vector;
    assessment = Assess(view);
    EXPECT_EQ(assessment.contact_error, SemanticContactError::kVelocity);
    EXPECT_EQ(assessment.contact_index, 1);
  }
}

TEST(SemanticContactsPolicyTest, ClassificationIsOpaqueAndRequired) {
  SemanticContactsView view = ValidContacts();
  view.contacts[0].classification = "";
  EXPECT_EQ(Assess(view).contact_error, SemanticContactError::kClassification);
  for (const std::string_view label : {"unknown", " ", "Buoy", "a,b"}) {
    view.contacts[0].classification = label;
    EXPECT_TRUE(Assess(view).accepted) << label;
  }
}

TEST(SemanticContactsPolicyTest,
     ConfidenceUnsetIsNotZeroAndBoundsAreInclusive) {
  SemanticContactsView view = ValidContacts();
  view.contacts[0].confidence_present = false;
  view.contacts[0].confidence = kNan;
  EXPECT_TRUE(Assess(view).accepted);

  view.contacts[0].confidence_present = true;
  for (const double confidence : {0.0, -0.0, 0.5, 1.0}) {
    view.contacts[0].confidence = confidence;
    EXPECT_TRUE(Assess(view).accepted) << confidence;
  }
  for (const double confidence :
       {-1e-12, std::nextafter(1.0, 2.0), 2.0, kNan, kInf, -kInf}) {
    view.contacts[0].confidence = confidence;
    const SemanticContactsAssessment assessment = Assess(view);
    EXPECT_EQ(assessment.contact_error, SemanticContactError::kConfidence)
        << confidence;
    EXPECT_EQ(assessment.contact_index, 0);
  }
}

TEST(SemanticContactsPolicyTest, AgeMustBePresentAndNonNegative) {
  SemanticContactsView view = ValidContacts();
  view.contacts[0].age_present = false;
  EXPECT_EQ(Assess(view).contact_error, SemanticContactError::kAge);

  view.contacts[0].age_present = true;
  for (const TimeParts age :
       {TimeParts{0, 0}, TimeParts{0, 999999999}, TimeParts{3600, 0}}) {
    view.contacts[0].age = age;
    EXPECT_TRUE(Assess(view).accepted) << age.seconds << "." << age.nanos;
  }
  for (const TimeParts age : {TimeParts{-1, 0}, TimeParts{-1, 500000000},
                              TimeParts{0, -1}, TimeParts{1, 1000000000}}) {
    view.contacts[0].age = age;
    const SemanticContactsAssessment assessment = Assess(view);
    EXPECT_EQ(assessment.contact_error, SemanticContactError::kAge)
        << age.seconds << "." << age.nanos;
    EXPECT_EQ(assessment.contact_index, 0);
  }
}

TEST(SemanticContactsPolicyTest, PerContactCheckOrder) {
  SemanticContactView bad;
  EXPECT_EQ(AssessSemanticContact(bad), SemanticContactError::kContactId);
  bad.contact_id = "c";
  EXPECT_EQ(AssessSemanticContact(bad), SemanticContactError::kPose);
  bad.pose_present = true;
  EXPECT_EQ(AssessSemanticContact(bad), SemanticContactError::kVelocity);
  bad.velocity_present = true;
  EXPECT_EQ(AssessSemanticContact(bad), SemanticContactError::kClassification);
  bad.classification = "unknown";
  EXPECT_EQ(AssessSemanticContact(bad), SemanticContactError::kAge);
  bad.confidence_present = true;
  bad.confidence = 2.0;
  EXPECT_EQ(AssessSemanticContact(bad), SemanticContactError::kConfidence);
  bad.confidence_present = false;
  bad.age_present = true;
  EXPECT_EQ(AssessSemanticContact(bad), SemanticContactError::kNone);
}

TEST(SemanticContactsPolicyTest, SameNumbersAcceptedInEachFrame) {
  SemanticContactsView view = ValidContacts();
  for (const std::string_view frame_id :
       {embodiment::kWorldEnuFrameId, embodiment::kWorldNedFrameId}) {
    view.frame_id = frame_id;
    EXPECT_TRUE(Assess(view).accepted) << frame_id;
    EXPECT_DOUBLE_EQ(view.contacts[0].position.x, 12.0);
    EXPECT_DOUBLE_EQ(view.contacts[0].position.y, -3.5);
    EXPECT_DOUBLE_EQ(view.contacts[0].position.z, -1.25);
    EXPECT_DOUBLE_EQ(view.contacts[0].linear_velocity_m_s.z, -0.05);
  }
}

TEST(SemanticContactsPolicyTest, AcceptedExampleAtDeadline) {
  const SemanticContactsAssessment fresh =
      AssessSemanticContacts(ValidContacts(), Deadline());
  EXPECT_EQ(fresh.error, SemanticContactsError::kNone);
  EXPECT_EQ(fresh.validity.freshness, ComponentFreshness::kFresh);
  EXPECT_EQ(fresh.validity.validity, ValidityKind::kValid);
  EXPECT_TRUE(fresh.accepted);

  const SemanticContactsAssessment expired = AssessSemanticContacts(
      ValidContacts(), TimeParts{Deadline().seconds, Deadline().nanos + 1});
  EXPECT_EQ(expired.error, SemanticContactsError::kNone);
  EXPECT_EQ(expired.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_EQ(expired.validity.error, ComponentValidityError::kNone);
  EXPECT_FALSE(expired.accepted);
}

TEST(SemanticContactsPolicyTest, UnknownAndInvalidStayNested) {
  SemanticContactsView view = ValidContacts();
  view.validity.validity_present = false;
  SemanticContactsAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.error, SemanticContactsError::kNone);
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kAbsent);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);

  view = ValidContacts();
  view.validity.validity_state = 2;
  assessment = Assess(view);
  EXPECT_EQ(assessment.error, SemanticContactsError::kNone);
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);

  view.validity.validity_state = 99;
  assessment = Assess(view);
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kUnspecified);
  EXPECT_EQ(assessment.error, SemanticContactsError::kNone);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SemanticContactsPolicyTest, BadEmbeddedConfidenceIsAValidityDefect) {
  SemanticContactsView view = ValidContacts();
  view.validity.confidence = 1.5;
  const SemanticContactsAssessment assessment = Assess(view);
  EXPECT_EQ(assessment.error, SemanticContactsError::kValidity);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kConfidence);
}

TEST(SemanticContactsPolicyTest, ExpiredContactDefectStaysAContactError) {
  SemanticContactsView view = ValidContacts();
  view.contacts[0].classification = "";
  const SemanticContactsAssessment assessment = AssessSemanticContacts(
      view, TimeParts{Deadline().seconds, Deadline().nanos + 1});
  EXPECT_EQ(assessment.error, SemanticContactsError::kContact);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_FALSE(assessment.accepted);
}

}  // namespace
}  // namespace intrinsic::world
