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

// E9: the EpisodeEvent schema freeze. Every tag, type, name, and oneof
// membership is permanent. A change fails this test and needs senior review.
// Keep in sync with episode_event_schema_freeze_test.py.

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "google/protobuf/descriptor.h"
#include "gtest/gtest.h"
#include "intrinsic/episode/proto/episode_event.pb.h"

namespace intrinsic::episode {
namespace {

std::string ReadFreezeFile() {
  const char* src = std::getenv("TEST_SRCDIR");
  const char* workspace = std::getenv("TEST_WORKSPACE");
  const std::string relative =
      "intrinsic_data/episode/testdata/episode_event_schema_freeze.txt";
  std::vector<std::string> candidates;
  if (src != nullptr) {
    if (workspace != nullptr) {
      candidates.push_back(std::string(src) + "/" + workspace + "/" + relative);
    }
    candidates.push_back(std::string(src) + "/_main/" + relative);
  }
  for (const std::string& path : candidates) {
    std::ifstream in(path);
    if (!in) {
      continue;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
  }
  ADD_FAILURE() << "missing runfile " << relative;
  return "";
}

std::vector<std::string> Lines(const std::string& text) {
  std::vector<std::string> lines;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty()) {
      lines.push_back(line);
    }
  }
  return lines;
}

std::string FieldLine(const google::protobuf::Descriptor& message,
                      const google::protobuf::FieldDescriptor& field) {
  std::string type(field.type_name());
  if (field.type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    type += ":" + std::string(field.message_type()->full_name());
  }
  std::string label = field.is_repeated() ? "repeated" : "optional";
  if (field.real_containing_oneof() != nullptr) {
    label = "oneof:" + std::string(field.real_containing_oneof()->name());
  }
  return std::string(message.name()) + "." + std::string(field.name()) + " " +
         std::to_string(field.number()) + " " + type + " " + label;
}

std::vector<std::string> DescriptorLines() {
  const google::protobuf::FileDescriptor* file =
      intrinsic_proto::episode::EpisodeEvent::descriptor()->file();
  std::vector<std::string> lines;
  for (int i = 0; i < file->message_type_count(); ++i) {
    const google::protobuf::Descriptor* message = file->message_type(i);
    for (int j = 0; j < message->field_count(); ++j) {
      lines.push_back(FieldLine(*message, *message->field(j)));
    }
  }
  std::sort(lines.begin(), lines.end());
  return lines;
}

TEST(EpisodeEventSchemaFreezeTest, DescriptorEqualsTheFreezeFile) {
  const std::vector<std::string> frozen = Lines(ReadFreezeFile());
  ASSERT_FALSE(frozen.empty());
  EXPECT_EQ(DescriptorLines(), frozen);
}

TEST(EpisodeEventSchemaFreezeTest, FreezeFileIsSortedAndUnique) {
  const std::vector<std::string> frozen = Lines(ReadFreezeFile());
  EXPECT_TRUE(std::is_sorted(frozen.begin(), frozen.end()));
  EXPECT_EQ(std::set<std::string>(frozen.begin(), frozen.end()).size(),
            frozen.size());
}

TEST(EpisodeEventSchemaFreezeTest, PackageAndMessagesAreFrozen) {
  const google::protobuf::FileDescriptor* file =
      intrinsic_proto::episode::EpisodeEvent::descriptor()->file();
  EXPECT_EQ(file->package(), "intrinsic_proto.episode");
  std::set<std::string> messages;
  for (int i = 0; i < file->message_type_count(); ++i) {
    messages.insert(std::string(file->message_type(i)->name()));
  }
  EXPECT_EQ(messages,
            (std::set<std::string>{"EpisodeEvent", "EpisodeMetadata"}));
  EXPECT_EQ(file->enum_type_count(), 0);
}

TEST(EpisodeEventSchemaFreezeTest, NoFieldIsDeclaredReserved) {
  const google::protobuf::Descriptor* event =
      intrinsic_proto::episode::EpisodeEvent::descriptor();
  EXPECT_EQ(event->reserved_range_count(), 0);
  EXPECT_EQ(event->reserved_name_count(), 0);
}

}  // namespace
}  // namespace intrinsic::episode
