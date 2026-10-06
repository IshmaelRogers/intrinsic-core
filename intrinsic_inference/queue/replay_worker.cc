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

#include "intrinsic_inference/queue/replay_worker.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"
#include "intrinsic_inference/queue/stub_worker.h"
#include "nlohmann/json.hpp"

namespace intrinsic::inference {
namespace {

using Json = nlohmann::json;

std::optional<std::string> GetString(const Json& object, const char* key) {
  const auto it = object.find(key);
  if (it == object.end() || !it->is_string()) {
    return std::nullopt;
  }
  return it->get<std::string>();
}

std::optional<int64_t> GetInt(const Json& object, const char* key, int64_t min,
                              int64_t max) {
  const auto it = object.find(key);
  if (it == object.end() || !it->is_number_integer()) {
    return std::nullopt;
  }
  if (it->is_number_unsigned()) {
    const uint64_t value = it->get<uint64_t>();
    if (value > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) ||
        static_cast<int64_t>(value) > max) {
      return std::nullopt;
    }
    return static_cast<int64_t>(value);
  }
  const int64_t value = it->get<int64_t>();
  if (value < min || value > max) {
    return std::nullopt;
  }
  return value;
}

std::optional<uint64_t> GetUnsigned(const Json& object, const char* key) {
  const auto it = object.find(key);
  if (it == object.end() || !it->is_number_unsigned()) {
    return std::nullopt;
  }
  return it->get<uint64_t>();
}

std::optional<double> GetNumber(const Json& object, const char* key) {
  const auto it = object.find(key);
  if (it == object.end() || !it->is_number()) {
    return std::nullopt;
  }
  return it->get<double>();
}

// Reads {"seconds": int64, "nanos": int32}.
std::optional<std::pair<int64_t, int32_t>> GetTime(const Json& object,
                                                   const char* key) {
  const auto it = object.find(key);
  if (it == object.end() || !it->is_object()) {
    return std::nullopt;
  }
  const auto seconds =
      GetInt(*it, "seconds", std::numeric_limits<int64_t>::min(),
             std::numeric_limits<int64_t>::max());
  const auto nanos = GetInt(*it, "nanos", std::numeric_limits<int32_t>::min(),
                            std::numeric_limits<int32_t>::max());
  if (!seconds.has_value() || !nanos.has_value()) {
    return std::nullopt;
  }
  return std::make_pair(*seconds, static_cast<int32_t>(*nanos));
}

struct ParsedKey {
  std::string model;
  std::string digest;
  uint64_t epoch = 0;
};

// The match key is the only thing that lets a fixture be addressed. A file
// without a complete key cannot be keyed and is skipped by the caller.
std::optional<ParsedKey> ParseKey(const Json& root) {
  const auto match = root.find("match");
  if (match == root.end() || !match->is_object()) {
    return std::nullopt;
  }
  const auto model = GetString(*match, "provenance_model_id");
  const auto digest = GetString(*match, "input_digest");
  const auto epoch = GetUnsigned(*match, "state_epoch");
  if (!model.has_value() || !digest.has_value() || !epoch.has_value()) {
    return std::nullopt;
  }
  return ParsedKey{*model, *digest, *epoch};
}

// Builds the recorded result shell from a fixture. Returns nullopt when a
// required field is missing or mistyped, or when the shell would not pass
// AssessInferenceResult. The match key supplies the shell's provenance model
// id, input digest, and state epoch, so the two can never disagree.
std::optional<InferenceResultShell> ParseResult(
    const Json& root, const ParsedKey& key,
    embodiment::ClockReading* deadline_out) {
  const auto result_it = root.find("result");
  if (result_it == root.end() || !result_it->is_object()) {
    return std::nullopt;
  }
  const Json& result = *result_it;

  const auto frame_id = GetString(result, "frame_id");
  const auto validity_state = GetInt(result, "validity_state", 0, 3);
  const auto source_time = GetTime(result, "source_time");
  const auto deadline = GetTime(result, "deadline");
  const auto horizon = GetTime(result, "validity_horizon");
  const auto output_digest = GetString(result, "output_digest");
  const auto oip_it = result.find("oip");
  if (!frame_id || !validity_state || !source_time || !deadline || !horizon ||
      !output_digest || oip_it == result.end() || !oip_it->is_object()) {
    return std::nullopt;
  }
  const auto model_name = GetString(*oip_it, "model_name");
  const auto model_version = GetString(*oip_it, "model_version");
  const auto request_id = GetString(*oip_it, "request_id");
  if (!model_name || !request_id) {
    return std::nullopt;
  }

  std::string world_snapshot_id;
  if (result.contains("world_snapshot_id")) {
    const auto value = GetString(result, "world_snapshot_id");
    if (!value) {
      return std::nullopt;
    }
    world_snapshot_id = *value;
  }
  std::optional<double> confidence;
  if (result.contains("confidence")) {
    confidence = GetNumber(result, "confidence");
    if (!confidence) {
      return std::nullopt;
    }
  }
  std::optional<double> uncertainty;
  if (result.contains("uncertainty")) {
    uncertainty = GetNumber(result, "uncertainty");
    if (!uncertainty) {
      return std::nullopt;
    }
  }

  const std::string model_version_text = model_version.value_or("");
  InferenceResultView view;
  InferenceCommonView& common = view.common;
  common.header_present = true;
  common.validity_present = true;
  common.validity_state = static_cast<int>(*validity_state);
  common.frame_id = *frame_id;
  common.source_time_present = true;
  common.source_time = {source_time->first, source_time->second};
  common.state_epoch = key.epoch;
  common.world_snapshot_id = world_snapshot_id;
  common.deadline_present = true;
  common.deadline = {deadline->first, deadline->second};
  common.validity_horizon_present = true;
  common.validity_horizon = {horizon->first, horizon->second};
  common.confidence_present = confidence.has_value();
  common.confidence = confidence.value_or(0.0);
  common.uncertainty_present = uncertainty.has_value();
  common.uncertainty = uncertainty.value_or(0.0);
  common.input_digest = key.digest;
  common.provenance_present = true;
  common.provenance_model_id = key.model;
  view.oip_result.message_set = true;
  view.oip_result.model_name = *model_name;
  view.oip_result.model_version = model_version_text;
  view.oip_result.request_id = *request_id;
  view.output_digest = *output_digest;

  const InferenceEnvelopeContractAssessment assessment =
      AssessInferenceResult(view);
  if (assessment.error != InferenceEnvelopeContractError::kNone) {
    return std::nullopt;
  }
  *deadline_out = common.deadline;
  return InferenceResultShell(view);
}

}  // namespace

ReplayWorker::ReplayWorker(const std::string& fixture_dir) {
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::directory_iterator it(fixture_dir, ec);
  if (ec) {
    return;
  }
  std::vector<fs::path> files;
  for (const fs::directory_iterator end; it != end; it.increment(ec)) {
    if (ec) {
      break;
    }
    std::error_code status_ec;
    if (it->is_regular_file(status_ec) && it->path().extension() == ".json") {
      files.push_back(it->path());
    }
  }
  std::sort(files.begin(), files.end());

  for (const fs::path& path : files) {
    const std::string name = path.filename().string();
    std::ifstream stream(path, std::ios::binary);
    std::ostringstream text;
    text << stream.rdbuf();
    if (!stream && !stream.eof()) {
      skipped_files_.push_back(name);
      continue;
    }
    const Json root =
        Json::parse(text.str(), nullptr, /*allow_exceptions=*/false);
    if (root.is_discarded() || !root.is_object()) {
      skipped_files_.push_back(name);
      continue;
    }
    const std::optional<ParsedKey> key = ParseKey(root);
    if (!key.has_value()) {
      skipped_files_.push_back(name);
      continue;
    }
    Key map_key{key->model, key->digest, key->epoch};
    if (rows_.find(map_key) != rows_.end()) {
      // Two files claim the same key. Neither can be trusted.
      rows_[map_key] = Row{};
      continue;
    }
    Row row;
    row.shell = ParseResult(root, *key, &row.deadline);
    rows_.emplace(std::move(map_key), std::move(row));
  }
}

WorkerOutcome ReplayWorker::Complete(const OwnedInferenceEnvelope& envelope,
                                     std::string_view /*queue_request_id*/,
                                     embodiment::ClockReading now) const {
  const InferenceEnvelopeView request = envelope.View();
  const Key key{std::string(request.common.provenance_model_id),
                std::string(request.common.input_digest),
                request.common.state_epoch};
  WorkerOutcome outcome;
  const auto row = rows_.find(key);
  if (row == rows_.end()) {
    outcome.status = WorkerStatus::kReplayMiss;
    return outcome;
  }
  if (!row->second.shell.has_value()) {
    outcome.status = WorkerStatus::kCorruptFixture;
    return outcome;
  }
  outcome.result = row->second.shell;
  outcome.status = TimeBefore(row->second.deadline, now)
                       ? WorkerStatus::kReplayExpired
                       : WorkerStatus::kComplete;
  return outcome;
}

}  // namespace intrinsic::inference
