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

#include "intrinsic_inference/queue/inference_client.h"

#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"
#include "intrinsic_inference/queue/stub_worker.h"

namespace intrinsic::inference {
namespace {

using Error = InferenceEnvelopeContractError;
using Status = InferenceQueueStatus;

constexpr std::string_view kIdPrefix = "inf-";

std::string FormatId(uint64_t id) {
  return std::string(kIdPrefix) + std::to_string(id);
}

// Accepts only the canonical form FormatId produces.
std::optional<uint64_t> ParseId(std::string_view text) {
  if (text.substr(0, kIdPrefix.size()) != kIdPrefix) {
    return std::nullopt;
  }
  const std::string_view digits = text.substr(kIdPrefix.size());
  if (digits.empty() || digits.size() > 20) {
    return std::nullopt;
  }
  uint64_t value = 0;
  const auto [end, ec] =
      std::from_chars(digits.data(), digits.data() + digits.size(), value);
  if (ec != std::errc() || end != digits.data() + digits.size() ||
      FormatId(value) != text) {
    return std::nullopt;
  }
  return value;
}

embodiment::ClockReading SystemUtcNow() {
  const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
  const auto seconds =
      std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
      since_epoch - seconds);
  return embodiment::ClockReading{static_cast<int64_t>(seconds.count()),
                                  static_cast<int32_t>(nanos.count())};
}

}  // namespace

InferenceClient::InferenceClient()
    : InferenceClient(InferenceClientOptions{}) {}

InferenceClient::InferenceClient(InferenceClientOptions options)
    : options_(std::move(options)) {
  if (!options_.clock) {
    options_.clock = SystemUtcNow;
  }
}

size_t InferenceClient::OutstandingLocked() const {
  size_t count = 0;
  for (const Slot& slot : slots_) {
    if (slot.state != SlotState::kFree) {
      ++count;
    }
  }
  return count;
}

InferenceClient::Slot* InferenceClient::FindLocked(
    std::string_view request_id) {
  const std::optional<uint64_t> id = ParseId(request_id);
  if (!id.has_value()) {
    return nullptr;
  }
  for (Slot& slot : slots_) {
    if (slot.state != SlotState::kFree && slot.id == *id) {
      return &slot;
    }
  }
  return nullptr;
}

bool InferenceClient::WasDiscardedLocked(std::string_view request_id) const {
  const std::optional<uint64_t> id = ParseId(request_id);
  if (!id.has_value()) {
    return false;
  }
  for (size_t i = 0; i < discarded_count_; ++i) {
    if (discarded_ids_[i] == *id) {
      return true;
    }
  }
  return false;
}

void InferenceClient::CompleteLocked(Slot& slot) {
  slot.result = worker_.Run(slot.envelope, FormatId(slot.id));
  slot.state = SlotState::kComplete;
}

SubmitOutcome InferenceClient::Submit(const InferenceEnvelopeView& request) {
  std::lock_guard<std::mutex> lock(mutex_);
  SubmitOutcome outcome;
  if (shutdown_) {
    outcome.status = Status::kShutdown;
    return outcome;
  }

  const InferenceEnvelopeContractAssessment assessment =
      AssessInferenceEnvelope(request);
  outcome.envelope_error = assessment.error;
  if (assessment.error == Error::kDeadline && request.common.deadline_present &&
      embodiment::NanosInRange(request.common.deadline.nanos)) {
    // #118 rejects deadline < header.source_time; that is the same
    // deadline-before-creation rule.
    outcome.status = Status::kDeadlineExpiredAtSubmit;
    return outcome;
  }
  if (assessment.error != Error::kNone || !assessment.accepted) {
    outcome.status = Status::kInvalidEnvelope;
    return outcome;
  }
  if (TimeBefore(request.common.deadline, options_.clock())) {
    outcome.status = Status::kDeadlineExpiredAtSubmit;
    return outcome;
  }

  Slot* free_slot = nullptr;
  for (Slot& slot : slots_) {
    if (slot.state == SlotState::kFree) {
      free_slot = &slot;
      break;
    }
  }
  if (free_slot == nullptr) {
    outcome.status = Status::kQueueFull;
    return outcome;
  }

  free_slot->id = next_id_++;
  free_slot->envelope = OwnedInferenceEnvelope(request);
  free_slot->result.reset();
  free_slot->state = SlotState::kInProgress;
  if (options_.complete_on_submit) {
    CompleteLocked(*free_slot);
  }
  outcome.status = Status::kOk;
  outcome.request_id = FormatId(free_slot->id);
  return outcome;
}

PollOutcome InferenceClient::Poll(std::string_view request_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  PollOutcome outcome;
  const Slot* slot = FindLocked(request_id);
  if (slot == nullptr) {
    outcome.status =
        WasDiscardedLocked(request_id) ? Status::kShutdown : Status::kUnknownId;
    return outcome;
  }
  if (slot->state == SlotState::kComplete) {
    outcome.status = Status::kComplete;
    outcome.result = slot->result;
  } else {
    outcome.status = Status::kOk;
  }
  return outcome;
}

CancelOutcome InferenceClient::Cancel(std::string_view request_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  CancelOutcome outcome;
  Slot* slot = FindLocked(request_id);
  if (slot == nullptr) {
    outcome.status =
        WasDiscardedLocked(request_id) ? Status::kShutdown : Status::kUnknownId;
    return outcome;
  }
  if (slot->state == SlotState::kComplete) {
    outcome.status = Status::kAlreadyComplete;
    return outcome;
  }
  *slot = Slot{};
  outcome.status = Status::kCancelled;
  return outcome;
}

PollOutcome InferenceClient::Reap(std::string_view request_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  PollOutcome outcome;
  Slot* slot = FindLocked(request_id);
  if (slot == nullptr) {
    outcome.status =
        WasDiscardedLocked(request_id) ? Status::kShutdown : Status::kUnknownId;
    return outcome;
  }
  if (slot->state != SlotState::kComplete) {
    outcome.status = Status::kOk;
    return outcome;
  }
  outcome.status = Status::kComplete;
  outcome.result = std::move(slot->result);
  *slot = Slot{};
  return outcome;
}

size_t InferenceClient::Tick(size_t max_completions) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (shutdown_) {
    return 0;
  }
  size_t completed = 0;
  while (completed < max_completions) {
    Slot* oldest = nullptr;
    for (Slot& slot : slots_) {
      if (slot.state == SlotState::kInProgress &&
          (oldest == nullptr || slot.id < oldest->id)) {
        oldest = &slot;
      }
    }
    if (oldest == nullptr) {
      break;
    }
    CompleteLocked(*oldest);
    ++completed;
  }
  return completed;
}

void InferenceClient::Shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (shutdown_) {
    return;
  }
  shutdown_ = true;
  for (Slot& slot : slots_) {
    if (slot.state != SlotState::kFree) {
      discarded_ids_[discarded_count_++] = slot.id;
      slot = Slot{};
    }
  }
}

size_t InferenceClient::Outstanding() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return OutstandingLocked();
}

bool InferenceClient::IsShutdown() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return shutdown_;
}

}  // namespace intrinsic::inference
