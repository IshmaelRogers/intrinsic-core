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

#include "intrinsic/simulation/gazebo/scenarios/uuv_reference/shadow_policy_runner.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"
#include "intrinsic_inference/queue/inference_client.h"
#include "intrinsic_inference/queue/policy_worker.h"
#include "intrinsic_inference/queue/stub_worker.h"

namespace intrinsic::simulation::uuv_reference {
namespace {

using inference::InferenceClient;
using inference::InferenceClientOptions;
using inference::InferenceEnvelopeView;
using inference::InferenceQueueStatus;
using inference::PollOutcome;
using Status = InferenceQueueStatus;

constexpr int64_t kNanosPerSecond = embodiment::kNanosPerSecond;
// Largest origin that cannot overflow when sim time up to INT64_MAX ns is
// added in whole seconds.
constexpr int64_t kMaxOriginSeconds =
    std::numeric_limits<int64_t>::max() - 10'000'000'000;

}  // namespace

ShadowPolicyOutcome ClassifySubmitStatus(InferenceQueueStatus status) {
  switch (status) {
    case Status::kQueueFull:
    case Status::kInvalidEnvelope:
    case Status::kDeadlineExpiredAtSubmit:
    case Status::kShutdown:
    default:
      return ShadowPolicyOutcome::kFailed;
  }
}

ShadowPolicyOutcome ClassifyTerminal(InferenceQueueStatus status,
                                     bool motion_found) {
  switch (status) {
    case Status::kComplete:
      return motion_found ? ShadowPolicyOutcome::kActive
                          : ShadowPolicyOutcome::kFailed;
    case Status::kReplayExpired:
    case Status::kCancelled:
      return ShadowPolicyOutcome::kLate;
    default:
      return ShadowPolicyOutcome::kFailed;
  }
}

ShadowPolicyRunner::ShadowPolicyRunner(ShadowPolicyRunnerOptions options)
    : options_(std::move(options)), now_(options_.clock_origin) {
  if (options_.worker == nullptr) {
    return;
  }
  InferenceClientOptions client_options;
  client_options.clock = [this] { return now_; };
  client_options.complete_on_submit = options_.complete_on_submit;
  client_options.worker = options_.worker;
  client_ = std::make_unique<InferenceClient>(std::move(client_options));
}

bool ShadowPolicyRunner::ValidStep(int64_t k, int64_t sim_time_ns) const {
  const int64_t dt_ns = options_.dt_ns;
  if (dt_ns <= 0 || dt_ns > std::numeric_limits<int64_t>::max() / 4) {
    return false;
  }
  if (options_.worker != nullptr && options_.store == nullptr) {
    return false;
  }
  const embodiment::ClockReading& origin = options_.clock_origin;
  if (origin.seconds < 0 || origin.seconds > kMaxOriginSeconds ||
      !embodiment::NanosInRange(origin.nanos)) {
    return false;
  }
  if (sim_time_ns < 0 || sim_time_ns > std::numeric_limits<int64_t>::max() -
                                           kShadowDeadlineSteps * dt_ns) {
    return false;
  }
  if (has_step_ && (k <= last_k_ || sim_time_ns <= last_sim_time_ns_)) {
    return false;
  }
  return true;
}

embodiment::ClockReading ShadowPolicyRunner::AtNs(int64_t ns) const {
  embodiment::ClockReading reading = options_.clock_origin;
  reading.seconds += ns / kNanosPerSecond;
  int64_t nanos = reading.nanos + ns % kNanosPerSecond;
  if (nanos >= kNanosPerSecond) {
    nanos -= kNanosPerSecond;
    ++reading.seconds;
  }
  reading.nanos = static_cast<int32_t>(nanos);
  return reading;
}

void ShadowPolicyRunner::Classify(
    const Pending& pending, const PollOutcome& reaped,
    std::vector<ShadowPolicyResolution>& out) const {
  ShadowPolicyResolution resolution;
  resolution.submit_k = pending.submit_k;
  resolution.request_id = pending.request_id;
  resolution.status = reaped.status;
  resolution.result = reaped.result;
  const intrinsic_proto::vehicle::DesiredMotion* motion = nullptr;
  if (reaped.status == Status::kComplete && reaped.result.has_value()) {
    motion = options_.store->Find(reaped.result->View().output_digest);
  }
  resolution.outcome = ClassifyTerminal(reaped.status, motion != nullptr);
  if (reaped.status == Status::kComplete && motion == nullptr) {
    resolution.digest_mismatch = true;
  }
  if (resolution.outcome == ShadowPolicyOutcome::kActive) {
    resolution.motion = *motion;
  }
  out.push_back(std::move(resolution));
}

StepResult ShadowPolicyRunner::Step(int64_t k, int64_t sim_time_ns,
                                    const ShadowEnvelopeInputs& inputs) {
  StepResult step;
  if (!ValidStep(k, sim_time_ns)) {
    step.error = true;
    return step;
  }
  has_step_ = true;
  last_k_ = k;
  last_sim_time_ns_ = sim_time_ns;
  now_ = AtNs(sim_time_ns);

  if (client_ == nullptr) {
    ShadowPolicyResolution absent;
    absent.submit_k = k;
    absent.outcome = ShadowPolicyOutcome::kAbsent;
    step.resolutions.push_back(std::move(absent));
    return step;
  }

  if (options_.ticks_per_step > 0) {
    client_->Tick(options_.ticks_per_step);
  }

  std::deque<Pending> still_pending;
  for (Pending& pending : pending_) {
    PollOutcome polled = client_->Poll(pending.request_id);
    if (polled.status != Status::kOk) {
      Classify(pending, client_->Reap(pending.request_id), step.resolutions);
      continue;
    }
    if (sim_time_ns <= pending.deadline_ns) {
      still_pending.push_back(std::move(pending));
      continue;
    }
    const inference::CancelOutcome cancelled =
        client_->Cancel(pending.request_id);
    if (cancelled.status == Status::kAlreadyComplete) {
      Classify(pending, client_->Reap(pending.request_id), step.resolutions);
      continue;
    }
    PollOutcome terminal;
    terminal.status = cancelled.status;
    Classify(pending, terminal, step.resolutions);
  }
  pending_ = std::move(still_pending);

  const int64_t dt_ns = options_.dt_ns;
  const int64_t deadline_ns = sim_time_ns + kShadowDeadlineSteps * dt_ns;
  const std::string request_id = "shadow-" + std::to_string(k);
  InferenceEnvelopeView envelope;
  envelope.common.header_present = true;
  envelope.common.validity_present = true;
  envelope.common.validity_state = 1;
  envelope.common.frame_id = options_.frame_id;
  envelope.common.source_time_present = true;
  envelope.common.source_time = now_;
  envelope.common.state_epoch = inputs.state_epoch;
  envelope.common.world_snapshot_id = inputs.world_snapshot_id;
  envelope.common.deadline_present = true;
  envelope.common.deadline = AtNs(deadline_ns);
  envelope.common.validity_horizon_present = true;
  const int64_t horizon_ns = kShadowDeadlineSteps * dt_ns;
  envelope.common.validity_horizon = {
      horizon_ns / kNanosPerSecond,
      static_cast<int32_t>(horizon_ns % kNanosPerSecond)};
  envelope.common.input_digest = inputs.input_digest;
  envelope.common.provenance_present = true;
  envelope.common.provenance_model_id = options_.model_id;
  envelope.oip_request.message_set = true;
  envelope.oip_request.model_name = options_.model_name;
  envelope.oip_request.model_version = options_.model_version;
  envelope.oip_request.request_id = request_id;

  const inference::SubmitOutcome submitted = client_->Submit(envelope);
  if (submitted.status != Status::kOk) {
    ShadowPolicyResolution rejected;
    rejected.submit_k = k;
    rejected.outcome = ClassifySubmitStatus(submitted.status);
    rejected.status = submitted.status;
    rejected.envelope_error = submitted.envelope_error;
    step.resolutions.push_back(std::move(rejected));
    return step;
  }

  step.submitted_request_id = submitted.request_id;
  Pending pending{submitted.request_id, k, deadline_ns};
  const PollOutcome polled = client_->Poll(pending.request_id);
  if (polled.status != Status::kOk) {
    Classify(pending, client_->Reap(pending.request_id), step.resolutions);
  } else {
    pending_.push_back(std::move(pending));
  }
  return step;
}

}  // namespace intrinsic::simulation::uuv_reference
