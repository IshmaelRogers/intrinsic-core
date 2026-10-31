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

#ifndef INTRINSIC_SIMULATION_GAZEBO_SCENARIOS_UUV_REFERENCE_SHADOW_POLICY_RUNNER_H_
#define INTRINSIC_SIMULATION_GAZEBO_SCENARIOS_UUV_REFERENCE_SHADOW_POLICY_RUNNER_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"
#include "intrinsic_inference/queue/inference_client.h"
#include "intrinsic_inference/queue/policy_worker.h"
#include "intrinsic_inference/queue/stub_worker.h"

namespace intrinsic::simulation::uuv_reference {

// Shadow policy runner (L44-2). Contract: issue #266, LOCK
// l44-2-shadow-policy-runner-v1, on top of S44-1 (#260) D44-4 A and D44-1 A.
//
// Once per sim step the runner builds an inference envelope, submits it to its
// own InferenceClient, collects every request that reached a terminal class,
// and classifies each one as absent, failed, late or active. A request is
// late when its deadline, N = 2 steps after submission, has passed. A late
// request has no usable motion, even if the worker completed it (lock v1.1).
//
// The runner owns the sim clock. Its client reads a clock that Step sets from
// sim_time_ns before any queue call, so the client never falls back to the
// system clock. Plain C++: no Gazebo, HAL, driver, socket, hardware, OIP or
// Triton. The runner is single threaded and is the only caller of its client,
// so a Cancel cannot race a completion beyond the kAlreadyComplete branch in
// Step.

// D44-4 N. Not configurable.
inline constexpr int64_t kShadowDeadlineSteps = 2;

enum class ShadowPolicyOutcome {
  kAbsent = 0,
  kFailed = 1,
  kLate = 2,
  kActive = 3,
};

struct ShadowPolicyRunnerOptions {
  // Null means no policy: Step reports kAbsent and no client is constructed.
  // (A null worker inside the queue would mean the stub worker.)
  std::shared_ptr<const inference::InferenceWorker> worker;
  // Side store the worker's output_digest keys. Required with a worker.
  std::shared_ptr<const inference::DesiredMotionStore> store;
  // Sim step length in nanoseconds. Must be > 0.
  int64_t dt_ns = 0;
  // Clock reading at sim_time_ns == 0. The recorded L44-1 fixtures use
  // absolute times near 1 700 000 000 s, so tests that replay them set this.
  embodiment::ClockReading clock_origin{};
  // Passed to the queue: true completes a request inside Submit. False leaves
  // it in progress until ticks_per_step completions are granted by a later
  // Step, so a request resolves one step after it was submitted.
  bool complete_on_submit = true;
  size_t ticks_per_step = 0;
  std::string frame_id = "world_enu";
  std::string model_id = "uuv_shadow_policy";
  std::string model_name = "uuv_shadow_policy";
  std::string model_version = "1";
};

struct ShadowEnvelopeInputs {
  // Verbatim. What it digests is decided by the scenario, not here.
  std::string input_digest;
  uint64_t state_epoch = 0;
  // Empty or 64 lowercase hex.
  std::string world_snapshot_id;
};

struct ShadowPolicyResolution {
  // The step whose Step call submitted the request (the current step for
  // kAbsent and for a rejected submit).
  int64_t submit_k = 0;
  // Empty for a rejected submit and for kAbsent.
  std::string request_id;
  ShadowPolicyOutcome outcome = ShadowPolicyOutcome::kFailed;
  // The terminal queue status, or the submit status for a rejected submit.
  inference::InferenceQueueStatus status = inference::InferenceQueueStatus::kOk;
  inference::InferenceEnvelopeContractError envelope_error =
      inference::InferenceEnvelopeContractError::kNone;
  // kComplete whose output_digest has no motion in the store.
  bool digest_mismatch = false;
  // Set for kComplete and kReplayExpired.
  std::optional<inference::InferenceResultShell> result;
  // Set for kActive only.
  std::optional<intrinsic_proto::vehicle::DesiredMotion> motion;
};

struct StepResult {
  // True when the step arguments or options were rejected. No queue call was
  // made, the state is unchanged, and resolutions is empty.
  bool error = false;
  // Request id accepted by this step, else empty.
  std::string submitted_request_id;
  // Requests that reached a terminal class this step, oldest submit_k first.
  // A rejected submit of this step comes last. Absent: exactly one kAbsent.
  std::vector<ShadowPolicyResolution> resolutions;
};

// Class of a rejected submit. Only called with a non-kOk status; every value,
// including kQueueFull, kInvalidEnvelope, kDeadlineExpiredAtSubmit and
// kShutdown, is kFailed.
ShadowPolicyOutcome ClassifySubmitStatus(
    inference::InferenceQueueStatus status);

// Class of a terminal queue status, before the deadline rule. kComplete is
// kActive when the motion was found in the store, else kFailed. kReplayExpired
// and kCancelled are kLate. Everything else is kFailed. Step additionally
// makes a kComplete reaped with sim_time_ns > deadline kLate (lock v1.1).
ShadowPolicyOutcome ClassifyTerminal(inference::InferenceQueueStatus status,
                                     bool motion_found);

// Step order:
//   1. Validate the arguments and set the clock.
//   2. No worker: one kAbsent resolution, return.
//   3. Grant ticks_per_step completions when > 0.
//   4. Oldest first, poll every pending request. A terminal one is reaped and
//      classified; a reaped kComplete with sim_time_ns > its deadline is kLate
//      (result shell kept, no motion), and sim_time_ns == deadline stays
//      kActive. An in-progress one with sim_time_ns > its deadline is
//      cancelled and resolves kLate.
//   5. Submit this step's envelope. A rejected submit is kFailed. An accepted
//      one is polled and, if already terminal, reaped and classified.
// A request submitted in tick mode therefore never completes in the step that
// submitted it.
class ShadowPolicyRunner {
 public:
  explicit ShadowPolicyRunner(ShadowPolicyRunnerOptions options);
  ShadowPolicyRunner(const ShadowPolicyRunner&) = delete;
  // Not movable either: the client clock captures this.
  ShadowPolicyRunner& operator=(const ShadowPolicyRunner&) = delete;

  // k and sim_time_ns must both increase on every call. The envelope created
  // at sim_time_ns has deadline sim_time_ns + 2 * dt_ns. A step that violates
  // a bound below returns error = true, makes no queue call and leaves the
  // state unchanged. Bounds that keep the int64 arithmetic from overflowing:
  //   dt_ns                <= INT64_MAX / 4
  //   sim_time_ns          <= INT64_MAX - 2 * dt_ns  (the deadline fits)
  //   clock_origin.seconds <= INT64_MAX - 10'000'000'000
  StepResult Step(int64_t k, int64_t sim_time_ns,
                  const ShadowEnvelopeInputs& inputs);

  // Pending requests tracked by the runner.
  size_t outstanding() const { return pending_.size(); }
  // True when a worker is set.
  bool configured() const { return options_.worker != nullptr; }

 private:
  struct Pending {
    std::string request_id;
    int64_t submit_k = 0;
    int64_t deadline_ns = 0;
  };

  bool ValidStep(int64_t k, int64_t sim_time_ns) const;
  embodiment::ClockReading AtNs(int64_t ns) const;
  void Classify(const Pending& pending, const inference::PollOutcome& reaped,
                int64_t sim_time_ns,
                std::vector<ShadowPolicyResolution>& out) const;

  ShadowPolicyRunnerOptions options_;
  embodiment::ClockReading now_;
  std::unique_ptr<inference::InferenceClient> client_;
  std::deque<Pending> pending_;
  bool has_step_ = false;
  int64_t last_k_ = 0;
  int64_t last_sim_time_ns_ = 0;
};

}  // namespace intrinsic::simulation::uuv_reference

#endif  // INTRINSIC_SIMULATION_GAZEBO_SCENARIOS_UUV_REFERENCE_SHADOW_POLICY_RUNNER_H_
