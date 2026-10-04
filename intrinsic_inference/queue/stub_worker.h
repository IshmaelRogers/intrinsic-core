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

#ifndef INTRINSIC_INFERENCE_QUEUE_STUB_WORKER_H_
#define INTRINSIC_INFERENCE_QUEUE_STUB_WORKER_H_

#include <string>
#include <string_view>

#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"

namespace intrinsic::inference {

// Owning copy of an InferenceEnvelopeView. The queue keeps one per accepted
// request so the caller's string storage need not outlive Submit.
class OwnedInferenceEnvelope {
 public:
  OwnedInferenceEnvelope() = default;
  explicit OwnedInferenceEnvelope(const InferenceEnvelopeView& view);

  // The returned string_views point into this object.
  InferenceEnvelopeView View() const;

 private:
  InferenceEnvelopeView base_;  // string_view members are always empty.
  std::string frame_id_;
  std::string world_snapshot_id_;
  std::string input_digest_;
  std::string provenance_model_id_;
  std::string model_name_;
  std::string model_version_;
  std::string request_id_;
};

// Owning InferenceResult shell returned by a completed request.
class InferenceResultShell {
 public:
  InferenceResultShell() = default;
  explicit InferenceResultShell(const InferenceResultView& view);

  // The returned string_views point into this object. Suitable for
  // AssessInferenceResult.
  InferenceResultView View() const;

 private:
  InferenceResultView base_;  // string_view members are always empty.
  std::string frame_id_;
  std::string world_snapshot_id_;
  std::string input_digest_;
  std::string provenance_model_id_;
  std::string model_name_;
  std::string model_version_;
  std::string request_id_;
  std::string output_digest_;
};

// Fake worker. Completes a request on a deterministic path with no I/O: no
// OIP or Triton call, no model, no socket, no replay store, no clock, no
// sleep. The result echoes the envelope context and stamps a stub output
// digest. Real backends are a later leaf.
class StubWorker {
 public:
  InferenceResultShell Run(const OwnedInferenceEnvelope& envelope,
                           std::string_view queue_request_id) const;
};

}  // namespace intrinsic::inference

#endif  // INTRINSIC_INFERENCE_QUEUE_STUB_WORKER_H_
