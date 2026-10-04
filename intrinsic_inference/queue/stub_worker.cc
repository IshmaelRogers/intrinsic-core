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

#include "intrinsic_inference/queue/stub_worker.h"

#include <string>
#include <string_view>
#include <utility>

#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"

namespace intrinsic::inference {
namespace {

// Strips and copies the strings shared by envelope and result views.
struct CommonStrings {
  std::string frame_id;
  std::string world_snapshot_id;
  std::string input_digest;
  std::string provenance_model_id;
  std::string model_name;
  std::string model_version;
  std::string request_id;
};

CommonStrings TakeStrings(InferenceCommonView& common, OipIdentifierView& oip) {
  CommonStrings out{
      std::string(common.frame_id),     std::string(common.world_snapshot_id),
      std::string(common.input_digest), std::string(common.provenance_model_id),
      std::string(oip.model_name),      std::string(oip.model_version),
      std::string(oip.request_id),
  };
  common.frame_id = {};
  common.world_snapshot_id = {};
  common.input_digest = {};
  common.provenance_model_id = {};
  oip.model_name = {};
  oip.model_version = {};
  oip.request_id = {};
  return out;
}

void Restore(InferenceCommonView& common, OipIdentifierView& oip,
             const std::string& frame_id, const std::string& world_snapshot_id,
             const std::string& input_digest,
             const std::string& provenance_model_id,
             const std::string& model_name, const std::string& model_version,
             const std::string& request_id) {
  common.frame_id = frame_id;
  common.world_snapshot_id = world_snapshot_id;
  common.input_digest = input_digest;
  common.provenance_model_id = provenance_model_id;
  oip.model_name = model_name;
  oip.model_version = model_version;
  oip.request_id = request_id;
}

}  // namespace

OwnedInferenceEnvelope::OwnedInferenceEnvelope(
    const InferenceEnvelopeView& view)
    : base_(view) {
  CommonStrings s = TakeStrings(base_.common, base_.oip_request);
  frame_id_ = std::move(s.frame_id);
  world_snapshot_id_ = std::move(s.world_snapshot_id);
  input_digest_ = std::move(s.input_digest);
  provenance_model_id_ = std::move(s.provenance_model_id);
  model_name_ = std::move(s.model_name);
  model_version_ = std::move(s.model_version);
  request_id_ = std::move(s.request_id);
}

InferenceEnvelopeView OwnedInferenceEnvelope::View() const {
  InferenceEnvelopeView view = base_;
  Restore(view.common, view.oip_request, frame_id_, world_snapshot_id_,
          input_digest_, provenance_model_id_, model_name_, model_version_,
          request_id_);
  return view;
}

InferenceResultShell::InferenceResultShell(const InferenceResultView& view)
    : base_(view) {
  CommonStrings s = TakeStrings(base_.common, base_.oip_result);
  frame_id_ = std::move(s.frame_id);
  world_snapshot_id_ = std::move(s.world_snapshot_id);
  input_digest_ = std::move(s.input_digest);
  provenance_model_id_ = std::move(s.provenance_model_id);
  model_name_ = std::move(s.model_name);
  model_version_ = std::move(s.model_version);
  request_id_ = std::move(s.request_id);
  output_digest_ = std::string(base_.output_digest);
  base_.output_digest = {};
}

InferenceResultView InferenceResultShell::View() const {
  InferenceResultView view = base_;
  Restore(view.common, view.oip_result, frame_id_, world_snapshot_id_,
          input_digest_, provenance_model_id_, model_name_, model_version_,
          request_id_);
  view.output_digest = output_digest_;
  return view;
}

InferenceResultShell StubWorker::Run(const OwnedInferenceEnvelope& envelope,
                                     std::string_view queue_request_id) const {
  const InferenceEnvelopeView request = envelope.View();
  const std::string output_digest =
      "stub-output:" + std::string(queue_request_id);
  InferenceResultView result;
  result.common = request.common;
  result.oip_result = request.oip_request;
  result.output_digest = output_digest;
  return InferenceResultShell(result);
}

}  // namespace intrinsic::inference
