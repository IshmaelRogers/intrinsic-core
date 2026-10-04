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

"""Fake inference worker for the bounded queue.

Completes a request on a deterministic path with no I/O: no OIP or Triton
call, no model, no socket, no replay store, no clock, no sleep. The result
echoes the envelope context and stamps a stub output digest. Real backends
are a later leaf.
"""

import dataclasses

from intrinsic_inference.envelope import inference_envelope_contract_policy as policy


class StubWorker:
  """Builds an InferenceResultView for an accepted InferenceEnvelopeView."""

  def run(
      self, envelope: policy.InferenceEnvelopeView, queue_request_id: str
  ) -> policy.InferenceResultView:
    return policy.InferenceResultView(
        common=dataclasses.replace(envelope.common),
        oip_result=dataclasses.replace(envelope.oip_request),
        output_digest=f"stub-output:{queue_request_id}",
    )
