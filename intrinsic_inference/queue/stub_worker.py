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
echoes the envelope context and stamps a stub output digest.
"""

import dataclasses
import enum
from typing import Optional

from intrinsic_inference.envelope import inference_envelope_contract_policy as policy


class WorkerStatus(enum.Enum):
  """Terminal outcome of a worker. Names match InferenceQueueStatus."""

  COMPLETE = 0
  # Replay worker only (#120).
  REPLAY_MISS = 1
  REPLAY_EXPIRED = 2
  CORRUPT_FIXTURE = 3


@dataclasses.dataclass(frozen=True)
class WorkerOutcome:
  status: WorkerStatus = WorkerStatus.COMPLETE
  # Set for COMPLETE. Also set for REPLAY_EXPIRED, so the recorded timing
  # stays assertable. None for every other status.
  result: Optional[policy.InferenceResultView] = None


class StubWorker:
  """Builds an InferenceResultView for an accepted InferenceEnvelopeView.

  The default worker of InferenceClient. Any object with a matching
  complete(envelope, queue_request_id, now) method can be injected instead.
  """

  def complete(
      self,
      envelope: policy.InferenceEnvelopeView,
      queue_request_id: str,
      now: policy.TimeParts,
  ) -> WorkerOutcome:
    del now
    return WorkerOutcome(
        WorkerStatus.COMPLETE, self.run(envelope, queue_request_id)
    )

  def run(
      self, envelope: policy.InferenceEnvelopeView, queue_request_id: str
  ) -> policy.InferenceResultView:
    return policy.InferenceResultView(
        common=dataclasses.replace(envelope.common),
        oip_result=dataclasses.replace(envelope.oip_request),
        output_digest=f"stub-output:{queue_request_id}",
    )
