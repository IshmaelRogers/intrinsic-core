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

"""Proto wrappers for the authority transition policy.

Policy: intrinsic_apis/intrinsic/safety/proto/README.md.

Unknown enum numbers are passed to the plain-value policy as raw ints, so they
are classified as unknown, fail closed, and are not rewritten.
"""

from intrinsic.safety import authority_transition_policy as policy
from intrinsic.safety import safety_decision_policy


def assess_authority_transition(
    current: int, event: int
) -> policy.AuthorityTransitionResult:
  """Applies apply_authority_transition to AuthorityMode and Event numbers."""
  return policy.apply_authority_transition(
      policy.classify_authority_mode(int(current)),
      policy.classify_authority_event(int(event)),
  )


def assess_decision_kind_under_authority(mode: int, kind: int) -> bool:
  """Applies decision_allowed_under_authority to wire numbers."""
  return policy.decision_allowed_under_authority(
      policy.classify_authority_mode(int(mode)),
      safety_decision_policy.classify_safety_decision_kind(int(kind)),
  )
