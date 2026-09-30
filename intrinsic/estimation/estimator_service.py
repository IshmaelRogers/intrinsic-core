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

"""Persistent estimation service interface.

Policy: intrinsic/estimation/README.md.

The published state type is the #17 VehicleState. This interface has no
filter equations and no ICON or safety wiring.
"""

import abc
from typing import Optional

from google.protobuf import timestamp_pb2

from intrinsic.estimation import estimator_pb2
from intrinsic.vehicle.proto import vehicle_state_pb2


class EstimatorService(abc.ABC):
  """Initialize, Ingest, Predict, Reset and GetState."""

  @abc.abstractmethod
  def initialize(
      self, config: estimator_pb2.EstimatorConfig
  ) -> estimator_pb2.EstimatorResult:
    """Loads config, resets every buffer, bumps the epoch, mode INITIALIZING.

    Calling it again is a full reset. An invalid config changes nothing.
    """

  @abc.abstractmethod
  def ingest(
      self, envelope: estimator_pb2.MeasurementEnvelope
  ) -> estimator_pb2.EstimatorResult:
    """Accepts or rejects one sample.

    A reject changes no published kinematics, mode or epoch. It only updates
    the source bookkeeping of a named source.
    """

  @abc.abstractmethod
  def predict(
      self, to_time: timestamp_pb2.Timestamp
  ) -> estimator_pb2.EstimatorResult:
    """Advances the snapshot clock to to_time and applies the mode rule."""

  @abc.abstractmethod
  def reset(
      self, prior: Optional[vehicle_state_pb2.VehicleState] = None
  ) -> estimator_pb2.EstimatorResult:
    """Clears buffers and bumps the epoch.

    A prior that passes AssessVehicleState becomes the next body. Otherwise the
    published body is empty. Either way the mode is INITIALIZING.
    """

  @abc.abstractmethod
  def get_state(self) -> estimator_pb2.GetStateResponse:
    """Returns the latched snapshot. state is unset before initialize."""
