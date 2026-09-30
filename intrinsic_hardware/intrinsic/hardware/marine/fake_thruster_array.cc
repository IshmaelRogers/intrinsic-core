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

#include "intrinsic/hardware/marine/fake_thruster_array.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/thruster_array.pb.h"
#include "intrinsic/hardware/marine/thruster_array_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::ThrusterArrayCommand;
using intrinsic_proto::hardware::marine::ThrusterArrayFeedback;
using intrinsic_proto::hardware::marine::ThrusterHealth;

constexpr int kHealthNominal = ThrusterHealth::THRUSTER_HEALTH_NOMINAL;
constexpr int kHealthDisabled = ThrusterHealth::THRUSTER_HEALTH_DISABLED;
constexpr int kHealthDerated = ThrusterHealth::THRUSTER_HEALTH_DERATED;
constexpr int kHealthStuckOff = ThrusterHealth::THRUSTER_HEALTH_STUCK_OFF;
constexpr int kHealthFailed = ThrusterHealth::THRUSTER_HEALTH_FAILED;

enum class FaultRank {
  kNone = 0,
  kDerated = 1,
  kDisabled = 2,
  kFailed = 3,
  kStuckOff = 4,
};

FakeThrusterSlotConfig Slot(const char *name, double forward_n,
                            double reverse_n, double forward_slew_n_per_s,
                            double reverse_slew_n_per_s) {
  FakeThrusterSlotConfig slot;
  slot.name = name;
  slot.min_thrust_n = -reverse_n;
  slot.max_thrust_n = forward_n;
  slot.max_forward_slew_n_per_s = forward_slew_n_per_s;
  slot.max_reverse_slew_n_per_s = reverse_slew_n_per_s;
  slot.efficiency = 1.0;
  return slot;
}

const FakeThrusterSlotConfig &FallbackSlot() {
  static const FakeThrusterSlotConfig slot = Slot("", 50, 35, 250, 175);
  return slot;
}

double ClampThrust(double value, double min_n, double max_n) {
  if (value > max_n) {
    return max_n;
  }
  if (value < min_n) {
    return min_n;
  }
  return value;
}

// Moves `previous` toward `target` by at most rate * dt. A non-positive
// limit leaves `previous` in place.
double SlewToward(double previous, double target, double forward_slew,
                  double reverse_slew, double dt) {
  const double delta = target - previous;
  if (delta == 0.0) {
    return previous;
  }
  const double rate = delta > 0.0 ? forward_slew : reverse_slew;
  const double limit = rate * dt;
  if (!(limit > 0.0)) {
    return previous;
  }
  if (std::fabs(delta) <= limit) {
    return target;
  }
  return previous + std::copysign(limit, delta);
}

FaultRank RankFault(const FakeThrusterSlotConfig &slot, bool enabled) {
  if (slot.fault_health_present && slot.fault_health == kHealthStuckOff) {
    return FaultRank::kStuckOff;
  }
  if (slot.fault_health_present && slot.fault_health == kHealthFailed) {
    return FaultRank::kFailed;
  }
  if (!enabled ||
      (slot.fault_health_present && slot.fault_health == kHealthDisabled)) {
    return FaultRank::kDisabled;
  }
  if (slot.fault_health_present && slot.fault_health == kHealthDerated) {
    return FaultRank::kDerated;
  }
  if (slot.efficiency > 0.0 && slot.efficiency < 1.0) {
    return FaultRank::kDerated;
  }
  return FaultRank::kNone;
}

int HealthWire(FaultRank rank) {
  switch (rank) {
  case FaultRank::kStuckOff:
    return kHealthStuckOff;
  case FaultRank::kFailed:
    return kHealthFailed;
  case FaultRank::kDisabled:
    return kHealthDisabled;
  case FaultRank::kDerated:
    return kHealthDerated;
  case FaultRank::kNone:
    return kHealthNominal;
  }
  return kHealthNominal;
}

double HealthDerate(FaultRank rank, const FakeThrusterSlotConfig &slot) {
  switch (rank) {
  case FaultRank::kStuckOff:
  case FaultRank::kFailed:
  case FaultRank::kDisabled:
    return 0.0;
  case FaultRank::kDerated:
    if (slot.fault_health_present && slot.fault_health == kHealthDerated) {
      return slot.fault_derate;
    }
    return slot.efficiency;
  case FaultRank::kNone:
    return 1.0;
  }
  return 1.0;
}

bool NeutralRank(FaultRank rank) {
  return rank == FaultRank::kStuckOff || rank == FaultRank::kFailed ||
         rank == FaultRank::kDisabled;
}

void FillDefaultHeader(intrinsic_proto::embodiment::StampedHeader *header) {
  header->set_source_id("thruster_array");
  header->set_frame_id("body");
  header->set_clock_domain(std::string(embodiment::kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_VALID);
  header->mutable_source_time()->set_seconds(1700000000);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000001);
}

} // namespace

std::vector<FakeThrusterSlotConfig> SixThrusterFixtureSlots() {
  return {
      Slot("surge_port", 50, 35, 250, 175),
      Slot("surge_starboard", 50, 35, 250, 175),
      Slot("sway_fore", 30, 30, 150, 150),
      Slot("sway_aft", 30, 30, 150, 150),
      Slot("heave_fore", 40, 25, 200, 125),
      Slot("heave_aft", 40, 25, 200, 125),
  };
}

std::optional<ThrusterArrayFeedback>
FakeThrusterArray::Apply(const ThrusterArrayCommand &command) {
  std::vector<RecordedSlot> current;
  current.reserve(command.thrusters_size());
  for (const auto &element : command.thrusters()) {
    RecordedSlot slot;
    slot.thrust_n = element.has_thrust_n() ? element.thrust_n() : 0.0;
    ThrusterCommandElementView enable_view;
    enable_view.enable_present = element.has_enable();
    enable_view.enable = element.enable();
    slot.enabled = ThrusterCommandEnabled(enable_view);
    slot.name_present = element.has_name();
    if (slot.name_present) {
      slot.name = element.name();
    }
    current.push_back(std::move(slot));
  }
  history_.push_back(current);
  const uint64_t sequence = config_.seed + step_;
  ++step_;
  if (config_.dropout) {
    return std::nullopt;
  }

  const int lag = config_.lag_steps < 0 ? 0 : config_.lag_steps;
  const int delayed_index = static_cast<int>(history_.size()) - 1 - lag;
  std::vector<RecordedSlot> neutral;
  const std::vector<RecordedSlot> *applied = nullptr;
  if (delayed_index >= 0) {
    applied = &history_[delayed_index];
  } else {
    neutral.assign(current.size(), RecordedSlot{});
    applied = &neutral;
  }

  ThrusterArrayFeedback feedback;
  intrinsic_proto::embodiment::StampedHeader *header =
      feedback.mutable_header();
  if (command.has_header()) {
    *header = command.header();
  } else {
    FillDefaultHeader(header);
  }
  header->set_sequence(sequence);

  for (int i = 0; i < static_cast<int>(applied->size()); ++i) {
    const RecordedSlot &recorded = (*applied)[i];
    const FakeThrusterSlotConfig &slot =
        (i >= 0 && i < static_cast<int>(config_.slots.size()))
            ? config_.slots[i]
            : FallbackSlot();
    if (static_cast<int>(slew_thrust_n_.size()) <= i) {
      slew_thrust_n_.resize(i + 1, 0.0);
    }
    const FaultRank rank = RankFault(slot, recorded.enabled);
    double applied_thrust = 0.0;
    bool saturated = false;
    if (NeutralRank(rank)) {
      slew_thrust_n_[i] = 0.0;
    } else {
      const double pre =
          config_.slew ? SlewToward(slew_thrust_n_[i], recorded.thrust_n,
                                    slot.max_forward_slew_n_per_s,
                                    slot.max_reverse_slew_n_per_s, config_.dt_s)
                       : recorded.thrust_n;
      const double clamped =
          ClampThrust(pre, slot.min_thrust_n, slot.max_thrust_n);
      slew_thrust_n_[i] = clamped;
      saturated = clamped != pre;
      applied_thrust = clamped * slot.efficiency;
    }

    auto *element = feedback.add_thrusters();
    if (recorded.name_present) {
      element->set_name(recorded.name);
    } else if (!slot.name.empty()) {
      element->set_name(slot.name);
    }
    element->set_commanded_thrust_n(recorded.thrust_n);
    element->set_measured_thrust_n(applied_thrust);
    element->set_saturated(saturated);
    element->set_health(static_cast<ThrusterHealth>(HealthWire(rank)));
    element->set_health_derate(HealthDerate(rank, slot));
    element->set_efficiency(slot.efficiency);
  }
  return feedback;
}

} // namespace intrinsic::hardware::marine
