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

#include "intrinsic/motion_planning/vehicle/vehicle_motion_primitives.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::BodyVector;

constexpr std::size_t kNumAxes = 6;

std::array<double, kNumAxes> ToArray(const BodyVector& v) {
  return {v.linear_x,  v.linear_y,  v.linear_z,
          v.angular_x, v.angular_y, v.angular_z};
}

BodyVector FromArray(const std::array<double, kNumAxes>& a) {
  BodyVector v;
  v.linear_x = a[0];
  v.linear_y = a[1];
  v.linear_z = a[2];
  v.angular_x = a[3];
  v.angular_y = a[4];
  v.angular_z = a[5];
  return v;
}

std::string PrimitiveId(std::size_t index) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "uuv-prim-%03zu", index);
  return buffer;
}

bool MaxIsValid(const std::array<double, kNumAxes>& max) {
  for (double m : max) {
    if (!std::isfinite(m) || m < 0.0) return false;
  }
  return true;
}

bool ControlIsWithin(const std::array<double, kNumAxes>& control,
                     const std::array<double, kNumAxes>& max) {
  for (std::size_t i = 0; i < kNumAxes; ++i) {
    if (!std::isfinite(control[i]) || std::fabs(control[i]) > max[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace

PrimitiveSetResult GenerateUuvMotionPrimitives(
    const VehicleMotionPrimitiveConfig& config) {
  PrimitiveSetResult result;
  result.error = PrimitiveSetError::kBadConfig;

  const std::array<double, kNumAxes> max = ToArray(config.max_force_torque);
  if (!MaxIsValid(max)) return result;
  if (!std::isfinite(config.duration_s) || config.duration_s <= 0.0) {
    return result;
  }

  std::vector<std::array<double, kNumAxes>> controls;
  if (config.mode == PrimitiveGenerationMode::kAxisAligned) {
    controls.reserve(1 + 2 * kNumAxes);
    controls.push_back({});
    for (std::size_t axis = 0; axis < kNumAxes; ++axis) {
      std::array<double, kNumAxes> positive{};
      std::array<double, kNumAxes> negative{};
      // A zero bound stays +0.0 on both entries so controls are comparable
      // bit for bit.
      if (max[axis] != 0.0) {
        positive[axis] = max[axis];
        negative[axis] = -max[axis];
      }
      controls.push_back(positive);
      controls.push_back(negative);
    }
  } else if (config.mode == PrimitiveGenerationMode::kCustom) {
    controls.reserve(config.custom_controls.size());
    for (const BodyVector& control : config.custom_controls) {
      const std::array<double, kNumAxes> values = ToArray(control);
      if (!ControlIsWithin(values, max)) return result;
      controls.push_back(values);
    }
    if (controls.empty()) {
      result.error = PrimitiveSetError::kEmpty;
      return result;
    }
  } else {
    return result;
  }

  result.primitives.reserve(controls.size());
  for (std::size_t i = 0; i < controls.size(); ++i) {
    VehicleMotionPrimitive primitive;
    primitive.id = PrimitiveId(i);
    primitive.control = FromArray(controls[i]);
    primitive.duration_s = config.duration_s;
    result.primitives.push_back(std::move(primitive));
  }
  result.error = PrimitiveSetError::kOk;
  return result;
}

}  // namespace intrinsic::motion_planning::vehicle
