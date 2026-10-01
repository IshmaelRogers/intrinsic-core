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

#include <cmath>
#include <string>

namespace intrinsic::motion_planning::vehicle {
namespace {

bool SpeedBoundValid(bool present, double value) {
  return !present || (std::isfinite(value) && value >= 0.0);
}

bool LevelValid(double level) { return std::isfinite(level) && level > 0.0; }

BodyVector ControlOnAxis(MotionPrimitiveAxis axis, double value) {
  BodyVector control;
  switch (axis) {
    case MotionPrimitiveAxis::kSurge:
      control.linear_x = value;
      break;
    case MotionPrimitiveAxis::kSway:
      control.linear_y = value;
      break;
    case MotionPrimitiveAxis::kHeave:
      control.linear_z = value;
      break;
    case MotionPrimitiveAxis::kRoll:
      control.angular_x = value;
      break;
    case MotionPrimitiveAxis::kPitch:
      control.angular_y = value;
      break;
    case MotionPrimitiveAxis::kYaw:
      control.angular_z = value;
      break;
  }
  return control;
}

double LinearSpeed(const BodyVector& control) {
  return std::hypot(control.linear_x, control.linear_y, control.linear_z);
}

double AngularSpeed(const BodyVector& control) {
  return std::hypot(control.angular_x, control.angular_y, control.angular_z);
}

bool WithinBounds(const BodyVector& control,
                  const VehicleMotionPrimitiveBounds& bounds) {
  if (bounds.max_linear_speed_present) {
    if (!(LinearSpeed(control) <= bounds.max_linear_speed_m_s)) {
      return false;
    }
  }
  if (bounds.max_angular_speed_present) {
    if (!(AngularSpeed(control) <= bounds.max_angular_speed_rad_s)) {
      return false;
    }
  }
  return true;
}

struct AxisSpec {
  bool present;
  double level;
  MotionPrimitiveAxis axis;
  const char* token;
};

GenerateMotionPrimitivesResult Error(MotionPrimitiveSetError error) {
  GenerateMotionPrimitivesResult result;
  result.error = error;
  return result;
}

}  // namespace

GenerateMotionPrimitivesResult GenerateMotionPrimitives(
    const VehicleMotionPrimitiveSetConfig& config) {
  if (!(std::isfinite(config.duration_s) && config.duration_s > 0.0)) {
    return Error(MotionPrimitiveSetError::kBadConfig);
  }
  if (!SpeedBoundValid(config.bounds.max_linear_speed_present,
                       config.bounds.max_linear_speed_m_s) ||
      !SpeedBoundValid(config.bounds.max_angular_speed_present,
                       config.bounds.max_angular_speed_rad_s)) {
    return Error(MotionPrimitiveSetError::kBadConfig);
  }

  const AxisSpec axes[] = {
      {config.surge_present, config.surge_m_s, MotionPrimitiveAxis::kSurge,
       "surge"},
      {config.sway_present, config.sway_m_s, MotionPrimitiveAxis::kSway,
       "sway"},
      {config.heave_present, config.heave_m_s, MotionPrimitiveAxis::kHeave,
       "heave"},
      {config.roll_present, config.roll_rad_s, MotionPrimitiveAxis::kRoll,
       "roll"},
      {config.pitch_present, config.pitch_rad_s, MotionPrimitiveAxis::kPitch,
       "pitch"},
      {config.yaw_present, config.yaw_rad_s, MotionPrimitiveAxis::kYaw, "yaw"},
  };
  for (const AxisSpec& axis : axes) {
    if (axis.present && !LevelValid(axis.level)) {
      return Error(MotionPrimitiveSetError::kBadConfig);
    }
  }

  GenerateMotionPrimitivesResult result;
  if (config.include_hover) {
    VehicleMotionPrimitive hover;
    hover.id = "hover";
    hover.duration_s = config.duration_s;
    result.primitives.push_back(hover);
  }
  for (const AxisSpec& axis : axes) {
    if (!axis.present) {
      continue;
    }
    VehicleMotionPrimitive positive;
    positive.id = std::string(axis.token) + "_pos";
    positive.control = ControlOnAxis(axis.axis, axis.level);
    positive.duration_s = config.duration_s;
    result.primitives.push_back(positive);

    VehicleMotionPrimitive negative;
    negative.id = std::string(axis.token) + "_neg";
    negative.control = ControlOnAxis(axis.axis, -axis.level);
    negative.duration_s = config.duration_s;
    result.primitives.push_back(negative);
  }

  for (const VehicleMotionPrimitive& primitive : result.primitives) {
    if (!WithinBounds(primitive.control, config.bounds)) {
      return Error(MotionPrimitiveSetError::kBoundsViolation);
    }
  }
  return result;
}

}  // namespace intrinsic::motion_planning::vehicle
