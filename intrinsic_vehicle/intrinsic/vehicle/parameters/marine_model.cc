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

#include "intrinsic/vehicle/parameters/marine_model.h"

#include <cmath>
#include <string>
#include <string_view>
#include <vector>

namespace intrinsic::vehicle::parameters {
namespace {

FieldError Error(std::string field, ModelErrorCode code, std::string message) {
  return FieldError{std::move(field), code, std::move(message)};
}

std::string Entry(std::string_view field, int row, int col) {
  return std::string(field) + "[" + std::to_string(row) + "," +
         std::to_string(col) + "]";
}

std::string Indexed(std::string_view field, int index) {
  return std::string(field) + "[" + std::to_string(index) + "]";
}

std::string ThrusterField(int index, std::string_view suffix) {
  return "thrusters[" + std::to_string(index) + "]." + std::string(suffix);
}

void AppendStrictlyPositive(std::vector<FieldError>* errors,
                            std::string_view field, double value) {
  if (!std::isfinite(value)) {
    errors->push_back(Error(std::string(field), ModelErrorCode::kNonFinite,
                            "must be finite"));
  } else if (!(value > 0.0)) {
    errors->push_back(Error(std::string(field), ModelErrorCode::kNotPositive,
                            "must be greater than zero"));
  }
}

void AppendNonNegative(std::vector<FieldError>* errors, std::string_view field,
                       double value) {
  if (!std::isfinite(value)) {
    errors->push_back(Error(std::string(field), ModelErrorCode::kNonFinite,
                            "must be finite"));
  } else if (value < 0.0) {
    errors->push_back(Error(std::string(field), ModelErrorCode::kNegative,
                            "must be greater than or equal to zero"));
  }
}

bool ComponentFinite(std::vector<FieldError>* errors, std::string_view field,
                     double value) {
  if (std::isfinite(value)) {
    return true;
  }
  errors->push_back(
      Error(std::string(field), ModelErrorCode::kNonFinite, "must be finite"));
  return false;
}

bool AppendVec3Finite(std::vector<FieldError>* errors, std::string_view field,
                      Vec3 value) {
  const double components[] = {value.x, value.y, value.z};
  const char* names[] = {"x", "y", "z"};
  bool finite = true;
  for (int i = 0; i < 3; ++i) {
    const std::string component = std::string(field) + "." + names[i];
    if (!ComponentFinite(errors, component, components[i])) {
      finite = false;
    }
  }
  return finite;
}

// Unpivoted Cholesky. `a` is row-major and symmetric. A pivot that is not
// strictly positive fails. n is 3 or 6.
bool IsPositiveDefinite(const double* a, int n, int* failed_pivot) {
  double factor[kSpatialDof * kSpatialDof] = {};
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j <= i; ++j) {
      double sum = a[i * n + j];
      for (int k = 0; k < j; ++k) {
        sum -= factor[i * n + k] * factor[j * n + k];
      }
      if (i == j) {
        if (!(sum > 0.0)) {
          *failed_pivot = i;
          return false;
        }
        factor[i * n + i] = std::sqrt(sum);
      } else {
        factor[i * n + j] = sum / factor[j * n + j];
      }
    }
  }
  return true;
}

void AppendMatrixErrors(std::vector<FieldError>* errors, std::string_view field,
                        const double* values, int n) {
  bool finite = true;
  for (int row = 0; row < n; ++row) {
    for (int col = 0; col < n; ++col) {
      if (!std::isfinite(values[row * n + col])) {
        finite = false;
        errors->push_back(Error(Entry(field, row, col),
                                ModelErrorCode::kNonFinite, "must be finite"));
      }
    }
  }
  if (!finite) {
    return;
  }

  bool symmetric = true;
  for (int row = 0; row < n; ++row) {
    for (int col = row + 1; col < n; ++col) {
      const double upper = values[row * n + col];
      const double lower = values[col * n + row];
      if (std::abs(upper - lower) > kMatrixSymmetryTolerance) {
        symmetric = false;
        errors->push_back(
            Error(Entry(field, row, col), ModelErrorCode::kAsymmetric,
                  "differs from the transpose by more than 1e-9"));
      }
    }
  }
  if (!symmetric) {
    return;
  }

  double symmetric_values[kSpatialDof * kSpatialDof] = {};
  for (int row = 0; row < n; ++row) {
    symmetric_values[row * n + row] = values[row * n + row];
    for (int col = row + 1; col < n; ++col) {
      const double upper = values[row * n + col];
      symmetric_values[row * n + col] = upper;
      symmetric_values[col * n + row] = upper;
    }
  }
  int pivot = -1;
  if (!IsPositiveDefinite(symmetric_values, n, &pivot)) {
    errors->push_back(
        Error(std::string(field), ModelErrorCode::kNotPositiveDefinite,
              "symmetric matrix is not positive definite; Cholesky pivot " +
                  std::to_string(pivot) + " is not positive"));
  }
}

bool AllowedCurrentFrame(std::string_view frame_id) {
  return frame_id == kWorldEnuFrameId || frame_id == kWorldNedFrameId ||
         frame_id == kBodyFrameId;
}

void AppendThrusterErrors(std::vector<FieldError>* errors,
                          const MarineModel& model) {
  const int count = static_cast<int>(model.thrusters.size());
  for (int i = 0; i < count; ++i) {
    const ThrusterGeometry& thruster = model.thrusters[i];
    if (thruster.name.empty()) {
      errors->push_back(Error(ThrusterField(i, "name"), ModelErrorCode::kEmpty,
                              "must be non-empty"));
    } else {
      for (int earlier = 0; earlier < i; ++earlier) {
        if (model.thrusters[earlier].name == thruster.name) {
          errors->push_back(Error(ThrusterField(i, "name"),
                                  ModelErrorCode::kDuplicate,
                                  "duplicates an earlier thruster name"));
          break;
        }
      }
    }

    AppendVec3Finite(errors, ThrusterField(i, "position_m"),
                     thruster.position_m);
    const bool direction_finite = AppendVec3Finite(
        errors, ThrusterField(i, "direction_body"), thruster.direction_body);
    if (direction_finite) {
      const double norm =
          std::sqrt((thruster.direction_body.x * thruster.direction_body.x) +
                    (thruster.direction_body.y * thruster.direction_body.y) +
                    (thruster.direction_body.z * thruster.direction_body.z));
      if (!std::isfinite(norm)) {
        errors->push_back(Error(ThrusterField(i, "direction_body"),
                                ModelErrorCode::kNonFinite, "must be finite"));
      } else if (std::abs(norm - 1.0) > kUnitVectorTolerance) {
        errors->push_back(Error(ThrusterField(i, "direction_body"),
                                ModelErrorCode::kNotUnit,
                                "must be a unit vector"));
      }
    }

    bool bounds_comparable = true;
    if (!std::isfinite(thruster.max_forward_thrust_n)) {
      bounds_comparable = false;
      errors->push_back(Error(ThrusterField(i, "max_forward_thrust_n"),
                              ModelErrorCode::kNonFinite, "must be finite"));
    } else if (thruster.max_forward_thrust_n < 0.0) {
      bounds_comparable = false;
      errors->push_back(Error(ThrusterField(i, "max_forward_thrust_n"),
                              ModelErrorCode::kNegative,
                              "must be greater than or equal to zero"));
    }
    if (!std::isfinite(thruster.max_reverse_thrust_n)) {
      bounds_comparable = false;
      errors->push_back(Error(ThrusterField(i, "max_reverse_thrust_n"),
                              ModelErrorCode::kNonFinite, "must be finite"));
    } else if (thruster.max_reverse_thrust_n < 0.0) {
      bounds_comparable = false;
      errors->push_back(Error(ThrusterField(i, "max_reverse_thrust_n"),
                              ModelErrorCode::kNegative,
                              "must be greater than or equal to zero"));
    }
    if (bounds_comparable && thruster.max_forward_thrust_n == 0.0 &&
        thruster.max_reverse_thrust_n == 0.0) {
      errors->push_back(
          Error(ThrusterField(i, "thrust_bounds"), ModelErrorCode::kNotPositive,
                "forward and reverse thrust bounds are both zero"));
    }
  }
}

}  // namespace

ValidationResult ValidateMarineModel(const MarineModel& model) {
  ValidationResult result;
  std::vector<FieldError>* errors = &result.errors;

  if (model.model_id.empty()) {
    errors->push_back(
        Error("model_id", ModelErrorCode::kEmpty, "must be non-empty"));
  }

  AppendStrictlyPositive(errors, "mass_inertia.mass_kg",
                         model.mass_inertia.mass_kg);
  AppendMatrixErrors(
      errors, "mass_inertia.inertia_about_center_of_gravity_kg_m2",
      model.mass_inertia.inertia_about_center_of_gravity_kg_m2.data(),
      kInertiaDof);

  AppendVec3Finite(errors, "centers.center_of_gravity_m",
                   model.centers.center_of_gravity_m);
  AppendVec3Finite(errors, "centers.center_of_buoyancy_m",
                   model.centers.center_of_buoyancy_m);

  AppendStrictlyPositive(errors, "buoyancy.displaced_volume_m3",
                         model.buoyancy.displaced_volume_m3);

  AppendMatrixErrors(errors, "added_mass.coefficients",
                     model.added_mass.coefficients.data(), kSpatialDof);

  AppendMatrixErrors(errors, "damping.linear_coefficients",
                     model.damping.linear_coefficients.data(), kSpatialDof);
  for (int i = 0; i < kSpatialDof; ++i) {
    AppendNonNegative(errors, Indexed("damping.quadratic_coefficients", i),
                      model.damping.quadratic_coefficients[i]);
  }

  AppendStrictlyPositive(errors, "environment.gravity_m_s2",
                         model.environment.gravity_m_s2);
  AppendStrictlyPositive(errors, "environment.fluid_density_kg_m3",
                         model.environment.fluid_density_kg_m3);
  AppendVec3Finite(errors, "environment.current_velocity_m_s",
                   model.environment.current_velocity_m_s);
  if (model.environment.current_frame_id.empty()) {
    errors->push_back(Error("environment.current_frame_id",
                            ModelErrorCode::kEmpty, "must be non-empty"));
  } else if (!AllowedCurrentFrame(model.environment.current_frame_id)) {
    errors->push_back(Error("environment.current_frame_id",
                            ModelErrorCode::kFrame,
                            "must be world_enu, world_ned, or body"));
  }

  AppendThrusterErrors(errors, model);
  return result;
}

}  // namespace intrinsic::vehicle::parameters
