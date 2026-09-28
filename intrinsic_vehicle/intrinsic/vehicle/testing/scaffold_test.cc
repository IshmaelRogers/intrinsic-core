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

#include "intrinsic/vehicle/testing/scaffold.h"

#include <string_view>

#include "gtest/gtest.h"

namespace {

TEST(VehicleScaffold, CompilesEveryPackage) {
  EXPECT_EQ(std::string_view(intrinsic::vehicle::testing::kScaffoldPackages),
            std::string_view(
                "allocation,control,dynamics,guidance,parameters,state"));
  EXPECT_EQ(intrinsic::vehicle::allocation::kPackageName,
            std::string_view("intrinsic_vehicle/intrinsic/vehicle/allocation"));
  EXPECT_EQ(intrinsic::vehicle::control::kPackageName,
            std::string_view("intrinsic_vehicle/intrinsic/vehicle/control"));
  EXPECT_EQ(intrinsic::vehicle::dynamics::kPackageName,
            std::string_view("intrinsic_vehicle/intrinsic/vehicle/dynamics"));
  EXPECT_EQ(intrinsic::vehicle::guidance::kPackageName,
            std::string_view("intrinsic_vehicle/intrinsic/vehicle/guidance"));
  EXPECT_EQ(intrinsic::vehicle::parameters::kPackageName,
            std::string_view("intrinsic_vehicle/intrinsic/vehicle/parameters"));
  EXPECT_EQ(intrinsic::vehicle::state::kPackageName,
            std::string_view("intrinsic_vehicle/intrinsic/vehicle/state"));
}

}  // namespace
