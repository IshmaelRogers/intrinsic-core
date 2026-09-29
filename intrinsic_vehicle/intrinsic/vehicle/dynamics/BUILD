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

load("//bazel:cc_macros.bzl", "cc_library", "cc_test")

# Opt-in vehicle dynamics interface, zero-force test double, rigid-body
# mass matrix, added-mass matrix, rigid-body Coriolis matrix, added-mass
# Coriolis matrix, linear/quadratic damping wrench, and gravity/buoyancy
# restoring wrench. Not part of the protected manipulator baseline. Do not
# add this package to .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "package_files",
    srcs = [
        "BUILD",
        "added_mass_coriolis_matrix.cc",
        "added_mass_coriolis_matrix.h",
        "added_mass_matrix.cc",
        "added_mass_matrix.h",
        "dynamics.h",
        "gravity_buoyancy_restoring_wrench.cc",
        "gravity_buoyancy_restoring_wrench.h",
        "linear_quadratic_damping_wrench.cc",
        "linear_quadratic_damping_wrench.h",
        "rigid_body_coriolis_matrix.cc",
        "rigid_body_coriolis_matrix.h",
        "rigid_body_mass_matrix.cc",
        "rigid_body_mass_matrix.h",
        "vehicle_dynamics.cc",
        "vehicle_dynamics.h",
        "zero_force_dynamics.cc",
        "zero_force_dynamics.h",
    ],
)

cc_library(
    name = "dynamics",
    srcs = [
        "added_mass_coriolis_matrix.cc",
        "added_mass_matrix.cc",
        "gravity_buoyancy_restoring_wrench.cc",
        "linear_quadratic_damping_wrench.cc",
        "rigid_body_coriolis_matrix.cc",
        "rigid_body_mass_matrix.cc",
        "vehicle_dynamics.cc",
        "zero_force_dynamics.cc",
    ],
    hdrs = [
        "added_mass_coriolis_matrix.h",
        "added_mass_matrix.h",
        "dynamics.h",
        "gravity_buoyancy_restoring_wrench.h",
        "linear_quadratic_damping_wrench.h",
        "rigid_body_coriolis_matrix.h",
        "rigid_body_mass_matrix.h",
        "vehicle_dynamics.h",
        "zero_force_dynamics.h",
    ],
    deps = [
        "//intrinsic_vehicle/intrinsic/vehicle/parameters",
    ],
)

cc_test(
    name = "added_mass_coriolis_matrix_test",
    size = "small",
    srcs = ["added_mass_coriolis_matrix_test.cc"],
    deps = [
        ":dynamics",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "added_mass_matrix_test",
    size = "small",
    srcs = ["added_mass_matrix_test.cc"],
    deps = [
        ":dynamics",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "dynamics_test",
    size = "small",
    srcs = ["dynamics_test.cc"],
    deps = [
        ":dynamics",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "gravity_buoyancy_restoring_wrench_test",
    size = "small",
    srcs = ["gravity_buoyancy_restoring_wrench_test.cc"],
    deps = [
        ":dynamics",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "linear_quadratic_damping_wrench_test",
    size = "small",
    srcs = ["linear_quadratic_damping_wrench_test.cc"],
    deps = [
        ":dynamics",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "rigid_body_coriolis_matrix_test",
    size = "small",
    srcs = ["rigid_body_coriolis_matrix_test.cc"],
    deps = [
        ":dynamics",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "rigid_body_mass_matrix_test",
    size = "small",
    srcs = ["rigid_body_mass_matrix_test.cc"],
    deps = [
        ":dynamics",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vehicle_dynamics_test",
    size = "small",
    srcs = ["vehicle_dynamics_test.cc"],
    deps = [
        ":dynamics",
        "@com_google_googletest//:gtest_main",
    ],
)
