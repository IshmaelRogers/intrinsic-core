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

# Opt-in vehicle allocation. The thruster effectiveness matrix, the
# unconstrained least-squares allocator, the bounded thrust clamp, and the
# thruster-health input adapter are the real-time helpers. Not part of the
# protected manipulator baseline.
# Do not add this package to .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "package_files",
    srcs = [
        "BUILD",
        "allocation.h",
        "bounded_allocator.cc",
        "bounded_allocator.h",
        "least_squares_allocator.cc",
        "least_squares_allocator.h",
        "thruster_effectiveness_matrix.cc",
        "thruster_effectiveness_matrix.h",
        "thruster_health_adapter.cc",
        "thruster_health_adapter.h",
    ],
)

cc_library(
    name = "allocation",
    srcs = [
        "bounded_allocator.cc",
        "least_squares_allocator.cc",
        "thruster_effectiveness_matrix.cc",
        "thruster_health_adapter.cc",
    ],
    hdrs = [
        "allocation.h",
        "bounded_allocator.h",
        "least_squares_allocator.h",
        "thruster_effectiveness_matrix.h",
        "thruster_health_adapter.h",
    ],
    deps = [
        "//intrinsic_vehicle/intrinsic/vehicle/parameters",
    ],
)

cc_test(
    name = "allocation_test",
    size = "small",
    srcs = ["allocation_test.cc"],
    deps = [
        ":allocation",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "bounded_allocator_test",
    size = "small",
    srcs = ["bounded_allocator_test.cc"],
    deps = [
        ":allocation",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "least_squares_allocator_test",
    size = "small",
    srcs = ["least_squares_allocator_test.cc"],
    deps = [
        ":allocation",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "thruster_effectiveness_matrix_test",
    size = "small",
    srcs = ["thruster_effectiveness_matrix_test.cc"],
    deps = [
        ":allocation",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "thruster_health_adapter_test",
    size = "small",
    srcs = ["thruster_health_adapter_test.cc"],
    deps = [
        ":allocation",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)
