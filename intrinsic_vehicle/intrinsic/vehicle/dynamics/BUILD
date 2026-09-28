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

# Opt-in vehicle dynamics interface and zero-force test double. Not part of
# the protected manipulator baseline. Do not add this package to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "package_files",
    srcs = [
        "BUILD",
        "dynamics.h",
        "vehicle_dynamics.cc",
        "vehicle_dynamics.h",
        "zero_force_dynamics.cc",
        "zero_force_dynamics.h",
    ],
)

cc_library(
    name = "dynamics",
    srcs = [
        "vehicle_dynamics.cc",
        "zero_force_dynamics.cc",
    ],
    hdrs = [
        "dynamics.h",
        "vehicle_dynamics.h",
        "zero_force_dynamics.h",
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
    name = "vehicle_dynamics_test",
    size = "small",
    srcs = ["vehicle_dynamics_test.cc"],
    deps = [
        ":dynamics",
        "@com_google_googletest//:gtest_main",
    ],
)
