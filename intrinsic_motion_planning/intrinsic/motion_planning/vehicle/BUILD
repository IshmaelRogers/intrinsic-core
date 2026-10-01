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
load("//bazel:python.bzl", "py_library", "py_test")

# Opt-in vehicle StateSpace and planner registry. No search, sampling, or
# collision checking. Not part of the protected manipulator baseline. Do not
# add these labels to .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_state_space",
    srcs = ["vehicle_state_space.cc"],
    hdrs = ["vehicle_state_space.h"],
    deps = [
        "//intrinsic/eigenmath:interpolation",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_gitlab_libeigen_eigen//:eigen",
    ],
)

cc_test(
    name = "vehicle_state_space_test",
    size = "small",
    srcs = ["vehicle_state_space_test.cc"],
    deps = [
        ":vehicle_state_space",
        "//intrinsic/eigenmath:interpolation",
        "//intrinsic/embodiment:frame_policy",
        "@com_gitlab_libeigen_eigen//:eigen",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_state_space_py",
    srcs = ["vehicle_state_space.py"],
    deps = [
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "vehicle_state_space_test_py",
    size = "small",
    srcs = ["vehicle_state_space_test.py"],
    main = "vehicle_state_space_test.py",
    deps = [
        ":vehicle_state_space_py",
        "//intrinsic/embodiment:frame_policy_py",
    ],
)

cc_library(
    name = "vehicle_planner_registry",
    srcs = ["vehicle_planner_registry.cc"],
    hdrs = ["vehicle_planner_registry.h"],
    deps = ["//intrinsic/vehicle:trajectory_contract_policy"],
)

cc_library(
    name = "fake_vehicle_planner",
    srcs = ["fake_vehicle_planner.cc"],
    hdrs = ["fake_vehicle_planner.h"],
    deps = [
        ":vehicle_planner_registry",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:trajectory_contract_policy",
    ],
)

cc_test(
    name = "vehicle_planner_registry_test",
    size = "small",
    srcs = ["vehicle_planner_registry_test.cc"],
    deps = [
        ":fake_vehicle_planner",
        ":vehicle_planner_registry",
        "//intrinsic/vehicle:trajectory_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_planner_registry_py",
    srcs = ["vehicle_planner_registry.py"],
    deps = ["//intrinsic/vehicle:trajectory_contract_policy_py"],
)

py_library(
    name = "fake_vehicle_planner_py",
    srcs = ["fake_vehicle_planner.py"],
    deps = [
        ":vehicle_planner_registry_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:trajectory_contract_policy_py",
    ],
)

py_test(
    name = "vehicle_planner_registry_test_py",
    size = "small",
    srcs = ["vehicle_planner_registry_test.py"],
    main = "vehicle_planner_registry_test.py",
    deps = [
        ":fake_vehicle_planner_py",
        ":vehicle_planner_registry_py",
        "//intrinsic/vehicle:trajectory_contract_policy_py",
    ],
)

cc_library(
    name = "vehicle_planner_deadline",
    srcs = ["vehicle_planner_deadline.cc"],
    hdrs = ["vehicle_planner_deadline.h"],
    deps = [":vehicle_planner_registry"],
)

cc_test(
    name = "vehicle_planner_deadline_test",
    size = "small",
    srcs = ["vehicle_planner_deadline_test.cc"],
    deps = [
        ":fake_vehicle_planner",
        ":vehicle_planner_deadline",
        ":vehicle_planner_registry",
        "//intrinsic/vehicle:trajectory_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_planner_deadline_py",
    srcs = ["vehicle_planner_deadline.py"],
    deps = [":vehicle_planner_registry_py"],
)

py_test(
    name = "vehicle_planner_deadline_test_py",
    size = "small",
    srcs = ["vehicle_planner_deadline_test.py"],
    main = "vehicle_planner_deadline_test.py",
    deps = [
        ":fake_vehicle_planner_py",
        ":vehicle_planner_deadline_py",
        ":vehicle_planner_registry_py",
        "//intrinsic/vehicle:trajectory_contract_policy_py",
    ],
)

cc_library(
    name = "vehicle_motion_primitives",
    srcs = ["vehicle_motion_primitives.cc"],
    hdrs = ["vehicle_motion_primitives.h"],
    deps = ["//intrinsic/vehicle:vehicle_contract_policy"],
)

cc_test(
    name = "vehicle_motion_primitives_test",
    size = "small",
    srcs = ["vehicle_motion_primitives_test.cc"],
    deps = [
        ":vehicle_motion_primitives",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_motion_primitives_py",
    srcs = ["vehicle_motion_primitives.py"],
    deps = ["//intrinsic/vehicle:vehicle_contract_policy_py"],
)

py_test(
    name = "vehicle_motion_primitives_test_py",
    size = "small",
    srcs = ["vehicle_motion_primitives_test.py"],
    main = "vehicle_motion_primitives_test.py",
    deps = [":vehicle_motion_primitives_py"],
)
