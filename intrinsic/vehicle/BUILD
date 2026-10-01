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

# Opt-in vehicle contract helpers. Not part of the protected manipulator
# baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_contract_policy",
    hdrs = ["vehicle_contract_policy.h"],
    deps = ["//intrinsic/embodiment:frame_policy"],
)

cc_test(
    name = "vehicle_contract_test",
    size = "small",
    srcs = ["vehicle_contract_test.cc"],
    deps = [
        ":vehicle_contract_policy",
        "//intrinsic/embodiment:capability_policy",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vehicle_contract_serialization_test",
    size = "small",
    srcs = ["vehicle_contract_serialization_test.cc"],
    data = ["@intrinsic_apis//intrinsic/vehicle/proto:examples"],
    deps = [
        ":vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

py_library(
    name = "vehicle_contract_policy_py",
    srcs = ["vehicle_contract_policy.py"],
    deps = ["//intrinsic/embodiment:frame_policy_py"],
)

py_test(
    name = "vehicle_contract_test_py",
    size = "small",
    srcs = ["vehicle_contract_test.py"],
    main = "vehicle_contract_test.py",
    deps = [
        ":vehicle_contract_policy_py",
        "//intrinsic/embodiment:capability_policy_py",
    ],
)

py_test(
    name = "vehicle_contract_serialization_test_py",
    size = "small",
    srcs = ["vehicle_contract_serialization_test.py"],
    data = ["@intrinsic_apis//intrinsic/vehicle/proto:examples"],
    main = "vehicle_contract_serialization_test.py",
    deps = [
        ":vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_py_pb2",
    ],
)

cc_library(
    name = "trajectory_contract_policy",
    hdrs = ["trajectory_contract_policy.h"],
    deps = [
        ":vehicle_contract_policy",
        "//intrinsic/embodiment:frame_policy",
    ],
)

cc_test(
    name = "trajectory_contract_test",
    size = "small",
    srcs = ["trajectory_contract_test.cc"],
    deps = [
        ":trajectory_contract_policy",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "trajectory_contract_serialization_test",
    size = "small",
    srcs = ["trajectory_contract_serialization_test.cc"],
    data = ["@intrinsic_apis//intrinsic/vehicle/proto:examples"],
    deps = [
        ":trajectory_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_trajectory_cc_proto",
    ],
)

py_library(
    name = "trajectory_contract_policy_py",
    srcs = ["trajectory_contract_policy.py"],
    deps = [
        ":vehicle_contract_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
    ],
)

py_test(
    name = "trajectory_contract_test_py",
    size = "small",
    srcs = ["trajectory_contract_test.py"],
    main = "trajectory_contract_test.py",
    deps = [
        ":trajectory_contract_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
    ],
)

py_test(
    name = "trajectory_contract_serialization_test_py",
    size = "small",
    srcs = ["trajectory_contract_serialization_test.py"],
    data = ["@intrinsic_apis//intrinsic/vehicle/proto:examples"],
    main = "trajectory_contract_serialization_test.py",
    deps = [
        ":trajectory_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_trajectory_py_pb2",
    ],
)
