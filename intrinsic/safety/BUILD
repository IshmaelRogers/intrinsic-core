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

# Opt-in safety decision helpers. Not part of the protected manipulator
# baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "safety_decision_policy",
    hdrs = ["safety_decision_policy.h"],
    deps = [
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_library(
    name = "safety_decision_assessor",
    srcs = ["safety_decision_assessor.cc"],
    hdrs = ["safety_decision_assessor.h"],
    deps = [
        ":safety_decision_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@boringssl//:crypto",
        "@intrinsic_apis//intrinsic/safety/proto:safety_decision_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_cc_proto",
    ],
)

cc_test(
    name = "safety_decision_policy_test",
    size = "small",
    srcs = ["safety_decision_policy_test.cc"],
    deps = [
        ":safety_decision_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "safety_decision_assessor_test",
    size = "small",
    srcs = ["safety_decision_assessor_test.cc"],
    deps = [
        ":safety_decision_assessor",
        ":safety_decision_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "//intrinsic/world/world_snapshot:world_snapshot_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/safety/proto:safety_decision_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_cc_proto",
    ],
)

cc_test(
    name = "safety_decision_serialization_test",
    size = "small",
    srcs = ["safety_decision_serialization_test.cc"],
    data = ["@intrinsic_apis//intrinsic/safety/proto:examples"],
    deps = [
        ":safety_decision_assessor",
        ":safety_decision_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/safety/proto:safety_decision_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_cc_proto",
    ],
)

py_library(
    name = "safety_decision_policy_py",
    srcs = ["safety_decision_policy.py"],
    deps = [
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_library(
    name = "safety_decision_assessor_py",
    srcs = ["safety_decision_assessor.py"],
    deps = [
        ":safety_decision_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/safety/proto:safety_decision_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_py_pb2",
    ],
)

py_test(
    name = "safety_decision_policy_test_py",
    size = "small",
    srcs = ["safety_decision_policy_test.py"],
    main = "safety_decision_policy_test.py",
    deps = [
        ":safety_decision_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "safety_decision_assessor_test_py",
    size = "small",
    srcs = ["safety_decision_assessor_test.py"],
    main = "safety_decision_assessor_test.py",
    deps = [
        ":safety_decision_assessor_py",
        ":safety_decision_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "//intrinsic/world/world_snapshot:world_snapshot_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
        "@intrinsic_apis//intrinsic/safety/proto:safety_decision_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_py_pb2",
    ],
)

py_test(
    name = "safety_decision_serialization_test_py",
    size = "small",
    srcs = ["safety_decision_serialization_test.py"],
    data = ["@intrinsic_apis//intrinsic/safety/proto:examples"],
    main = "safety_decision_serialization_test.py",
    deps = [
        ":safety_decision_assessor_py",
        ":safety_decision_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
        "@intrinsic_apis//intrinsic/safety/proto:safety_decision_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_py_pb2",
    ],
)
