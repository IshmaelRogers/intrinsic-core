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

# Opt-in safety decision and authority transition helpers. Not part of the protected manipulator
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

cc_library(
    name = "authority_transition_policy",
    hdrs = ["authority_transition_policy.h"],
    deps = [":safety_decision_policy"],
)

cc_library(
    name = "authority_transition_assessor",
    srcs = ["authority_transition_assessor.cc"],
    hdrs = ["authority_transition_assessor.h"],
    deps = [
        ":authority_transition_policy",
        ":safety_decision_policy",
        "@intrinsic_apis//intrinsic/safety/proto:authority_mode_cc_proto",
        "@intrinsic_apis//intrinsic/safety/proto:safety_decision_cc_proto",
    ],
)

cc_test(
    name = "authority_transition_policy_test",
    size = "small",
    srcs = ["authority_transition_policy_test.cc"],
    deps = [
        ":authority_transition_policy",
        ":safety_decision_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "authority_transition_assessor_test",
    size = "small",
    srcs = ["authority_transition_assessor_test.cc"],
    deps = [
        ":authority_transition_assessor",
        ":authority_transition_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/safety/proto:authority_mode_cc_proto",
        "@intrinsic_apis//intrinsic/safety/proto:safety_decision_cc_proto",
    ],
)

py_library(
    name = "authority_transition_policy_py",
    srcs = ["authority_transition_policy.py"],
    deps = [":safety_decision_policy_py"],
)

py_library(
    name = "authority_transition_assessor_py",
    srcs = ["authority_transition_assessor.py"],
    deps = [
        ":authority_transition_policy_py",
        ":safety_decision_policy_py",
    ],
)

py_test(
    name = "authority_transition_policy_test_py",
    size = "small",
    srcs = ["authority_transition_policy_test.py"],
    main = "authority_transition_policy_test.py",
    deps = [
        ":authority_transition_policy_py",
        ":safety_decision_policy_py",
    ],
)

py_test(
    name = "authority_transition_assessor_test_py",
    size = "small",
    srcs = ["authority_transition_assessor_test.py"],
    data = ["@intrinsic_apis//intrinsic/safety/proto:examples"],
    main = "authority_transition_assessor_test.py",
    deps = [
        ":authority_transition_assessor_py",
        ":authority_transition_policy_py",
        "@intrinsic_apis//intrinsic/safety/proto:authority_mode_py_pb2",
        "@intrinsic_apis//intrinsic/safety/proto:safety_decision_py_pb2",
    ],
)

cc_library(
    name = "safety_rule_result",
    hdrs = ["safety_rule_result.h"],
)

cc_library(
    name = "state_age_rule",
    srcs = ["state_age_rule.cc"],
    hdrs = ["state_age_rule.h"],
    deps = [
        ":safety_rule_result",
        "//intrinsic/embodiment:frame_policy",
    ],
)

cc_library(
    name = "non_finite_rule",
    srcs = ["non_finite_rule.cc"],
    hdrs = ["non_finite_rule.h"],
    deps = [
        ":safety_rule_result",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_test(
    name = "state_age_rule_test",
    size = "small",
    srcs = ["state_age_rule_test.cc"],
    deps = [
        ":safety_decision_policy",
        ":state_age_rule",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "non_finite_rule_test",
    size = "small",
    srcs = ["non_finite_rule_test.cc"],
    deps = [
        ":non_finite_rule",
        ":safety_decision_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "safety_rule_result_py",
    srcs = ["safety_rule_result.py"],
)

py_library(
    name = "state_age_rule_py",
    srcs = ["state_age_rule.py"],
    deps = [
        ":safety_rule_result_py",
        "//intrinsic/embodiment:frame_policy_py",
    ],
)

py_library(
    name = "non_finite_rule_py",
    srcs = ["non_finite_rule.py"],
    deps = [
        ":safety_rule_result_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "state_age_rule_test_py",
    size = "small",
    srcs = ["state_age_rule_test.py"],
    main = "state_age_rule_test.py",
    deps = [
        ":safety_decision_policy_py",
        ":safety_rule_result_py",
        ":state_age_rule_py",
    ],
)

py_test(
    name = "non_finite_rule_test_py",
    size = "small",
    srcs = ["non_finite_rule_test.py"],
    main = "non_finite_rule_test.py",
    deps = [
        ":non_finite_rule_py",
        ":safety_decision_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

cc_library(
    name = "depth_max_rule",
    srcs = ["depth_max_rule.cc"],
    hdrs = ["depth_max_rule.h"],
    deps = [
        ":safety_rule_result",
        "//intrinsic/embodiment:frame_policy",
    ],
)

cc_test(
    name = "depth_max_rule_test",
    size = "small",
    srcs = ["depth_max_rule_test.cc"],
    deps = [
        ":depth_max_rule",
        ":safety_decision_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "depth_max_rule_py",
    srcs = ["depth_max_rule.py"],
    deps = [":safety_rule_result_py"],
)

py_test(
    name = "depth_max_rule_test_py",
    size = "small",
    srcs = ["depth_max_rule_test.py"],
    main = "depth_max_rule_test.py",
    deps = [
        ":depth_max_rule_py",
        ":safety_decision_policy_py",
        ":safety_rule_result_py",
    ],
)

cc_library(
    name = "altitude_min_rule",
    srcs = ["altitude_min_rule.cc"],
    hdrs = ["altitude_min_rule.h"],
    deps = [
        ":safety_rule_result",
        "//intrinsic/embodiment:frame_policy",
    ],
)

cc_test(
    name = "altitude_min_rule_test",
    size = "small",
    srcs = ["altitude_min_rule_test.cc"],
    deps = [
        ":altitude_min_rule",
        ":safety_decision_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "altitude_min_rule_py",
    srcs = ["altitude_min_rule.py"],
    deps = [":safety_rule_result_py"],
)

py_test(
    name = "altitude_min_rule_test_py",
    size = "small",
    srcs = ["altitude_min_rule_test.py"],
    main = "altitude_min_rule_test.py",
    deps = [
        ":altitude_min_rule_py",
        ":safety_decision_policy_py",
        ":safety_rule_result_py",
    ],
)
