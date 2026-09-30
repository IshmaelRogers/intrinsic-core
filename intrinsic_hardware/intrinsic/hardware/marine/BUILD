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

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("//bazel:cc_macros.bzl", "cc_library", "cc_test")
load("//bazel:protos.bzl", "proto_library")
load("//bazel:python.bzl", "py_library", "py_test")

# Opt-in marine measurement health. Not part of the protected manipulator
# baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "measurement_health_proto",
    srcs = ["measurement_health.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_proto",
    ],
)

cc_proto_library(
    name = "measurement_health_cc_proto",
    deps = [":measurement_health_proto"],
)

py_proto_library(
    name = "measurement_health_py_pb2",
    deps = [":measurement_health_proto"],
)

cc_library(
    name = "measurement_health_policy",
    hdrs = ["measurement_health_policy.h"],
    deps = [
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_test(
    name = "measurement_health_policy_test",
    size = "small",
    srcs = ["measurement_health_policy_test.cc"],
    deps = [
        ":measurement_health_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "measurement_health_serialization_test",
    size = "small",
    srcs = ["measurement_health_serialization_test.cc"],
    data = [":examples"],
    deps = [
        ":measurement_health_cc_proto",
        ":measurement_health_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

py_library(
    name = "measurement_health_policy_py",
    srcs = ["measurement_health_policy.py"],
    deps = [
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "measurement_health_policy_test_py",
    size = "small",
    srcs = ["measurement_health_policy_test.py"],
    main = "measurement_health_policy_test.py",
    deps = [
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "measurement_health_serialization_test_py",
    size = "small",
    srcs = ["measurement_health_serialization_test.py"],
    data = [":examples"],
    main = "measurement_health_serialization_test.py",
    deps = [
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_py_pb2",
    ],
)

filegroup(
    name = "examples",
    srcs = [
        "examples/measurement_health.textproto",
        "examples/measurement_health_unknown_covariance.textproto",
    ],
)
