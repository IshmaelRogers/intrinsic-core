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

proto_library(
    name = "dvl_proto",
    srcs = ["dvl.proto"],
    deps = [":measurement_health_proto"],
)

cc_proto_library(
    name = "dvl_cc_proto",
    deps = [":dvl_proto"],
)

py_proto_library(
    name = "dvl_py_pb2",
    deps = [":dvl_proto"],
)

cc_library(
    name = "dvl_policy",
    hdrs = ["dvl_policy.h"],
    deps = [
        ":measurement_health_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_library(
    name = "fake_dvl",
    srcs = ["fake_dvl.cc"],
    hdrs = ["fake_dvl.h"],
    deps = [
        ":dvl_cc_proto",
        ":measurement_health_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

cc_test(
    name = "dvl_policy_test",
    size = "small",
    srcs = ["dvl_policy_test.cc"],
    deps = [
        ":dvl_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "dvl_serialization_test",
    size = "small",
    srcs = ["dvl_serialization_test.cc"],
    data = [":examples"],
    deps = [
        ":dvl_cc_proto",
        ":dvl_policy",
        ":fake_dvl",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

cc_test(
    name = "fake_dvl_test",
    size = "small",
    srcs = ["fake_dvl_test.cc"],
    deps = [
        ":dvl_cc_proto",
        ":dvl_policy",
        ":fake_dvl",
        ":measurement_health_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "dvl_policy_py",
    srcs = ["dvl_policy.py"],
    deps = [
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_library(
    name = "fake_dvl_py",
    srcs = ["fake_dvl.py"],
    deps = [
        ":dvl_py_pb2",
        ":measurement_health_py_pb2",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "dvl_policy_test_py",
    size = "small",
    srcs = ["dvl_policy_test.py"],
    main = "dvl_policy_test.py",
    deps = [
        ":dvl_policy_py",
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "dvl_serialization_test_py",
    size = "small",
    srcs = ["dvl_serialization_test.py"],
    data = [":examples"],
    main = "dvl_serialization_test.py",
    deps = [
        ":dvl_policy_py",
        ":dvl_py_pb2",
        ":fake_dvl_py",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "fake_dvl_test_py",
    size = "small",
    srcs = ["fake_dvl_test.py"],
    main = "fake_dvl_test.py",
    deps = [
        ":dvl_policy_py",
        ":dvl_py_pb2",
        ":fake_dvl_py",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
    ],
)

filegroup(
    name = "examples",
    srcs = [
        "examples/dvl_bottom_track.textproto",
        "examples/measurement_health.textproto",
        "examples/measurement_health_unknown_covariance.textproto",
    ],
)
