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

# Opt-in estimation service (intrinsic_estimation). Not part of the protected
# manipulator baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "estimator_proto",
    srcs = ["estimator.proto"],
    deps = [
        "@com_google_protobuf//:duration_proto",
        "@com_google_protobuf//:timestamp_proto",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_proto",
    ],
)

cc_proto_library(
    name = "estimator_cc_proto",
    deps = [":estimator_proto"],
)

py_proto_library(
    name = "estimator_py_pb2",
    deps = [":estimator_proto"],
)

cc_library(
    name = "estimator_service",
    hdrs = ["estimator_service.h"],
    deps = [
        ":estimator_cc_proto",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

cc_library(
    name = "fake_estimator_service",
    srcs = ["fake_estimator_service.cc"],
    hdrs = ["fake_estimator_service.h"],
    deps = [
        ":estimator_cc_proto",
        ":estimator_service",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

cc_test(
    name = "fake_estimator_service_test",
    size = "small",
    srcs = ["fake_estimator_service_test.cc"],
    data = ["testdata/nominal_vehicle_state.textproto"],
    deps = [
        ":estimator_cc_proto",
        ":fake_estimator_service",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

py_library(
    name = "estimator_service_py",
    srcs = ["estimator_service.py"],
    deps = [
        ":estimator_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_py_pb2",
    ],
)

py_library(
    name = "fake_estimator_service_py",
    srcs = ["fake_estimator_service.py"],
    deps = [
        ":estimator_py_pb2",
        ":estimator_service_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_py_pb2",
    ],
)

py_test(
    name = "fake_estimator_service_test_py",
    size = "small",
    srcs = ["fake_estimator_service_test.py"],
    data = ["testdata/nominal_vehicle_state.textproto"],
    main = "fake_estimator_service_test.py",
    deps = [
        ":estimator_py_pb2",
        ":fake_estimator_service_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_py_pb2",
    ],
)

cc_library(
    name = "eskf_state",
    srcs = ["eskf_state.cc"],
    hdrs = ["eskf_state.h"],
)

cc_test(
    name = "eskf_state_test",
    size = "small",
    srcs = ["eskf_state_test.cc"],
    deps = [
        ":eskf_state",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "eskf_state_py",
    srcs = ["eskf_state.py"],
)

py_test(
    name = "eskf_state_test_py",
    size = "small",
    srcs = ["eskf_state_test.py"],
    main = "eskf_state_test.py",
    deps = [":eskf_state_py"],
)

cc_library(
    name = "eskf_propagate",
    srcs = ["eskf_propagate.cc"],
    hdrs = ["eskf_propagate.h"],
    deps = [":eskf_state"],
)

cc_test(
    name = "eskf_propagate_test",
    size = "small",
    srcs = ["eskf_propagate_test.cc"],
    deps = [
        ":eskf_propagate",
        ":eskf_state",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "eskf_propagate_py",
    srcs = ["eskf_propagate.py"],
    deps = [":eskf_state_py"],
)

py_test(
    name = "eskf_propagate_test_py",
    size = "small",
    srcs = ["eskf_propagate_test.py"],
    main = "eskf_propagate_test.py",
    deps = [
        ":eskf_propagate_py",
        ":eskf_state_py",
    ],
)

cc_library(
    name = "eskf_cov_propagate",
    srcs = ["eskf_cov_propagate.cc"],
    hdrs = ["eskf_cov_propagate.h"],
    deps = [
        ":eskf_propagate",
        ":eskf_state",
    ],
)

cc_test(
    name = "eskf_cov_propagate_test",
    size = "small",
    srcs = ["eskf_cov_propagate_test.cc"],
    deps = [
        ":eskf_cov_propagate",
        ":eskf_propagate",
        ":eskf_state",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "eskf_cov_propagate_py",
    srcs = ["eskf_cov_propagate.py"],
    deps = [
        ":eskf_propagate_py",
        ":eskf_state_py",
    ],
)

py_test(
    name = "eskf_cov_propagate_test_py",
    size = "small",
    srcs = ["eskf_cov_propagate_test.py"],
    main = "eskf_cov_propagate_test.py",
    deps = [
        ":eskf_cov_propagate_py",
        ":eskf_propagate_py",
        ":eskf_state_py",
    ],
)
