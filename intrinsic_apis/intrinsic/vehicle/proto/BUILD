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
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@io_bazel_rules_go//proto:def.bzl", "go_proto_library")

# Opt-in vehicle contracts. Not part of the protected manipulator baseline.

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "vehicle_state_proto",
    srcs = ["vehicle_state.proto"],
    deps = [
        "//intrinsic/embodiment/proto:stamped_header_proto",
        "//intrinsic/math/proto:pose_proto",
    ],
)

cc_proto_library(
    name = "vehicle_state_cc_proto",
    deps = [":vehicle_state_proto"],
)

py_proto_library(
    name = "vehicle_state_py_pb2",
    deps = [":vehicle_state_proto"],
)

go_proto_library(
    name = "vehicle_state_go_proto",
    importpath = "intrinsic/vehicle/proto/vehicle_state_go_proto",
    protos = [":vehicle_state_proto"],
    deps = [
        "//intrinsic/embodiment/proto:stamped_header_go_proto",
        "//intrinsic/math/proto:pose_go_proto",
    ],
)

proto_library(
    name = "vehicle_command_proto",
    srcs = ["vehicle_command.proto"],
    deps = [
        ":vehicle_state_proto",
        "//intrinsic/embodiment/proto:model_provenance_proto",
        "//intrinsic/embodiment/proto:stamped_header_proto",
        "//intrinsic/math/proto:pose_proto",
        "@com_google_protobuf//:duration_proto",
    ],
)

cc_proto_library(
    name = "vehicle_command_cc_proto",
    deps = [":vehicle_command_proto"],
)

py_proto_library(
    name = "vehicle_command_py_pb2",
    deps = [":vehicle_command_proto"],
)

go_proto_library(
    name = "vehicle_command_go_proto",
    importpath = "intrinsic/vehicle/proto/vehicle_command_go_proto",
    protos = [":vehicle_command_proto"],
    deps = [
        ":vehicle_state_go_proto",
        "//intrinsic/embodiment/proto:model_provenance_go_proto",
        "//intrinsic/embodiment/proto:stamped_header_go_proto",
        "//intrinsic/math/proto:pose_go_proto",
        "@org_golang_google_protobuf//types/known/durationpb",
    ],
)

filegroup(
    name = "examples",
    srcs = [
        "examples/body_wrench.textproto",
        "examples/desired_motion_pose.textproto",
        "examples/desired_motion_twist.textproto",
        "examples/vehicle_state.textproto",
        "examples/vehicle_state_unknown_covariance.textproto",
    ],
)
