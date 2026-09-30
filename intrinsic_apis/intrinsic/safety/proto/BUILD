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

# Opt-in safety decision contract. Not part of the protected manipulator
# baseline.

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "safety_decision_proto",
    srcs = ["safety_decision.proto"],
    deps = [
        "//intrinsic/embodiment/proto:stamped_header_proto",
        "//intrinsic/vehicle/proto:vehicle_command_proto",
    ],
)

cc_proto_library(
    name = "safety_decision_cc_proto",
    deps = [":safety_decision_proto"],
)

py_proto_library(
    name = "safety_decision_py_pb2",
    deps = [":safety_decision_proto"],
)

go_proto_library(
    name = "safety_decision_go_proto",
    importpath = "intrinsic/safety/proto/safety_decision_go_proto",
    protos = [":safety_decision_proto"],
    deps = [
        "//intrinsic/embodiment/proto:stamped_header_go_proto",
        "//intrinsic/vehicle/proto:vehicle_command_go_proto",
    ],
)

filegroup(
    name = "examples",
    srcs = [
        "examples/abort.textproto",
        "examples/accept.textproto",
        "examples/project.textproto",
        "examples/reject.textproto",
        "examples/surface.textproto",
    ],
)
