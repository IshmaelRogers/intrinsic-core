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

# Opt-in Physical AI episode event envelope. Not part of the protected
# manipulator baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "episode_event_proto",
    srcs = ["episode_event.proto"],
    deps = [
        "//intrinsic/embodiment/proto:model_provenance_proto",
        "//intrinsic/embodiment/proto:stamped_header_proto",
        "//intrinsic/inference/proto:inference_envelope_proto",
        "//intrinsic/safety/proto:safety_decision_proto",
        "//intrinsic/vehicle/proto:vehicle_command_proto",
        "//intrinsic/vehicle/proto:vehicle_trajectory_proto",
        "//intrinsic/world/proto:world_snapshot_descriptor_proto",
        "@com_google_protobuf//:duration_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "episode_event_cc_proto",
    deps = [":episode_event_proto"],
)

py_proto_library(
    name = "episode_event_py_pb2",
    deps = [":episode_event_proto"],
)

go_proto_library(
    name = "episode_event_go_proto",
    importpath = "intrinsic/episode/proto/episode_event_go_proto",
    protos = [":episode_event_proto"],
    deps = [
        "//intrinsic/embodiment/proto:model_provenance_go_proto",
        "//intrinsic/embodiment/proto:stamped_header_go_proto",
        "//intrinsic/inference/proto:inference_envelope_go_proto",
        "//intrinsic/safety/proto:safety_decision_go_proto",
        "//intrinsic/vehicle/proto:vehicle_command_go_proto",
        "//intrinsic/vehicle/proto:vehicle_trajectory_go_proto",
        "//intrinsic/world/proto:world_snapshot_descriptor_go_proto",
        "@org_golang_google_protobuf//types/known/durationpb",
        "@org_golang_google_protobuf//types/known/timestamppb",
    ],
)

filegroup(
    name = "examples",
    srcs = [
        "examples/episode_event_v1_desired_motion.textproto",
        "examples/episode_event_v1_inference_envelope.textproto",
        "examples/episode_event_v1_inference_result.textproto",
        "examples/episode_event_v1_metadata.textproto",
        "examples/episode_event_v1_recorded_message.textproto",
        "examples/episode_event_v1_safety_decision.textproto",
        "examples/episode_event_v1_vehicle_trajectory.textproto",
        "examples/episode_event_v1_world_snapshot_record.textproto",
    ],
)
