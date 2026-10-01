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

# Opt-in Physical AI inference envelope. Not part of the protected
# manipulator baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "inference_envelope_proto",
    srcs = ["inference_envelope.proto"],
    deps = [
        "//intrinsic/embodiment/proto:model_provenance_proto",
        "//intrinsic/embodiment/proto:stamped_header_proto",
        "@com_google_protobuf//:duration_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "inference_envelope_cc_proto",
    deps = [":inference_envelope_proto"],
)

py_proto_library(
    name = "inference_envelope_py_pb2",
    deps = [":inference_envelope_proto"],
)

go_proto_library(
    name = "inference_envelope_go_proto",
    importpath = "intrinsic/inference/proto/inference_envelope_go_proto",
    protos = [":inference_envelope_proto"],
    deps = [
        "//intrinsic/embodiment/proto:model_provenance_go_proto",
        "//intrinsic/embodiment/proto:stamped_header_go_proto",
        "@org_golang_google_protobuf//types/known/durationpb",
        "@org_golang_google_protobuf//types/known/timestamppb",
    ],
)

filegroup(
    name = "examples",
    srcs = [
        "examples/inference_envelope_bad_confidence.textproto",
        "examples/inference_envelope_bad_snapshot_id.textproto",
        "examples/inference_envelope_bad_uncertainty.textproto",
        "examples/inference_envelope_deadline_before_creation.textproto",
        "examples/inference_envelope_deadline_equal_creation.textproto",
        "examples/inference_envelope_empty_oip_request_id.textproto",
        "examples/inference_envelope_nominal.textproto",
        "examples/inference_result_nominal.textproto",
    ],
)
