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

# Opt-in sonar frame contracts (forward-looking and side-scan) and the
# multimodal observation bundle. Not part of the protected manipulator
# baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "fls_frame_proto",
    srcs = ["fls_frame.proto"],
    deps = [
        "//intrinsic/embodiment/proto:stamped_header_proto",
        "//intrinsic/math/proto:pose_proto",
    ],
)

cc_proto_library(
    name = "fls_frame_cc_proto",
    deps = [":fls_frame_proto"],
)

py_proto_library(
    name = "fls_frame_py_pb2",
    deps = [":fls_frame_proto"],
)

go_proto_library(
    name = "fls_frame_go_proto",
    importpath = "intrinsic/perception/proto/sonar/fls_frame_go_proto",
    protos = [":fls_frame_proto"],
    deps = [
        "//intrinsic/embodiment/proto:stamped_header_go_proto",
        "//intrinsic/math/proto:pose_go_proto",
    ],
)

proto_library(
    name = "sss_frame_proto",
    srcs = ["sss_frame.proto"],
    deps = [
        ":fls_frame_proto",
        "//intrinsic/embodiment/proto:stamped_header_proto",
        "//intrinsic/math/proto:pose_proto",
    ],
)

cc_proto_library(
    name = "sss_frame_cc_proto",
    deps = [":sss_frame_proto"],
)

py_proto_library(
    name = "sss_frame_py_pb2",
    deps = [":sss_frame_proto"],
)

go_proto_library(
    name = "sss_frame_go_proto",
    importpath = "intrinsic/perception/proto/sonar/sss_frame_go_proto",
    protos = [":sss_frame_proto"],
    deps = [
        ":fls_frame_go_proto",
        "//intrinsic/embodiment/proto:stamped_header_go_proto",
        "//intrinsic/math/proto:pose_go_proto",
    ],
)

proto_library(
    name = "observation_bundle_proto",
    srcs = ["observation_bundle.proto"],
    deps = [
        "//intrinsic/embodiment/proto:stamped_header_proto",
        "@com_google_protobuf//:duration_proto",
    ],
)

cc_proto_library(
    name = "observation_bundle_cc_proto",
    deps = [":observation_bundle_proto"],
)

py_proto_library(
    name = "observation_bundle_py_pb2",
    deps = [":observation_bundle_proto"],
)

go_proto_library(
    name = "observation_bundle_go_proto",
    importpath = "intrinsic/perception/proto/sonar/observation_bundle_go_proto",
    protos = [":observation_bundle_proto"],
    deps = [
        "//intrinsic/embodiment/proto:stamped_header_go_proto",
        "@org_golang_google_protobuf//types/known/durationpb",
    ],
)

filegroup(
    name = "examples",
    srcs = [
        "examples/fls_frame_bad_frame.textproto",
        "examples/fls_frame_bad_sound_speed.textproto",
        "examples/fls_frame_blob_nominal.textproto",
        "examples/fls_frame_empty_blob_id.textproto",
        "examples/fls_frame_inline_nominal.textproto",
        "examples/fls_frame_payload_mismatch.textproto",
        "examples/observation_bundle_all_modalities.textproto",
        "examples/observation_bundle_bad_snapshot_id.textproto",
        "examples/observation_bundle_camera_only.textproto",
        "examples/observation_bundle_excessive_skew.textproto",
        "examples/observation_bundle_frame_mismatch.textproto",
        "examples/observation_bundle_missing_absent_reason.textproto",
        "examples/observation_bundle_sonar_only.textproto",
        "examples/observation_bundle_time_reversal.textproto",
        "examples/sss_frame_bad_frame.textproto",
        "examples/sss_frame_bad_sound_speed.textproto",
        "examples/sss_frame_blob_nominal.textproto",
        "examples/sss_frame_empty_blob_id.textproto",
        "examples/sss_frame_inline_nominal.textproto",
        "examples/sss_frame_payload_mismatch.textproto",
    ],
)
