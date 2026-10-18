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

# Opt-in Physical AI episode event envelope host policy. Not part of the
# protected manipulator baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "testdata",
    srcs = glob(["testdata/*"]),
)

cc_library(
    name = "episode_event_policy",
    hdrs = ["episode_event_policy.h"],
    deps = [
        "//intrinsic/embodiment:frame_policy",
        "@boringssl//:crypto",
    ],
)

cc_test(
    name = "episode_event_policy_test",
    size = "small",
    srcs = ["episode_event_policy_test.cc"],
    deps = [
        ":episode_event_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "episode_event_serialization_test",
    size = "small",
    srcs = ["episode_event_serialization_test.cc"],
    data = [
        ":testdata",
        "@intrinsic_apis//intrinsic/episode/proto:examples",
    ],
    deps = [
        ":episode_event_policy",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_test(
    name = "episode_event_schema_freeze_test",
    size = "small",
    srcs = ["episode_event_schema_freeze_test.cc"],
    data = [":testdata"],
    deps = [
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_library(
    name = "episode_event_view",
    hdrs = ["episode_event_view.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_library(
    name = "episode_writer",
    srcs = ["episode_writer.cc"],
    hdrs = ["episode_writer.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_event_view",
        "//intrinsic/embodiment:frame_policy",
        "@abseil-cpp//absl/crc:crc32c",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/types:span",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_test(
    name = "episode_writer_test",
    size = "small",
    srcs = ["episode_writer_test.cc"],
    data = [":testdata"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_writer",
        "//intrinsic/embodiment:frame_policy",
        "@abseil-cpp//absl/crc:crc32c",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/types:span",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_library(
    name = "episode_reader",
    srcs = ["episode_reader.cc"],
    hdrs = ["episode_reader.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_event_view",
        "//intrinsic/embodiment:frame_policy",
        "@abseil-cpp//absl/crc:crc32c",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/types:span",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_test(
    name = "episode_reader_test",
    size = "small",
    srcs = ["episode_reader_test.cc"],
    data = [":testdata"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_reader",
        ":episode_writer",
        "//intrinsic/embodiment:frame_policy",
        "@abseil-cpp//absl/crc:crc32c",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/types:span",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_library(
    name = "episode_replay",
    srcs = ["episode_replay.cc"],
    hdrs = ["episode_replay.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_event_view",
        ":episode_reader",
        "//intrinsic/embodiment:frame_policy",
        "@abseil-cpp//absl/types:span",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_test(
    name = "episode_replay_test",
    size = "small",
    srcs = ["episode_replay_test.cc"],
    data = [":testdata"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_reader",
        ":episode_replay",
        "//intrinsic/embodiment:frame_policy",
        "@abseil-cpp//absl/crc:crc32c",
        "@abseil-cpp//absl/types:span",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_test(
    name = "episode_replay_payloads_test",
    size = "small",
    srcs = ["episode_replay_payloads_test.cc"],
    data = [":testdata"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_reader",
        ":episode_replay",
        ":episode_writer",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/types:span",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_library(
    name = "episode_replay_adapter_common",
    hdrs = ["episode_replay_adapter_common.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_replay",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_library(
    name = "estimator_replay_adapter",
    srcs = ["estimator_replay_adapter.cc"],
    hdrs = ["estimator_replay_adapter.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_replay",
        ":episode_replay_adapter_common",
        "//intrinsic/estimation:estimator_cc_proto",
        "//intrinsic/estimation:estimator_service",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "estimator_replay_adapter_test",
    size = "small",
    srcs = ["estimator_replay_adapter_test.cc"],
    data = [":testdata"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_reader",
        ":episode_replay",
        ":episode_replay_adapter_common",
        ":episode_writer",
        ":estimator_replay_adapter",
        "//intrinsic/estimation:estimator_cc_proto",
        "//intrinsic/estimation:estimator_service",
        "//intrinsic/estimation:fake_estimator_service",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
    ],
)

cc_library(
    name = "world_replay_adapter",
    srcs = ["world_replay_adapter.cc"],
    hdrs = ["world_replay_adapter.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_replay",
        ":episode_replay_adapter_common",
        "//intrinsic/world/marine_component_validity:marine_component_validity_policy",
        "//intrinsic/world/world_snapshot:world_snapshot_builder",
        "//intrinsic/world/world_snapshot:world_snapshot_policy",
        "//intrinsic/world/world_snapshot:world_snapshot_skew_assessor",
        "//intrinsic/world/world_snapshot:world_snapshot_skew_policy",
        "@intrinsic_apis//intrinsic/world/proto:world_snapshot_descriptor_cc_proto",
    ],
)

cc_test(
    name = "world_replay_adapter_test",
    size = "small",
    srcs = ["world_replay_adapter_test.cc"],
    data = [":testdata"],
    visibility = ["//visibility:private"],
    deps = [
        ":episode_event_policy",
        ":episode_reader",
        ":episode_replay",
        ":episode_replay_adapter_common",
        ":episode_writer",
        ":world_replay_adapter",
        "//intrinsic/world/marine_component_validity:marine_component_validity_policy",
        "//intrinsic/world/world_snapshot:world_snapshot_builder",
        "//intrinsic/world/world_snapshot:world_snapshot_policy",
        "//intrinsic/world/world_snapshot:world_snapshot_skew_assessor",
        "//intrinsic/world/world_snapshot:world_snapshot_skew_policy",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_cc_proto",
        "@intrinsic_apis//intrinsic/world/proto:world_snapshot_descriptor_cc_proto",
    ],
)

py_library(
    name = "episode_event_policy_py",
    srcs = ["episode_event_policy.py"],
    deps = ["//intrinsic/embodiment:frame_policy_py"],
)

py_test(
    name = "episode_event_policy_test_py",
    size = "small",
    srcs = ["episode_event_policy_test.py"],
    main = "episode_event_policy_test.py",
    deps = [":episode_event_policy_py"],
)

py_test(
    name = "episode_event_serialization_test_py",
    size = "small",
    srcs = ["episode_event_serialization_test.py"],
    data = [
        ":testdata",
        "@intrinsic_apis//intrinsic/episode/proto:examples",
    ],
    main = "episode_event_serialization_test.py",
    deps = [
        ":episode_event_policy_py",
        "@intrinsic_apis//intrinsic/episode/proto:episode_event_py_pb2",
    ],
)

py_test(
    name = "episode_event_schema_freeze_test_py",
    size = "small",
    srcs = ["episode_event_schema_freeze_test.py"],
    data = [":testdata"],
    main = "episode_event_schema_freeze_test.py",
    deps = ["@intrinsic_apis//intrinsic/episode/proto:episode_event_py_pb2"],
)
