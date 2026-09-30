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

# Opt-in World snapshot identity helpers. Not part of the protected
# manipulator baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "world_snapshot_policy",
    hdrs = ["world_snapshot_policy.h"],
    deps = [
        "//intrinsic/embodiment:frame_policy",
        "@boringssl//:crypto",
    ],
)

cc_library(
    name = "world_snapshot_builder",
    srcs = ["world_snapshot_builder.cc"],
    hdrs = ["world_snapshot_builder.h"],
    deps = [
        ":world_snapshot_policy",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@intrinsic_apis//intrinsic/world/proto:world_snapshot_descriptor_cc_proto",
    ],
)

cc_test(
    name = "world_snapshot_policy_test",
    size = "small",
    srcs = ["world_snapshot_policy_test.cc"],
    deps = [
        ":world_snapshot_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "world_snapshot_serialization_test",
    size = "small",
    srcs = ["world_snapshot_serialization_test.cc"],
    deps = [
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/world/proto:world_snapshot_descriptor_cc_proto",
    ],
)

cc_test(
    name = "world_snapshot_builder_test",
    size = "small",
    srcs = ["world_snapshot_builder_test.cc"],
    deps = [
        ":world_snapshot_builder",
        ":world_snapshot_policy",
        "@abseil-cpp//absl/status",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/world/proto:world_snapshot_descriptor_cc_proto",
    ],
)

py_library(
    name = "world_snapshot_policy_py",
    srcs = ["world_snapshot_policy.py"],
    deps = ["//intrinsic/embodiment:frame_policy_py"],
)

py_library(
    name = "world_snapshot_builder_py",
    srcs = ["world_snapshot_builder.py"],
    deps = [
        ":world_snapshot_policy_py",
        "@intrinsic_apis//intrinsic/world/proto:world_snapshot_descriptor_py_pb2",
    ],
)

py_test(
    name = "world_snapshot_policy_test_py",
    size = "small",
    srcs = ["world_snapshot_policy_test.py"],
    main = "world_snapshot_policy_test.py",
    deps = [":world_snapshot_policy_py"],
)

py_test(
    name = "world_snapshot_serialization_test_py",
    size = "small",
    srcs = ["world_snapshot_serialization_test.py"],
    main = "world_snapshot_serialization_test.py",
    deps = [
        "@intrinsic_apis//intrinsic/world/proto:world_snapshot_descriptor_py_pb2",
    ],
)

py_test(
    name = "world_snapshot_builder_test_py",
    size = "small",
    srcs = ["world_snapshot_builder_test.py"],
    main = "world_snapshot_builder_test.py",
    deps = [
        ":world_snapshot_builder_py",
        ":world_snapshot_policy_py",
        "@intrinsic_apis//intrinsic/world/proto:world_snapshot_descriptor_py_pb2",
    ],
)
