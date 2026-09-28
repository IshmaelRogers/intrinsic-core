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

# Opt-in embodiment helpers. Not part of the protected manipulator baseline.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "frame_policy",
    hdrs = [
        "frame_policy.h",
        "stamped_header_policy.h",
    ],
)

cc_test(
    name = "frame_policy_test",
    size = "small",
    srcs = ["frame_policy_test.cc"],
    deps = [
        ":frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "stamped_header_serialization_test",
    size = "small",
    srcs = ["stamped_header_serialization_test.cc"],
    deps = [
        ":frame_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

py_library(
    name = "frame_policy_py",
    srcs = [
        "frame_policy.py",
        "stamped_header_policy.py",
    ],
)

py_test(
    name = "frame_policy_test_py",
    size = "small",
    srcs = ["frame_policy_test.py"],
    deps = [":frame_policy_py"],
)

py_test(
    name = "stamped_header_serialization_test_py",
    size = "small",
    srcs = ["stamped_header_serialization_test.py"],
    deps = [
        ":frame_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)
