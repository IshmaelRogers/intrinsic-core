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

# Opt-in marine component validity helpers. Not part of the protected
# manipulator baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "marine_component_validity_policy",
    hdrs = ["marine_component_validity_policy.h"],
    deps = ["//intrinsic/embodiment:frame_policy"],
)

cc_test(
    name = "marine_component_validity_policy_test",
    size = "small",
    srcs = ["marine_component_validity_policy_test.cc"],
    deps = [
        ":marine_component_validity_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "marine_component_validity_serialization_test",
    size = "small",
    srcs = ["marine_component_validity_serialization_test.cc"],
    deps = [
        ":marine_component_validity_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/world/proto:marine_component_validity_cc_proto",
    ],
)

py_library(
    name = "marine_component_validity_policy_py",
    srcs = ["marine_component_validity_policy.py"],
    deps = ["//intrinsic/embodiment:frame_policy_py"],
)

py_test(
    name = "marine_component_validity_policy_test_py",
    size = "small",
    srcs = ["marine_component_validity_policy_test.py"],
    main = "marine_component_validity_policy_test.py",
    deps = [":marine_component_validity_policy_py"],
)

py_test(
    name = "marine_component_validity_serialization_test_py",
    size = "small",
    srcs = ["marine_component_validity_serialization_test.py"],
    main = "marine_component_validity_serialization_test.py",
    deps = [
        ":marine_component_validity_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:marine_component_validity_py_pb2",
    ],
)
