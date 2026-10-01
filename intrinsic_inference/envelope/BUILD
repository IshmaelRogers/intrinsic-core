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

# Opt-in Physical AI inference envelope host policy. Not wired into the
# Triton inference service and not part of the protected manipulator
# baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "inference_envelope_contract_policy",
    hdrs = ["inference_envelope_contract_policy.h"],
    deps = ["//intrinsic/embodiment:frame_policy"],
)

cc_test(
    name = "inference_envelope_contract_test",
    size = "small",
    srcs = ["inference_envelope_contract_test.cc"],
    deps = [
        ":inference_envelope_contract_policy",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "inference_envelope_contract_serialization_test",
    size = "small",
    srcs = ["inference_envelope_contract_serialization_test.cc"],
    data = ["@intrinsic_apis//intrinsic/inference/proto:examples"],
    deps = [
        ":inference_envelope_contract_policy",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@intrinsic_apis//intrinsic/inference/proto:inference_envelope_cc_proto",
    ],
)

py_library(
    name = "inference_envelope_contract_policy_py",
    srcs = ["inference_envelope_contract_policy.py"],
    deps = ["//intrinsic/embodiment:frame_policy_py"],
)

py_test(
    name = "inference_envelope_contract_test_py",
    size = "small",
    srcs = ["inference_envelope_contract_test.py"],
    main = "inference_envelope_contract_test.py",
    deps = [
        ":inference_envelope_contract_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
    ],
)

py_test(
    name = "inference_envelope_contract_serialization_test_py",
    size = "small",
    srcs = ["inference_envelope_contract_serialization_test.py"],
    data = ["@intrinsic_apis//intrinsic/inference/proto:examples"],
    main = "inference_envelope_contract_serialization_test.py",
    deps = [
        ":inference_envelope_contract_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "@intrinsic_apis//intrinsic/inference/proto:inference_envelope_py_pb2",
    ],
)
