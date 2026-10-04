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

# Opt-in bounded async inference request queue (capacity 8, reject new) with a
# stub worker. Contract: issue #119, comment 5978277220. Not wired into the
# Triton inference service, ICON, or a HAL, and not part of the protected
# manipulator baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stub_worker",
    srcs = ["stub_worker.cc"],
    hdrs = ["stub_worker.h"],
    deps = ["//intrinsic_inference/envelope:inference_envelope_contract_policy"],
)

cc_library(
    name = "inference_client",
    srcs = ["inference_client.cc"],
    hdrs = ["inference_client.h"],
    deps = [
        ":stub_worker",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_inference/envelope:inference_envelope_contract_policy",
    ],
)

cc_test(
    name = "inference_queue_test",
    size = "small",
    srcs = ["inference_queue_test.cc"],
    deps = [
        ":inference_client",
        ":stub_worker",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_inference/envelope:inference_envelope_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "stub_worker_py",
    srcs = ["stub_worker.py"],
    deps = ["//intrinsic_inference/envelope:inference_envelope_contract_policy_py"],
)

py_library(
    name = "inference_client_py",
    srcs = ["inference_client.py"],
    deps = [
        ":stub_worker_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic_inference/envelope:inference_envelope_contract_policy_py",
    ],
)

py_test(
    name = "inference_queue_test_py",
    size = "small",
    srcs = ["inference_queue_test.py"],
    main = "inference_queue_test.py",
    deps = [
        ":inference_client_py",
        ":stub_worker_py",
        "//intrinsic_inference/envelope:inference_envelope_contract_policy_py",
    ],
)
