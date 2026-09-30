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

# Opt-in vehicle guidance. GuidanceStep, EchoGuidance, and NullGuidance.
# Not a guidance law. Not part of the protected manipulator baseline.
# Do not add this package to .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "package_files",
    srcs = [
        "BUILD",
        "echo_guidance.cc",
        "echo_guidance.h",
        "guidance.h",
        "guidance_step.cc",
        "guidance_step.h",
        "null_guidance.cc",
        "null_guidance.h",
    ],
)

cc_library(
    name = "guidance",
    srcs = [
        "echo_guidance.cc",
        "guidance_step.cc",
        "null_guidance.cc",
    ],
    hdrs = [
        "echo_guidance.h",
        "guidance.h",
        "guidance_step.h",
        "null_guidance.h",
    ],
)

cc_test(
    name = "guidance_test",
    size = "small",
    srcs = ["guidance_test.cc"],
    deps = [
        ":guidance",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "motion_guidance_test",
    size = "small",
    srcs = ["motion_guidance_test.cc"],
    deps = [
        ":guidance",
        "@com_google_googletest//:gtest_main",
    ],
)
