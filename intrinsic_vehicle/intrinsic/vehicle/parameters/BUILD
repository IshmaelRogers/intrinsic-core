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

# Opt-in marine model parameters. Not part of the protected manipulator
# baseline. Do not add this package to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "package_files",
    srcs = [
        "BUILD",
        "marine_model.cc",
        "marine_model.h",
        "parameters.h",
        "six_thruster_uuv_example.cc",
        "six_thruster_uuv_example.h",
    ],
)

cc_library(
    name = "parameters",
    srcs = ["marine_model.cc"],
    hdrs = [
        "marine_model.h",
        "parameters.h",
    ],
)

cc_library(
    name = "six_thruster_uuv_example",
    srcs = ["six_thruster_uuv_example.cc"],
    hdrs = ["six_thruster_uuv_example.h"],
    deps = [":parameters"],
)

cc_test(
    name = "marine_model_test",
    size = "small",
    srcs = ["marine_model_test.cc"],
    deps = [
        ":parameters",
        ":six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)
