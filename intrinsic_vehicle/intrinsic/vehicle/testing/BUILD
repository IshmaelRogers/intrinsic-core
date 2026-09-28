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
load("//bazel:python.bzl", "py_test")

# Opt-in vehicle testing scaffold. Not part of the protected manipulator
# baseline. Do not add this package to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "package_files",
    srcs = [
        "BUILD",
        "scaffold.h",
    ],
)

cc_library(
    name = "scaffold",
    testonly = 1,
    hdrs = ["scaffold.h"],
    deps = [
        "//intrinsic_vehicle/intrinsic/vehicle/allocation",
        "//intrinsic_vehicle/intrinsic/vehicle/control",
        "//intrinsic_vehicle/intrinsic/vehicle/dynamics",
        "//intrinsic_vehicle/intrinsic/vehicle/guidance",
        "//intrinsic_vehicle/intrinsic/vehicle/state",
    ],
)

cc_test(
    name = "scaffold_test",
    size = "small",
    srcs = ["scaffold_test.cc"],
    deps = [
        ":scaffold",
        "@com_google_googletest//:gtest_main",
    ],
)

py_test(
    name = "package_graph_test",
    size = "small",
    srcs = ["package_graph_test.py"],
    data = [
        "//intrinsic_vehicle/intrinsic/vehicle:package_files",
        "//intrinsic_vehicle/intrinsic/vehicle/allocation:package_files",
        "//intrinsic_vehicle/intrinsic/vehicle/control:package_files",
        "//intrinsic_vehicle/intrinsic/vehicle/dynamics:package_files",
        "//intrinsic_vehicle/intrinsic/vehicle/guidance:package_files",
        "//intrinsic_vehicle/intrinsic/vehicle/state:package_files",
        "//intrinsic_vehicle/intrinsic/vehicle/testing:package_files",
    ],
    main = "package_graph_test.py",
)
