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

# Opt-in vehicle reference control. ReferenceController,
# ZeroWrenchController, ReferenceDepthController,
# ReferenceHeadingController, ReferenceForwardSpeedController, and
# scalar control math. The depth controller is the heave law. The
# heading controller is the yaw law. The forward-speed controller is
# the surge law. Not part of the protected manipulator baseline. Do
# not add this package to .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "package_files",
    srcs = [
        "BUILD",
        "control.h",
        "control_math.cc",
        "control_math.h",
        "reference_control.cc",
        "reference_control.h",
        "reference_depth_controller.cc",
        "reference_depth_controller.h",
        "reference_forward_speed_controller.cc",
        "reference_forward_speed_controller.h",
        "reference_heading_controller.cc",
        "reference_heading_controller.h",
        "zero_wrench_controller.cc",
        "zero_wrench_controller.h",
    ],
)

cc_library(
    name = "control",
    srcs = [
        "control_math.cc",
        "reference_control.cc",
        "reference_depth_controller.cc",
        "reference_forward_speed_controller.cc",
        "reference_heading_controller.cc",
        "zero_wrench_controller.cc",
    ],
    hdrs = [
        "control.h",
        "control_math.h",
        "reference_control.h",
        "reference_depth_controller.h",
        "reference_forward_speed_controller.h",
        "reference_heading_controller.h",
        "zero_wrench_controller.h",
    ],
    deps = [
        "//intrinsic_vehicle/intrinsic/vehicle/guidance",
    ],
)

cc_test(
    name = "control_math_test",
    size = "small",
    srcs = ["control_math_test.cc"],
    deps = [
        ":control",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "control_test",
    size = "small",
    srcs = ["control_test.cc"],
    deps = [
        ":control",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "reference_depth_controller_test",
    size = "small",
    srcs = ["reference_depth_controller_test.cc"],
    deps = [
        ":control",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "reference_forward_speed_controller_test",
    size = "small",
    srcs = ["reference_forward_speed_controller_test.cc"],
    deps = [
        ":control",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "reference_heading_controller_test",
    size = "small",
    srcs = ["reference_heading_controller_test.cc"],
    deps = [
        ":control",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "reference_control_test",
    size = "small",
    srcs = ["reference_control_test.cc"],
    deps = [
        ":control",
        "//intrinsic_vehicle/intrinsic/vehicle/guidance",
        "@com_google_googletest//:gtest_main",
    ],
)
