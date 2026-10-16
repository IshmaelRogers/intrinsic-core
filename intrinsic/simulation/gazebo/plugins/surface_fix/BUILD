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

# Opt-in surface-fix sensor system plugin. Not part of the protected manipulator
# baseline. Do not add these targets to
# .github/baseline/manipulator_targets.tsv.

load("//bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:private"])

cc_library(
    name = "surface_fix_adapter",
    srcs = ["surface_fix_adapter.cc"],
    hdrs = ["surface_fix_adapter.h"],
    deps = [
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_imu",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_surface_fix",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_cc_proto",
        "@abseil-cpp//absl/strings",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@sdformat",
    ],
)

cc_library(
    name = "surface_fix_plugin",
    srcs = ["surface_fix_plugin.cc"],
    hdrs = ["surface_fix_plugin.h"],
    deps = [
        ":surface_fix_adapter",
        "@abseil-cpp//absl/log",
        "@gz-math",
        "@gz-sim",
        "@sdformat",
    ],
)

cc_library(
    name = "surface_fix_plugin_register",
    srcs = ["surface_fix_plugin_register.cc"],
    visibility = ["//intrinsic/simulation/gazebo/plugins:__pkg__"],
    deps = [
        ":surface_fix_plugin",
        "@gz-plugin//:register",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_test(
    name = "surface_fix_adapter_test",
    size = "small",
    srcs = ["surface_fix_adapter_test.cc"],
    data = [
        "surface_fix_adapter.cc",
        "surface_fix_adapter.h",
        "surface_fix_plugin.cc",
        "surface_fix_plugin.h",
        "surface_fix_plugin_register.cc",
        "testdata/surface_fix_adapter_seed_42.golden.txt",
    ],
    deps = [
        ":surface_fix_adapter",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_imu",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_surface_fix",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "surface_fix_plugin_test",
    size = "small",
    srcs = ["surface_fix_plugin_test.cc"],
    deps = [
        ":surface_fix_adapter",
        ":surface_fix_plugin",
        ":surface_fix_plugin_register",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_policy",
        "@abseil-cpp//absl/log:log_sink",
        "@abseil-cpp//absl/log:log_sink_registry",
        "@com_google_googletest//:gtest_main",
        "@gz-math",
        "@gz-msgs",
        "@gz-sim",
        "@sdformat",
    ],
)
