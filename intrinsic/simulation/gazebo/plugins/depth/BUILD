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

# Opt-in depth sensor system plugin. Not part of the protected manipulator
# baseline. Do not add these targets to
# .github/baseline/manipulator_targets.tsv.

load("//bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:private"])

cc_library(
    name = "depth_adapter",
    srcs = ["depth_adapter.cc"],
    hdrs = ["depth_adapter.h"],
    deps = [
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_imu",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_pressure_depth",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_cc_proto",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters",
        "@abseil-cpp//absl/strings",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@sdformat",
    ],
)

cc_library(
    name = "depth_plugin",
    srcs = ["depth_plugin.cc"],
    hdrs = ["depth_plugin.h"],
    visibility = ["//intrinsic/simulation/gazebo/scenarios/dive_resurface:__pkg__"],
    deps = [
        ":depth_adapter",
        "@abseil-cpp//absl/log",
        "@gz-math",
        "@gz-sim",
        "@sdformat",
    ],
)

cc_library(
    name = "depth_plugin_register",
    srcs = ["depth_plugin_register.cc"],
    visibility = ["//intrinsic/simulation/gazebo/plugins:__pkg__"],
    deps = [
        ":depth_plugin",
        "@gz-plugin//:register",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_test(
    name = "depth_adapter_test",
    size = "small",
    srcs = ["depth_adapter_test.cc"],
    data = [
        "depth_adapter.cc",
        "depth_adapter.h",
        "depth_plugin.cc",
        "depth_plugin.h",
        "depth_plugin_register.cc",
        "testdata/depth_adapter_seed_42.golden.txt",
    ],
    deps = [
        ":depth_adapter",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_imu",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_pressure_depth",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "depth_plugin_test",
    size = "small",
    srcs = ["depth_plugin_test.cc"],
    deps = [
        ":depth_adapter",
        ":depth_plugin",
        ":depth_plugin_register",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_policy",
        "@abseil-cpp//absl/log:log_sink",
        "@abseil-cpp//absl/log:log_sink_registry",
        "@com_google_googletest//:gtest_main",
        "@gz-math",
        "@gz-msgs",
        "@gz-sim",
        "@sdformat",
    ],
)
