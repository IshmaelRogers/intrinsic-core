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

# Opt-in IMU sensor system plugin. Not part of the protected manipulator
# baseline. Do not add these targets to
# .github/baseline/manipulator_targets.tsv.

load("//bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:private"])

cc_library(
    name = "imu_adapter",
    srcs = ["imu_adapter.cc"],
    hdrs = ["imu_adapter.h"],
    deps = [
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/estimation:eskf_propagate",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_imu",
        "//intrinsic_hardware/intrinsic/hardware/marine:imu_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters",
        "@abseil-cpp//absl/strings",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@sdformat",
    ],
)

cc_library(
    name = "imu_plugin",
    srcs = ["imu_plugin.cc"],
    hdrs = ["imu_plugin.h"],
    deps = [
        ":imu_adapter",
        "@abseil-cpp//absl/log",
        "@gz-math",
        "@gz-sim",
        "@sdformat",
    ],
)

cc_library(
    name = "imu_plugin_register",
    srcs = ["imu_plugin_register.cc"],
    visibility = ["//intrinsic/simulation/gazebo/plugins:__pkg__"],
    deps = [
        ":imu_plugin",
        "@gz-plugin//:register",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_test(
    name = "imu_adapter_test",
    size = "small",
    srcs = ["imu_adapter_test.cc"],
    data = [
        "imu_adapter.cc",
        "imu_adapter.h",
        "imu_plugin.cc",
        "imu_plugin.h",
        "imu_plugin_register.cc",
        "testdata/imu_adapter_seed_42.golden.txt",
    ],
    deps = [
        ":imu_adapter",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/estimation:eskf_propagate",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_imu",
        "//intrinsic_hardware/intrinsic/hardware/marine:imu_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:imu_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "imu_plugin_test",
    size = "small",
    srcs = ["imu_plugin_test.cc"],
    deps = [
        ":imu_adapter",
        ":imu_plugin",
        ":imu_plugin_register",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/estimation:eskf_propagate",
        "//intrinsic/simulation/gazebo/plugins/hydrodynamics:hydrodynamics_plugin_register",
        "//intrinsic_hardware/intrinsic/hardware/marine:imu_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:imu_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "@abseil-cpp//absl/log:log_sink",
        "@abseil-cpp//absl/log:log_sink_registry",
        "@com_google_googletest//:gtest_main",
        "@gz-math",
        "@gz-msgs",
        "@gz-physics//bullet-featherstone:libgz-physics-bullet-featherstone-plugin-static",
        "@gz-sim",
        "@gz-sim//:gz-sim-physics-system-static",
        "@sdformat",
    ],
)
