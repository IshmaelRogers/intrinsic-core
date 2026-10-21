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

# Fixed-seed dive and resurface scenario (#134,
# uuv-dive-resurface-scenario-v1). Sim and test only. Not part of the protected
# manipulator baseline. Do not add these targets to
# .github/baseline/manipulator_targets.tsv.

load("//bazel:cc_macros.bzl", "cc_test")

package(default_visibility = ["//visibility:private"])

filegroup(
    name = "scenario_assets",
    srcs = [
        "dive_resurface.world.sdf",
        "dive_resurface_profile.txt",
        "dive_resurface_sensors.sdf",
    ],
)

cc_test(
    name = "dive_resurface_scenario_test",
    size = "small",
    srcs = ["dive_resurface_scenario_test.cc"],
    data = [":scenario_assets"],
    deps = [
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/estimation:estimator_cc_proto",
        "//intrinsic/estimation:fake_estimator_service",
        "//intrinsic/simulation/gazebo/plugins/altimeter:altimeter_plugin",
        "//intrinsic/simulation/gazebo/plugins/depth:depth_plugin",
        "//intrinsic/simulation/gazebo/plugins/dvl:dvl_plugin",
        "//intrinsic/simulation/gazebo/plugins/surface_fix:surface_fix_plugin",
        "//intrinsic_hardware/intrinsic/hardware/marine:altimeter_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:altimeter_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:dvl_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:dvl_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_policy",
        "@com_google_googletest//:gtest_main",
        "@gz-math",
        "@gz-sim",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
        "@sdformat",
    ],
)
