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

# UUV reference-solution simulation leaves (ADR 0002, #43). Sim and test only,
# not in the manifest. Do not add these targets to
# .github/baseline/manipulator_targets.tsv.

load("//bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:private"])

filegroup(
    name = "playback_test_assets",
    srcs = ["testdata/playback_test.world.sdf"],
)

cc_library(
    name = "trajectory_playback",
    srcs = ["trajectory_playback.cc"],
    hdrs = ["trajectory_playback.h"],
    deps = [
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:trajectory_contract_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_planner_registry",
        "//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_state_space",
        "@gz-math",
        "@gz-sim",
    ],
)

cc_test(
    name = "trajectory_playback_test",
    size = "small",
    srcs = ["trajectory_playback_test.cc"],
    data = [":playback_test_assets"],
    deps = [
        ":trajectory_playback",
        "//intrinsic/world/world_snapshot:world_snapshot_policy",
        "//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_kinodynamic_baseline",
        "//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_motion_primitives",
        "//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_primitive_propagation",
        "//intrinsic_vehicle/intrinsic/vehicle/dynamics",
        "@com_google_googletest//:gtest_main",
        "@gz-math",
        "@gz-sim",
        "@sdformat",
    ],
)

# Later leaves append below.
