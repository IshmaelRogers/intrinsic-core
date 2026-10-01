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

# Opt-in vehicle StateSpace, planner registry, motion primitives, primitive
# propagation, snapshot-backed trajectory validity, and the discrete
# kinodynamic lattice Dijkstra baseline. Not part of the protected manipulator
# baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_state_space",
    srcs = ["vehicle_state_space.cc"],
    hdrs = ["vehicle_state_space.h"],
    deps = [
        "//intrinsic/eigenmath:interpolation",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_gitlab_libeigen_eigen//:eigen",
    ],
)

cc_test(
    name = "vehicle_state_space_test",
    size = "small",
    srcs = ["vehicle_state_space_test.cc"],
    deps = [
        ":vehicle_state_space",
        "//intrinsic/eigenmath:interpolation",
        "//intrinsic/embodiment:frame_policy",
        "@com_gitlab_libeigen_eigen//:eigen",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_state_space_py",
    srcs = ["vehicle_state_space.py"],
    deps = [
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "vehicle_state_space_test_py",
    size = "small",
    srcs = ["vehicle_state_space_test.py"],
    main = "vehicle_state_space_test.py",
    deps = [
        ":vehicle_state_space_py",
        "//intrinsic/embodiment:frame_policy_py",
    ],
)

cc_library(
    name = "vehicle_planner_registry",
    srcs = ["vehicle_planner_registry.cc"],
    hdrs = ["vehicle_planner_registry.h"],
    deps = [
        ":vehicle_state_space",
        "//intrinsic/vehicle:trajectory_contract_policy",
    ],
)

cc_library(
    name = "fake_vehicle_planner",
    srcs = ["fake_vehicle_planner.cc"],
    hdrs = ["fake_vehicle_planner.h"],
    deps = [
        ":vehicle_planner_registry",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:trajectory_contract_policy",
    ],
)

cc_test(
    name = "vehicle_planner_registry_test",
    size = "small",
    srcs = ["vehicle_planner_registry_test.cc"],
    deps = [
        ":fake_vehicle_planner",
        ":vehicle_planner_registry",
        "//intrinsic/vehicle:trajectory_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_planner_registry_py",
    srcs = ["vehicle_planner_registry.py"],
    deps = [
        ":vehicle_state_space_py",
        "//intrinsic/vehicle:trajectory_contract_policy_py",
    ],
)

py_library(
    name = "fake_vehicle_planner_py",
    srcs = ["fake_vehicle_planner.py"],
    deps = [
        ":vehicle_planner_registry_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:trajectory_contract_policy_py",
    ],
)

py_test(
    name = "vehicle_planner_registry_test_py",
    size = "small",
    srcs = ["vehicle_planner_registry_test.py"],
    main = "vehicle_planner_registry_test.py",
    deps = [
        ":fake_vehicle_planner_py",
        ":vehicle_planner_registry_py",
        "//intrinsic/vehicle:trajectory_contract_policy_py",
    ],
)

cc_library(
    name = "vehicle_planner_deadline",
    srcs = ["vehicle_planner_deadline.cc"],
    hdrs = ["vehicle_planner_deadline.h"],
    deps = [":vehicle_planner_registry"],
)

cc_test(
    name = "vehicle_planner_deadline_test",
    size = "small",
    srcs = ["vehicle_planner_deadline_test.cc"],
    deps = [
        ":fake_vehicle_planner",
        ":vehicle_planner_deadline",
        ":vehicle_planner_registry",
        "//intrinsic/vehicle:trajectory_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_planner_deadline_py",
    srcs = ["vehicle_planner_deadline.py"],
    deps = [":vehicle_planner_registry_py"],
)

py_test(
    name = "vehicle_planner_deadline_test_py",
    size = "small",
    srcs = ["vehicle_planner_deadline_test.py"],
    main = "vehicle_planner_deadline_test.py",
    deps = [
        ":fake_vehicle_planner_py",
        ":vehicle_planner_deadline_py",
        ":vehicle_planner_registry_py",
        "//intrinsic/vehicle:trajectory_contract_policy_py",
    ],
)

cc_library(
    name = "vehicle_motion_primitives",
    srcs = ["vehicle_motion_primitives.cc"],
    hdrs = ["vehicle_motion_primitives.h"],
    deps = ["//intrinsic/vehicle:vehicle_contract_policy"],
)

cc_test(
    name = "vehicle_motion_primitives_test",
    size = "small",
    srcs = ["vehicle_motion_primitives_test.cc"],
    deps = [
        ":vehicle_motion_primitives",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_motion_primitives_py",
    srcs = ["vehicle_motion_primitives.py"],
    deps = ["//intrinsic/vehicle:vehicle_contract_policy_py"],
)

py_test(
    name = "vehicle_motion_primitives_test_py",
    size = "small",
    srcs = ["vehicle_motion_primitives_test.py"],
    main = "vehicle_motion_primitives_test.py",
    deps = [":vehicle_motion_primitives_py"],
)

cc_library(
    name = "vehicle_primitive_propagation",
    srcs = ["vehicle_primitive_propagation.cc"],
    hdrs = ["vehicle_primitive_propagation.h"],
    deps = [
        ":vehicle_motion_primitives",
        ":vehicle_state_space",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "//intrinsic_vehicle/intrinsic/vehicle/dynamics",
    ],
)

cc_test(
    name = "vehicle_primitive_propagation_test",
    size = "small",
    srcs = ["vehicle_primitive_propagation_test.cc"],
    deps = [
        ":vehicle_primitive_propagation",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "//intrinsic_vehicle/intrinsic/vehicle/dynamics",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_primitive_propagation_py",
    srcs = ["vehicle_primitive_propagation.py"],
    deps = [
        ":vehicle_motion_primitives_py",
        ":vehicle_state_space_py",
        "//intrinsic/embodiment:frame_policy_py",
    ],
)

py_test(
    name = "vehicle_primitive_propagation_test_py",
    size = "small",
    srcs = ["vehicle_primitive_propagation_test.py"],
    main = "vehicle_primitive_propagation_test.py",
    deps = [":vehicle_primitive_propagation_py"],
)

cc_library(
    name = "vehicle_trajectory_validity",
    srcs = ["vehicle_trajectory_validity.cc"],
    hdrs = ["vehicle_trajectory_validity.h"],
    deps = [
        ":vehicle_primitive_propagation",
        ":vehicle_state_space",
        "//intrinsic/safety:clearance_rule",
        "//intrinsic/safety:geofence_rule",
        "//intrinsic/safety:safety_rule_result",
        "//intrinsic/world/marine_component_validity:marine_component_validity_policy",
        "//intrinsic/world/world_snapshot:world_snapshot_policy",
        "//intrinsic/world/world_snapshot:world_snapshot_skew_policy",
    ],
)

cc_test(
    name = "vehicle_trajectory_validity_test",
    size = "small",
    srcs = ["vehicle_trajectory_validity_test.cc"],
    deps = [
        ":vehicle_primitive_propagation",
        ":vehicle_state_space",
        ":vehicle_trajectory_validity",
        "//intrinsic/safety:clearance_rule",
        "//intrinsic/safety:geofence_rule",
        "//intrinsic/world/world_snapshot:world_snapshot_policy",
        "//intrinsic/world/world_snapshot:world_snapshot_skew_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_trajectory_validity_py",
    srcs = ["vehicle_trajectory_validity.py"],
    deps = [
        ":vehicle_primitive_propagation_py",
        ":vehicle_state_space_py",
        "//intrinsic/safety:clearance_rule_py",
        "//intrinsic/safety:geofence_rule_py",
        "//intrinsic/safety:safety_rule_result_py",
        "//intrinsic/world/world_snapshot:world_snapshot_policy_py",
        "//intrinsic/world/world_snapshot:world_snapshot_skew_policy_py",
    ],
)

py_test(
    name = "vehicle_trajectory_validity_test_py",
    size = "small",
    srcs = ["vehicle_trajectory_validity_test.py"],
    main = "vehicle_trajectory_validity_test.py",
    deps = [
        ":vehicle_state_space_py",
        ":vehicle_trajectory_validity_py",
        "//intrinsic/world/world_snapshot:world_snapshot_policy_py",
    ],
)

cc_library(
    name = "vehicle_kinodynamic_baseline",
    srcs = ["vehicle_kinodynamic_baseline.cc"],
    hdrs = ["vehicle_kinodynamic_baseline.h"],
    deps = [
        ":vehicle_motion_primitives",
        ":vehicle_planner_deadline",
        ":vehicle_planner_registry",
        ":vehicle_primitive_propagation",
        ":vehicle_state_space",
        ":vehicle_trajectory_validity",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:trajectory_contract_policy",
        "//intrinsic_vehicle/intrinsic/vehicle/dynamics",
    ],
)

cc_test(
    name = "vehicle_kinodynamic_baseline_test",
    size = "small",
    srcs = ["vehicle_kinodynamic_baseline_test.cc"],
    deps = [
        ":fake_vehicle_planner",
        ":vehicle_kinodynamic_baseline",
        ":vehicle_motion_primitives",
        ":vehicle_planner_deadline",
        ":vehicle_planner_registry",
        ":vehicle_primitive_propagation",
        ":vehicle_state_space",
        "//intrinsic/vehicle:trajectory_contract_policy",
        "//intrinsic/world/world_snapshot:world_snapshot_policy",
        "//intrinsic/world/world_snapshot:world_snapshot_skew_policy",
        "//intrinsic_vehicle/intrinsic/vehicle/dynamics",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "vehicle_kinodynamic_baseline_py",
    srcs = ["vehicle_kinodynamic_baseline.py"],
    deps = [
        ":vehicle_motion_primitives_py",
        ":vehicle_planner_deadline_py",
        ":vehicle_planner_registry_py",
        ":vehicle_primitive_propagation_py",
        ":vehicle_state_space_py",
        ":vehicle_trajectory_validity_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/safety:clearance_rule_py",
        "//intrinsic/vehicle:trajectory_contract_policy_py",
        "//intrinsic/world/world_snapshot:world_snapshot_policy_py",
        "//intrinsic/world/world_snapshot:world_snapshot_skew_policy_py",
    ],
)

py_test(
    name = "vehicle_kinodynamic_baseline_test_py",
    size = "small",
    srcs = ["vehicle_kinodynamic_baseline_test.py"],
    main = "vehicle_kinodynamic_baseline_test.py",
    deps = [
        ":fake_vehicle_planner_py",
        ":vehicle_kinodynamic_baseline_py",
        ":vehicle_motion_primitives_py",
        ":vehicle_planner_deadline_py",
        ":vehicle_planner_registry_py",
        ":vehicle_primitive_propagation_py",
        ":vehicle_state_space_py",
        ":vehicle_trajectory_validity_py",
        "//intrinsic/vehicle:trajectory_contract_policy_py",
        "//intrinsic/world/world_snapshot:world_snapshot_policy_py",
    ],
)
