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
load("//intrinsic_control/intrinsic/icon/testing:cc_test_and_malloc_test.bzl", "cc_test_and_malloc_test")

cc_library(
    name = "fake_realtime_part",
    testonly = True,
    hdrs = ["fake_realtime_part.h"],
    visibility = ["//intrinsic_control/intrinsic/icon:rtcl_users"],
    deps = [
        "//intrinsic_control/intrinsic/icon/cc_client:operational_status",
        "//intrinsic_control/intrinsic/icon/common:part_properties",
        "//intrinsic_control/intrinsic/icon/control:realtime_bridge_types",
        "//intrinsic_control/intrinsic/icon/control/parts:feature_interface_registry",
        "//intrinsic_control/intrinsic/icon/control/parts:realtime_part_interface",
        "//intrinsic_control/intrinsic/icon/control/parts:realtime_part_property_access",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status_or",
        "@abseil-cpp//absl/algorithm:container",
        "@abseil-cpp//absl/container:fixed_array",
        "@abseil-cpp//absl/log:check",
    ],
)

cc_library(
    name = "fake_vehicle_realtime_part",
    testonly = True,
    srcs = ["fake_vehicle_realtime_part.cc"],
    hdrs = ["fake_vehicle_realtime_part.h"],
    visibility = ["//intrinsic_control/intrinsic/icon:rtcl_users"],
    deps = [
        "//intrinsic_control/intrinsic/icon/common:part_properties",
        "//intrinsic_control/intrinsic/icon/control:realtime_bridge_types",
        "//intrinsic_control/intrinsic/icon/control:realtime_operational_status",
        "//intrinsic_control/intrinsic/icon/control/parts:feature_interface_registry",
        "//intrinsic_control/intrinsic/icon/control/parts:feature_interfaces",
        "//intrinsic_control/intrinsic/icon/control/parts:realtime_part_interface",
        "//intrinsic_control/intrinsic/icon/control/parts:realtime_part_property_access",
        "//intrinsic_control/intrinsic/icon/control/parts/feature_interfaces:vehicle_body_features",
        "//intrinsic_control/intrinsic/icon/hal:hardware_interface_handle",
        "//intrinsic_control/intrinsic/icon/hal:hardware_interface_registry",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:vehicle_hal_fbs_cc",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:vehicle_hal_utils",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:vehicle_hardware_interfaces",
        "//intrinsic_control/intrinsic/icon/interprocess/shared_memory_manager",
        "//intrinsic_control/intrinsic/icon/utils:core_time",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status_or",
        "@abseil-cpp//absl/container:fixed_array",
        "@abseil-cpp//absl/memory",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
    ],
)

cc_library(
    name = "vehicle_cycle_guard",
    testonly = True,
    srcs = ["vehicle_cycle_guard.cc"],
    hdrs = ["vehicle_cycle_guard.h"],
    deps = [
        "//intrinsic_control/intrinsic/icon/testing:malloc_test",
        "//intrinsic_control/intrinsic/icon/utils:core_time",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status",
        "@abseil-cpp//absl/log:check",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cc_test_and_malloc_test(
    name = "vehicle_cycle_guard_test",
    size = "small",
    srcs = ["vehicle_cycle_guard_test.cc"],
    deps = [
        ":fake_realtime_part",
        ":fake_vehicle_realtime_part",
        ":vehicle_cycle_guard",
        "//intrinsic_control/intrinsic/icon/actions:bounded_body_wrench_info",
        "//intrinsic_control/intrinsic/icon/actions:bounded_body_wrench_signature",
        "//intrinsic_control/intrinsic/icon/common:id_types",
        "//intrinsic_control/intrinsic/icon/common:part_properties",
        "//intrinsic_control/intrinsic/icon/control:action_factory_context",
        "//intrinsic_control/intrinsic/icon/control:realtime_signal_access",
        "//intrinsic_control/intrinsic/icon/control:realtime_signal_storage",
        "//intrinsic_control/intrinsic/icon/control:realtime_slot_map",
        "//intrinsic_control/intrinsic/icon/control:slot_types",
        "//intrinsic_control/intrinsic/icon/control:streaming_io_helpers",
        "//intrinsic_control/intrinsic/icon/control:streaming_io_realtime",
        "//intrinsic_control/intrinsic/icon/control:streaming_io_storage",
        "//intrinsic_control/intrinsic/icon/control/actions:bounded_body_wrench_action",
        "//intrinsic_control/intrinsic/icon/control/parts:feature_interface_registry",
        "//intrinsic_control/intrinsic/icon/control/parts:feature_interfaces",
        "//intrinsic_control/intrinsic/icon/control/parts:realtime_part_interface",
        "//intrinsic_control/intrinsic/icon/control/parts:realtime_part_property_access",
        "//intrinsic_control/intrinsic/icon/utils:core_time",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status",
        "//intrinsic_kinematics/intrinsic/kinematics/types:joint_trajectories",
        "@abseil-cpp//absl/container:fixed_array",
        "@abseil-cpp//absl/container:flat_hash_map",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/strings",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/icon/proto/v1:types_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_cc_proto",
    ],
)

cc_test(
    name = "fake_vehicle_realtime_part_test",
    size = "small",
    srcs = ["fake_vehicle_realtime_part_test.cc"],
    deps = [
        ":fake_vehicle_realtime_part",
        "//intrinsic_control/intrinsic/icon/control:realtime_operational_status",
        "//intrinsic_control/intrinsic/icon/control/parts:feature_interfaces",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:vehicle_hal_fbs_cc",
        "//intrinsic_control/intrinsic/icon/utils:core_time",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status_matchers",
        "@abseil-cpp//absl/status",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/icon/proto/v1:types_cc_proto",
    ],
)
