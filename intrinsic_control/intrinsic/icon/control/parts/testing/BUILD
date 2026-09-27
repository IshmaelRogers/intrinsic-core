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

cc_test(
    name = "arm_feature_registration_golden_test",
    size = "small",
    srcs = ["arm_feature_registration_golden_test.cc"],
    data = ["testdata/arm_feature_registration.golden"],
    deps = [
        "//intrinsic_control/intrinsic/icon/control/parts:fake_feature_interfaces",
        "//intrinsic_control/intrinsic/icon/control/parts:feature_interface_registry",
        "//intrinsic_control/intrinsic/icon/control/parts:feature_interfaces",
        "//intrinsic_kinematics/intrinsic/kinematics/types:cartesian_limits",
        "//intrinsic_kinematics/intrinsic/kinematics/types:joint_limits",
        "//intrinsic_sdk/intrinsic/eigenmath",
        "//intrinsic_sdk/intrinsic/math:pose3",
        "@abseil-cpp//absl/container:flat_hash_set",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@com_google_googletest//:gtest",
        "@intrinsic_apis//intrinsic/icon/proto/v1:types_cc_proto",
    ],
)
