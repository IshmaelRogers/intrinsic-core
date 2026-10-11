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

# Opt-in DVL sensor system plugin. Not part of the protected manipulator
# baseline. Do not add these targets to
# .github/baseline/manipulator_targets.tsv.

load("//bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dvl_adapter",
    srcs = ["dvl_adapter.cc"],
    hdrs = ["dvl_adapter.h"],
    deps = [
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/world/current_field_component:current_field_component_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:dvl_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_dvl",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_imu",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters",
        "@abseil-cpp//absl/strings",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@sdformat",
    ],
)

cc_library(
    name = "dvl_plugin",
    srcs = ["dvl_plugin.cc"],
    hdrs = ["dvl_plugin.h"],
    deps = [
        ":dvl_adapter",
        "@abseil-cpp//absl/log",
        "@gz-math",
        "@gz-sim",
        "@sdformat",
    ],
)

cc_library(
    name = "dvl_plugin_register",
    srcs = ["dvl_plugin_register.cc"],
    deps = [
        ":dvl_plugin",
        "@gz-plugin//:register",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_test(
    name = "dvl_adapter_test",
    size = "small",
    srcs = ["dvl_adapter_test.cc"],
    data = [
        "dvl_adapter.cc",
        "dvl_adapter.h",
        "dvl_plugin.cc",
        "dvl_plugin.h",
        "dvl_plugin_register.cc",
        "testdata/dvl_adapter_seed_42.golden.txt",
        "//intrinsic/scene/sdf:testdata/current_field_hydrodynamics_constant.golden.sdf",
    ],
    deps = [
        ":dvl_adapter",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:dvl_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:dvl_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_dvl",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_imu",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "dvl_plugin_test",
    size = "small",
    srcs = ["dvl_plugin_test.cc"],
    deps = [
        ":dvl_adapter",
        ":dvl_plugin",
        ":dvl_plugin_register",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:dvl_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:dvl_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "@com_google_googletest//:gtest_main",
        "@gz-math",
        "@gz-msgs",
        "@gz-sim",
        "@sdformat",
    ],
)
