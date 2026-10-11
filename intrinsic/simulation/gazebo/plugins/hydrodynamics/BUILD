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

# Opt-in hydrodynamics system plugin. Not part of the protected
# manipulator baseline. Do not add these targets to
# .github/baseline/manipulator_targets.tsv.

load("//bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hydrodynamics_adapter",
    srcs = ["hydrodynamics_adapter.cc"],
    hdrs = ["hydrodynamics_adapter.h"],
    deps = [
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/world/current_field_component:current_field_component_policy",
        "//intrinsic_vehicle/intrinsic/vehicle/dynamics",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@abseil-cpp//absl/strings",
        "@sdformat",
    ],
)

cc_library(
    name = "hydrodynamics_plugin",
    srcs = ["hydrodynamics_plugin.cc"],
    hdrs = ["hydrodynamics_plugin.h"],
    deps = [
        ":hydrodynamics_adapter",
        "//intrinsic/simulation/gazebo/plugins:priority_constants",
        "@abseil-cpp//absl/log",
        "@gz-math",
        "@gz-sim",
        "@sdformat",
    ],
)

cc_library(
    name = "hydrodynamics_plugin_register",
    srcs = ["hydrodynamics_plugin_register.cc"],
    deps = [
        ":hydrodynamics_plugin",
        "@gz-plugin//:register",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_test(
    name = "hydrodynamics_adapter_test",
    size = "small",
    srcs = ["hydrodynamics_adapter_test.cc"],
    data = [
        "hydrodynamics_adapter.cc",
        "hydrodynamics_adapter.h",
        "hydrodynamics_plugin.cc",
        "hydrodynamics_plugin.h",
        "hydrodynamics_plugin_register.cc",
        "//intrinsic/scene/sdf:testdata/current_field_hydrodynamics_constant.golden.sdf",
    ],
    deps = [
        ":hydrodynamics_adapter",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/scene/sdf:current_field_to_sdf",
        "//intrinsic_vehicle/intrinsic/vehicle/dynamics",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "hydrodynamics_plugin_test",
    size = "small",
    srcs = ["hydrodynamics_plugin_test.cc"],
    deps = [
        ":hydrodynamics_plugin",
        ":hydrodynamics_plugin_register",
        "//intrinsic/simulation/gazebo/plugins:priority_constants",
        "@com_google_googletest//:gtest_main",
        "@gz-math",
        "@gz-msgs",
        "@gz-sim",
    ],
)
