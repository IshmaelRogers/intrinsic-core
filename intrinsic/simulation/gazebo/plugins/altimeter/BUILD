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

# Opt-in altimeter sensor system plugin. Not part of the protected manipulator
# baseline. Do not add these targets to
# .github/baseline/manipulator_targets.tsv.

load("//bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:private"])

cc_library(
    name = "altimeter_adapter",
    srcs = ["altimeter_adapter.cc"],
    hdrs = ["altimeter_adapter.h"],
    deps = [
        "//intrinsic_hardware/intrinsic/hardware/marine:altimeter_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_altimeter",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters",
        "@abseil-cpp//absl/strings",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@sdformat",
    ],
)

cc_library(
    name = "altimeter_plugin",
    srcs = ["altimeter_plugin.cc"],
    hdrs = ["altimeter_plugin.h"],
    visibility = ["//intrinsic/simulation/gazebo/scenarios/dive_resurface:__pkg__"],
    deps = [
        ":altimeter_adapter",
        "@abseil-cpp//absl/log",
        "@gz-math",
        "@gz-sim",
        "@sdformat",
    ],
)

cc_library(
    name = "altimeter_plugin_register",
    srcs = ["altimeter_plugin_register.cc"],
    visibility = ["//intrinsic/simulation/gazebo/plugins:__pkg__"],
    deps = [
        ":altimeter_plugin",
        "@gz-plugin//:register",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_test(
    name = "altimeter_adapter_test",
    size = "small",
    srcs = ["altimeter_adapter_test.cc"],
    data = [
        "altimeter_adapter.cc",
        "altimeter_adapter.h",
        "altimeter_plugin.cc",
        "altimeter_plugin.h",
        "altimeter_plugin_register.cc",
        "testdata/altimeter_adapter_seed_42.golden.txt",
    ],
    deps = [
        ":altimeter_adapter",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:altimeter_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:altimeter_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_altimeter",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "altimeter_plugin_test",
    size = "small",
    srcs = ["altimeter_plugin_test.cc"],
    deps = [
        ":altimeter_adapter",
        ":altimeter_plugin",
        ":altimeter_plugin_register",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:altimeter_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:altimeter_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy",
        "@abseil-cpp//absl/log:log_sink",
        "@abseil-cpp//absl/log:log_sink_registry",
        "@com_google_googletest//:gtest_main",
        "@gz-math",
        "@gz-msgs",
        "@gz-sim",
        "@sdformat",
    ],
)
