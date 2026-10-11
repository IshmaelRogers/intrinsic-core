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

# Opt-in seeded marine parameter sampling. Not part of the protected
# manipulator baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "marine_parameter_sampler",
    srcs = ["marine_parameter_sampler.cc"],
    hdrs = ["marine_parameter_sampler.h"],
    deps = [
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/world/current_field_component:current_field_component_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_imu",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)

cc_test(
    name = "marine_parameter_sampler_test",
    size = "small",
    srcs = ["marine_parameter_sampler_test.cc"],
    data = ["testdata/marine_parameter_sampler_seed_42.golden.txt"],
    deps = [
        ":marine_parameter_sampler",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/world/current_field_component:current_field_component_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:fake_imu",
        "//intrinsic_hardware/intrinsic/hardware/marine:imu_cc_proto",
        "//intrinsic_hardware/intrinsic/hardware/marine:imu_policy",
        "//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@com_google_googletest//:gtest_main",
    ],
)
