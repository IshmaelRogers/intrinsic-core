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

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("//bazel:cc_macros.bzl", "cc_library", "cc_test")
load("//bazel:protos.bzl", "proto_library")
load("//bazel:python.bzl", "py_library", "py_test")

# Opt-in marine measurement health. Not part of the protected manipulator
# baseline. Do not add these labels to
# .github/baseline/manipulator_targets.tsv.

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "measurement_health_proto",
    srcs = ["measurement_health.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_proto",
    ],
)

cc_proto_library(
    name = "measurement_health_cc_proto",
    deps = [":measurement_health_proto"],
)

py_proto_library(
    name = "measurement_health_py_pb2",
    deps = [":measurement_health_proto"],
)

cc_library(
    name = "measurement_health_policy",
    hdrs = ["measurement_health_policy.h"],
    deps = [
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_test(
    name = "measurement_health_policy_test",
    size = "small",
    srcs = ["measurement_health_policy_test.cc"],
    deps = [
        ":measurement_health_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "measurement_health_serialization_test",
    size = "small",
    srcs = ["measurement_health_serialization_test.cc"],
    data = [":examples"],
    deps = [
        ":measurement_health_cc_proto",
        ":measurement_health_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

py_library(
    name = "measurement_health_policy_py",
    srcs = ["measurement_health_policy.py"],
    deps = [
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "measurement_health_policy_test_py",
    size = "small",
    srcs = ["measurement_health_policy_test.py"],
    main = "measurement_health_policy_test.py",
    deps = [
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "measurement_health_serialization_test_py",
    size = "small",
    srcs = ["measurement_health_serialization_test.py"],
    data = [":examples"],
    main = "measurement_health_serialization_test.py",
    deps = [
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_py_pb2",
    ],
)

proto_library(
    name = "dvl_proto",
    srcs = ["dvl.proto"],
    deps = [":measurement_health_proto"],
)

cc_proto_library(
    name = "dvl_cc_proto",
    deps = [":dvl_proto"],
)

py_proto_library(
    name = "dvl_py_pb2",
    deps = [":dvl_proto"],
)

cc_library(
    name = "dvl_policy",
    hdrs = ["dvl_policy.h"],
    deps = [
        ":measurement_health_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_library(
    name = "fake_dvl",
    srcs = ["fake_dvl.cc"],
    hdrs = ["fake_dvl.h"],
    deps = [
        ":dvl_cc_proto",
        ":measurement_health_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

cc_test(
    name = "dvl_policy_test",
    size = "small",
    srcs = ["dvl_policy_test.cc"],
    deps = [
        ":dvl_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "dvl_serialization_test",
    size = "small",
    srcs = ["dvl_serialization_test.cc"],
    data = [":examples"],
    deps = [
        ":dvl_cc_proto",
        ":dvl_policy",
        ":fake_dvl",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

cc_test(
    name = "fake_dvl_test",
    size = "small",
    srcs = ["fake_dvl_test.cc"],
    deps = [
        ":dvl_cc_proto",
        ":dvl_policy",
        ":fake_dvl",
        ":measurement_health_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "dvl_policy_py",
    srcs = ["dvl_policy.py"],
    deps = [
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_library(
    name = "fake_dvl_py",
    srcs = ["fake_dvl.py"],
    deps = [
        ":dvl_py_pb2",
        ":measurement_health_py_pb2",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "dvl_policy_test_py",
    size = "small",
    srcs = ["dvl_policy_test.py"],
    main = "dvl_policy_test.py",
    deps = [
        ":dvl_policy_py",
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "dvl_serialization_test_py",
    size = "small",
    srcs = ["dvl_serialization_test.py"],
    data = [":examples"],
    main = "dvl_serialization_test.py",
    deps = [
        ":dvl_policy_py",
        ":dvl_py_pb2",
        ":fake_dvl_py",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "fake_dvl_test_py",
    size = "small",
    srcs = ["fake_dvl_test.py"],
    main = "fake_dvl_test.py",
    deps = [
        ":dvl_policy_py",
        ":dvl_py_pb2",
        ":fake_dvl_py",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
    ],
)

proto_library(
    name = "pressure_depth_proto",
    srcs = ["pressure_depth.proto"],
    deps = [":measurement_health_proto"],
)

cc_proto_library(
    name = "pressure_depth_cc_proto",
    deps = [":pressure_depth_proto"],
)

py_proto_library(
    name = "pressure_depth_py_pb2",
    deps = [":pressure_depth_proto"],
)

cc_library(
    name = "pressure_depth_policy",
    hdrs = ["pressure_depth_policy.h"],
    deps = [
        ":measurement_health_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_library(
    name = "fake_pressure_depth",
    srcs = ["fake_pressure_depth.cc"],
    hdrs = ["fake_pressure_depth.h"],
    deps = [
        ":measurement_health_cc_proto",
        ":pressure_depth_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

cc_test(
    name = "pressure_depth_policy_test",
    size = "small",
    srcs = ["pressure_depth_policy_test.cc"],
    deps = [
        ":pressure_depth_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pressure_depth_serialization_test",
    size = "small",
    srcs = ["pressure_depth_serialization_test.cc"],
    data = [":examples"],
    deps = [
        ":fake_pressure_depth",
        ":pressure_depth_cc_proto",
        ":pressure_depth_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

cc_test(
    name = "fake_pressure_depth_test",
    size = "small",
    srcs = ["fake_pressure_depth_test.cc"],
    deps = [
        ":fake_pressure_depth",
        ":measurement_health_cc_proto",
        ":pressure_depth_cc_proto",
        ":pressure_depth_policy",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "pressure_depth_policy_py",
    srcs = ["pressure_depth_policy.py"],
    deps = [
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_library(
    name = "fake_pressure_depth_py",
    srcs = ["fake_pressure_depth.py"],
    deps = [
        ":measurement_health_py_pb2",
        ":pressure_depth_py_pb2",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "pressure_depth_policy_test_py",
    size = "small",
    srcs = ["pressure_depth_policy_test.py"],
    main = "pressure_depth_policy_test.py",
    deps = [
        ":measurement_health_policy_py",
        ":pressure_depth_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "pressure_depth_serialization_test_py",
    size = "small",
    srcs = ["pressure_depth_serialization_test.py"],
    data = [":examples"],
    main = "pressure_depth_serialization_test.py",
    deps = [
        ":fake_pressure_depth_py",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        ":pressure_depth_policy_py",
        ":pressure_depth_py_pb2",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "fake_pressure_depth_test_py",
    size = "small",
    srcs = ["fake_pressure_depth_test.py"],
    main = "fake_pressure_depth_test.py",
    deps = [
        ":fake_pressure_depth_py",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        ":pressure_depth_policy_py",
        ":pressure_depth_py_pb2",
    ],
)

proto_library(
    name = "altimeter_proto",
    srcs = ["altimeter.proto"],
    deps = [":measurement_health_proto"],
)

cc_proto_library(
    name = "altimeter_cc_proto",
    deps = [":altimeter_proto"],
)

py_proto_library(
    name = "altimeter_py_pb2",
    deps = [":altimeter_proto"],
)

cc_library(
    name = "altimeter_policy",
    hdrs = ["altimeter_policy.h"],
    deps = [
        ":measurement_health_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_library(
    name = "fake_altimeter",
    srcs = ["fake_altimeter.cc"],
    hdrs = ["fake_altimeter.h"],
    deps = [
        ":altimeter_cc_proto",
        ":measurement_health_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

cc_test(
    name = "altimeter_policy_test",
    size = "small",
    srcs = ["altimeter_policy_test.cc"],
    deps = [
        ":altimeter_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "altimeter_serialization_test",
    size = "small",
    srcs = ["altimeter_serialization_test.cc"],
    data = [":examples"],
    deps = [
        ":altimeter_cc_proto",
        ":altimeter_policy",
        ":fake_altimeter",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

cc_test(
    name = "fake_altimeter_test",
    size = "small",
    srcs = ["fake_altimeter_test.cc"],
    deps = [
        ":altimeter_cc_proto",
        ":altimeter_policy",
        ":fake_altimeter",
        ":measurement_health_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "altimeter_policy_py",
    srcs = ["altimeter_policy.py"],
    deps = [
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_library(
    name = "fake_altimeter_py",
    srcs = ["fake_altimeter.py"],
    deps = [
        ":altimeter_py_pb2",
        ":measurement_health_py_pb2",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "altimeter_policy_test_py",
    size = "small",
    srcs = ["altimeter_policy_test.py"],
    main = "altimeter_policy_test.py",
    deps = [
        ":altimeter_policy_py",
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "altimeter_serialization_test_py",
    size = "small",
    srcs = ["altimeter_serialization_test.py"],
    data = [":examples"],
    main = "altimeter_serialization_test.py",
    deps = [
        ":altimeter_policy_py",
        ":altimeter_py_pb2",
        ":fake_altimeter_py",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "fake_altimeter_test_py",
    size = "small",
    srcs = ["fake_altimeter_test.py"],
    main = "fake_altimeter_test.py",
    deps = [
        ":altimeter_policy_py",
        ":altimeter_py_pb2",
        ":fake_altimeter_py",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
    ],
)

proto_library(
    name = "imu_proto",
    srcs = ["imu.proto"],
    deps = [":measurement_health_proto"],
)

cc_proto_library(
    name = "imu_cc_proto",
    deps = [":imu_proto"],
)

py_proto_library(
    name = "imu_py_pb2",
    deps = [":imu_proto"],
)

cc_library(
    name = "imu_policy",
    hdrs = ["imu_policy.h"],
    deps = [
        ":measurement_health_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_library(
    name = "fake_imu",
    srcs = ["fake_imu.cc"],
    hdrs = ["fake_imu.h"],
    deps = [
        ":imu_cc_proto",
        ":imu_policy",
        ":measurement_health_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

cc_test(
    name = "imu_policy_test",
    size = "small",
    srcs = ["imu_policy_test.cc"],
    deps = [
        ":imu_policy",
        ":ins_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "imu_serialization_test",
    size = "small",
    srcs = ["imu_serialization_test.cc"],
    data = [":examples"],
    deps = [
        ":fake_imu",
        ":imu_cc_proto",
        ":imu_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

cc_test(
    name = "fake_imu_test",
    size = "small",
    srcs = ["fake_imu_test.cc"],
    deps = [
        ":fake_imu",
        ":imu_cc_proto",
        ":imu_policy",
        ":measurement_health_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "imu_policy_py",
    srcs = ["imu_policy.py"],
    deps = [
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_library(
    name = "fake_imu_py",
    srcs = ["fake_imu.py"],
    deps = [
        ":imu_policy_py",
        ":imu_py_pb2",
        ":measurement_health_py_pb2",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "imu_policy_test_py",
    size = "small",
    srcs = ["imu_policy_test.py"],
    main = "imu_policy_test.py",
    deps = [
        ":imu_policy_py",
        ":ins_policy_py",
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "imu_serialization_test_py",
    size = "small",
    srcs = ["imu_serialization_test.py"],
    data = [":examples"],
    main = "imu_serialization_test.py",
    deps = [
        ":fake_imu_py",
        ":imu_policy_py",
        ":imu_py_pb2",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "fake_imu_test_py",
    size = "small",
    srcs = ["fake_imu_test.py"],
    main = "fake_imu_test.py",
    deps = [
        ":fake_imu_py",
        ":imu_policy_py",
        ":imu_py_pb2",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
    ],
)

proto_library(
    name = "ins_proto",
    srcs = ["ins.proto"],
    deps = [":measurement_health_proto"],
)

cc_proto_library(
    name = "ins_cc_proto",
    deps = [":ins_proto"],
)

py_proto_library(
    name = "ins_py_pb2",
    deps = [":ins_proto"],
)

cc_library(
    name = "ins_policy",
    hdrs = ["ins_policy.h"],
    deps = [
        ":measurement_health_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_library(
    name = "fake_ins",
    srcs = ["fake_ins.cc"],
    hdrs = ["fake_ins.h"],
    deps = [
        ":ins_cc_proto",
        ":ins_policy",
        ":measurement_health_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

cc_test(
    name = "ins_policy_test",
    size = "small",
    srcs = ["ins_policy_test.cc"],
    deps = [
        ":ins_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ins_serialization_test",
    size = "small",
    srcs = ["ins_serialization_test.cc"],
    data = [":examples"],
    deps = [
        ":fake_ins",
        ":ins_cc_proto",
        ":ins_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

cc_test(
    name = "fake_ins_test",
    size = "small",
    srcs = ["fake_ins_test.cc"],
    deps = [
        ":fake_ins",
        ":ins_cc_proto",
        ":ins_policy",
        ":measurement_health_cc_proto",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "ins_policy_py",
    srcs = ["ins_policy.py"],
    deps = [
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_library(
    name = "fake_ins_py",
    srcs = ["fake_ins.py"],
    deps = [
        ":ins_policy_py",
        ":ins_py_pb2",
        ":measurement_health_py_pb2",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "ins_policy_test_py",
    size = "small",
    srcs = ["ins_policy_test.py"],
    main = "ins_policy_test.py",
    deps = [
        ":ins_policy_py",
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "ins_serialization_test_py",
    size = "small",
    srcs = ["ins_serialization_test.py"],
    data = [":examples"],
    main = "ins_serialization_test.py",
    deps = [
        ":fake_ins_py",
        ":ins_policy_py",
        ":ins_py_pb2",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "fake_ins_test_py",
    size = "small",
    srcs = ["fake_ins_test.py"],
    main = "fake_ins_test.py",
    deps = [
        ":fake_ins_py",
        ":ins_policy_py",
        ":ins_py_pb2",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
    ],
)

proto_library(
    name = "surface_fix_proto",
    srcs = ["surface_fix.proto"],
    deps = [":measurement_health_proto"],
)

cc_proto_library(
    name = "surface_fix_cc_proto",
    deps = [":surface_fix_proto"],
)

py_proto_library(
    name = "surface_fix_py_pb2",
    deps = [":surface_fix_proto"],
)

cc_library(
    name = "surface_fix_policy",
    hdrs = ["surface_fix_policy.h"],
    deps = [
        ":measurement_health_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_library(
    name = "fake_surface_fix",
    srcs = ["fake_surface_fix.cc"],
    hdrs = ["fake_surface_fix.h"],
    deps = [
        ":measurement_health_cc_proto",
        ":surface_fix_cc_proto",
        ":surface_fix_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
        "@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto",
    ],
)

cc_test(
    name = "surface_fix_policy_test",
    size = "small",
    srcs = ["surface_fix_policy_test.cc"],
    deps = [
        ":surface_fix_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "surface_fix_serialization_test",
    size = "small",
    srcs = ["surface_fix_serialization_test.cc"],
    data = [":examples"],
    deps = [
        ":fake_surface_fix",
        ":measurement_health_cc_proto",
        ":surface_fix_cc_proto",
        ":surface_fix_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

cc_test(
    name = "fake_surface_fix_test",
    size = "small",
    srcs = ["fake_surface_fix_test.cc"],
    deps = [
        ":fake_surface_fix",
        ":measurement_health_cc_proto",
        ":surface_fix_cc_proto",
        ":surface_fix_policy",
        "//intrinsic/embodiment:frame_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

py_library(
    name = "surface_fix_policy_py",
    srcs = ["surface_fix_policy.py"],
    deps = [
        ":measurement_health_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_library(
    name = "fake_surface_fix_py",
    srcs = ["fake_surface_fix.py"],
    deps = [
        ":measurement_health_py_pb2",
        ":surface_fix_policy_py",
        ":surface_fix_py_pb2",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "surface_fix_policy_test_py",
    size = "small",
    srcs = ["surface_fix_policy_test.py"],
    main = "surface_fix_policy_test.py",
    deps = [
        ":measurement_health_policy_py",
        ":surface_fix_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "surface_fix_serialization_test_py",
    size = "small",
    srcs = ["surface_fix_serialization_test.py"],
    data = [":examples"],
    main = "surface_fix_serialization_test.py",
    deps = [
        ":fake_surface_fix_py",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        ":surface_fix_policy_py",
        ":surface_fix_py_pb2",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "fake_surface_fix_test_py",
    size = "small",
    srcs = ["fake_surface_fix_test.py"],
    main = "fake_surface_fix_test.py",
    deps = [
        ":fake_surface_fix_py",
        ":measurement_health_policy_py",
        ":measurement_health_py_pb2",
        ":surface_fix_policy_py",
        ":surface_fix_py_pb2",
    ],
)

proto_library(
    name = "thruster_array_proto",
    srcs = ["thruster_array.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_proto",
    ],
)

cc_proto_library(
    name = "thruster_array_cc_proto",
    deps = [":thruster_array_proto"],
)

py_proto_library(
    name = "thruster_array_py_pb2",
    deps = [":thruster_array_proto"],
)

cc_library(
    name = "thruster_array_policy",
    hdrs = ["thruster_array_policy.h"],
    deps = [
        "//intrinsic/vehicle:vehicle_contract_policy",
    ],
)

cc_library(
    name = "fake_thruster_array",
    srcs = ["fake_thruster_array.cc"],
    hdrs = ["fake_thruster_array.h"],
    deps = [
        ":thruster_array_cc_proto",
        ":thruster_array_policy",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

cc_test(
    name = "thruster_array_policy_test",
    size = "small",
    srcs = ["thruster_array_policy_test.cc"],
    deps = [
        ":thruster_array_policy",
        "//intrinsic/embodiment:frame_policy",
        "//intrinsic/vehicle:vehicle_contract_policy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "thruster_array_serialization_test",
    size = "small",
    srcs = ["thruster_array_serialization_test.cc"],
    data = [":examples"],
    deps = [
        ":fake_thruster_array",
        ":thruster_array_cc_proto",
        ":thruster_array_policy",
        "//intrinsic_vehicle/intrinsic/vehicle/parameters:parameters",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

cc_test(
    name = "fake_thruster_array_test",
    size = "small",
    srcs = ["fake_thruster_array_test.cc"],
    deps = [
        ":fake_thruster_array",
        ":thruster_array_cc_proto",
        ":thruster_array_policy",
        "@com_google_googletest//:gtest_main",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto",
    ],
)

py_library(
    name = "thruster_array_policy_py",
    srcs = ["thruster_array_policy.py"],
    deps = [
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_library(
    name = "fake_thruster_array_py",
    srcs = ["fake_thruster_array.py"],
    deps = [
        ":thruster_array_policy_py",
        ":thruster_array_py_pb2",
        "//intrinsic/embodiment:frame_policy_py",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "thruster_array_policy_test_py",
    size = "small",
    srcs = ["thruster_array_policy_test.py"],
    main = "thruster_array_policy_test.py",
    deps = [
        ":thruster_array_policy_py",
        "//intrinsic/embodiment:frame_policy_py",
        "//intrinsic/vehicle:vehicle_contract_policy_py",
    ],
)

py_test(
    name = "thruster_array_serialization_test_py",
    size = "small",
    srcs = ["thruster_array_serialization_test.py"],
    data = [":examples"],
    main = "thruster_array_serialization_test.py",
    deps = [
        ":fake_thruster_array_py",
        ":thruster_array_policy_py",
        ":thruster_array_py_pb2",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

py_test(
    name = "fake_thruster_array_test_py",
    size = "small",
    srcs = ["fake_thruster_array_test.py"],
    main = "fake_thruster_array_test.py",
    deps = [
        ":fake_thruster_array_py",
        ":thruster_array_policy_py",
        ":thruster_array_py_pb2",
        "@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2",
    ],
)

filegroup(
    name = "examples",
    srcs = [
        "examples/altimeter_nominal.textproto",
        "examples/dvl_bottom_track.textproto",
        "examples/imu_nominal.textproto",
        "examples/imu_rates_only.textproto",
        "examples/ins_nominal.textproto",
        "examples/measurement_health.textproto",
        "examples/measurement_health_unknown_covariance.textproto",
        "examples/pressure_depth_from_pressure.textproto",
        "examples/surface_fix_acoustic.textproto",
        "examples/surface_fix_gnss.textproto",
        "examples/thruster_array_command_nominal.textproto",
        "examples/thruster_array_feedback_nominal.textproto",
    ],
)
