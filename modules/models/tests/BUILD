cc_test(
    name = "dynamic_tests",
    srcs = [
        "dynamic_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/models/dynamic",
        "@boost//:geometry",
        "@gtest//:main",
    ],
)

cc_test(
    name = "execution_test",
    srcs = [
        "execution_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/models/dynamic",
        #"//modules/models/execution/mpc:mpc",
        "//modules/models/execution/interpolation",
        "@gtest//:main",
    ],
)

cc_test(
    name = "behavior_motion_primitive_test",
    srcs = [
        "behavior_motion_primitive_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/models/behavior/motion_primitives",
        "//modules/models/dynamic",
        "//modules/world",
        "@com_github_eigen_eigen//:eigen",
        "@gtest//:main",
    ],
)

cc_test(
    name = "behavior_longitudinal_acceleration_test",
    srcs = [
        "behavior_longitudinal_acceleration_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":make_test_world",
        "//modules/geometry",
        "//modules/models/behavior/longitudinal_acceleration",
        "//modules/models/dynamic",
        "//modules/world",
        "@com_github_eigen_eigen//:eigen",
        "@gtest//:main",
    ],
)

cc_test(
    name = "behavior_idm_classic_test",
    srcs = [
        "behavior_idm_classic_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":make_test_world",
        "//modules/geometry",
        "//modules/models/behavior/constant_velocity",
        "//modules/models/behavior/idm:idm_classic",
        "//modules/models/execution/interpolation",
        "@com_github_eigen_eigen//:eigen",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "behavior_static_trajectory_test",
    srcs = [
        "behavior_static_trajectory_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/models/behavior/static_trajectory",
        "@com_github_eigen_eigen//:eigen",
        "@gtest//:main",
    ],
)

cc_library(
    name = "make_test_world",
    srcs = [
        "make_test_world.cpp",
    ],
    hdrs = [
        "make_test_world.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/geometry",
        "//modules/models/behavior/constant_velocity",
        "//modules/models/behavior/idm:idm_classic",
        "//modules/models/execution/interpolation",
        "@com_github_eigen_eigen//:eigen",
    ],
)

py_test(
    name = "py_behavior_model_test",
    srcs = ["py_behavior_model_test.py"],
    data = [
        "//modules/runtime/tests:xodr_data",
        "//python:bark.so",
    ],
    imports = ["../../../python/"],
    deps = [
        "//modules/runtime",
        "//modules/runtime/commons:parameters",
        "//modules/runtime/commons:xodr_parser",
        "//modules/runtime/scenario/scenario_generation",
    ],
)
