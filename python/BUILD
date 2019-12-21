package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "bark.so",
    srcs = glob([
        "*.cpp",
        "*.hpp",
        "commons/*.cpp",
        "commons/*.hpp",
        "geometry/*.cpp",
        "geometry/*.hpp",
        "world/*.cpp",
        "world/*.hpp",
        "models/*.cpp",
        "models/*.hpp",
        "viewer/*.cpp",
        "viewer/*.hpp",
        "runtime.hpp",
    ]),
    linkshared = 1,
    linkstatic = 1,
    deps = [
        "@python_linux//:python-lib",
        "@pybind11",
        "@com_github_eigen_eigen//:eigen",
        "//modules/geometry",
        "//modules/world/opendrive",
        "//modules/world/map:roadgraph",
        "//modules/world/goal_definition",
        "//modules/world/evaluation",
        "//modules/commons/params",
        "//modules/models/execution/interpolation",
        "//modules/models/behavior/constant_velocity",
        "//modules/models/behavior/motion_primitives",
        "//modules/models/behavior/dynamic_model",
        "//modules/models/behavior/idm:idm_classic",
        "//modules/models/behavior/static_trajectory",
        #"//modules/models/execution/mpc:mpc",
        "//modules/runtime/viewer",
        "//modules/world",
        "//modules/runtime:cc_runtime",
        "//python/models/plan:planners",
    ],
)

# needed for external python wrapping (planners)
cc_library(
    name = "common",
    hdrs = ["common.hpp"],
    deps = [
        "@com_github_eigen_eigen//:eigen",
        "@pybind11",
        "@python_linux//:python-lib",
    ],
)

# needed to test external python wrapping of planners in their own modules
cc_library(
    name = "behavior",
    srcs = ["models/behavior.cpp"],
    hdrs = ["models/behavior.hpp"],
    deps = [
        ":common",
        "//modules/models/behavior/constant_velocity",
        "//modules/models/behavior/dynamic_model",
        "//modules/models/behavior/idm:idm_classic",
        "//modules/models/behavior/motion_primitives",
        "//modules/models/behavior/static_trajectory",
        "//python/models/plan:planners",
        "@com_github_eigen_eigen//:eigen",
        "@pybind11",
        "@python_linux//:python-lib",
    ],
)
