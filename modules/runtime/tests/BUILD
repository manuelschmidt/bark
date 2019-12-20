filegroup(
    name = "xodr_data",
    srcs = glob(["data/**"]),
    visibility = ["//visibility:public"],
)

py_test(
    name = "py_importer_tests",
    srcs = ["py_importer_tests.py"],
    data = [
        ":xodr_data",
        "//python:bark.so",
    ],
    imports = ["../../../python/"],
    deps = [
        "//modules/runtime",
        "//modules/runtime/commons:parameters",
        "//modules/runtime/commons:xodr_parser",
    ],
)

py_test(
    name = "py_scenario_generation_tests",
    srcs = ["py_scenario_generation_tests.py"],
    data = [
        ":xodr_data",
        "//python:bark.so",
        "@com_github_interaction-dataset_interaction-dataset//:interaction_test_data",
    ],
    imports = ["../../../python/"],
    deps = [
        "//modules/runtime/commons:parameters",
        "//modules/runtime/commons:xodr_parser",
        "//modules/runtime/scenario/scenario_generation",
    ],
)

py_test(
    name = "py_runtime_tests",
    srcs = ["py_runtime_tests.py"],
    data = [
        ":xodr_data",
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

py_test(
    name = "py_evaluation_tests",
    srcs = ["py_evaluation_tests.py"],
    data = ["//python:bark.so"],
    imports = ["../python/"],
    deps = [
        "//modules/runtime",
        "//modules/runtime/commons:parameters",
    ],
)
