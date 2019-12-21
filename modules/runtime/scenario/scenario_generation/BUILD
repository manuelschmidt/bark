py_library(
    name = "scenario_generation",
    srcs = [
        "deterministic.py",
        "deterministic_drone_challenge.py",
        "drone_challenge.py",
        "interaction_dataset_reader.py",
        "model_json_conversion.py",
        "scenario_generation.py",
        "uniform_vehicle_distribution.py",
    ],
    data = ["//python:bark.so"],
    imports = ["../../../python"],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/runtime/scenario",
        "@com_github_interaction-dataset_interaction-dataset//:interaction-tools",
    ],
)
