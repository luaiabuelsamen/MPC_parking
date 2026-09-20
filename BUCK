SANITIZER_FLAGS = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"] if read_config("mpcpark", "sanitizers", "false") == "true" else []

CXX_FLAGS = [
    "-std=c++17",
    "-pthread",
    "-O2",
    "-Wall",
    "-Wextra",
    "-Wpedantic",
] + SANITIZER_FLAGS

cxx_library(
    name = "mpcpark",
    srcs = [
        "src/geometry.cpp",
        "src/ilqr.cpp",
        "src/multi_agent.cpp",
        "src/planner.cpp",
        "src/scenario.cpp",
        "src/safety.cpp",
        "src/simulator.cpp",
        "src/trajectory.cpp",
        "src/vehicle.cpp",
    ],
    exported_headers = {path.removeprefix("include/"): path for path in glob(["include/**/*.hpp"])},
    header_namespace = "",
    exported_linker_flags = ["-pthread"] + SANITIZER_FLAGS,
    compiler_flags = CXX_FLAGS,
    tests = [":mpcpark_tests", ":safety_tests"],
    visibility = ["PUBLIC"],
)

cxx_binary(
    name = "mpc_parking",
    srcs = ["apps/mpc_parking.cpp"],
    compiler_flags = CXX_FLAGS,
    deps = [":mpcpark"],
)

cxx_binary(
    name = "mpc_simulator",
    srcs = ["apps/mpc_simulator.cpp"],
    compiler_flags = CXX_FLAGS,
    deps = [":mpcpark"],
)

cxx_binary(
    name = "distributed_ilqr",
    srcs = ["apps/distributed_ilqr.cpp"],
    compiler_flags = CXX_FLAGS,
    deps = [":mpcpark"],
)

cxx_binary(
    name = "distributed_stress",
    srcs = ["apps/distributed_stress.cpp"],
    compiler_flags = CXX_FLAGS,
    deps = [":mpcpark"],
)

cxx_test(
    name = "mpcpark_tests",
    srcs = ["tests/mpcpark_tests.cpp"],
    compiler_flags = CXX_FLAGS,
    deps = [":mpcpark"],
)

cxx_test(
    name = "safety_tests",
    srcs = ["tests/safety_tests.cpp"],
    compiler_flags = CXX_FLAGS,
    deps = [":mpcpark"],
)
