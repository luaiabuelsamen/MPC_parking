CXX_FLAGS = [
    "-std=c++17",
    "-Wall",
    "-Wextra",
    "-Wpedantic",
]

cxx_library(
    name = "mpcpark",
    srcs = [
        "src/geometry.cpp",
        "src/ilqr.cpp",
        "src/multi_agent.cpp",
        "src/planner.cpp",
        "src/scenario.cpp",
        "src/simulator.cpp",
        "src/trajectory.cpp",
        "src/vehicle.cpp",
    ],
    headers = glob(["include/**/*.hpp"]),
    public_include_directories = ["include"],
    compiler_flags = CXX_FLAGS,
    tests = [":mpcpark_tests"],
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

cxx_test(
    name = "mpcpark_tests",
    srcs = ["tests/mpcpark_tests.cpp"],
    compiler_flags = CXX_FLAGS,
    deps = [":mpcpark"],
)
