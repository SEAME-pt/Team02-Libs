cc_library(
    name = "CAN",
    srcs = ["src/CAN.cpp"],
    hdrs = glob(["include/**/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    copts = ["-DTEST_MODE", "--coverage"],  # Add this line to enable coverage
    linkopts = ["--coverage"],  # Add this line to enable coverage
)