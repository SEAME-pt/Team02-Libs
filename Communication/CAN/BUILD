cc_library(
    name = "CAN",
    srcs = ["src/CAN.cpp"],
    hdrs = glob(["include/**/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    copts = ["--coverage"],  # Add --coverage to enable coverage
    linkopts = ["--coverage"],  # Add this line to enable coverage
)