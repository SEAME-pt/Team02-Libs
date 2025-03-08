cc_library(
    name = "I2C",
    srcs = ["src/I2C.cpp" , "src/MockFunctions.cpp"],
    hdrs = glob(["include/**/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    copts = ["-DTEST_MODE", "--coverage"],  # Add this line to enable coverage
    linkopts = ["--coverage"],  # Add this line to enable coverage
)