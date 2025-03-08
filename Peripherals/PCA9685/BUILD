cc_library(
    name = "PCA9685",
    srcs = ["src/PCA9685.cpp"],
    hdrs = glob(["include/**/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    copts = ["-DTEST_MODE", "--coverage"],  # Add this line to enable coverage
    linkopts = ["--coverage"],
    deps = [
        "//Communication/I2C:I2C",  # Add this line to include the I2C library as a dependency
    ],# Add this line to enable coverage
)