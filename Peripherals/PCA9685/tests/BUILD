cc_test (
    name = "pca9685_tests",
    srcs = ["pca9685_tests.cpp"],
    deps = [
        "//Peripherals/PCA9685:PCA9685",
        "@catch2//:catch2",
    ],
    visibility = ["//visibility:public"],
    copts = ["--coverage"],  # Add this line to enable coverage
    linkopts = ["--coverage"],  # Add this line to enable coverage
)