cc_test (
    name = "i2c_tests",
    srcs = ["i2c_tests.cpp",  "MockFunctions.cpp"],
    deps = [
        "//Communication/I2C:I2C",
        "@catch2//:catch2",
    ],
    visibility = ["//visibility:public"],
    copts = ["--coverage"],  # Add this line to enable coverage
    linkopts = ["--coverage"],  # Add this line to enable coverage
)