cc_test (
    name = "ina219_tests",
    srcs = ["ina219_tests.cpp"],
    deps = [
        "//Peripherals/INA219:INA219",
        "@catch2//:catch2",
    ],
    visibility = ["//visibility:public"],
    copts = ["--coverage"],  # Add this line to enable coverage
    linkopts = ["--coverage"],  # Add this line to enable coverage
)