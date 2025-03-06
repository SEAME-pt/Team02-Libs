cc_test (
    name = "can_tests",
    srcs = ["can_tests.cpp", "MockFunctions.cpp"],
    deps = [
        "//Communication/CAN:CAN",
        "@catch2//:catch2",
    ],
    visibility = ["//visibility:public"],
    copts = ["--coverage"],  # Add this line to enable coverage
    linkopts = ["--coverage"],  # Add this line to enable coverage
)