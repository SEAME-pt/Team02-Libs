## Run tests using Catch2 and Bazel

```sh
    sudo apt-get install lcov
    bazel coverage --test_output=all --combined_report=lcov //Communication/I2C/tests:i2c_tests
    genhtml -o coverage_report bazel-out/_coverage/_coverage_report.dat
```

can't ruin test coverage using root user

open coverage_report/index.html