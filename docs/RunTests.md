## Run tests using Catch2 and Bazel

```sh
    sudo apt-get install lcov
    bazel coverage --test_output=all --combined_report=lcov //...
    genhtml -o coverage_report bazel-out/_coverage/_coverage_report.dat
```

can't run test coverage using root user

open coverage_report/index.html