#!/bin/bash

projectDir=JetsonNano
UserName=team02
IpAddress=10.21.221.71
PathBin=/opt/vehicle/bin
PathEtc=/opt/vehicle/etc/zenoh
Pass=seameteam2
architecture=$(uname -m)


check_ssh_connection() {
    local host=$1
    local user=$2
    echo "Checking connection to $host..."
    # Use nc (netcat) to test connection with 5 second timeout
    if nc -G 20 -z "$host" 22 >/dev/null 2>&1; then
        # Test SSH login
        if sshpass -p "$Pass" ssh -q -o ConnectTimeout=2000 "$user@$host" exit; then
            return 0
        fi
    fi
    return 1
}


# Build docker image with appropriate platform flag
echo "Building docker image to build app..."
if [ "$architecture" = "arm64" ] || [ "$architecture" = "aarch64" ]; then
    echo "Building for ARM64 architecture..."
    docker build -f ./tests/DockerfileDeployJetson \
        --build-arg projectDir=/$projectDir \
        -t final-app .
else
    echo "Building for non-ARM64 architecture with platform emulation..."
    docker buildx build -f ./tests/DockerfileDeployTests \
        --platform linux/arm64 --load \
        --build-arg projectDir=/$projectDir \
        -t final-app .
fi

docker rm -f tmpapp
docker create --name tmpapp final-app

docker start tmpapp
# docker exec tmpapp bash -c "apt-get install -y lcov && cd /home/$projectDir && bazel coverage --test_output=all --combined_report=lcov //... && genhtml -o coverage_report bazel-out/_coverage/_coverage_report.dat"


# echo "Copy the coverage report from tmp container"
# mkdir -p coverage_report
# docker cp tmpapp:/home/$projectDir/coverage_report/. ./coverage_report

