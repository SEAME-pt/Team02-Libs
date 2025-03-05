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
    docker buildx build -f ./tests/DockerfileDeployJetson \
        --platform linux/arm64 --load \
        --build-arg projectDir=/$projectDir \
        -t final-app .
fi

echo "Remove tmpapp container if it is exist"
docker rm -f tmpapp
echo "Create a tmp container to copy binary"
docker create --name tmpapp final-app
echo "Copy the binary from tmp container"
mkdir package
mkdir package/bazel-bin
mkdir package/bazel-out
mkdir package/bazel-testlogs
docker cp tmpapp:/home/$projectDir/package/bazel-bin/. ./package/bazel-bin
docker cp tmpapp:/home/$projectDir/package/bazel-out/. ./package/bazel-out
docker cp tmpapp:/home/$projectDir/package/bazel-testlogs/. ./package/bazel-testlogs

if check_ssh_connection "$IpAddress" "$UserName"; then
    
    echo "Copying files to JetsonNano..."
    sshpass -p "$Pass" scp -r ./package/* "$UserName"@"$IpAddress":"/home/team02"
    
    echo "Running tests on JetsonNano..."
    sshpass -p "$Pass" ssh -t "$UserName"@"$IpAddress" "
        cd $PathBin &&
        bazel coverage --test_output=all --combined_report=lcov //... &&
        genhtml -o $PathCoverage bazel-out/_coverage/_coverage_report.dat
    "
    
    echo "Copying coverage report back to local machine..."
    sshpass -p "$Pass" scp -r "$UserName"@"$IpAddress":"$PathCoverage" ./coverage_report
    
#     echo "Opening coverage report in local browser..."
#     report_path=$(wslpath -w $(pwd)/coverage_report/index.html)
#     cmd.exe /c start $report_path
else
    echo "ERROR: Cannot connect to $UserName at $IpAddress"
fi
