#!/usr/bin/env bash
set -euo pipefail

echo "OS:       $(. /etc/os-release && echo "${PRETTY_NAME}")"
echo "Compiler: $(g++ --version | head -n 1)"
echo "CMake:    $(cmake --version | head -n 1)"
echo "Ninja:    $(ninja --version)"
echo "Python:   $(python3 --version)"

for package in eigen3 opencv4 pcl_common yaml-cpp; do
    printf '%-10s %s\n' \
        "${package}:" \
        "$(pkg-config --modversion "${package}")"
done

echo "Development environment is ready."
