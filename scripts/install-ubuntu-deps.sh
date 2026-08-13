#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  apt=(apt-get)
elif command -v sudo >/dev/null 2>&1; then
  apt=(sudo apt-get)
else
  echo "Run as root or install sudo first." >&2
  exit 1
fi

"${apt[@]}" update
"${apt[@]}" install -y --no-install-recommends \
  build-essential \
  ca-certificates \
  clang-format \
  cmake \
  gdb \
  git \
  libceres-dev \
  libeigen3-dev \
  libgtest-dev \
  libopencv-dev \
  libpcl-dev \
  libgl1-mesa-dev \
  libyaml-cpp-dev \
  ninja-build \
  nlohmann-json3-dev \
  pkg-config \
  python3 \
  python3-pip \
  python3-venv \
  qt6-base-dev \
  qt6-base-dev-tools \
  sudo

echo "Ubuntu dependencies installed. Run ./scripts/verify-dev-env.sh next."
