#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
work_dir="$script_dir/build-blickfeld-scanner-library"
mkdir "$work_dir"
cd "$work_dir"

sudo apt update
sudo apt install -y \
     git \
     build-essential \
     libprotobuf-dev \
     libprotoc-dev \
     protobuf-compiler

git clone --recursive https://github.com/Blickfeld/blickfeld-scanner-lib.git
cd blickfeld-scanner-lib

cmake -S . -B build
cmake --build build --parallel
sudo cmake --install build
