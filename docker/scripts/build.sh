#!/usr/bin/env bash
set -e

print_usage() {
    echo "Usage: $0 SRC_DIR BUILD_DIR"
}


src_dir="$1"
shift || {
    print_usage
    exit 1
}

build_dir="$1"
shift || {
    print_usage
    exit 1
}

cp -r "$src_dir/" "$build_dir"
cd "$build_dir"
make prepare
make build
