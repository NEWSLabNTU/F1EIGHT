#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

pkgver=4.8.1

download() {
    aria2c -c --auto-file-renaming=false "$1"
}

# Download source files
download "https://github.com/opencv/opencv/archive/refs/tags/$pkgver.tar.gz"
download "https://github.com/opencv/opencv_contrib/archive/refs/tags/$pkgver.tar.gz"

# Unpack downloaded files
bsdtar -xvf opencv-$pkgver.tar.gz
bsdtar -xvf opencv_contrib-$pkgver.tar.gz
patch -d opencv-$pkgver -p1 < cuda-12.2.patch # Fix build with CUDA 12.2

# Configure CMake options
_opts="-DWITH_OPENCL=ON \
         -DWITH_OPENGL=ON \
         -DOpenGL_GL_PREFERENCE=LEGACY \
         -DCMAKE_CXX_STANDARD=17 \
         -DWITH_TBB=ON \
         -DWITH_VULKAN=ON \
         -DWITH_QT=ON \
         -DBUILD_TESTS=OFF \
         -DBUILD_PERF_TESTS=OFF \
         -DBUILD_EXAMPLES=ON \
         -DBUILD_PROTOBUF=OFF \
         -DPROTOBUF_UPDATE_FILES=ON \
         -DINSTALL_C_EXAMPLES=ON \
         -DINSTALL_PYTHON_EXAMPLES=ON \
         -DOPENCV_EXTRA_MODULES_PATH=$PWD/opencv_contrib-$pkgver/modules \
         -DOPENCV_SKIP_PYTHON_LOADER=ON \
         -DOPENCV_GENERATE_PKGCONFIG=ON \
         -DOPENCV_ENABLE_NONFREE=ON \
         -DOPENCV_GENERATE_SETUPVARS=OFF \
         -DEIGEN_INCLUDE_PATH=/usr/include/eigen3 \
         -DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON \
         -Dprotobuf_MODULE_COMPATIBLE=ON"

# Build the source code
env \
    CFLAGS="${CFLAGS} -fno-lto" \
    CXXFLAGS="${CXXFLAGS} -fno-lto" \
    LDFLAGS="${LDFLAGS} -fno-lto" \
    cmake -B build -S opencv-$pkgver $_opts \
    -DBUILD_WITH_DEBUG_INFO=OFF \
    -DWITH_CUDA=ON \
    -DWITH_CUDNN=ON \
    -DCMAKE_C_COMPILER=gcc-12 \
    -DCMAKE_CXX_COMPILER=g++-12 \
    -DCUDA_ARCH_BIN='52-real;53-real;60-real;61-real;62-real;70-real;72-real;75-real;80-real;86-real;87-real;89-real;90-real;90-virtual' \
    -DCUDA_ARCH_PTX='90-virtual'
cmake --build build --parallel

# Install
cmake --install build
