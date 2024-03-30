# Installation Guide

## System Requirement

This project was tested on NVIDIA AGX Orin w/ **JetPack 6**. If you're
using non-Orin platform, Ubuntu 22.04 is recommended, and it's best to
have a CUDA-enabled graphics card.

- ROS 2 Humble

  Please follow the official installation
  [guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

- Blickfeld Scanner Library 2.20.6

  There is no official deb package provided for JetPack 6. It's
  recommended to follow the instructions in the
  [appendix](#appendix-install-blickfeld-scanner-library) at the end
  of this article to install the library.

- ZED SDK 4.0.8

  This is required for the ZED X mini camera. Please visit the
  official [download
  page](https://www.stereolabs.com/developers/release) and download
  the _ZED SDK for Ubuntu 22 4.0.8_ version.

## Prepare F1EIGHTH Workspace

Clone this repository and its submodules.

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/F1EIGHT.git
cd F1EIGHT
```

## Build This Project

This is a meta step for all following commands. Always enable ROS
environment whenever you start a new shell.

```bash
source /opt/ros/humble/setup.bash
```

Install required dependencies.

```bash
make prepare
```

Build the whole project.

```bash
make build
```



## Appendix: Install Blickfeld Scanner Library

Download the source repository.

```bash
git clone --recurse-submodules --branch v2.20.6 https://github.com/Blickfeld/blickfeld-scanner-lib.git
cd blickfeld-scanner-lib
```

Compile and install this library.

```bash
cmake -S . -B build && \
cmake --build build --parallel && \
sudo cmake --install build
```

### (Optional) Install to Home Directory

If you prefer installing this library to home directory instead,
please patch this line at the end of `python/CMakeLists.txt` first.

```bash
     add_custom_target(blickfeld-scanner-python ALL DEPENDS ${OUTPUT})
 
-    install(CODE "execute_process(WORKING_DIRECTORY \"${CMAKE_CURRENT_BINARY_DIR}\" COMMAND ${Python_EXECUTABLE} ${SETUP_PY} install)")
+    install(CODE "execute_process(WORKING_DIRECTORY \"${CMAKE_CURRENT_BINARY_DIR}\" COMMAND ${Python_EXECUTABLE} ${SETUP_PY} install --user)")
 endif()
```

Configure CMake with our prefix path and install this library.

```bash
cmake -S . -B build -DCMAKE_INSTALL_PREFIX:PATH=$HOME/.local && \
cmake --build build --parallel && \
cmake --install build
```

Append these lines to your favorite shell rc (.bashrc, etc) and
restart your shell. It ensures your later compilation steps can find
the Blickfeld Scanner Library.

```bash
export PATH="$HOME/.local/bin:$PATH"
export CPATH="$HOME/.local/include:$CPATH"
export LIBRARY_PATH="$HOME/.local/lib:$LIBRARY_PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"
```
