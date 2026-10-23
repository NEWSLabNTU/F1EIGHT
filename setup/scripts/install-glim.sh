#!/usr/bin/env bash
# Install GLIM (LiDAR-IMU mapping) from koide3's PPA, for `just lab3 map`.
#
# Two variants, and they do not coexist -- the CUDA packages conflict with the
# CPU ones of the same name:
#
#   cuda  ros-humble-glim-ros-cuda12.6 + libgtsam-points-cuda12.6-dev
#         JetPack 6.2 and the workstation both carry CUDA 12.6.
#   cpu   ros-humble-glim-ros + libgtsam-points-dev
#         a machine with no NVIDIA GPU; `lab3_map --cpu` selects the matching
#         modules either way.
#
# Auto-detected (a Jetson, or nvidia-smi on PATH, means cuda); override with
# AUTOSDV_GLIM_VARIANT=cpu|cuda.
set -euo pipefail

if [[ -f /opt/ros/humble/share/glim_ros/package.xml ]]; then
    echo "GLIM is already installed: $(dpkg-query -W -f='${Package} ${Version}\n' \
        'ros-humble-glim-ros*' 2>/dev/null | head -1)"
    exit 0
fi

variant="${AUTOSDV_GLIM_VARIANT:-}"
if [[ -z "${variant}" ]]; then
    if [[ -f /etc/nv_tegra_release ]] || command -v nvidia-smi >/dev/null 2>&1; then
        variant=cuda
    else
        variant=cpu
    fi
fi

case "${variant}" in
    cuda) pkgs=(libgtsam-points-cuda12.6-dev ros-humble-glim-ros-cuda12.6) ;;
    cpu)  pkgs=(libgtsam-points-dev ros-humble-glim-ros) ;;
    *)    echo "AUTOSDV_GLIM_VARIANT must be cpu or cuda, got '${variant}'" >&2; exit 2 ;;
esac

echo "Installing GLIM (${variant} variant)"

# The PPA script adds the key and the source list, then runs apt-get update.
curl -fsSL https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

sudo apt-get install -y --no-install-recommends \
    libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev "${pkgs[@]}"

# gtsam_points lands in /usr/local/lib, which the loader cache does not know
# about until ldconfig runs; glim_rosbag then fails to start with a missing
# libgtsam_points.so.1.
sudo ldconfig

echo "GLIM installed: $(dpkg-query -W -f='${Package} ${Version}\n' 'ros-humble-glim-ros*' | head -1)"
