FROM nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel

# Update system packages
RUN apt update -y
RUN apt upgrade -y

# Configure the locale and timezone
RUN apt install -y locales
RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen
RUN locale-gen
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN ln -snf /usr/share/zoneinfo/Asia/Taipei /etc/localtime && echo Asia/Taipei > /etc/timezone

# Enable 'universe' repository
RUN apt install -y software-properties-common
RUN apt install -y apt-transport-https
RUN add-apt-repository universe

# Install ROS Humble
RUN apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'

RUN apt update -y
RUN apt install -y ros-humble-desktop
RUN apt install -y ros-dev-tools
RUN apt install -y ros-humble-pacmod3
RUN apt install -y ros-humble-rmw-cyclonedds-cpp

# Install APT packages
RUN apt install -y git
RUN apt install -y sudo
RUN apt install -y ccache git-lfs golang ros-humble-plotjuggler-ros clang-format geographiclib-tools
RUN geographiclib-get-geoids egm2008-1
RUN apt install -y python3-pip

# Remove /usr/local CMake binaries 
RUN rm /usr/local/bin/cmake /usr/local/bin/cpack /usr/local/bin/ctest

# Initialize rosdep
RUN rosdep init

# Create the 'jetson' user
RUN useradd -m -u 1000 jetson
RUN usermod -aG sudo jetson
RUN passwd --delete jetson

# Switch to the 'jetson' user
USER jetson
WORKDIR /home/jetson

# Update rosdep packages
RUN rosdep update --rosdistro=humble

# Install Git LFS
RUN git lfs install

# Install user local dependencies
RUN pip3 install pre-commit
RUN pip3 install gdown

# Set envs
RUN echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/jetson/.bashrc
RUN echo '' >> ~/.bashrc && echo "source /opt/ros/humble/setup.bash" >> /home/jetson/.bashrc

# Install ultimate bashrc scripts.
RUN wget -O ~/.bashrc https://sourceforge.net/projects/ultimate-bashrc/files/_bashrc/download
RUN wget -O ~/.bashrc_help https://sourceforge.net/projects/ultimate-bashrc/files/_bashrc_help/download

# Install rosdep dependencies
# COPY --chmod=644 apt.txt /home/jetson/apt.txt
# COPY --chmod=644 pip.txt /home/jetson/pip.txt
# RUN sudo apt install -y $(cat apt.txt)
# RUN pip install -U $(cat pip.txt)

# Install Blickfeld Scanner library
COPY --chmod=755 scripts/install-blickfeld-scanner-library.sh /home/jetson/install-blickfeld-scanner-library.sh
RUN ./install-blickfeld-scanner-library.sh

# Copy the build script
COPY --chmod=755 scripts/build.sh /home/jetson/build.sh

# Set default process to bash
CMD ["/bin/bash"]
