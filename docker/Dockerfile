FROM nvcr.io/nvidia/l4t-cuda:12.2.12-devel

RUN apt update -y
RUN apt upgrade -y

RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN ln -snf /usr/share/zoneinfo/Asia/Taipei /etc/localtime && echo Asia/Taipei > /etc/timezone

RUN apt install -y software-properties-common
RUN apt install -y apt-transport-https
RUN add-apt-repository universe

RUN apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'

RUN apt update -y
RUN apt install -y ros-humble-desktop
RUN apt install -y ros-dev-tools
RUN apt install -y ros-humble-pacmod3
RUN apt install -y ros-humble-rmw-cyclonedds-cpp

RUN apt install -y git
RUN apt install -y sudo
RUN apt install -y ccache git-lfs golang ros-humble-plotjuggler-ros clang-format geographiclib-tools
RUN geographiclib-get-geoids egm2008-1
RUN apt install -y python3-pip

RUN rosdep init

RUN useradd -m -u 1000 jetson
RUN usermod -aG sudo jetson
USER jetson
WORKDIR /home/jetson

RUN rosdep update --rosdistro=humble
RUN git lfs install

RUN pip3 install pre-commit
RUN pip3 install gdown
RUN echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/jetson/.bashrc
RUN echo '' >> ~/.bashrc && echo "source /opt/ros/humble/setup.bash" >> /home/jetson/.bashrc

RUN wget -O ~/.bashrc https://sourceforge.net/projects/ultimate-bashrc/files/_bashrc/download
RUN wget -O ~/.bashrc_help https://sourceforge.net/projects/ultimate-bashrc/files/_bashrc_help/download

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh /dev/stdin -t stable -y

USER root
RUN passwd --delete jetson

USER jetson

COPY --chmod=755 build.sh /home/jetson/build.sh

CMD ["/bin/bash"]
