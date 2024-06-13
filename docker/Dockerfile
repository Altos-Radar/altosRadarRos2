FROM osrf/ros:humble-desktop-full

# updates from toolchain 
RUN apt-get update && apt-get dist-upgrade -y

RUN apt-get update && apt-get install -y \
    apt-utils \
    apt-transport-https \
    software-properties-common \
    locales \
    wget \
    curl \
    gnupg \
    locate \
    lsb-release \
    git \
    dbus-x11 \
    gnome-terminal \
    openssh-client \
    doxygen \
    net-tools \
    libgoogle-glog-dev \
    libgflags-dev \
    libgtest-dev \
    libtbb-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libeigen3-dev \
    libglew-dev \
    liblz4-dev \
    libbz2-dev \
    libboost-all-dev \
    libboost-regex-dev \
    libboost-filesystem-dev \
    libboost-date-time-dev \
    libboost-program-options-dev \
    lsb-core \
    gcovr \
    # ggcov \
    lcov \
    graphviz \
    libfmt-dev \
    ffmpeg \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff5-dev \
    libopencv-dev \
    libopenexr-dev \
    libpcap-dev \
    libv4l-dev

# RUN apt-get clean all

# C++
RUN apt-get update && apt-get install -y \
    build-essential \
    g++-11 \
    libc++-dev \
    cmake \
    pkg-config \
    git-core \
    git-lfs \
    ccache 

# Python
# RUN apt-get update && apt-get install -y \
#     python3 \
#     python3-pip \
#     libpython3-all-dev
# RUN pip3 install -U pip setuptools
# RUN pip3 install -U cmake cmake_format
# RUN pip3 install -U yapf
# RUN pip3 install -U pybind11
# ENV PATH="/usr/local/bin:${PATH}"

# ROS packages
RUN apt-get update && apt-get install -y \
    ros-humble-usb-cam \
    ros-humble-rosbag2-cpp \
    ros-humble-rqt

# WORKDIR /home/altos/
