FROM osrf/ros:humble-desktop

ARG USER_ID
ARG GID

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
RUN update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    curl \
    git \
    wget \
    psmisc \
    inxi \
    python3-pip \
    pcl-tools \
    libgoogle-glog-dev \
    software-properties-common

RUN add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update && \
    apt-get install -y gcc-9 g++-9

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 10
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 10

RUN apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential

# COPY ./docker/Livox-SDK2 /root/software/Livox-SDK2
# WORKDIR /root/software/Livox-SDK2/build
# RUN cmake .. && make -j8 && make install

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /root/workspace