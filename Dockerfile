FROM registry.cn-guangzhou.aliyuncs.com/zijiechenrobotics/ros:noetic-desktop-full

ARG USER_ID
ARG GID

# set time zone
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' > /etc/timezone

# config English environment
ENV LANG=en_US.UTF-8

# remove old ros public key
RUN rm /etc/apt/sources.list.d/ros1-latest.list && \    
    rm /usr/share/keyrings/ros1-latest-archive-keyring.gpg && \    
    apt-get update 
RUN apt-get install -y curl
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&\    
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update

# gcc9 g++9
RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt-get install -y gcc-9 g++-9
# switch to gcc9 & g++9
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 10
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 10

# tools
RUN apt-get update && \
    apt-get install -y libgoogle-glog-dev \
    git \ 
    wget \
    psmisc \
    inxi \
    pip \
    pcl-tools

# livox sdk1
COPY ./docker/Livox-SDK /root/software/Livox-SDK
WORKDIR /root/software/Livox-SDK/build
RUN cmake .. && make -j8 && make install

# soucre ROS的环境
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"
WORKDIR /root/workspace