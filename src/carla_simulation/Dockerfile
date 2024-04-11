FROM nvidia/opengl:base-ubuntu18.04

WORKDIR /opt/workspace

RUN mkdir src && mkdir share

RUN apt update && \
    apt install -y \
        software-properties-common \
        ca-certificates \
        gpg \
        wget \
        curl
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
        gpg --dearmor - | \
        tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'

RUN apt update && \
    apt install -y cmake \
        libjsoncpp-dev \
        python3.8 \
        python3.8-venv \
        python3-pip

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive \
    apt install -y \
        build-essential \
        software-properties-common \
        ros-melodic-desktop-full
RUN apt install -y \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool
RUN rosdep init
RUN rosdep update
RUN rosdep fix-permissions

RUN apt update && \
    apt install -y \
        python3-catkin-pkg-modules \
        python3-rospkg-modules

RUN python3.8 -m pip install --upgrade pip

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

WORKDIR /

RUN apt update && \
    apt install -y \
        git

RUN git clone https://github.com/carla-simulator/carla.git /opt/CARLA/Simulator
RUN git clone https://github.com/carla-simulator/scenario_runner.git /opt/CARLA/Runner
RUN git clone https://git.fortiss.org/opensbt/carla-runner.git /opt/OpenSBT/Runner
RUN git clone https://git.fortiss.org/ff1/rosco.git /opt/workspace/src/rosco

WORKDIR /opt/workspace/src/rosco

RUN git checkout tags/2.0
